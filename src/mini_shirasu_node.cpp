/// @file mini_shirasu_node.cpp
/// mini-shirasu (ブラシ付き DC モータドライバ) 1 枚と、robomas_plugins の CAN ブリッジ越しに話すノード。
///
/// - `robomas_can_tx` / `robomas_can_rx` (robomas_plugins/msg/Frame) でフレームをやりとりする
/// - 起動時 (と `~/reconfigure`) に、SetMode(無効) -> 全設定の SetParam -> SetMode(mode) を、
///   1 つずつ Ack を待ちながら送る
/// - 目標値を `~/target_current` などで受け、`control_rate` の周期で送り続ける。
///   ファームには通信タイムアウトが無いので、目標が `target_timeout` 以上途切れたら
///   電流・速度モードではゼロを送る (位置モードでは最後の目標のまま)
/// - Status を `~/status` に出す。Status が `status_timeout` 以上途切れたら
///   基板が再起動したとみなして設定し直す
///
/// 符号化・復号は ROS 非依存 (protocol.cpp)。ここは ROS の入出力と手順だけを見る。

#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <format>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <robomas_plugins/msg/frame.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "mini_shirasu_ros/msg/status.hpp"
#include "mini_shirasu_ros/protocol.hpp"
#include "mini_shirasu_ros/settings.hpp"

namespace {
	namespace proto = mini_shirasu_ros::protocol;
	using robomas_plugins::msg::Frame;

	auto mode_from_name(const std::string& name) -> std::uint8_t {
		if (name == "current") { return proto::mode::current; }
		if (name == "velocity") { return proto::mode::velocity; }
		if (name == "position") { return proto::mode::position; }
		throw std::invalid_argument(std::format("mode must be current, velocity or position (got '{}')", name));
	}

	auto nack_reason_text(const std::uint8_t reason) -> const char* {
		switch (reason) {
		case proto::nack_reason::bad_frame: return "bad frame";
		case proto::nack_reason::output_enabled: return "output is enabled";
		case proto::nack_reason::config_error: return "configuration error";
		case proto::nack_reason::fault_latched: return "fault is latched";
		case proto::nack_reason::invalid_value: return "invalid value";
		case proto::nack_reason::nfault_stuck: return "NFAULT did not clear";
		default: return "unknown reason";
		}
	}

	auto command_text(const proto::Message& m) -> std::string {
		return std::visit(
			[]<class T>(const T& c) -> std::string {
				if constexpr (std::is_same_v<T, proto::SetMode>) {
					return std::format("SetMode({})", c.mode);
				} else if constexpr (std::is_same_v<T, proto::SetParam>) {
					const auto name = mini_shirasu_ros::setting_name(c.id);
					return std::format("SetParam({} = {})", name ? *name : "?", c.value);
				} else if constexpr (std::is_same_v<T, proto::ResetFault>) {
					return "ResetFault";
				} else if constexpr (std::is_same_v<T, proto::SetOrigin>) {
					return "SetOrigin";
				} else {
					return "?";
				}
			},
			m
		);
	}

	class MiniShirasuNode final : public rclcpp::Node {
	public:
		MiniShirasuNode() : rclcpp::Node("mini_shirasu_node") {
			// --- 起動時のみ ---
			const auto tx_topic = this->declare_parameter<std::string>("can_tx_topic", "robomas_can_tx");
			const auto rx_topic = this->declare_parameter<std::string>("can_rx_topic", "robomas_can_rx");
			this->target_id_ = static_cast<std::uint32_t>(this->declare_parameter<std::int64_t>("can_id.target", 0x110));
			this->status_id_ = static_cast<std::uint32_t>(this->declare_parameter<std::int64_t>("can_id.status", 0x120));
			this->command_id_ = static_cast<std::uint32_t>(this->declare_parameter<std::int64_t>("can_id.command", 0x130));
			this->response_id_ =
				static_cast<std::uint32_t>(this->declare_parameter<std::int64_t>("can_id.response", 0x140));
			this->mode_ = mode_from_name(this->declare_parameter<std::string>("mode", "velocity"));
			this->want_enabled_ = this->declare_parameter<bool>("enable_on_start", true);
			const double rate = this->declare_parameter<double>("control_rate", 50.0);
			if (!(rate > 0.0)) {
				throw std::invalid_argument("control_rate must be positive");
			}

			this->load_settings();

			// --- 実行中に変えられる ---
			this->declare_parameter<double>("target_timeout", 0.2);
			this->declare_parameter<double>("command_timeout", 0.1);
			this->declare_parameter<int>("command_retries", 3);
			this->declare_parameter<double>("status_timeout", 0.5);
			this->read_runtime_params();
			this->param_cb_ = this->add_post_set_parameters_callback(
				[this](const std::vector<rclcpp::Parameter>&) { this->read_runtime_params(); }
			);

			// --- 入出力 ---
			this->tx_pub_ = this->create_publisher<Frame>(tx_topic, 100);
			this->status_pub_ = this->create_publisher<mini_shirasu_ros::msg::Status>("~/status", 10);
			this->rx_sub_ = this->create_subscription<Frame>(
				rx_topic, 100, [this](const Frame& f) { this->on_frame(f); }
			);
			this->target_subs_.push_back(this->create_subscription<std_msgs::msg::Float64>(
				"~/target_current", 10,
				[this](const std_msgs::msg::Float64& m) { this->on_target(proto::mode::current, m.data); }
			));
			this->target_subs_.push_back(this->create_subscription<std_msgs::msg::Float64>(
				"~/target_velocity", 10,
				[this](const std_msgs::msg::Float64& m) { this->on_target(proto::mode::velocity, m.data); }
			));
			this->target_subs_.push_back(this->create_subscription<std_msgs::msg::Float64>(
				"~/target_position", 10,
				[this](const std_msgs::msg::Float64& m) { this->on_target(proto::mode::position, m.data); }
			));

			this->enable_srv_ = this->create_service<std_srvs::srv::SetBool>(
				"~/enable",
				[this](
					const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
					std::shared_ptr<std_srvs::srv::SetBool::Response> res
				) {
					this->want_enabled_ = req->data;
					if (!this->configured_ && !this->configuring_) {
						res->success = false;
						res->message = "the board is not configured; call ~/reconfigure (or ~/reset_fault)";
						return;
					}
					this->enqueue(proto::SetMode{.mode = req->data ? this->mode_ : proto::mode::disabled});
					res->success = true;
					res->message = "queued";
				}
			);
			this->reset_fault_srv_ = this->create_service<std_srvs::srv::Trigger>(
				"~/reset_fault",
				[this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
					if (this->configured_) {
						this->enqueue(proto::ResetFault{});
						if (this->want_enabled_) {
							this->enqueue(proto::SetMode{.mode = this->mode_});
						}
					} else {
						// 起動時に異常ラッチ中だった場合など。解除してから設定し直す
						this->start_configuration(true);
					}
					res->success = true;
					res->message = "queued";
				}
			);
			this->set_origin_srv_ = this->create_service<std_srvs::srv::Trigger>(
				"~/set_origin",
				[this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
					// SetOrigin は無効中しか受け付けないので、いったん止める
					this->enqueue(proto::SetMode{.mode = proto::mode::disabled});
					this->enqueue(proto::SetOrigin{});
					if (this->configured_ && this->want_enabled_) {
						this->enqueue(proto::SetMode{.mode = this->mode_});
					}
					res->success = true;
					res->message = "queued";
				}
			);
			this->reconfigure_srv_ = this->create_service<std_srvs::srv::Trigger>(
				"~/reconfigure",
				[this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
					this->start_configuration();
					res->success = true;
					res->message = "queued";
				}
			);

			this->timer_ = this->create_wall_timer(
				std::chrono::duration<double>(1.0 / rate), [this] { this->on_timer(); }
			);

			this->start_configuration();
		}

		~MiniShirasuNode() override {
			// 終了時は出力を止める (届くかは保証できない)
			try {
				this->send(this->command_id_, proto::SetMode{.mode = proto::mode::disabled});
			} catch (...) {
			}
		}

	private:
		struct Pending {
			proto::Message command;
			int attempts{};
			rclcpp::Time sent_at{};
		};

		void load_settings() {
			for (const auto& s : mini_shirasu_ros::settings) {
				const std::string name = std::format("settings.{}", s.name);
				this->declare_parameter(name, rclcpp::ParameterType::PARAMETER_DOUBLE);
				rclcpp::Parameter p{};
				if (this->get_parameter(name, p) && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
					this->settings_.push_back(proto::SetParam{.id = s.id, .value = static_cast<float>(p.as_double())});
				} else if (s.required) {
					throw std::invalid_argument(std::format("parameter '{}' is not set", name));
				}
			}
		}

		void read_runtime_params() {
			this->target_timeout_ = this->get_parameter("target_timeout").as_double();
			this->command_timeout_ = this->get_parameter("command_timeout").as_double();
			this->command_retries_ = static_cast<int>(this->get_parameter("command_retries").as_int());
			this->status_timeout_ = this->get_parameter("status_timeout").as_double();
		}

		/// (ResetFault ->) SetMode(無効) -> 全設定 -> (有効なら) SetMode(mode)
		void start_configuration(const bool reset_fault_first = false) {
			this->queue_.clear();
			this->pending_.reset();
			this->configured_ = false;
			this->enabled_ = false;
			this->retry_configuration_ = false;
			this->last_status_.reset();
			this->last_attempt_ = this->now();

			if (reset_fault_first) {
				this->enqueue(proto::ResetFault{});
			}
			this->enqueue(proto::SetMode{.mode = proto::mode::disabled});
			for (const auto& s : this->settings_) {
				this->enqueue(s);
			}
			this->configuring_ = true;
			if (this->want_enabled_) {
				this->enqueue(proto::SetMode{.mode = this->mode_});
			}
		}

		void enqueue(proto::Message command) {
			this->queue_.push_back(std::move(command));
			this->send_next_command();
		}

		void send_next_command() {
			if (this->pending_ || this->queue_.empty()) {
				return;
			}
			this->pending_ = Pending{.command = this->queue_.front(), .attempts = 1, .sent_at = this->now()};
			this->queue_.pop_front();
			this->send(this->command_id_, this->pending_->command);
		}

		void send(const std::uint32_t id, const proto::Message& message) {
			for (const auto& chunk : proto::split_into_frames(proto::encode(message))) {
				Frame f{};
				f.id = id;
				f.is_rtr = false;
				f.is_extended = false;
				f.is_error = false;
				f.dlc = chunk.dlc;
				std::copy(chunk.data.begin(), chunk.data.end(), f.data.begin());
				this->tx_pub_->publish(f);
			}
		}

		void on_target(const std::uint8_t mode, const double value) {
			if (mode != this->mode_) {
				RCLCPP_WARN_THROTTLE(
					this->get_logger(), *this->get_clock(), 5000,
					"ignoring a target for mode %u; this node runs in mode %u", mode, this->mode_
				);
				return;
			}
			if (!std::isfinite(value)) {
				return;
			}
			this->target_ = value;
			this->target_received_ = this->now();
		}

		void on_timer() {
			const auto now = this->now();

			// コマンドの再送とタイムアウト
			if (this->pending_ && (now - this->pending_->sent_at).seconds() > this->command_timeout_) {
				if (this->pending_->attempts > this->command_retries_) {
					if (this->configured_) {
						RCLCPP_ERROR(
							this->get_logger(), "%s: no response; giving up",
							command_text(this->pending_->command).c_str()
						);
					} else {
						RCLCPP_WARN(
							this->get_logger(), "%s: no response; retrying the configuration in 1 s",
							command_text(this->pending_->command).c_str()
						);
					}
					// 基板やブリッジがまだ起きていないだけかもしれないので、設定中なら後でやり直す
					this->retry_configuration_ = !this->configured_;
					this->abort_commands();
				} else {
					++this->pending_->attempts;
					this->pending_->sent_at = now;
					this->send(this->command_id_, this->pending_->command);
				}
			}

			if (this->retry_configuration_ && (now - this->last_attempt_).seconds() > 1.0) {
				this->start_configuration();
				return;
			}

			// 基板が再起動すると設定が消えて Status も来なくなるので、設定し直す
			if (this->configured_ && this->status_timeout_ > 0.0 && this->last_status_
				&& (now - *this->last_status_).seconds() > this->status_timeout_) {
				RCLCPP_WARN(this->get_logger(), "status timed out; reconfiguring the board");
				this->start_configuration();
				return;
			}

			if (!this->enabled_) {
				return;
			}
			const bool fresh =
				this->target_received_ && (now - *this->target_received_).seconds() <= this->target_timeout_;
			switch (this->mode_) {
			case proto::mode::current:
				this->send(this->target_id_, proto::TargetCurrent{.current = fresh ? static_cast<float>(this->target_) : 0.f});
				break;
			case proto::mode::velocity:
				this->send(
					this->target_id_,
					proto::TargetVelocity{.velocity = fresh ? static_cast<float>(this->target_) : 0.f, .accel_ff = 0.f}
				);
				break;
			case proto::mode::position:
				// 位置はゼロに戻すと動いてしまうので、途切れたら何も送らない (ファームは最後の目標を保つ)
				if (fresh) {
					this->send(
						this->target_id_,
						proto::TargetPosition{.position = proto::revolutions_to_q16_16(this->target_)}
					);
				}
				break;
			default: break;
			}
		}

		void on_frame(const Frame& f) {
			if (f.is_extended || f.is_rtr || f.is_error) {
				return;
			}
			proto::StreamParser* parser = nullptr;
			if (f.id == this->status_id_) {
				parser = &this->status_parser_;
			} else if (f.id == this->response_id_) {
				parser = &this->response_parser_;
			} else {
				return;
			}
			const std::size_t n = std::min<std::size_t>(f.dlc, f.data.size());
			for (std::size_t i = 0; i < n; ++i) {
				if (auto result = parser->push(f.data[i])) {
					if (*result) {
						this->on_message(**result);
					} else {
						RCLCPP_DEBUG(this->get_logger(), "dropped a broken message on id 0x%x", f.id);
					}
				}
			}
		}

		void on_message(const proto::Message& m) {
			if (const auto* s = std::get_if<proto::Status>(&m)) {
				this->on_status(*s);
			} else if (const auto* a = std::get_if<proto::Ack>(&m)) {
				this->on_ack(a->command);
			} else if (const auto* n = std::get_if<proto::Nack>(&m)) {
				this->on_nack(*n);
			} else if (std::holds_alternative<proto::FaultNotice>(m)) {
				RCLCPP_ERROR(this->get_logger(), "driver fault; call ~/reset_fault to recover");
				this->enabled_ = false;
			}
		}

		void on_status(const proto::Status& s) {
			this->last_status_ = this->now();
			mini_shirasu_ros::msg::Status msg{};
			msg.header.stamp = *this->last_status_;
			msg.mode = s.mode;
			msg.saturated_upper = (s.flags & proto::status_flag::saturated_upper) != 0;
			msg.saturated_lower = (s.flags & proto::status_flag::saturated_lower) != 0;
			msg.vdc_low = (s.flags & proto::status_flag::vdc_low) != 0;
			msg.current = s.current;
			msg.velocity = s.velocity;
			msg.position = proto::q16_16_to_revolutions(s.position);
			msg.vdc = s.vdc;
			msg.temperature_raw = s.temp;
			this->status_pub_->publish(msg);
		}

		void on_ack(const std::uint8_t command) {
			if (!this->pending_ || proto::kind_of(this->pending_->command) != command) {
				return;
			}
			const auto done = std::move(this->pending_->command);
			this->pending_.reset();

			if (const auto* m = std::get_if<proto::SetMode>(&done)) {
				this->enabled_ = m->mode != proto::mode::disabled;
				if (this->enabled_) {
					// 有効化したらファームは目標を初期化するので、こちらも古い目標を捨てる
					this->target_received_.reset();
				}
			}
			if (this->configuring_ && this->queue_.empty()
				&& (std::holds_alternative<proto::SetParam>(done) || std::holds_alternative<proto::SetMode>(done))) {
				this->configuring_ = false;
				this->configured_ = true;
				RCLCPP_INFO(this->get_logger(), "configured (%s)", this->enabled_ ? "enabled" : "disabled");
			}
			this->send_next_command();
		}

		void on_nack(const proto::Nack& n) {
			if (!this->pending_ || proto::kind_of(this->pending_->command) != n.command) {
				if (n.reason == proto::nack_reason::bad_frame) {
					RCLCPP_WARN(this->get_logger(), "the board received a broken command frame");
				}
				return;
			}
			std::string detail{};
			if (n.reason == proto::nack_reason::config_error) {
				const auto name = mini_shirasu_ros::setting_name(n.param);
				detail = std::format(" (setting {})", name ? *name : std::format("0x{:02x}", n.param));
			}
			RCLCPP_ERROR(
				this->get_logger(), "%s rejected: %s%s",
				command_text(this->pending_->command).c_str(), nack_reason_text(n.reason), detail.c_str()
			);
			this->abort_commands();
		}

		/// 失敗したら残りのコマンドは送らない (手順が前提を失うので)
		void abort_commands() {
			this->pending_.reset();
			this->queue_.clear();
			this->configuring_ = false;
		}

		// 設定
		std::uint32_t target_id_{};
		std::uint32_t status_id_{};
		std::uint32_t command_id_{};
		std::uint32_t response_id_{};
		std::uint8_t mode_{};
		std::vector<proto::SetParam> settings_{};
		double target_timeout_{};
		double command_timeout_{};
		int command_retries_{};
		double status_timeout_{};

		// 状態
		bool want_enabled_{true};
		bool configuring_{false};
		bool configured_{false};
		bool enabled_{false};
		bool retry_configuration_{false};
		rclcpp::Time last_attempt_{};
		std::deque<proto::Message> queue_{};
		std::optional<Pending> pending_{};
		double target_{};
		std::optional<rclcpp::Time> target_received_{};
		std::optional<rclcpp::Time> last_status_{};
		proto::StreamParser status_parser_{};
		proto::StreamParser response_parser_{};

		rclcpp::Publisher<Frame>::SharedPtr tx_pub_{};
		rclcpp::Publisher<mini_shirasu_ros::msg::Status>::SharedPtr status_pub_{};
		rclcpp::Subscription<Frame>::SharedPtr rx_sub_{};
		std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> target_subs_{};
		rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_{};
		rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_fault_srv_{};
		rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_origin_srv_{};
		rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reconfigure_srv_{};
		rclcpp::TimerBase::SharedPtr timer_{};
		rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_{};
	};
} // namespace

auto main(int argc, char** argv) -> int {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MiniShirasuNode>());
	rclcpp::shutdown();
	return 0;
}
