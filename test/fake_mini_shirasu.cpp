/// @file fake_mini_shirasu.cpp
/// mini-shirasu の模擬基板 (手動テスト用)。robomas_can_tx を受けて robomas_can_rx に返す。
///
/// minishirasu-firm の設計書の状態機械のうち、ノードが使う部分だけを真似る。
/// - SetParam は無効中だけ受け付ける。SetMode は必須の設定が揃っていないと Nack (理由 3)
/// - 速度は目標に 1 次遅れで追従し、位置はその積分。Status を status_period_ms ごとに返す

#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <numbers>
#include <variant>

#include <rclcpp/rclcpp.hpp>
#include <robomas_plugins/msg/frame.hpp>

#include "mini_shirasu_ros/protocol.hpp"
#include "mini_shirasu_ros/settings.hpp"

namespace {
	namespace proto = mini_shirasu_ros::protocol;
	using robomas_plugins::msg::Frame;

	class FakeMiniShirasu final : public rclcpp::Node {
	public:
		FakeMiniShirasu() : rclcpp::Node("fake_mini_shirasu") {
			this->target_id_ = static_cast<std::uint32_t>(this->declare_parameter<std::int64_t>("can_id.target", 0x100));
			this->status_id_ = static_cast<std::uint32_t>(this->declare_parameter<std::int64_t>("can_id.status", 0x101));
			this->command_id_ = static_cast<std::uint32_t>(this->declare_parameter<std::int64_t>("can_id.command", 0x200));
			this->response_id_ =
				static_cast<std::uint32_t>(this->declare_parameter<std::int64_t>("can_id.response", 0x201));
			this->tau_ = this->declare_parameter<double>("tau", 0.05);

			this->rx_pub_ = this->create_publisher<Frame>("robomas_can_rx", 100);
			this->tx_sub_ = this->create_subscription<Frame>(
				"robomas_can_tx", 100, [this](const Frame& f) { this->on_frame(f); }
			);
			this->timer_ = this->create_wall_timer(std::chrono::milliseconds(1), [this] { this->step(); });
		}

	private:
		void on_frame(const Frame& f) {
			proto::StreamParser* parser = f.id == this->target_id_ ? &this->target_parser_
				: f.id == this->command_id_                         ? &this->command_parser_
																	: nullptr;
			if (!parser) {
				return;
			}
			for (std::size_t i = 0; i < f.dlc; ++i) {
				if (auto r = parser->push(f.data[i])) {
					if (!*r) {
						if (parser == &this->command_parser_) {
							this->reply(proto::Nack{.command = 0, .reason = proto::nack_reason::bad_frame});
						}
						continue;
					}
					this->on_message(**r);
				}
			}
		}

		void on_message(const proto::Message& m) {
			if (const auto* t = std::get_if<proto::TargetVelocity>(&m)) {
				if (this->mode_ == proto::mode::velocity) { this->target_ = t->velocity; }
			} else if (const auto* c = std::get_if<proto::TargetCurrent>(&m)) {
				if (this->mode_ == proto::mode::current) { this->target_ = c->current; }
			} else if (const auto* p = std::get_if<proto::SetParam>(&m)) {
				if (this->mode_ != proto::mode::disabled) {
					this->reply(proto::Nack{.command = proto::kind::set_param, .reason = proto::nack_reason::output_enabled});
					return;
				}
				this->params_[p->id] = p->value;
				this->reply(proto::Ack{.command = proto::kind::set_param});
			} else if (const auto* s = std::get_if<proto::SetMode>(&m)) {
				if (this->mode_ == proto::mode::fault) {
					this->reply(proto::Nack{.command = proto::kind::set_mode, .reason = proto::nack_reason::fault_latched});
					return;
				}
				if (s->mode != proto::mode::disabled) {
					for (const auto& info : mini_shirasu_ros::settings) {
						if (info.required && !this->params_.contains(info.id)) {
							this->reply(proto::Nack{
								.command = proto::kind::set_mode,
								.reason = proto::nack_reason::config_error,
								.param = info.id,
							});
							return;
						}
					}
				}
				this->mode_ = s->mode;
				this->target_ = 0.0;
				this->reply(proto::Ack{.command = proto::kind::set_mode});
			} else if (std::holds_alternative<proto::ResetFault>(m)) {
				if (this->mode_ == proto::mode::fault) { this->mode_ = proto::mode::disabled; }
				this->reply(proto::Ack{.command = proto::kind::reset_fault});
			} else if (std::holds_alternative<proto::SetOrigin>(m)) {
				this->position_ = 0.0;
				this->reply(proto::Ack{.command = proto::kind::set_origin});
			}
		}

		void step() {
			constexpr double dt = 1e-3;
			const double target = this->mode_ == proto::mode::velocity ? this->target_ : 0.0;
			this->velocity_ += (target - this->velocity_) * dt / this->tau_;
			this->position_ += this->velocity_ * dt / (2.0 * std::numbers::pi);

			const auto period = this->params_.find(0x24);
			if (period == this->params_.end() || period->second <= 0.f) {
				return;
			}
			if (++this->ticks_ < static_cast<int>(period->second)) {
				return;
			}
			this->ticks_ = 0;
			this->send(this->status_id_, proto::Status{
				.mode = this->mode_,
				.velocity = static_cast<float>(this->velocity_),
				.position = proto::revolutions_to_q16_16(this->position_),
				.vdc = 24.0f,
			});
		}

		void reply(const proto::Message& m) { this->send(this->response_id_, m); }

		void send(const std::uint32_t id, const proto::Message& m) {
			for (const auto& chunk : proto::split_into_frames(proto::encode(m))) {
				Frame f{};
				f.id = id;
				f.dlc = chunk.dlc;
				std::copy(chunk.data.begin(), chunk.data.end(), f.data.begin());
				this->rx_pub_->publish(f);
			}
		}

		std::uint32_t target_id_{}, status_id_{}, command_id_{}, response_id_{};
		double tau_{};
		std::uint8_t mode_{proto::mode::disabled};
		std::map<std::uint8_t, float> params_{};
		double target_{}, velocity_{}, position_{};
		int ticks_{};
		proto::StreamParser target_parser_{}, command_parser_{};
		rclcpp::Publisher<Frame>::SharedPtr rx_pub_{};
		rclcpp::Subscription<Frame>::SharedPtr tx_sub_{};
		rclcpp::TimerBase::SharedPtr timer_{};
	};
}

auto main(int argc, char** argv) -> int {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<FakeMiniShirasu>());
	rclcpp::shutdown();
	return 0;
}
