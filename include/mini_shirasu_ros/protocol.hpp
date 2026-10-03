#pragma once
/// @file protocol.hpp
/// mini-shirasu (minishirasu-firm) の通信プロトコル。ROS非依存。
///
/// 仕様は mini-shirasu2 の docs/superpowers/specs/2026-10-03-minishirasu-firm-design.md の「プロトコル」「CAN」。
///
/// - メッセージは COBS( 種別 1B | ペイロード | CRC-8/SMBUS 1B ) のあとに区切りの 0x00
/// - 数値はリトルエンディアン
/// - CAN ではストリームごとに ID を1つ使い、フレームのデータ (1〜8 バイト) をそのまま連結する。
///   フレーム境界に意味はない

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <span>
#include <variant>
#include <vector>

namespace mini_shirasu_ros::protocol {
	/// 1メッセージの符号化後の最大長 (区切りの 0x00 込み)。パーサのバッファもこの大きさ
	inline constexpr std::size_t max_encoded = 32;

	namespace kind {
		inline constexpr std::uint8_t target_current = 0x01;
		inline constexpr std::uint8_t target_velocity = 0x02;
		inline constexpr std::uint8_t target_position = 0x03;
		inline constexpr std::uint8_t set_mode = 0x10;
		inline constexpr std::uint8_t set_param = 0x11;
		inline constexpr std::uint8_t reset_fault = 0x12;
		inline constexpr std::uint8_t set_origin = 0x13;
		inline constexpr std::uint8_t status = 0x20;
		inline constexpr std::uint8_t ack = 0x30;
		inline constexpr std::uint8_t nack = 0x31;
		inline constexpr std::uint8_t fault_notice = 0x32;
	}

	/// 制御モード (SetMode, Status)
	namespace mode {
		inline constexpr std::uint8_t disabled = 0;
		inline constexpr std::uint8_t current = 1;
		inline constexpr std::uint8_t velocity = 2;
		inline constexpr std::uint8_t position = 3;
		/// Status でだけ使う
		inline constexpr std::uint8_t fault = 4;
	}

	/// Status のフラグ
	namespace status_flag {
		inline constexpr std::uint8_t saturated_upper = 1u << 0;
		inline constexpr std::uint8_t saturated_lower = 1u << 1;
		inline constexpr std::uint8_t vdc_low = 1u << 2;
	}

	/// Nack の理由
	namespace nack_reason {
		inline constexpr std::uint8_t bad_frame = 1;
		inline constexpr std::uint8_t output_enabled = 2;
		inline constexpr std::uint8_t config_error = 3;
		inline constexpr std::uint8_t fault_latched = 4;
		inline constexpr std::uint8_t invalid_value = 5;
		inline constexpr std::uint8_t nfault_stuck = 6;
	}

	struct TargetCurrent {
		float current{};  ///< [A]
		auto operator==(const TargetCurrent&) const -> bool = default;
	};
	struct TargetVelocity {
		float velocity{};  ///< [rad/s]
		float accel_ff{};  ///< [rad/s^2]
		auto operator==(const TargetVelocity&) const -> bool = default;
	};
	struct TargetPosition {
		std::int32_t position{};  ///< [回転] Q16.16
		float velocity_ff{};      ///< [rad/s]
		float accel_ff{};         ///< [rad/s^2]
		auto operator==(const TargetPosition&) const -> bool = default;
	};
	struct SetMode {
		std::uint8_t mode{};
		auto operator==(const SetMode&) const -> bool = default;
	};
	struct SetParam {
		std::uint8_t id{};
		float value{};
		auto operator==(const SetParam&) const -> bool = default;
	};
	struct ResetFault {
		auto operator==(const ResetFault&) const -> bool = default;
	};
	struct SetOrigin {
		auto operator==(const SetOrigin&) const -> bool = default;
	};
	struct Status {
		std::uint8_t mode{};
		std::uint8_t flags{};
		float current{};          ///< [A]
		float velocity{};         ///< [rad/s]
		std::int32_t position{};  ///< [回転] Q16.16
		float vdc{};              ///< [V]
		std::uint16_t temp{};     ///< ADC 生値
		auto operator==(const Status&) const -> bool = default;
	};
	struct Ack {
		std::uint8_t command{};
		auto operator==(const Ack&) const -> bool = default;
	};
	struct Nack {
		std::uint8_t command{};
		std::uint8_t reason{};
		std::uint8_t param{};
		auto operator==(const Nack&) const -> bool = default;
	};
	struct FaultNotice {
		auto operator==(const FaultNotice&) const -> bool = default;
	};

	using Message = std::variant<
		TargetCurrent,
		TargetVelocity,
		TargetPosition,
		SetMode,
		SetParam,
		ResetFault,
		SetOrigin,
		Status,
		Ack,
		Nack,
		FaultNotice>;

	/// メッセージの種別
	auto kind_of(const Message& message) -> std::uint8_t;

	enum class DecodeError {
		cobs,
		too_short,
		crc,
		unknown_kind,
		length,
		overflow,
	};

	/// CRC-8/SMBUS (多項式 0x07、初期値 0、反転なし)
	auto crc8(std::span<const std::uint8_t> data) -> std::uint8_t;

	/// COBS 符号化。区切りの 0x00 は付けない
	auto cobs_encode(std::span<const std::uint8_t> data) -> std::vector<std::uint8_t>;
	/// COBS 復号。区切りの 0x00 は含めないこと
	auto cobs_decode(std::span<const std::uint8_t> data) -> std::optional<std::vector<std::uint8_t>>;

	/// メッセージ -> 送るバイト列 (区切りの 0x00 込み)
	auto encode(const Message& message) -> std::vector<std::uint8_t>;

	/// COBS を外したあとのバイト列 (種別 | ペイロード | CRC) -> メッセージ
	auto decode_body(std::span<const std::uint8_t> body) -> std::expected<Message, DecodeError>;

	/// バイトストリームのパーサ。1バイトずつ push し、0x00 を受けた時点で結果を返す。
	/// バッファがあふれたら次の 0x00 まで読み捨てる
	class StreamParser {
	public:
		auto push(std::uint8_t byte) -> std::optional<std::expected<Message, DecodeError>>;

	private:
		std::array<std::uint8_t, max_encoded> buffer_{};
		std::size_t length_{};
		bool overflowed_{};
	};

	/// 1つの CAN フレームのデータ
	struct CanChunk {
		std::array<std::uint8_t, 8> data{};
		std::uint8_t dlc{};
	};

	/// バイト列を 8 バイトずつの CAN フレームに分ける
	auto split_into_frames(std::span<const std::uint8_t> bytes) -> std::vector<CanChunk>;

	/// 回転数 <-> Q16.16 (最も近い値に丸め、範囲外は飽和)
	auto revolutions_to_q16_16(double revolutions) -> std::int32_t;
	auto q16_16_to_revolutions(std::int32_t q) -> double;
}
