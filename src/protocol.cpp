#include "mini_shirasu_ros/protocol.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
#include <type_traits>

namespace mini_shirasu_ros::protocol {
	namespace {
		/// 種別ごとの、種別 + ペイロード + CRC の長さ
		auto body_length(const std::uint8_t k) -> std::optional<std::size_t> {
			switch (k) {
			case kind::target_current: return 1 + 4 + 1;
			case kind::target_velocity: return 1 + 8 + 1;
			case kind::target_position: return 1 + 12 + 1;
			case kind::set_mode: return 1 + 1 + 1;
			case kind::set_param: return 1 + 5 + 1;
			case kind::reset_fault: return 1 + 0 + 1;
			case kind::set_origin: return 1 + 0 + 1;
			case kind::status: return 1 + 20 + 1;
			case kind::ack: return 1 + 1 + 1;
			case kind::nack: return 1 + 3 + 1;
			case kind::fault_notice: return 1 + 0 + 1;
			default: return std::nullopt;
			}
		}

		/// リトルエンディアンで書く
		class Writer {
		public:
			explicit Writer(const std::uint8_t k) { this->bytes_.push_back(k); }

			void u8(const std::uint8_t v) { this->bytes_.push_back(v); }
			void u16(const std::uint16_t v) { this->le(v); }
			void i32(const std::int32_t v) { this->le(std::bit_cast<std::uint32_t>(v)); }
			void f32(const float v) { this->le(std::bit_cast<std::uint32_t>(v)); }

			auto finish() -> std::vector<std::uint8_t> {
				this->bytes_.push_back(crc8(this->bytes_));
				return std::move(this->bytes_);
			}

		private:
			template <class T>
			void le(const T v) {
				for (std::size_t i = 0; i < sizeof(T); ++i) {
					this->bytes_.push_back(static_cast<std::uint8_t>(v >> (8 * i)));
				}
			}

			std::vector<std::uint8_t> bytes_{};
		};

		/// リトルエンディアンで読む (長さは呼び出し側で検査済み)
		class Reader {
		public:
			explicit Reader(const std::span<const std::uint8_t> payload) : payload_{payload} {}

			auto u8() -> std::uint8_t { return this->payload_[this->pos_++]; }
			auto u16() -> std::uint16_t { return this->le<std::uint16_t>(); }
			auto i32() -> std::int32_t { return std::bit_cast<std::int32_t>(this->le<std::uint32_t>()); }
			auto f32() -> float { return std::bit_cast<float>(this->le<std::uint32_t>()); }

		private:
			template <class T>
			auto le() -> T {
				T v{};
				for (std::size_t i = 0; i < sizeof(T); ++i) {
					v = static_cast<T>(v | (static_cast<T>(this->payload_[this->pos_++]) << (8 * i)));
				}
				return v;
			}

			std::span<const std::uint8_t> payload_;
			std::size_t pos_{};
		};

		template <class... Fs>
		struct Overloaded : Fs... {
			using Fs::operator()...;
		};
	}

	auto kind_of(const Message& message) -> std::uint8_t {
		return std::visit(
			Overloaded{
				[](const TargetCurrent&) { return kind::target_current; },
				[](const TargetVelocity&) { return kind::target_velocity; },
				[](const TargetPosition&) { return kind::target_position; },
				[](const SetMode&) { return kind::set_mode; },
				[](const SetParam&) { return kind::set_param; },
				[](const ResetFault&) { return kind::reset_fault; },
				[](const SetOrigin&) { return kind::set_origin; },
				[](const Status&) { return kind::status; },
				[](const Ack&) { return kind::ack; },
				[](const Nack&) { return kind::nack; },
				[](const FaultNotice&) { return kind::fault_notice; },
			},
			message
		);
	}

	auto crc8(const std::span<const std::uint8_t> data) -> std::uint8_t {
		std::uint8_t crc = 0;
		for (const std::uint8_t byte : data) {
			crc ^= byte;
			for (int bit = 0; bit < 8; ++bit) {
				crc = static_cast<std::uint8_t>((crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1));
			}
		}
		return crc;
	}

	auto cobs_encode(const std::span<const std::uint8_t> data) -> std::vector<std::uint8_t> {
		std::vector<std::uint8_t> out{};
		out.reserve(data.size() + data.size() / 254 + 1);
		std::size_t code_index = 0;
		out.push_back(0);
		std::uint8_t code = 1;
		for (const std::uint8_t byte : data) {
			if (byte == 0) {
				out[code_index] = code;
				code_index = out.size();
				out.push_back(0);
				code = 1;
				continue;
			}
			out.push_back(byte);
			++code;
			if (code == 0xFF) {
				out[code_index] = code;
				code_index = out.size();
				out.push_back(0);
				code = 1;
			}
		}
		out[code_index] = code;
		return out;
	}

	auto cobs_decode(const std::span<const std::uint8_t> data) -> std::optional<std::vector<std::uint8_t>> {
		std::vector<std::uint8_t> out{};
		out.reserve(data.size());
		std::size_t i = 0;
		while (i < data.size()) {
			const std::uint8_t code = data[i];
			if (code == 0 || i + code > data.size()) {
				return std::nullopt;
			}
			for (std::size_t j = 1; j < code; ++j) {
				if (data[i + j] == 0) {
					return std::nullopt;
				}
				out.push_back(data[i + j]);
			}
			i += code;
			// 最後のブロック以外で、0xFF でないブロックの後ろには 0 があった
			if (code != 0xFF && i < data.size()) {
				out.push_back(0);
			}
		}
		return out;
	}

	auto encode(const Message& message) -> std::vector<std::uint8_t> {
		Writer w{kind_of(message)};
		std::visit(
			Overloaded{
				[&](const TargetCurrent& m) { w.f32(m.current); },
				[&](const TargetVelocity& m) {
					w.f32(m.velocity);
					w.f32(m.accel_ff);
				},
				[&](const TargetPosition& m) {
					w.i32(m.position);
					w.f32(m.velocity_ff);
					w.f32(m.accel_ff);
				},
				[&](const SetMode& m) { w.u8(m.mode); },
				[&](const SetParam& m) {
					w.u8(m.id);
					w.f32(m.value);
				},
				[&](const ResetFault&) {},
				[&](const SetOrigin&) {},
				[&](const Status& m) {
					w.u8(m.mode);
					w.u8(m.flags);
					w.f32(m.current);
					w.f32(m.velocity);
					w.i32(m.position);
					w.f32(m.vdc);
					w.u16(m.temp);
				},
				[&](const Ack& m) { w.u8(m.command); },
				[&](const Nack& m) {
					w.u8(m.command);
					w.u8(m.reason);
					w.u8(m.param);
				},
				[&](const FaultNotice&) {},
			},
			message
		);
		auto out = cobs_encode(w.finish());
		out.push_back(0);
		return out;
	}

	auto decode_body(const std::span<const std::uint8_t> body) -> std::expected<Message, DecodeError> {
		if (body.size() < 2) {
			return std::unexpected{DecodeError::too_short};
		}
		if (crc8(body.first(body.size() - 1)) != body.back()) {
			return std::unexpected{DecodeError::crc};
		}
		const std::uint8_t k = body.front();
		const auto length = body_length(k);
		if (!length) {
			return std::unexpected{DecodeError::unknown_kind};
		}
		if (*length != body.size()) {
			return std::unexpected{DecodeError::length};
		}

		Reader r{body.subspan(1, body.size() - 2)};
		switch (k) {
		case kind::target_current: return TargetCurrent{.current = r.f32()};
		case kind::target_velocity: {
			const float v = r.f32();
			return TargetVelocity{.velocity = v, .accel_ff = r.f32()};
		}
		case kind::target_position: {
			const std::int32_t p = r.i32();
			const float v = r.f32();
			return TargetPosition{.position = p, .velocity_ff = v, .accel_ff = r.f32()};
		}
		case kind::set_mode: return SetMode{.mode = r.u8()};
		case kind::set_param: {
			const std::uint8_t id = r.u8();
			return SetParam{.id = id, .value = r.f32()};
		}
		case kind::reset_fault: return ResetFault{};
		case kind::set_origin: return SetOrigin{};
		case kind::status: {
			Status s{};
			s.mode = r.u8();
			s.flags = r.u8();
			s.current = r.f32();
			s.velocity = r.f32();
			s.position = r.i32();
			s.vdc = r.f32();
			s.temp = r.u16();
			return s;
		}
		case kind::ack: return Ack{.command = r.u8()};
		case kind::nack: {
			const std::uint8_t command = r.u8();
			const std::uint8_t reason = r.u8();
			return Nack{.command = command, .reason = reason, .param = r.u8()};
		}
		case kind::fault_notice: return FaultNotice{};
		default: return std::unexpected{DecodeError::unknown_kind};
		}
	}

	auto StreamParser::push(const std::uint8_t byte) -> std::optional<std::expected<Message, DecodeError>> {
		if (byte != 0) {
			if (this->overflowed_) {
				return std::nullopt;
			}
			if (this->length_ == this->buffer_.size()) {
				this->overflowed_ = true;
				this->length_ = 0;
				return std::unexpected{DecodeError::overflow};
			}
			this->buffer_[this->length_++] = byte;
			return std::nullopt;
		}

		// 区切り
		const std::size_t length = this->length_;
		const bool overflowed = this->overflowed_;
		this->length_ = 0;
		this->overflowed_ = false;
		if (overflowed || length == 0) {
			// あふれた残り、または連続した区切り
			return std::nullopt;
		}
		const auto body = cobs_decode(std::span{this->buffer_}.first(length));
		if (!body) {
			return std::unexpected{DecodeError::cobs};
		}
		return decode_body(*body);
	}

	auto split_into_frames(const std::span<const std::uint8_t> bytes) -> std::vector<CanChunk> {
		std::vector<CanChunk> frames{};
		for (std::size_t i = 0; i < bytes.size(); i += 8) {
			const std::size_t n = std::min<std::size_t>(8, bytes.size() - i);
			CanChunk chunk{};
			std::copy_n(bytes.begin() + static_cast<std::ptrdiff_t>(i), n, chunk.data.begin());
			chunk.dlc = static_cast<std::uint8_t>(n);
			frames.push_back(chunk);
		}
		return frames;
	}

	auto revolutions_to_q16_16(const double revolutions) -> std::int32_t {
		if (std::isnan(revolutions)) {
			return 0;
		}
		const double q = std::round(revolutions * 65536.0);
		constexpr auto lo = static_cast<double>(std::numeric_limits<std::int32_t>::min());
		constexpr auto hi = static_cast<double>(std::numeric_limits<std::int32_t>::max());
		return static_cast<std::int32_t>(std::clamp(q, lo, hi));
	}

	auto q16_16_to_revolutions(const std::int32_t q) -> double {
		return static_cast<double>(q) / 65536.0;
	}
}
