/// @file protocol_test.cpp
/// プロトコルのテスト (ROS不要)。minishirasu-firm の protocol.rs のテストと同じ性質を見る。

#include <cstdint>
#include <cstdio>
#include <source_location>
#include <vector>

#include "mini_shirasu_ros/protocol.hpp"

namespace {
	namespace proto = mini_shirasu_ros::protocol;
	using Bytes = std::vector<std::uint8_t>;
	using Result = std::expected<proto::Message, proto::DecodeError>;

	int failures = 0;

	void check(const bool ok, const char* what, const std::source_location loc = std::source_location::current()) {
		if (!ok) {
			std::printf("FAIL %s:%u: %s\n", loc.file_name(), static_cast<unsigned>(loc.line()), what);
			++failures;
		}
	}

	auto feed(proto::StreamParser& p, const Bytes& bytes) -> std::vector<Result> {
		std::vector<Result> out{};
		for (const auto b : bytes) {
			if (auto r = p.push(b)) {
				out.push_back(*r);
			}
		}
		return out;
	}

	auto samples() -> std::vector<proto::Message> {
		return {
			proto::TargetCurrent{.current = 1.5f},
			proto::TargetVelocity{.velocity = -12.25f, .accel_ff = 3.0f},
			proto::TargetPosition{.position = -(3 << 16), .velocity_ff = 1.0f, .accel_ff = -2.0f},
			proto::SetMode{.mode = proto::mode::velocity},
			proto::SetParam{.id = 0x21, .value = 8192.0f},
			proto::ResetFault{},
			proto::SetOrigin{},
			proto::Status{
				.mode = proto::mode::velocity,
				.flags = proto::status_flag::saturated_upper,
				.current = 0.5f,
				.velocity = 100.0f,
				.position = 0x00012345,
				.vdc = 24.0f,
				.temp = 0x0ABC,
			},
			proto::Ack{.command = proto::kind::set_mode},
			proto::Nack{.command = proto::kind::set_param, .reason = 3, .param = 0x04},
			proto::FaultNotice{},
		};
	}

	void test_crc() {
		// CRC-8/SMBUS の検査値
		const Bytes data{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
		check(proto::crc8(data) == 0xF4, "crc8 check value");
	}

	void test_cobs() {
		for (const Bytes& data : {Bytes{}, Bytes{0}, Bytes{0, 0}, Bytes{1, 0, 2}, Bytes{0x11, 0x22, 0x00, 0x33}}) {
			const auto e = proto::cobs_encode(data);
			bool no_zero = true;
			for (const auto b : e) {
				no_zero = no_zero && b != 0;
			}
			check(no_zero, "cobs output has no zero");
			const auto d = proto::cobs_decode(e);
			check(d && *d == data, "cobs roundtrip");
		}
		// 254 バイト以上の 0 を含まない列
		Bytes long_run(300, 0x55);
		const auto d = proto::cobs_decode(proto::cobs_encode(long_run));
		check(d && *d == long_run, "cobs roundtrip over 254 bytes");
		// 壊れた符号
		check(!proto::cobs_decode(Bytes{5, 1, 2}), "cobs rejects a block longer than the input");
	}

	void test_roundtrip() {
		for (const auto& m : samples()) {
			proto::StreamParser p{};
			const auto got = feed(p, proto::encode(m));
			check(got.size() == 1 && got[0] && *got[0] == m, "every message roundtrips");
		}
	}

	void test_lengths_and_kinds() {
		const std::vector<std::size_t> lengths{8, 12, 16, 5, 9, 4, 4, 24, 5, 7, 4};
		const std::vector<std::uint8_t> kinds{0x01, 0x02, 0x03, 0x10, 0x11, 0x12, 0x13, 0x20, 0x30, 0x31, 0x32};
		const auto s = samples();
		for (std::size_t i = 0; i < s.size(); ++i) {
			const auto e = proto::encode(s[i]);
			check(e.size() == lengths[i], "encoded length matches the spec");
			check(proto::kind_of(s[i]) == kinds[i], "kind matches the spec");
			check(e.back() == 0, "ends with the delimiter");
			for (std::size_t j = 0; j + 1 < e.size(); ++j) {
				check(e[j] != 0, "no zero before the delimiter");
			}
		}
	}

	void test_little_endian() {
		// SetParam(0x21, 1.0f): 本体 = 11 21 00 00 80 3F crc
		const Bytes body{0x11, 0x21, 0x00, 0x00, 0x80, 0x3F};
		Bytes full = body;
		full.push_back(proto::crc8(body));
		const auto m = proto::decode_body(full);
		check(m && *m == proto::Message{proto::SetParam{.id = 0x21, .value = 1.0f}}, "fields are little endian");
	}

	void test_stream() {
		proto::StreamParser p{};
		Bytes bytes{0, 0};
		for (const auto& m : {proto::Message{proto::Ack{.command = 0x10}}, proto::Message{proto::FaultNotice{}}}) {
			const auto e = proto::encode(m);
			bytes.insert(bytes.end(), e.begin(), e.end());
			bytes.push_back(0);  // 余分な区切り
		}
		const auto got = feed(p, bytes);
		check(got.size() == 2 && got[0] && got[1], "consecutive messages split, extra delimiters ignored");

		// 途中から受信: 壊れた1つだけ落ちて、次は読める
		proto::StreamParser q{};
		const auto first = proto::encode(proto::SetParam{.id = 4, .value = 6.0f});
		Bytes mid(first.end() - 3, first.end());
		const auto second = proto::encode(proto::SetMode{.mode = 1});
		mid.insert(mid.end(), second.begin(), second.end());
		const auto got2 = feed(q, mid);
		check(got2.size() == 2 && !got2[0] && got2[1], "joining mid message drops only the broken one");

		// 1 バイト化けたら CRC で落ちる
		proto::StreamParser r{};
		auto corrupt = proto::encode(proto::SetParam{.id = 4, .value = 6.0f});
		// COBS の符号バイトではなく中身を化けさせる (符号バイトが 0 になると区切りが増えるので)
		corrupt[2] ^= 0x01;
		const auto got3 = feed(r, corrupt);
		check(got3.size() == 1 && !got3[0], "corrupted byte is rejected");

		// あふれたら次の区切りまで捨てて、その後は読める
		proto::StreamParser o{};
		Bytes over(100, 0x55);
		over.push_back(0);
		const auto ok = proto::encode(proto::ResetFault{});
		over.insert(over.end(), ok.begin(), ok.end());
		const auto got4 = feed(o, over);
		check(
			got4.size() == 2 && !got4[0] && got4[0].error() == proto::DecodeError::overflow && got4[1],
			"overflow is reported once and the parser recovers"
		);
	}

	void test_decode_errors() {
		check(proto::decode_body(Bytes{0x12}).error() == proto::DecodeError::too_short, "too short");
		Bytes unknown{0x7F};
		unknown.push_back(proto::crc8(unknown));
		check(proto::decode_body(unknown).error() == proto::DecodeError::unknown_kind, "unknown kind");
		Bytes wrong_length{proto::kind::set_mode, 1, 2};
		wrong_length.push_back(proto::crc8(wrong_length));
		check(proto::decode_body(wrong_length).error() == proto::DecodeError::length, "wrong length");
	}

	void test_frames() {
		const auto e = proto::encode(proto::Status{});
		const auto frames = proto::split_into_frames(e);
		check(frames.size() == 3, "24 bytes become 3 frames");
		check(frames[0].dlc == 8 && frames[1].dlc == 8 && frames[2].dlc == 8, "frames are full");

		const auto v = proto::split_into_frames(proto::encode(proto::TargetVelocity{}));
		check(v.size() == 2 && v[0].dlc == 8 && v[1].dlc == 4, "12 bytes become 8 + 4");

		// フレームのデータを順に流せば元に戻る
		proto::StreamParser p{};
		std::vector<Result> got{};
		for (const auto& f : v) {
			for (std::size_t i = 0; i < f.dlc; ++i) {
				if (auto r = p.push(f.data[i])) {
					got.push_back(*r);
				}
			}
		}
		check(got.size() == 1 && got[0], "frames reassemble");
	}

	void test_q16_16() {
		check(proto::revolutions_to_q16_16(1.0) == 65536, "1 rev");
		check(proto::revolutions_to_q16_16(-0.5) == -32768, "-0.5 rev");
		check(proto::revolutions_to_q16_16(1e9) == 0x7FFFFFFF, "saturates high");
		check(proto::revolutions_to_q16_16(-1e9) == static_cast<std::int32_t>(0x80000000), "saturates low");
		check(proto::q16_16_to_revolutions(98304) == 1.5, "back to revolutions");
	}
}

auto main() -> int {
	test_crc();
	test_cobs();
	test_roundtrip();
	test_lengths_and_kinds();
	test_little_endian();
	test_stream();
	test_decode_errors();
	test_frames();
	test_q16_16();

	if (failures == 0) {
		std::printf("all protocol tests passed\n");
		return 0;
	}
	std::printf("%d failure(s)\n", failures);
	return 1;
}
