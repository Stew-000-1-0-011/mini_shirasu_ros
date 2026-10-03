#pragma once
/// @file settings.hpp
/// SetParam の設定 ID と名前の対応 (minishirasu-firm の設計書「設定値」)。ROS非依存。

#include <array>
#include <cstdint>
#include <optional>
#include <string_view>

namespace mini_shirasu_ros {
	struct SettingInfo {
		std::uint8_t id{};
		std::string_view name{};
		/// 有効化に必須か (status_period_ms だけが任意)
		bool required{true};
	};

	inline constexpr std::array<SettingInfo, 22> settings{{
		{0x00, "vdcmax"},
		{0x01, "ibase"},
		{0x02, "wbase"},
		{0x03, "ke"},
		{0x04, "ckp"},
		{0x05, "cki"},
		{0x06, "dead_duty"},
		{0x07, "i_threshold"},
		{0x08, "duty_max"},
		{0x09, "vmax"},
		{0x0A, "imax"},
		{0x0B, "wkp"},
		{0x0C, "wki"},
		{0x0D, "wb"},
		{0x0E, "wmax"},
		{0x0F, "pkp"},
		{0x10, "pmax"},
		{0x20, "accel_to_current"},
		{0x21, "encoder_cpr"},
		{0x22, "w_filter_alpha"},
		{0x23, "encoder_reversed"},
		{0x24, "status_period_ms", false},
	}};

	constexpr auto setting_name(const std::uint8_t id) -> std::optional<std::string_view> {
		for (const auto& s : settings) {
			if (s.id == id) {
				return s.name;
			}
		}
		return std::nullopt;
	}
}
