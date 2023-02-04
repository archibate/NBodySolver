#pragma once

#include <string>
#include "MathUtils.h"

// 格里高利历日期和儒略日数转换
// https://www.fourmilab.ch/documents/calendar/
struct GregorianDate {
    // 年 [0,inf)
    int year = 1950;
    // 月 [1,12]
    int month = 1;
    // 日 [1,31]
    int day = 1;

    // 是否为闰年？
    static bool leapGregorian(int year) {
        return ((year % 4) == 0) &&
            (!(((year % 100) == 0) && ((year % 400) != 0)));
    }

    // 儒略日数转日期
    static GregorianDate fromJulianDays(JulianDays instant) {
        Real jd = instant;
        Real wjd = std::floor(jd - 0.5) + 0.5;
        Real depoch = wjd - 1721425.5;
        Real quadricent = std::floor(depoch / 146097);
        Real dqc = std::fmod(depoch, 146097);
        Real cent = std::floor(dqc / 36524);
        Real dcent = std::fmod(dqc, 36524);
        Real quad = std::floor(dcent / 1461);
        Real dquad = std::fmod(dcent, 1461);
        Real yindex = std::floor(dquad / 365);
        int year = (int)std::rint((quadricent * 400) + (cent * 100) + (quad * 4) + yindex);
        if (!((cent == 4) || (yindex == 4))) {
            year++;
        }
        Real yearday = wjd - GregorianDate{year, 1, 1}.toJulianDays();
        Real leapadj = ((wjd < GregorianDate{year, 3, 1}.toJulianDays()) ? 0 :
                   (leapGregorian(year) ? 1 : 2));
        int month = (int)std::floor((((yearday + leapadj) * 12) + 373) / 367);
        int day = (int)std::rint((wjd - GregorianDate{year, month, 1}.toJulianDays()) + 1);
        return {year, month, day};
    }

    // 日期转儒略日数
    JulianDays toJulianDays() const {
        return (1721425.5 - 1) +
           (365 * (year - 1)) +
           std::floor((year - 1) / 4) +
           (-std::floor((year - 1) / 100)) +
           std::floor((year - 1) / 400) +
           std::floor((((367 * month) - 362) / 12) +
           ((month <= 2) ? 0 : (leapGregorian(year) ? -1 : -2)) +
           day);
    }

    // 用字符串表示
    std::string toString() const {
        return std::to_string(year)
            + "/" + std::to_string(month)
            + "/" + std::to_string(day);
    }
};

struct GregorianTime : GregorianDate {
    // 时 [0,24)
    int hour = 0;
    // 分 [0,60)
    int minute = 0;
    // 秒 [0,60)
    int second = 0;
    // 秒的非整数部分 [0,1)
    Real extraSeconds = 0.0;

    // 儒略日数转日期+时间
    static GregorianTime fromJulianDays(JulianDays instant) {
        auto date = GregorianDate::fromJulianDays(instant);
        JulianDays offset = instant - std::floor(instant);
        int hour = (int)std::floor(offset * 24.0);
        int minute = (int)std::floor((offset * 24.0 - hour) * 60.0);
        int second = (int)std::floor(((offset * 24.0 - hour) * 60.0 - minute) * 60.0);
        Real extraSeconds = ((offset * 24.0 - hour) * 60.0 - minute) * 60.0 - second;
        return {date, hour, minute, second, extraSeconds};
    }

    // 日期+时间转儒略日数
    JulianDays toJulianDays() const {
        JulianDays baseJd = GregorianDate::toJulianDays();
        return baseJd + (((second + extraSeconds) / 60.0 + minute) / 60.0 + hour) / 24.0;
    }

    // 用字符串表示
    std::string toString() const {
        return GregorianDate::toString()
            + " " + std::to_string(hour + 100).substr(1)
            + ":" + std::to_string(minute + 100).substr(1)
            + ":" + std::to_string(second + 100).substr(1)
            + "." + std::to_string((int)std::floor(extraSeconds * 1000) + 10000).substr(1);
    }
};
