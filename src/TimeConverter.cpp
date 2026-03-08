#include "SPP.h"

/****************************************************************************
  CommonTimeToMJDTime

  目的：定义通用时到儒略日转换的函数

****************************************************************************/
void CommonTimeToMJDTime(const COMMONTIME* CT, MJDTIME* MJDT) 
{
	if (CT == nullptr || MJDT == nullptr) {
		return;  // 检查指针是否为空
	}

    // 如果月份小于等于2，则年份减1，月份加12
    int y, m;
    if (CT->Month <= 2) {
        y = CT->Year - 1;
        m = CT->Month + 12;
    }
    else {
        y = CT->Year;
        m = CT->Month;
    }

    // 计算儒略日（JD）
    double JD = static_cast<int>(365.25 * y)+static_cast<int>(30.6001 * (m + 1)) + CT->Day + CT->Hour / 24.0 + CT->Minute / 1440.0 + CT->Second / 86400.0 + 1720981.5;

    // 计算简化儒略日（MJD）
    double MJD = JD - 2400000.5;

    // 计算MJD的整数部分和小数部分
    MJDT->Days = static_cast<int>(MJD);
    MJDT->FracDay = CT->Hour / 24.0 + CT->Minute / 1440.0 + CT->Second / 86400.0;

}

/****************************************************************************
  MJDTimeToCommonTime

  目的：定义儒略日到通用时的转换函数

****************************************************************************/
void MJDTimeToCommonTime(const MJDTIME* MJDT, COMMONTIME* CT) 
{
	if (MJDT == nullptr || CT == nullptr) {
		return; // 检查指针是否为空
	}

	// 计算完整的儒略日（整数部分 + 小数部分）
	double MJD = MJDT->Days + MJDT->FracDay;
	double JD = MJD + 2400000.5;

	// 计算年、月、日
	double a = static_cast<int>(JD + 0.5);
	double b = a + 1537;
	double c = static_cast<int>((b - 122.1) / 365.25);
	double d = static_cast<int>(365.25 * c);
	double e = static_cast<int>((b - d) / 30.6001);
	CT->Day = b - d - static_cast<int>(30.6001 * e) + (JD + 0.5 - a);
	CT->Month = e - 1 - 12 * static_cast<int>(e / 14);//确保月份在1到12之间
	CT->Year = c - 4715 - static_cast<int>((7 + CT->Month) / 10);
	
	// 计算小时、分钟和秒
	double fracDay = MJDT->FracDay; // 获取小数部分
	CT->Hour = (unsigned short)(fracDay * 24.0);
	double remainingFracHour = fracDay * 24.0 - CT->Hour; // 获取小时的小数部分
	CT->Minute = (unsigned short)(remainingFracHour * 60.0);
	double remainingFracMinute = remainingFracHour * 60.0 - CT->Minute; // 获取分钟的小数部分
	CT->Second = remainingFracMinute * 60.0;

}

/****************************************************************************
  MJDTimeToGPSTime

  目的：定义儒略日到GPS时的转换函数

****************************************************************************/
void MJDTimeToGPSTime(const MJDTIME* MJDT, GPSTIME* GT)
{
	if (MJDT == nullptr || GT == nullptr) {
		return; //检查指针是否为空
	}

	//计算GPS周，GPS周秒
	double MJD = MJDT->Days + MJDT->FracDay;
	GT->Week = (unsigned short)((MJD - 44244) / 7);
	GT->SecOfWeek = (MJDT->Days - 44244 - GT->Week * 7) * 86400 + MJDT->FracDay * 86400;

	// 确保秒数在合理范围内
	if (GT->SecOfWeek < 0.0) {
		GT->SecOfWeek += 604800.0;
		GT->Week -= 1;
	}
	else if (GT->SecOfWeek >= 604800.0) {
		GT->SecOfWeek -= 604800.0;
		GT->Week += 1;
	}
}

/****************************************************************************
  GPSTimeToMJDTime

  目的：定义GPS时到儒略日的转换函数

****************************************************************************/
void GPSTimeToMJDTime(const GPSTIME* GT, MJDTIME* MJDT) 
{
	if (GT == nullptr || MJDT == nullptr) {
		return; //检查指针是否为空
	}

	double MJD = 44244 + GT->Week * 7 + GT->SecOfWeek / 86400;
	MJDT->Days = static_cast<int>(MJD);
	MJDT->FracDay = GT->SecOfWeek / 86400 - static_cast<int>(GT->SecOfWeek / 86400);
}

/****************************************************************************
  CommonTimeToGPSTime

  目的：定义通用时到GPS时的转换函数

****************************************************************************/
void CommonTimeToGPSTime(const COMMONTIME* CT, GPSTIME* GT)
{
	if (CT == nullptr || GT == nullptr) {
		return; // 检查指针是否为空
	}

	// 第一步：由通用时转换到简化儒略日
	MJDTIME MJDT;
	CommonTimeToMJDTime(CT, &MJDT);

	// 第二步：由简化儒略日转换到GPS时
	MJDTimeToGPSTime(&MJDT, GT);
}

/****************************************************************************
 GPSTimeToCommonTime

  目的：定义GPS时到通用时的转换函数

****************************************************************************/
void GPSTimeToCommonTime(const GPSTIME* GT, COMMONTIME* CT) 
{
	if (GT == nullptr || CT == nullptr) {
		return; // 检查指针是否为空
	}

	// 第一步：由GPS时间转换到儒略日
	MJDTIME MJDT;
	GPSTimeToMJDTime(GT, &MJDT);

	// 第二步：由儒略日转换到通用时
	MJDTimeToCommonTime(&MJDT, CT);
}