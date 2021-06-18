#pragma once

#ifdef GEOALGOR_MODULE
#define EXPORTGEOALGOCLASS __declspec(dllexport)
#define EXPORTGEOALGOAPI __declspec(dllexport)
#else
#define EXPORTGEOALGOCLASS __declspec(dllimport)
#define EXPORTGEOALGOAPI __declspec(dllimport)
#endif

extern const double M_PI;

class EXPORTGEOALGOCLASS GeoAlgoTool
{
public:
	/// @brief 是否等于
	static bool Equal(const double& a, const double& b, const double& dTol = 0.0001);
	/// @brief 是否小于等于
	static bool LessEqualThan(const double& a, const double& b, const double& dTol = 0.0001);
	/// @brief 是否小于
	static bool LessThan(const double& a, const double& b, const double& dTol = 0.0001);
	/// @brief 是否大于等于
	static bool BiggerEqualThan(const double& a, const double& b, const double& dTol = 0.0001);
	/// @brief 是否大于
	static bool BiggerThan(const double& a, const double& b, const double& dTol = 0.0001);

	/// @brief 四舍五入取整/保留小数函数
	static double Round(double decimals, int digits = 0);

	/// @brief 弧度转角度
	static double TransRadianToAngle(const double& dRadian);
	/// @brief 角度转弧度
	static double TransAngleToRadian(const double& dAngle);
	/// @brief 标准化角度
	static double NormalizeAngle(double& theta, double min = 0, double max = 2 * M_PI);

	/// @brief 计算凸度(弧度)
	static double CalBulge(const double& dStartAngle, const double& dEndAngle);
	/// @brief 凸度转夹角(弧度)
	static double TransBulgeToIncludeAngle(const double& dBulge);
};
