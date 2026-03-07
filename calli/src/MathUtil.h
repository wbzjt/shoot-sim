#pragma once

#include <optional>
#include <string>
#include <vector>

namespace calibration::math {

std::string ToLower(std::string s);
std::string Trim(const std::string& s);
bool ParseDouble(const std::string& s, double* out);
std::optional<double> ParseOptionalDouble(const std::string& s);

bool ParseBoolLoose(const std::string& s, bool* out);

std::vector<double> SortCopy(std::vector<double> values);

double Mean(const std::vector<double>& values);
double Median(const std::vector<double>& values);
double StdDev(const std::vector<double>& values);
double Quantile(const std::vector<double>& values, double q);

double Clamp(double v, double lo, double hi);
double Lerp(double a, double b, double t);
double LinearInterpolate(const std::vector<double>& xs,
                         const std::vector<double>& ys,
                         double x,
                         bool clamp);

double RoundTo(double value, double step);

std::vector<double> SolveLinearSystem(std::vector<std::vector<double>> a,
                                      std::vector<double> b,
                                      bool* ok);

}  // namespace calibration::math
