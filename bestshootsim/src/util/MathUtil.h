#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace shootersim::util {

double DegToRad(double deg);
double RadToDeg(double rad);
double Clamp(double value, double lo, double hi);
bool NearlyEqual(double a, double b, double eps = 1e-9);
std::vector<double> Linspace(double start, double end, double step);

std::size_t HashCombine(std::size_t seed, std::size_t value);
std::size_t HashDouble(std::size_t seed, double value, double scale = 1e6);
std::string FormatDouble(double value, int precision = 6);

}  // namespace shootersim::util
