#pragma once

#include <cmath>
#include <functional>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace calib_test {

using TestFn = std::function<void()>;

struct TestCase {
  std::string name;
  TestFn fn;
};

inline std::vector<TestCase>& Registry() {
  static std::vector<TestCase> tests;
  return tests;
}

inline void Register(const std::string& name, const TestFn& fn) {
  Registry().push_back({name, fn});
}

inline void Fail(const std::string& message, const char* file, int line) {
  std::ostringstream oss;
  oss << file << ':' << line << " " << message;
  throw std::runtime_error(oss.str());
}

inline int RunAll() {
  int pass = 0;
  int fail = 0;
  for (const auto& t : Registry()) {
    try {
      t.fn();
      ++pass;
      std::cout << "[PASS] " << t.name << '\n';
    } catch (const std::exception& ex) {
      ++fail;
      std::cout << "[FAIL] " << t.name << " : " << ex.what() << '\n';
    }
  }
  std::cout << "Summary: " << pass << " passed, " << fail << " failed\n";
  return fail == 0 ? 0 : 1;
}

}  // namespace calib_test

#define CT_CONCAT_INNER(a, b) a##b
#define CT_CONCAT(a, b) CT_CONCAT_INNER(a, b)

#define TEST_CASE(name)                                                          \
  static void CT_CONCAT(TestFn_, __LINE__)();                                   \
  namespace {                                                                    \
  struct CT_CONCAT(TestReg_, __LINE__) {                                        \
    CT_CONCAT(TestReg_, __LINE__)() {                                            \
      calib_test::Register((name), CT_CONCAT(TestFn_, __LINE__));               \
    }                                                                            \
  } CT_CONCAT(kReg_, __LINE__);                                                 \
  }                                                                              \
  static void CT_CONCAT(TestFn_, __LINE__)()

#define EXPECT_TRUE(expr)                                                       \
  do {                                                                          \
    if (!(expr)) {                                                              \
      calib_test::Fail(std::string("EXPECT_TRUE failed: ") + #expr, __FILE__, __LINE__); \
    }                                                                           \
  } while (0)

#define EXPECT_FALSE(expr) EXPECT_TRUE(!(expr))

#define EXPECT_NEAR(a, b, eps)                                                 \
  do {                                                                          \
    const double _va = static_cast<double>(a);                                  \
    const double _vb = static_cast<double>(b);                                  \
    if (std::fabs(_va - _vb) > static_cast<double>(eps)) {                      \
      std::ostringstream _oss;                                                  \
      _oss << "EXPECT_NEAR failed: " << #a << "=" << _va << ", " << #b << "=" << _vb; \
      calib_test::Fail(_oss.str(), __FILE__, __LINE__);                         \
    }                                                                           \
  } while (0)

#define EXPECT_GT(a, b) EXPECT_TRUE((a) > (b))
#define EXPECT_LT(a, b) EXPECT_TRUE((a) < (b))
