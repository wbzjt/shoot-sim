#pragma once

#include <cmath>
#include <exception>
#include <functional>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace testfw {

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
  oss << file << ':' << line << " -> " << message;
  throw std::runtime_error(oss.str());
}

inline int RunAll() {
  int passed = 0;
  int failed = 0;

  for (const auto& test : Registry()) {
    try {
      test.fn();
      ++passed;
      std::cout << "[PASS] " << test.name << '\n';
    } catch (const std::exception& ex) {
      ++failed;
      std::cout << "[FAIL] " << test.name << " : " << ex.what() << '\n';
    }
  }

  std::cout << "\nSummary: " << passed << " passed, " << failed << " failed\n";
  return failed == 0 ? 0 : 1;
}

}  // namespace testfw

#define TESTFW_CONCAT_INNER(a, b) a##b
#define TESTFW_CONCAT(a, b) TESTFW_CONCAT_INNER(a, b)

#define TEST_CASE(name)                                                             \
  static void TESTFW_CONCAT(TestFunc_, __LINE__)();                                 \
  namespace {                                                                        \
  struct TESTFW_CONCAT(TestRegistrar_, __LINE__) {                                  \
    TESTFW_CONCAT(TestRegistrar_, __LINE__)() {                                     \
      testfw::Register((name), TESTFW_CONCAT(TestFunc_, __LINE__));                 \
    }                                                                                \
  } TESTFW_CONCAT(kRegistrar_, __LINE__);                                           \
  }                                                                                  \
  static void TESTFW_CONCAT(TestFunc_, __LINE__)()

#define EXPECT_TRUE(expr)                                                            \
  do {                                                                               \
    if (!(expr)) {                                                                   \
      testfw::Fail(std::string("EXPECT_TRUE failed: ") + #expr, __FILE__, __LINE__); \
    }                                                                                \
  } while (0)

#define EXPECT_FALSE(expr) EXPECT_TRUE(!(expr))

#define EXPECT_NEAR(a, b, eps)                                                         \
  do {                                                                                 \
    const double _va = static_cast<double>(a);                                         \
    const double _vb = static_cast<double>(b);                                         \
    if (std::fabs(_va - _vb) > static_cast<double>(eps)) {                             \
      std::ostringstream _oss;                                                         \
      _oss << "EXPECT_NEAR failed: " << #a << "=" << _va << ", " << #b << "=" << _vb \
           << ", eps=" << static_cast<double>(eps);                                   \
      testfw::Fail(_oss.str(), __FILE__, __LINE__);                                    \
    }                                                                                  \
  } while (0)

#define EXPECT_LT(a, b)                                                              \
  do {                                                                               \
    if (!((a) < (b))) {                                                              \
      testfw::Fail(std::string("EXPECT_LT failed: ") + #a + " < " + #b, __FILE__, __LINE__); \
    }                                                                                \
  } while (0)

#define EXPECT_GT(a, b)                                                              \
  do {                                                                               \
    if (!((a) > (b))) {                                                              \
      testfw::Fail(std::string("EXPECT_GT failed: ") + #a + " > " + #b, __FILE__, __LINE__); \
    }                                                                                \
  } while (0)
