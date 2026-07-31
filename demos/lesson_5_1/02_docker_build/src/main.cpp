#include <iostream>

int main()
{
#if defined(__aarch64__)
  constexpr auto arch = "aarch64";
#elif defined(__x86_64__)
  constexpr auto arch = "x86_64";
#elif defined(__arm__)
  constexpr auto arch = "arm";
#else
  constexpr auto arch = "unknown";
  static_assert<>
#endif

  std::cout << "rd51 build demo\n";
  std::cout << "compiled for: " << arch << '\n';
  return 0;
}
