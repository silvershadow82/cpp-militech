#include "ballistics.hpp"
#include <cstring>
#include <fstream>
#include <iostream>
#include <ostream>

auto readInput(char *fileName, BallisticInput &input)
{
  std::ifstream inputFile(fileName);

  if (!inputFile.is_open()) {
    std::cerr << "Unable to open " << fileName << '\n';
    return 1;
  }

  if (inputFile >> input.startPos.x >> input.startPos.y >> input.altitude >> input.targetPos.x >> input.targetPos.y >> input.attackSpeed >>
      input.accelPath >> input.ammoName) {
    inputFile.close();

    return 0;
  }

  inputFile.close();

  std::cerr << "Unable to read input data - check the data format" << '\n';
  return 1;
}

void printResult(const BallisticResult &result)
{
  std::cout << "Ballistic Result: " << '\n';
  std::cout << "  |- Ammo Type: " << result.ammoName  // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay,hicpp-no-array-decay)
            << '\n';
  std::cout << "  |- (fireX, fireY)=" << "(" << result.dropPoint.x << "," << result.dropPoint.y << ")" << '\n';
  std::cout << "  |- t=" << result.payloadDropTime << '\n';
}

int main(int argc, char *argv[]) // NOLINT(modernize-use-trailing-return-type)
{
  if (argc < 2) {
    std::cout << "Usage ballistic_cli <input_file.txt>" << '\n';
    return 0;
  }

  BallisticInput input{};
  auto input_ret_code = readInput(argv[1], input);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  if (input_ret_code > 0) {
    return input_ret_code;
  }

  BallisticResult result{};

  auto ballistics_ret_code = ballistics(result, input);

  if (ballistics_ret_code == 0) {
    printResult(result);
  }

  return ballistics_ret_code;
}