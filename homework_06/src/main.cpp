#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include "ballistics.hpp"
#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"

using nlohmann::json;

void clearInputAmmo(BallisticInput &input)
{
  if (input.ammo != nullptr) {
    delete[] input.ammo;
    input.ammo = nullptr;
  }
}

int readAmmoData(BallisticInput &input)
{
  std::ifstream ammoFile("ammo.json");
  if (!ammoFile.is_open()) {
    std::cerr << "Unable to open ammo file!" << std::endl;
    return 1;
  }
  json ja;
  ammoFile >> ja;

  input.ammoCount = static_cast<size_t>(ja.size());
  input.ammo = new AmmoParams[input.ammoCount];

  for (uint16_t i = 0; i < input.ammoCount; i++) {
    std::strncpy(input.ammo[i].name, ja[i]["name"].get<std::string>().c_str(), 31);
    input.ammo[i].mass = ja[i]["mass"];
    input.ammo[i].drag = ja[i]["drag"];
    input.ammo[i].lift = ja[i]["lift"];
  }

  ammoFile.close();
  return 0;
}

int readInput(char *fileName, BallisticInput &input)
{
  std::ifstream inputFile(fileName);

  if (!inputFile.is_open()) {
    std::cerr << "Unable to open " << fileName << std::endl;
    return 1;
  }

  if (inputFile >> input.startPos.x >> input.startPos.y >> input.altitude >> input.targetPos.x >> input.targetPos.y >> input.attackSpeed >>
      input.accelPath >> input.ammoName) {
    inputFile.close();

    return readAmmoData(input);
  }

  inputFile.close();

  std::cerr << "Unable to read input data - check the data format" << std::endl;
  return 1;

  // Work with json
  //   input.startPos.x = jc["drone"]["position"]["x"];
  //   input.startPos.y = jc["drone"]["position"]["y"];
  //   input.altitude = jc["drone"]["altitude"];
  //   input.attackSpeed = jc["drone"]["attackSpeed"];
  //   input.accelPath = jc["drone"]["accelerationPath"];

  //   std::strncpy(input.ammoName, jc["ammo"].get<std::string>().c_str(), MAX_AMMO_LENGTH - 1);
}

int main(int argc, char *argv[])
{
  if (argc < 2) {
    std::cout << "Usage ballistic_cli <input_file.txt>" << std::endl;
    return 0;
  }

  BallisticInput input{};
  int retCode = readInput(argv[1], input);
  if (retCode > 0) {
    return retCode;
  }

  BallisticResult result{};

  retCode = ballistics(result, input);

  return retCode;
}