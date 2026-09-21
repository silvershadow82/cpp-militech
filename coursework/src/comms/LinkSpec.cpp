#include "comms/LinkSpec.h"

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "comms/SerialLink.h"
#include "comms/SocketLink.h"
#include "interfaces/IByteLink.h"

namespace follow::comms {

namespace {

std::vector<std::string> split(const std::string& text, char separator)
{
  std::vector<std::string> parts;
  size_t start = 0;
  while (true) {
    size_t end = text.find(separator, start);
    parts.push_back(text.substr(start, end - start));
    if (end == std::string::npos) {
      return parts;
    }
    start = end + 1;
  }
}

// Whole-string decimal number in [minimum, maximum].
int parseNumber(const std::string& text, int minimum, int maximum, const std::string& what, const std::string& spec)
{
  size_t used = 0;
  long value = -1;
  try {
    value = std::stol(text, &used);
  }
  catch (const std::exception&) {
    used = 0;
  }
  if (text.empty() || used != text.size() || value < minimum || value > maximum) {
    throw std::invalid_argument("bad " + what + " '" + text + "' in link '" + spec + "'");
  }
  return static_cast<int>(value);
}

}  // namespace

LinkSpec parseLinkSpec(const std::string& text)
{
  std::vector<std::string> parts = split(text, ':');
  LinkSpec spec;
  if (parts[0] == "udp" && (parts.size() == 2 || parts.size() == 4)) {
    spec.kind = LinkSpec::Kind::Udp;
    spec.localPort = parseNumber(parts[1], 0, 65535, "port", text);
    if (parts.size() == 4) {
      spec.remoteHost = parts[2];
      spec.remotePort = parseNumber(parts[3], 1, 65535, "port", text);
    }
    return spec;
  }
  if (parts[0] == "uart" && parts.size() == 3 && !parts[1].empty()) {
    spec.kind = LinkSpec::Kind::Uart;
    spec.device = parts[1];
    spec.baud = parseNumber(parts[2], 1, 4000000, "baud rate", text);
    return spec;
  }
  throw std::invalid_argument("link '" + text + "': expected udp:PORT, udp:PORT:HOST:PORT or uart:DEVICE:BAUD");
}

std::unique_ptr<interfaces::IByteLink> openLink(const LinkSpec& spec)
{
  if (spec.kind == LinkSpec::Kind::Udp) {
    return std::make_unique<SocketLink>(spec.localPort, spec.remoteHost, spec.remotePort);
  }
  return std::make_unique<SerialLink>(spec.device, spec.baud);
}

}  // namespace follow::comms
