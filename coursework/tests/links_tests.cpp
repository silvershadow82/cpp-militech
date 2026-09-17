#include <gtest/gtest.h>
#include <unistd.h>
#if defined(__APPLE__)
#include <util.h>
#else
#include <pty.h>
#endif

#include <array>
#include <chrono>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>

#include "follow/mavlink/ByteLink.h"
#include "follow/mavlink/Links.h"

using namespace follow::mavlink;
using namespace std::chrono_literals;

namespace {

std::span<const uint8_t> bytesOf(const std::string& text)
{
  return {reinterpret_cast<const uint8_t*>(text.data()), text.size()};
}

// Waits for data and returns what one receive() call reads.
std::string receiveText(ByteLink& link)
{
  std::array<uint8_t, 256> buffer{};
  if (link.waitReadable(1000ms) != ByteLink::WaitStatus::Readable) {
    return "<timeout>";
  }
  int n = link.receive(buffer);
  return n > 0 ? std::string(buffer.begin(), buffer.begin() + n) : "<nothing>";
}

}  // namespace

TEST(LinkSpecTest, ParsesUdpAndUartSpecs)
{
  LinkSpec listen = parseLinkSpec("udp:14560");
  EXPECT_EQ(listen.kind, LinkSpec::Kind::Udp);
  EXPECT_EQ(listen.localPort, 14560);
  EXPECT_TRUE(listen.remoteHost.empty());

  LinkSpec connect = parseLinkSpec("udp:14551:127.0.0.1:14550");
  EXPECT_EQ(connect.localPort, 14551);
  EXPECT_EQ(connect.remoteHost, "127.0.0.1");
  EXPECT_EQ(connect.remotePort, 14550);

  LinkSpec uart = parseLinkSpec("uart:/dev/serial0:921600");
  EXPECT_EQ(uart.kind, LinkSpec::Kind::Uart);
  EXPECT_EQ(uart.device, "/dev/serial0");
  EXPECT_EQ(uart.baud, 921600);
}

TEST(LinkSpecTest, RejectsMalformedSpecs)
{
  EXPECT_THROW(parseLinkSpec("tcp:5760"), std::invalid_argument);
  EXPECT_THROW(parseLinkSpec("udp:port"), std::invalid_argument);
  EXPECT_THROW(parseLinkSpec("udp:70000"), std::invalid_argument);
  EXPECT_THROW(parseLinkSpec("udp:1:host"), std::invalid_argument);
  EXPECT_THROW(parseLinkSpec("uart:/dev/serial0"), std::invalid_argument);
  EXPECT_THROW(parseLinkSpec("uart:/dev/serial0:0"), std::invalid_argument);
}

TEST(UdpLinkTest, LearnsPeerFromFirstDatagramAndReplies)
{
  // Setup: `listener` has no remote, `client` sends to the listener's port
  UdpLink listener(0);
  UdpLink client(0, "127.0.0.1", listener.localPort());

  // Run + Assert
  EXPECT_EQ(listener.send(bytesOf("too early")), 0);
  EXPECT_EQ(client.send(bytesOf("hello")), 5);
  EXPECT_EQ(receiveText(listener), "hello");
  EXPECT_EQ(listener.send(bytesOf("back")), 4);
  EXPECT_EQ(receiveText(client), "back");
}

TEST(UdpLinkTest, WaitReadableTimesOutWithoutData)
{
  UdpLink link(0);
  auto start = std::chrono::steady_clock::now();

  EXPECT_EQ(link.waitReadable(30ms), ByteLink::WaitStatus::Timeout);
  EXPECT_GE(std::chrono::steady_clock::now() - start, 25ms);
  std::array<uint8_t, 16> buffer{};
  EXPECT_EQ(link.receive(buffer), 0);
}

TEST(UdpLinkTest, UnresolvableRemoteThrows)
{
  EXPECT_THROW(UdpLink(0, "no-such-host.invalid", 1), std::runtime_error);
}

TEST(UartLinkTest, PassesBytesBothWaysThroughPseudoTerminal)
{
  // Setup: the pty master plays the flight controller
  int master = -1;
  int slave = -1;
  std::array<char, 128> name{};
  ASSERT_EQ(openpty(&master, &slave, name.data(), nullptr, nullptr), 0);
  UartLink link(name.data(), 921600);

  // Run + Assert: FC -> link
  ASSERT_EQ(::write(master, "ping", 4), 4);
  EXPECT_EQ(receiveText(link), "ping");

  // link -> FC
  EXPECT_EQ(link.send(bytesOf("pong")), 4);
  std::array<char, 16> reply{};
  ssize_t n = ::read(master, reply.data(), reply.size());
  EXPECT_EQ(std::string(reply.data(), n > 0 ? static_cast<size_t>(n) : 0), "pong");

  ::close(slave);
  ::close(master);
}

TEST(UartLinkTest, MissingDeviceThrows)
{
  EXPECT_THROW(UartLink("/dev/does-not-exist", 921600), std::runtime_error);
  EXPECT_THROW(openLink(parseLinkSpec("uart:/dev/does-not-exist:921600")), std::runtime_error);
}
