#include <gtest/gtest.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <string>

#include "follow/config/ConfigJson.h"

using namespace follow;
using namespace follow::config;
using nlohmann::json;
using namespace std::chrono_literals;

namespace {

// Runs parseAppConfig and returns the ConfigError message, or "" if it did not throw.
std::string errorOf(const char* text)
{
  try {
    parseAppConfig(json::parse(text));
  }
  catch (const ConfigError& e) {
    return e.what();
  }
  return "";
}

void expectIntrinsicsNear(const core::Intrinsics& actual, const core::Intrinsics& expected)
{
  EXPECT_EQ(actual.width, expected.width);
  EXPECT_EQ(actual.height, expected.height);
  EXPECT_NEAR(actual.fx, expected.fx, 1e-6);
  EXPECT_NEAR(actual.fy, expected.fy, 1e-6);
  EXPECT_NEAR(actual.cx, expected.cx, 1e-6);
  EXPECT_NEAR(actual.cy, expected.cy, 1e-6);
}

}  // namespace

TEST(ConfigTest, EmptyDocumentKeepsDefaults)
{
  // Run
  AppConfig config = parseAppConfig(json::object());

  // Assert
  core::Config defaults{};
  EXPECT_EQ(config.core.rateHz, defaults.rateHz);
  EXPECT_EQ(config.core.control.vxMax, defaults.control.vxMax);
  EXPECT_EQ(config.core.supervisor.lostTimeout, defaults.supervisor.lostTimeout);
  EXPECT_FALSE(config.core.estimator.targetHeightM);
  EXPECT_EQ(config.mavlink.link, "uart:/dev/serial0:921600");
  EXPECT_EQ(config.camera.kind, CameraKind::Fisheye);
  expectIntrinsicsNear(config.camera.intrinsics, core::nominalFisheye(640, 480, 160.0));
}

TEST(ConfigTest, CommittedFollowJsonMatchesSpecDefaults)
{
  // Run
  AppConfig config = loadAppConfig(FOLLOW_CONFIG_DIR "/follow.json");

  // Assert: every tunable equals the built-in default from the spec
  core::Config d{};
  const core::Config& c = config.core;
  EXPECT_EQ(c.rateHz, d.rateHz);
  EXPECT_EQ(c.lockBoxFrac, d.lockBoxFrac);
  EXPECT_EQ(c.estimator.emaAlpha, d.estimator.emaAlpha);
  EXPECT_EQ(c.estimator.stale, d.estimator.stale);
  EXPECT_EQ(c.estimator.attitudeStale, d.estimator.attitudeStale);
  EXPECT_EQ(c.estimator.borderMarginPx, d.estimator.borderMarginPx);
  EXPECT_EQ(c.estimator.maxAreaJump, d.estimator.maxAreaJump);
  EXPECT_EQ(c.estimator.minConfidence, d.estimator.minConfidence);
  EXPECT_EQ(c.estimator.targetHeightM, d.estimator.targetHeightM);
  EXPECT_EQ(c.control.kYaw, d.control.kYaw);
  EXPECT_EQ(c.control.yawRateMaxDps, d.control.yawRateMaxDps);
  EXPECT_EQ(c.control.yawDeadbandDeg, d.control.yawDeadbandDeg);
  EXPECT_EQ(c.control.kD, d.control.kD);
  EXPECT_EQ(c.control.vxMax, d.control.vxMax);
  EXPECT_EQ(c.control.vxSlew, d.control.vxSlew);
  EXPECT_EQ(c.control.dSet, d.control.dSet);
  EXPECT_EQ(c.control.dNominal, d.control.dNominal);
  EXPECT_EQ(c.control.dMin, d.control.dMin);
  EXPECT_EQ(c.control.distDeadbandM, d.control.distDeadbandM);
  EXPECT_EQ(c.control.headingGateDeg, d.control.headingGateDeg);
  EXPECT_EQ(c.control.enableVx, d.control.enableVx);
  EXPECT_EQ(c.supervisor.fcTimeout, d.supervisor.fcTimeout);
  EXPECT_EQ(c.supervisor.lockTimeout, d.supervisor.lockTimeout);
  EXPECT_EQ(c.supervisor.lostTimeout, d.supervisor.lostTimeout);
  EXPECT_EQ(config.mavlink.link, "uart:/dev/serial0:921600");
  EXPECT_EQ(config.mavlink.sysid, 1);
  EXPECT_EQ(config.mavlink.compid, 191);
  EXPECT_EQ(config.vision.trackWidth, 640);
  EXPECT_EQ(config.vision.trackHeight, 480);
  EXPECT_EQ(config.camera.kind, CameraKind::Fisheye);
  expectIntrinsicsNear(config.camera.intrinsics, core::nominalFisheye(640, 480, 160.0));
}

TEST(ConfigTest, LoadAppliesOverridesAsMergePatch)
{
  AppConfig config = loadAppConfig(FOLLOW_CONFIG_DIR "/follow.json", json::parse(R"({"control": {"enable_vx": false}})"));

  EXPECT_FALSE(config.core.control.enableVx);
  EXPECT_EQ(config.core.control.vxMax, 1.5);
}

TEST(ConfigTest, ValuesOverrideDefaults)
{
  // Run
  AppConfig config = parseAppConfig(json::parse(R"({
    "mavlink": { "link": "udp:14560", "fc_timeout_ms": 1500 },
    "target": { "height_m": 1.7 },
    "supervisor": { "lost_timeout_ms": 5000, "stale_ms": 250 },
    "control": { "enable_vx": false, "vx_max": 0.5 }
  })"));

  // Assert
  EXPECT_EQ(config.mavlink.link, "udp:14560");
  EXPECT_EQ(config.core.supervisor.fcTimeout, 1500ms);
  EXPECT_EQ(config.core.estimator.targetHeightM, 1.7);
  EXPECT_EQ(config.core.supervisor.lostTimeout, 5000ms);
  EXPECT_EQ(config.core.estimator.stale, 250ms);
  EXPECT_FALSE(config.core.control.enableVx);
  EXPECT_EQ(config.core.control.vxMax, 0.5);
}

TEST(ConfigTest, UnknownKeyIsRejectedWithItsPath)
{
  EXPECT_EQ(errorOf(R"({"control": {"enable_vxx": false}})"), "control.enable_vxx: unknown key");
  EXPECT_EQ(errorOf(R"({"contrl": {}})"), "contrl: unknown key");
}

TEST(ConfigTest, WrongTypeIsRejectedWithItsPath)
{
  EXPECT_EQ(errorOf(R"({"control": {"k_d": "fast"}})").rfind("control.k_d: ", 0), 0u);
  EXPECT_EQ(errorOf(R"({"control": []})"), "control: expected an object");
}

TEST(ConfigTest, OutOfRangeValuesAreRejected)
{
  EXPECT_EQ(errorOf(R"({"control": {"d_min": 3.0}})"), "control.d_min: must be less than control.d_set");
  EXPECT_EQ(errorOf(R"({"control": {"rate_hz": 0}})"), "control.rate_hz: must be positive");
  EXPECT_EQ(errorOf(R"({"vision": {"lock_box_frac": 1.5}})"), "vision.lock_box_frac: must be in (0, 1]");
  EXPECT_EQ(errorOf(R"({"supervisor": {"lost_timeout_ms": -1}})"), "supervisor.lost_timeout_ms: must not be negative");
  EXPECT_EQ(errorOf(R"({"mavlink": {"compid": 300}})"), "mavlink.compid: must be 1..255");
}

TEST(ConfigTest, CameraIntrinsicsAreScaledToTrackingResolution)
{
  // Setup: a calibration done at the full 1640x1232 capture resolution
  json calibration = json::parse(R"({
    "model": "fisheye", "width": 1640, "height": 1232,
    "fx": 700.0, "fy": 701.0, "cx": 820.0, "cy": 616.0,
    "k1": 0.01, "k2": -0.002, "k3": 0.0, "k4": 0.0005, "tilt_deg": 10.0
  })");

  // Run
  CameraSettings camera = parseCamera(calibration, 640, 480);

  // Assert: focal lengths and principal point scale per axis, distortion does not
  EXPECT_EQ(camera.intrinsics.width, 640);
  EXPECT_EQ(camera.intrinsics.height, 480);
  EXPECT_NEAR(camera.intrinsics.fx, 700.0 * 640.0 / 1640.0, 1e-9);
  EXPECT_NEAR(camera.intrinsics.fy, 701.0 * 480.0 / 1232.0, 1e-9);
  EXPECT_NEAR(camera.intrinsics.cx, 320.0, 1e-9);
  EXPECT_NEAR(camera.intrinsics.cy, 240.0, 1e-9);
  EXPECT_EQ(camera.intrinsics.k1, 0.01);
  EXPECT_EQ(camera.intrinsics.k4, 0.0005);
  EXPECT_EQ(camera.mount.tiltUpDeg, 10.0);
}

TEST(ConfigTest, CameraModelFollowsTheModelField)
{
  // Setup
  json pinhole = json::parse(R"({"model": "pinhole", "width": 640, "height": 480, "fx": 500, "fy": 500, "cx": 320, "cy": 240})");
  json fisheye = pinhole;
  fisheye["model"] = "fisheye";

  // Run
  auto pinholeModel = makeCameraModel(parseCamera(pinhole, 640, 480));
  auto fisheyeModel = makeCameraModel(parseCamera(fisheye, 640, 480));

  // Assert
  EXPECT_NE(dynamic_cast<core::PinholeModel*>(pinholeModel.get()), nullptr);
  EXPECT_NE(dynamic_cast<core::FisheyeKbModel*>(fisheyeModel.get()), nullptr);
}

TEST(ConfigTest, MissingCameraFieldIsRejected)
{
  try {
    parseCamera(json::parse(R"({"model": "fisheye", "width": 640, "height": 480, "fx": 300, "fy": 300, "cx": 320})"), 640, 480);
    FAIL() << "expected ConfigError";
  }
  catch (const ConfigError& e) {
    EXPECT_STREQ(e.what(), "camera.cy: required");
  }
}
