#include "follow/config/ConfigJson.h"

#include <fstream>
#include <nlohmann/json.hpp>

#include "ObjectReader.h"

namespace follow::config {

using detail::ObjectReader;
using detail::require;
using nlohmann::json;

AppConfig parseAppConfig(const json& doc)
{
  AppConfig config;
  core::Config& core = config.core;
  ObjectReader root(doc, "", {"mavlink", "camera", "vision", "target", "estimator", "supervisor", "control"});

  ObjectReader mavlink = root.child("mavlink", {"link", "sysid", "compid", "fc_timeout_ms"});
  mavlink.read("link", config.mavlink.link);
  mavlink.read("sysid", config.mavlink.sysid);
  mavlink.read("compid", config.mavlink.compid);
  mavlink.readMs("fc_timeout_ms", core.supervisor.fcTimeout);
  require(config.mavlink.sysid >= 1 && config.mavlink.sysid <= 255, "mavlink.sysid: must be 1..255");
  require(config.mavlink.compid >= 1 && config.mavlink.compid <= 255, "mavlink.compid: must be 1..255");

  ObjectReader vision = root.child("vision",
                                   {"capture",
                                    "track",
                                    "tracker",
                                    "lock_box_frac",
                                    "min_confidence",
                                    "fps",
                                    "framebuffer",
                                    "overlay_fps",
                                    "reacquire_period_ms",
                                    "reacquire_expand"});
  vision.readSize("capture", config.vision.captureWidth, config.vision.captureHeight);
  vision.readSize("track", config.vision.trackWidth, config.vision.trackHeight);
  vision.read("tracker", config.vision.tracker);
  vision.read("lock_box_frac", core.lockBoxFrac);
  vision.read("min_confidence", core.estimator.minConfidence);
  vision.read("fps", config.vision.fps);
  vision.read("framebuffer", config.vision.framebuffer);
  vision.read("overlay_fps", config.vision.overlayFps);
  vision.read("reacquire_period_ms", config.vision.reacquirePeriodMs);
  vision.read("reacquire_expand", config.vision.reacquireExpand);
  require(config.vision.tracker == "kcf" || config.vision.tracker == "csrt", "vision.tracker: expected \"kcf\" or \"csrt\"");
  require(config.vision.fps > 0, "vision.fps: must be positive");
  require(config.vision.overlayFps > 0, "vision.overlay_fps: must be positive");
  require(config.vision.reacquirePeriodMs >= 0, "vision.reacquire_period_ms: must not be negative");
  require(config.vision.reacquireExpand >= 1.0, "vision.reacquire_expand: must be at least 1");
  require(core.lockBoxFrac > 0.0 && core.lockBoxFrac <= 1.0, "vision.lock_box_frac: must be in (0, 1]");

  root.child("target", {"height_m"}).readOptional("height_m", core.estimator.targetHeightM);

  ObjectReader estimator = root.child("estimator", {"ema_alpha", "max_area_jump", "attitude_stale_ms", "border_margin_px"});
  estimator.read("ema_alpha", core.estimator.emaAlpha);
  estimator.read("max_area_jump", core.estimator.maxAreaJump);
  estimator.readMs("attitude_stale_ms", core.estimator.attitudeStale);
  estimator.read("border_margin_px", core.estimator.borderMarginPx);

  ObjectReader supervisor = root.child("supervisor", {"lock_timeout_ms", "lost_timeout_ms", "stale_ms"});
  supervisor.readMs("lock_timeout_ms", core.supervisor.lockTimeout);
  supervisor.readMs("lost_timeout_ms", core.supervisor.lostTimeout);
  supervisor.readMs("stale_ms", core.estimator.stale);

  ObjectReader control = root.child("control",
                                    {"rate_hz",
                                     "k_yaw",
                                     "yaw_rate_max_dps",
                                     "yaw_deadband_deg",
                                     "k_d",
                                     "vx_max",
                                     "vx_slew",
                                     "d_set",
                                     "d_nominal",
                                     "d_min",
                                     "dist_deadband_m",
                                     "heading_gate_deg",
                                     "enable_vx"});
  control.read("rate_hz", core.rateHz);
  control.read("k_yaw", core.control.kYaw);
  control.read("yaw_rate_max_dps", core.control.yawRateMaxDps);
  control.read("yaw_deadband_deg", core.control.yawDeadbandDeg);
  control.read("k_d", core.control.kD);
  control.read("vx_max", core.control.vxMax);
  control.read("vx_slew", core.control.vxSlew);
  control.read("d_set", core.control.dSet);
  control.read("d_nominal", core.control.dNominal);
  control.read("d_min", core.control.dMin);
  control.read("dist_deadband_m", core.control.distDeadbandM);
  control.read("heading_gate_deg", core.control.headingGateDeg);
  control.read("enable_vx", core.control.enableVx);
  require(core.rateHz > 0.0, "control.rate_hz: must be positive");
  require(core.control.kYaw >= 0.0, "control.k_yaw: must not be negative");
  require(core.control.yawRateMaxDps > 0.0, "control.yaw_rate_max_dps: must be positive");
  require(core.control.yawDeadbandDeg >= 0.0, "control.yaw_deadband_deg: must not be negative");
  require(core.control.kD >= 0.0, "control.k_d: must not be negative");
  require(core.control.vxMax >= 0.0, "control.vx_max: must not be negative");
  require(core.control.vxSlew > 0.0, "control.vx_slew: must be positive");
  require(core.control.dSet > 0.0, "control.d_set: must be positive");
  require(core.control.dNominal > 0.0, "control.d_nominal: must be positive");
  require(core.control.dMin >= 0.0, "control.d_min: must not be negative");
  require(core.control.dMin < core.control.dSet, "control.d_min: must be less than control.d_set");
  require(core.control.distDeadbandM >= 0.0, "control.dist_deadband_m: must not be negative");
  require(core.control.headingGateDeg > 0.0, "control.heading_gate_deg: must be positive");
  require(core.estimator.emaAlpha > 0.0 && core.estimator.emaAlpha <= 1.0, "estimator.ema_alpha: must be in (0, 1]");
  require(core.estimator.maxAreaJump > 1.0, "estimator.max_area_jump: must be greater than 1");
  require(core.estimator.minConfidence >= 0.0 && core.estimator.minConfidence <= 1.0, "vision.min_confidence: must be in [0, 1]");
  require(core.estimator.borderMarginPx >= 0.0, "estimator.border_margin_px: must not be negative");

  return config;
}

CameraSettings parseCamera(const json& doc, int trackWidth, int trackHeight)
{
  ObjectReader camera(doc, "camera", {"model", "width", "height", "fx", "fy", "cx", "cy", "k1", "k2", "k3", "k4", "tilt_deg"});
  CameraSettings settings;

  std::string model;
  camera.readRequired("model", model);
  require(model == "fisheye" || model == "pinhole", "camera.model: expected \"fisheye\" or \"pinhole\"");
  settings.kind = model == "fisheye" ? CameraKind::Fisheye : CameraKind::Pinhole;

  core::Intrinsics k{};
  camera.readRequired("width", k.width);
  camera.readRequired("height", k.height);
  camera.readRequired("fx", k.fx);
  camera.readRequired("fy", k.fy);
  camera.readRequired("cx", k.cx);
  camera.readRequired("cy", k.cy);
  camera.read("k1", k.k1);
  camera.read("k2", k.k2);
  camera.read("k3", k.k3);
  camera.read("k4", k.k4);
  camera.read("tilt_deg", settings.mount.tiltUpDeg);
  require(k.width > 0 && k.height > 0 && k.fx > 0.0 && k.fy > 0.0, "camera: width, height, fx and fy must be positive");

  // Calibration may run at the capture resolution; tracking runs at trackWidth x trackHeight.
  double sx = static_cast<double>(trackWidth) / k.width;
  double sy = static_cast<double>(trackHeight) / k.height;
  k.fx *= sx;
  k.cx *= sx;
  k.fy *= sy;
  k.cy *= sy;
  k.width = trackWidth;
  k.height = trackHeight;
  settings.intrinsics = k;
  return settings;
}

json readJsonFile(const std::filesystem::path& path)
{
  std::ifstream in(path);
  require(static_cast<bool>(in), path.string() + ": cannot open");
  try {
    return json::parse(in);
  }
  catch (const json::parse_error& e) {
    throw ConfigError(path.string() + ": " + e.what());
  }
}

AppConfig loadAppConfig(const std::filesystem::path& path, const json& overrides)
{
  json doc = readJsonFile(path);
  doc.merge_patch(overrides);
  try {
    AppConfig config = parseAppConfig(doc);
    if (auto it = doc.find("camera"); it != doc.end()) {
      require(it->is_string(), "camera: expected a file name");
      std::filesystem::path cameraPath = path.parent_path() / it->get<std::string>();
      config.camera = parseCamera(readJsonFile(cameraPath), config.vision.trackWidth, config.vision.trackHeight);
    }
    return config;
  }
  catch (const ConfigError& e) {
    throw ConfigError(path.string() + ": " + e.what());
  }
}

AppConfig loadAppConfig(const std::filesystem::path& path)
{
  return loadAppConfig(path, json::object());
}

std::unique_ptr<core::CameraModel> makeCameraModel(const CameraSettings& settings)
{
  if (settings.kind == CameraKind::Pinhole) {
    return std::make_unique<core::PinholeModel>(settings.intrinsics);
  }
  return std::make_unique<core::FisheyeKbModel>(settings.intrinsics);
}

}  // namespace follow::config
