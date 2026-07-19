#pragma once

#include <cstdint>
#include <optional>

/*
  У чистій C++ логіці потрібно виділити щонайменше такі частини:
- команда yaw-серво: `Target.x` -> `ServoCommand.direction`, `target_x`,
  `error_x`;
- команда гімбала: `Target.y` -> `GimbalCommand.direction`, `target_y`,
  `error_y`;
- рішення щодо пострілу: `distance_m`, `max_distance_m`,
  останній `ActuatorStatus.state` -> `TRIGGER_SKIP`, `TRIGGER_REQUESTED` або
  `TRIGGER_RELOADING`;
- складання `TurretStatus` для перевірки через `/turret/status`.
*/

namespace antidrone_turret {

inline constexpr auto kFrameWidth = 640;
inline constexpr auto kFrameHight = 480;
inline constexpr auto kCenterX = 320;
inline constexpr auto kCenterY = 240;

enum class ActuatorState {
  kReady,
  kReloading,
};

struct TargetObservation {
  bool visible;
  float x;
  float y;
  float distance_m;
  float confidence;
};

struct ControllerConfig {
  double confidence_threshold = 0.80;  // if >= `0.80` =>достатньо надійне розпізнавання
  //  Значення нижче `0.80` потрібно трактувати як `TARGET_LOW_CONFIDENCE`
  double max_distance_m = 30.0;
};

enum class Action : std::uint8_t {  // описує, чи треба рухати наведення:
  kActionIdle,  //`ACTION_IDLE` - не публікувати нові команди наведення, бо ціль не придатна;
  kActionTrack  //`ACTION_TRACK` - публікувати `GimbalCommand` і `ServoCommand`.
};

enum class TargetState : std::uint8_t {
  kTargetNone,           //`TARGET_NONE` - ціль не видима;
  kTargetLowConfidence,  //`TARGET_LOW_CONFIDENCE` - ціль є, але `confidence < confidence_threshold` (default- менше 80%;)
  kTargetLocked          //`TARGET_LOCKED` - ціль видима і розпізнавання достатньо надійне.
};

enum class TriggerDecision : std::uint8_t {  // описує рішення щодо команди пострілу
  kTriggerSkip,                              //`TRIGGER_SKIP` - не викликати сервіс пострілу;
  kTriggerRequested,  //`TRIGGER_REQUESTED` - викликати сервіс пострілу, бо ціль близько і актуатор готовий
  kTriggerReloading  //`TRIGGER_RELOADING` - ціль близько, але актуатор ще перезаряджається
};

enum class GimbalDirection : std::int8_t {
  kDown = -1,   //`y > 240` -> `GimbalCommand.direction=DOWN`;
  kCenter = 0,  // `y == 240` -> `GimbalCommand.direction=CENTER`.
  kUp = 1       //`y < 240` -> `GimbalCommand.direction=UP`;
};

enum class ServoDirection : std::int8_t {
  kLeft = -1,   // `x < 320` -> `ServoCommand.direction=LEFT`;
  kCenter = 0,  //`x == 320` -> `ServoCommand.direction=CENTER`;
  kRight = 1    //`x > 320` -> `ServoCommand.direction=RIGHT`;
};

struct ServoCmd {
  float target_x;            // - скопійоване значення Target.x;
  float error_x;             // = Target.x - 320.0;
  ServoDirection direction;  //=RIGHT, якщо error_x > 0; =LEFT, якщо error_x < 0; =CENTER, якщо error_x == 0.
};

struct GimbalCmd {
  float target_y;             // - скопійоване значення Target.y;
  float error_y;              // = 240.0 - Target.y;
  GimbalDirection direction;  //=UP, якщо error_y > 0; =DOWN, якщо error_y < 0;=CENTER, якщо error_y == 0
};

struct TurretDecision {
  TargetState target_state{TargetState::kTargetNone};
  Action action{Action::kActionIdle};
  TriggerDecision trigger_decision{TriggerDecision::kTriggerSkip};
  ServoCmd servo;
  GimbalCmd gimbal;

  float confidence;  // оцінка надійності розпізнавання цілі 0.0..1.0 - в кадрі саме FPV-ціль, а не шум або інший об'єкт.
  float distance_m;

  TurretDecision(float confidence, float distance_m)
    : confidence(confidence)
    , distance_m(distance_m){};
};

// оцінка цілі: `visible`, `confidence_threshold` -> `TARGET_NONE`,
//  `TARGET_LOW_CONFIDENCE` або `TARGET_LOCKED`;
[[nodiscard]] TargetState evaluate_target(const TargetObservation& target, const antidrone_turret::ControllerConfig& config) 
{
  if (!target.visible) {
    return TargetState::kTargetNone;
  }

  if (target.confidence < config.confidence_threshold) {
    return TargetState::kTargetLowConfidence;
  }

  return TargetState::kTargetLocked;
}

GimbalCmd make_gimbal_decision(float target_y)
{
  GimbalCmd cmd;
  cmd.target_y = target_y;
  cmd.error_y = kCenterY - target_y;
  cmd.direction = cmd.error_y < 0 ? GimbalDirection::kDown : (cmd.error_y > 0 ? GimbalDirection::kUp : GimbalDirection::kCenter);
  return cmd;
}

ServoCmd make_servo_decision(float target_x)
{
  ServoCmd cmd;
  cmd.target_x = target_x;
  cmd.error_x = kCenterX - target_x;
  cmd.direction = cmd.error_x < 0 ? ServoDirection::kLeft : (cmd.error_x > 0 ? ServoDirection::kRight : ServoDirection::kCenter);
  return cmd;
}

/* 
- `distance_m <= max_distance_m` і актуатор `READY` ->  `TRIGGER_REQUESTED`;
- `distance_m <= max_distance_m` і актуатор `RELOADING` -> `TRIGGER_RELOADING`;
- `distance_m > max_distance_m` -> `TRIGGER_SKIP`. */
[[nodiscard]] auto make_trigger_decision(float distance_m, float max_distance_m, std::optional<ActuatorState> actuator_state) -> TriggerDecision
{
  if (distance_m > max_distance_m) { //далеко
    return TriggerDecision::kTriggerSkip;
  }

  if (!actuator_state.has_value()) {
    // Статус актуатора ще не отримано
    return TriggerDecision::kTriggerSkip;
  }

  if (*actuator_state == ActuatorState::kReady) {
    return TriggerDecision::kTriggerRequested;
  }

  return TriggerDecision::kTriggerReloading;
}


TurretDecision decide_on_target(const TargetObservation& target,
                                std::optional<ActuatorState> actuator_state,
                                const antidrone_turret::ControllerConfig& config)
{
  TurretDecision decision(target.confidence, target.distance_m);
  decision.target_state = evaluate_target(target, config);

  if ((decision.target_state == TargetState::kTargetNone) || (decision.target_state == TargetState::kTargetLowConfidence)) {
    //- `visible=false` -> `TARGET_NONE`, `ACTION_IDLE`, `TRIGGER_SKIP`;
    return decision;
  }

  decision.action = Action::kActionTrack;
  decision.gimbal = make_gimbal_decision(target.y);
  decision.servo = make_servo_decision(target.x);
  decision.trigger_decision = make_trigger_decision(target.distance_m, config.max_distance_m, actuator_state);

  return decision;
}

}  // namespace antidrone_turret