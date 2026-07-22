/* 
У ДЗ можливі такі сценарії:
- дальній проліт: ціль видима і має достатню оцінку надійності
  розпізнавання, тому турель має наводити гімбал і серво, але сервіс
  пострілу `/actuator/trigger` не викликається, бо
  `distance_m > max_distance_m`;
- ненадійне розпізнавання: ціль близько, але оцінка надійності розпізнавання
  нижче `confidence_threshold`. За замовчуванням це `0.80`, тобто нижче 80%.
  У цьому випадку контролер має перейти в `TARGET_LOW_CONFIDENCE`, не рухати
  наведення і не запитувати постріл;
- вдалий момент для пострілу: ціль видима, оцінка надійності розпізнавання
  достатня, `distance_m <= max_distance_m`, актуатор у стані `READY`;
  контролер має навести гімбал і серво, опублікувати статус і один раз
  викликати сервіс пострілу;
- перезаряджання: після прийнятої команди пострілу актуатор публікує
  `RELOADING`;
  поки цей стан активний, близька ціль може продовжувати приходити у
  `/perception/target`, але повторну команду пострілу викликати не потрібно;
- наступний епізод: після вдалого пострілу або після завершення поточного
  епізоду трек може перейти до наступної цілі, тому контролер має працювати
  як потокова система і не покладатися на одноразовий запуск;
- запізнілий запит пострілу: якщо близька ціль приходить під час
  `RELOADING`, її потрібно позначити як `TRIGGER_RELOADING`; це демонструє,
  що правильна поведінка системи залежить не тільки від дистанції і оцінки
  надійності розпізнавання, а й від останнього стану актуатора;
- тиск на перезаряджання: в `reload_pressure.csv` перший близький FPV може
  отримати постріл, але наступний FPV підлітає, поки актуатор ще не готовий.
  Контролер не має робити повторний запит пострілу під час `RELOADING`.

Мінімальні тести:
    • невидима ціль;
    • confidence нижче порога;
    • x > 320 → RIGHT;
    • x < 320 → LEFT;
    • y < 240 → UP;
    • y > 240 → DOWN;
    • близька ціль + READY → TRIGGER_REQUESTED;
    • близька ціль + RELOADING → TRIGGER_RELOADING;
    • далека валідна ціль → TRACK + TRIGGER_SKIP.
    */

/* Очікувані рішення для готових треків:

```text
approach_trigger.csv:
коли distance_m <= 30 і confidence >= 0.80:
розпізнавання достатньо надійне
READY -> TRIGGER_REQUESTED

far_flyby_no_trigger.csv:
distance_m завжди > 30:
TRIGGER_SKIP

low_confidence_no_trigger.csv:
confidence завжди < 0.80, тобто нижче 80%:
розпізнавання недостатньо надійне
ACTION_IDLE, TRIGGER_SKIP

reload_pressure.csv:
перший близький кадр із надійним розпізнаванням:
READY -> TRIGGER_REQUESTED
наступний близький FPV приходить, поки актуатор у RELOADING:
RELOADING -> TRIGGER_RELOADING
якщо FPV доходить до захищеної зони до READY:
повторний постріл заборонений, епізод вважається пропущеним


Для власної логіки
контролера потрібно додати мінімальні тести: хоча б один тест на кожну
частину рішення.

```text
оцінка цілі:
confidence нижче порога -> ACTION_IDLE, TRIGGER_SKIP

команда yaw-серво:
x > 320 -> ServoCommand RIGHT, error_x > 0

команда гімбала:
y < 240 -> GimbalCommand UP, error_y > 0

рішення щодо пострілу:
близька ціль + актуатор READY -> TRIGGER_REQUESTED
близька ціль + актуатор RELOADING -> TRIGGER_RELOADING

статус контролера:
далека коректна ціль -> TARGET_LOCKED, ACTION_TRACK, TRIGGER_SKIP

*/

#include "antidrone_turret/turret_controller_logic.hpp"
#include "antidrone_turret/target_sequence.hpp"
#include <gtest/gtest.h>

namespace {

antidrone_turret::ControllerConfig config{};
std::vector<antidrone_turret::TargetSample> tgts = antidrone_turret::default_target_samples();

// turret_controller_logic_test
/*  оцінка цілі:
confidence нижче порога -> ACTION_IDLE, TRIGGER_SKIP */
TEST(TurretControllerLogicTest, TargetLowConfidenceScenario)
{
  const auto& tgt_low_confidence = tgts.at(0);
  antidrone_turret::TargetObservation tgt{
    tgt_low_confidence.visible, tgt_low_confidence.x, tgt_low_confidence.y, tgt_low_confidence.distance_m, tgt_low_confidence.confidence};

  antidrone_turret::TurretDecision decision = antidrone_turret::decide_on_target(tgt, antidrone_turret::ActuatorState::kReady, config);
  EXPECT_EQ(decision.target_state, antidrone_turret::TargetState::kTargetLowConfidence);
  EXPECT_EQ(decision.action, antidrone_turret::Action::kActionIdle);
  EXPECT_EQ(decision.trigger_decision, antidrone_turret::TriggerDecision::kTriggerSkip);
}

/* команда yaw-серво:
x > 320 -> ServoCommand RIGHT, error_x > 0 */
TEST(TurretControllerLogicTest, YawServoTgtOnTheRightScenario)
{
  const auto& tgt_right = tgts.at(1);
  EXPECT_FLOAT_EQ(tgt_right.x, 340.0F);
  antidrone_turret::TargetObservation tgt{tgt_right.visible, tgt_right.x, tgt_right.y, tgt_right.distance_m, tgt_right.confidence};

  antidrone_turret::TurretDecision decision = antidrone_turret::decide_on_target(tgt, antidrone_turret::ActuatorState::kReady, config);
  EXPECT_EQ(decision.servo.direction, antidrone_turret::ServoDirection::kRight);
  EXPECT_GT(decision.servo.error_x, 0.0F);
}

/* команда гімбала:
y < 240 -> GimbalCommand UP, error_y > 0 */
TEST(TurretControllerLogicTest, GimbalTgtUpScenario)
{
  const auto& tgt_up = tgts.at(2);
  EXPECT_FLOAT_EQ(tgt_up.y, 215.0F);

  antidrone_turret::TargetObservation tgt{tgt_up.visible, tgt_up.x, tgt_up.y, tgt_up.distance_m, tgt_up.confidence};

  antidrone_turret::TurretDecision decision = antidrone_turret::decide_on_target(tgt, antidrone_turret::ActuatorState::kReady, config);

  EXPECT_EQ(decision.gimbal.direction, antidrone_turret::GimbalDirection::kUp);
  EXPECT_GT(decision.gimbal.error_y, 0.0F);
}

/* рішення щодо пострілу:
близька ціль + актуатор READY -> TRIGGER_REQUESTED
 */
TEST(TurretControllerLogicTest, TriggerDecisionReadyScenario)
{
  const auto& tgt_close = tgts.at(4);
  EXPECT_FLOAT_EQ(tgt_close.distance_m, 25.0F);

  antidrone_turret::TargetObservation tgt{tgt_close.visible, tgt_close.x, tgt_close.y, tgt_close.distance_m, tgt_close.confidence};

  antidrone_turret::TurretDecision decision = antidrone_turret::decide_on_target(tgt, antidrone_turret::ActuatorState::kReady, config);

  EXPECT_EQ(decision.trigger_decision, antidrone_turret::TriggerDecision::kTriggerRequested);
}

/* рішення щодо пострілу:
близька ціль + актуатор RELOADING -> TRIGGER_RELOADING
 */
TEST(TurretControllerLogicTest, TriggerDecisionRealodingScenario)
{
  const auto& tgt_close = tgts.at(4);
  EXPECT_FLOAT_EQ(tgt_close.distance_m, 25.0F);

  antidrone_turret::TargetObservation tgt{tgt_close.visible, tgt_close.x, tgt_close.y, tgt_close.distance_m, tgt_close.confidence};

  antidrone_turret::TurretDecision decision = antidrone_turret::decide_on_target(tgt, antidrone_turret::ActuatorState::kReloading, config);

  EXPECT_EQ(decision.trigger_decision, antidrone_turret::TriggerDecision::kTriggerReloading);
}

/*
статус контролера:
далека коректна ціль -> TARGET_LOCKED, ACTION_TRACK, TRIGGER_SKIP
 */
TEST(TurretControllerLogicTest, TurretStatusFarTargetScenario)
{
  const auto& tgt_far = tgts.at(3);
  EXPECT_FLOAT_EQ(tgt_far.distance_m, 31.0F);

  antidrone_turret::TargetObservation tgt{tgt_far.visible, tgt_far.x, tgt_far.y, tgt_far.distance_m, tgt_far.confidence};

  antidrone_turret::TurretDecision decision = antidrone_turret::decide_on_target(tgt, antidrone_turret::ActuatorState::kReady, config);

  EXPECT_EQ(decision.trigger_decision, antidrone_turret::TriggerDecision::kTriggerSkip);
  EXPECT_EQ(decision.target_state, antidrone_turret::TargetState::kTargetLocked);
  EXPECT_EQ(decision.action, antidrone_turret::Action::kActionTrack);
}
}  // namespace