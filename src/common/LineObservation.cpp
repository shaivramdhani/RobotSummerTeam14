#include "common/LineObservation.h"

namespace robot {

LineObservation observeDigitalLineSensors(
    const bool left_black, const bool right_black,
    const std::int8_t previous_last_known_side,
    const Milliseconds timestamp_ms) {
  const std::int8_t previous_side =
      normalizeLastKnownSide(previous_last_known_side);

  LineObservation observation{};
  observation.leftOnTape = left_black;
  observation.rightOnTape = right_black;
  observation.timestampMs = timestamp_ms;
  observation.left_black = left_black;
  observation.right_black = right_black;
  observation.timestamp_ms = timestamp_ms;
  observation.observed_at_ms = timestamp_ms;
  observation.lastKnownSide = previous_side;
  observation.last_known_side = previous_side;

  if (left_black && right_black) {
    observation.kind = LineObservationKind::Centered;
    observation.error = 0;
    observation.lineVisible = true;
    observation.line_visible = true;
    observation.hasHistory = previous_side != 0;
    observation.safe_to_drive = true;
  } else if (left_black) {
    observation.kind = LineObservationKind::LeftOnly;
    observation.error = 1;
    observation.lastKnownSide = 1;
    observation.last_known_side = 1;
    observation.lineVisible = true;
    observation.line_visible = true;
    observation.hasHistory = true;
    observation.safe_to_drive = true;
  } else if (right_black) {
    observation.kind = LineObservationKind::RightOnly;
    observation.error = -1;
    observation.lastKnownSide = -1;
    observation.last_known_side = -1;
    observation.lineVisible = true;
    observation.line_visible = true;
    observation.hasHistory = true;
    observation.safe_to_drive = true;
  } else if (previous_side > 0) {
    observation.kind = LineObservationKind::LostWithHistory;
    observation.error = 5;
    observation.lastKnownSide = 1;
    observation.last_known_side = 1;
    observation.lineVisible = false;
    observation.line_visible = false;
    observation.hasHistory = true;
    observation.safe_to_drive = true;
  } else if (previous_side < 0) {
    observation.kind = LineObservationKind::LostWithHistory;
    observation.error = -5;
    observation.lastKnownSide = -1;
    observation.last_known_side = -1;
    observation.lineVisible = false;
    observation.line_visible = false;
    observation.hasHistory = true;
    observation.safe_to_drive = true;
  } else {
    observation.kind = LineObservationKind::LostWithoutHistory;
    observation.error = 0;
    observation.lastKnownSide = 0;
    observation.last_known_side = 0;
    observation.lineVisible = false;
    observation.line_visible = false;
    observation.hasHistory = false;
    observation.safe_to_drive = false;
  }

  observation.line_error_milli =
      static_cast<std::int16_t>(observation.error) * 1000;
  return observation;
}

LineObservation observeDigitalLineSensorLevels(
    const bool left_electrical_high, const bool right_electrical_high,
    const std::int8_t previous_last_known_side,
    const Milliseconds timestamp_ms) {
  return observeDigitalLineSensors(left_electrical_high,
                                   right_electrical_high,
                                   previous_last_known_side, timestamp_ms);
}

LineObservation observeRearLineSensorsForReverseTravel(
    const bool back_left_black, const bool back_right_black,
    const std::int8_t previous_last_known_side,
    const Milliseconds timestamp_ms) {
  // In the reverse travel frame, the physical back-right sensor is on the
  // traveler's left and the physical back-left sensor is on its right.
  return observeDigitalLineSensors(back_right_black, back_left_black,
                                   previous_last_known_side, timestamp_ms);
}

LineObservation observeFrontLine(const FrontLineSensorSnapshot& snapshot) {
  if (!snapshot.valid || snapshot.left == LineSample::Unknown ||
      snapshot.right == LineSample::Unknown) {
    LineObservation observation{};
    observation.kind = LineObservationKind::Invalid;
    observation.timestampMs = snapshot.captured_at_ms;
    observation.timestamp_ms = snapshot.captured_at_ms;
    observation.observed_at_ms = snapshot.captured_at_ms;
    return observation;
  }

  return observeDigitalLineSensors(snapshot.left == LineSample::OnTape,
                                   snapshot.right == LineSample::OnTape, 0,
                                   snapshot.captured_at_ms);
}

}  // namespace robot
