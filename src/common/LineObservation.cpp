#include "common/LineObservation.h"

namespace robot {

LineObservation observeDigitalLineSensors(
    const bool left_black, const bool right_black,
    const std::int8_t previous_last_known_side,
    const Milliseconds timestamp_ms) {
  const std::int8_t previous_side =
      normalizeLastKnownSide(previous_last_known_side);

  LineObservation observation{};
  observation.left_black = left_black;
  observation.right_black = right_black;
  observation.timestamp_ms = timestamp_ms;
  observation.observed_at_ms = timestamp_ms;
  observation.last_known_side = previous_side;

  if (left_black && right_black) {
    observation.kind = LineObservationKind::Centered;
    observation.error = 0;
    observation.line_visible = true;
    observation.safe_to_drive = true;
  } else if (left_black) {
    observation.kind = LineObservationKind::LeftOnly;
    observation.error = 1;
    observation.last_known_side = 1;
    observation.line_visible = true;
    observation.safe_to_drive = true;
  } else if (right_black) {
    observation.kind = LineObservationKind::RightOnly;
    observation.error = -1;
    observation.last_known_side = -1;
    observation.line_visible = true;
    observation.safe_to_drive = true;
  } else if (previous_side > 0) {
    observation.kind = LineObservationKind::LostWithHistory;
    observation.error = 5;
    observation.last_known_side = 1;
    observation.line_visible = false;
    observation.safe_to_drive = true;
  } else if (previous_side < 0) {
    observation.kind = LineObservationKind::LostWithHistory;
    observation.error = -5;
    observation.last_known_side = -1;
    observation.line_visible = false;
    observation.safe_to_drive = true;
  } else {
    observation.kind = LineObservationKind::LostWithoutHistory;
    observation.error = 0;
    observation.last_known_side = 0;
    observation.line_visible = false;
    observation.safe_to_drive = false;
  }

  observation.line_error_milli =
      static_cast<std::int16_t>(observation.error) * 1000;
  return observation;
}

LineObservation observeFrontLine(const FrontLineSensorSnapshot& snapshot) {
  if (!snapshot.valid || snapshot.left == LineSample::Unknown ||
      snapshot.right == LineSample::Unknown) {
    LineObservation observation{};
    observation.kind = LineObservationKind::Invalid;
    observation.timestamp_ms = snapshot.captured_at_ms;
    observation.observed_at_ms = snapshot.captured_at_ms;
    return observation;
  }

  return observeDigitalLineSensors(snapshot.left == LineSample::OnTape,
                                   snapshot.right == LineSample::OnTape, 0,
                                   snapshot.captured_at_ms);
}

}  // namespace robot
