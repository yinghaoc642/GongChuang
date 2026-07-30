#include "CompetitionCore.h"

#include <string.h>

namespace competition {
namespace {

bool characterInRange(
    char value, char minimum, char maximum) {
  return value >= minimum && value <= maximum;
}

uint8_t threeDigitMask(
    const char *digits, char minimum, char maximum) {
  uint8_t mask = 0U;
  for (uint8_t index = 0U; index < 3U; ++index) {
    if (!characterInRange(
            digits[index], minimum, maximum)) {
      return 0U;
    }

    const uint8_t bit = static_cast<uint8_t>(
        1U << (digits[index] - minimum));
    if ((mask & bit) != 0U) {
      return 0U;
    }
    mask = static_cast<uint8_t>(mask | bit);
  }
  return mask;
}

int16_t normalizedCardinalHeading(int16_t degrees) {
  int16_t normalized = static_cast<int16_t>(degrees % 360);
  if (normalized < 0) {
    normalized = static_cast<int16_t>(normalized + 360);
  }
  return normalized;
}

} // namespace

TaskPlan::TaskPlan() { clear(); }

void TaskPlan::clear() {
  memset(colors, 0, sizeof(colors));
  memset(positions, 0, sizeof(positions));
}

TaskCodeStatus parseTaskCode(
    const char *code, TaskPlan &plan) {
  plan.clear();
  if (code == nullptr) {
    return TASK_CODE_NULL;
  }
  if (strlen(code) != TASK_CODE_LENGTH) {
    return TASK_CODE_WRONG_LENGTH;
  }
  if (code[3] != '+' ||
      code[7] != '+' ||
      code[11] != '+') {
    return TASK_CODE_WRONG_SEPARATOR;
  }

  const uint8_t firstColorMask =
      threeDigitMask(code, '1', '4');
  const uint8_t secondColorMask =
      threeDigitMask(code + 8, '1', '4');
  if (firstColorMask == 0U || secondColorMask == 0U) {
    return TASK_CODE_INVALID_COLOR_GROUP;
  }
  if (firstColorMask != secondColorMask) {
    return TASK_CODE_COLOR_SET_MISMATCH;
  }

  constexpr uint8_t POSITION_MASK_123 = 0x07U;
  if (threeDigitMask(code + 4, '1', '3') !=
          POSITION_MASK_123 ||
      threeDigitMask(code + 12, '1', '3') !=
          POSITION_MASK_123) {
    return TASK_CODE_INVALID_POSITION_GROUP;
  }

  for (uint8_t item = 0U; item < 3U; ++item) {
    plan.colors[0][item] =
        static_cast<uint8_t>(code[item] - '0');
    plan.positions[0][item] =
        static_cast<uint8_t>(code[4U + item] - '0');
    plan.colors[1][item] =
        static_cast<uint8_t>(code[8U + item] - '0');
    plan.positions[1][item] =
        static_cast<uint8_t>(code[12U + item] - '0');
  }
  return TASK_CODE_OK;
}

uint8_t stackedRingForColor(
    const TaskPlan &plan, uint8_t secondBatchColor) {
  for (uint8_t item = 0U; item < 3U; ++item) {
    if (plan.colors[0][item] == secondBatchColor) {
      return plan.positions[0][item];
    }
  }
  return 0U;
}

Rectangle startZoneBounds(
    StartZone zone,
    float fieldSizeMm,
    float zoneSizeMm) {
  const float minimumX = fieldSizeMm - zoneSizeMm;
  if (zone == START_ZONE_2) {
    return Rectangle(
        minimumX, fieldSizeMm, 0.0f, zoneSizeMm);
  }
  return Rectangle(
      minimumX,
      fieldSizeMm,
      fieldSizeMm - zoneSizeMm,
      fieldSizeMm);
}

Rectangle axisAlignedFootprint(
    float centerX,
    float centerY,
    float footprintX,
    float footprintY,
    int16_t cardinalHeadingDegrees) {
  const int16_t heading =
      normalizedCardinalHeading(cardinalHeadingDegrees);
  const bool swapsAxes = heading == 90 || heading == 270;
  const float halfX =
      (swapsAxes ? footprintY : footprintX) * 0.5f;
  const float halfY =
      (swapsAxes ? footprintX : footprintY) * 0.5f;
  return Rectangle(
      centerX - halfX,
      centerX + halfX,
      centerY - halfY,
      centerY + halfY);
}

bool rectangleContains(
    const Rectangle &outer, const Rectangle &inner) {
  return inner.minimumX >= outer.minimumX &&
         inner.maximumX <= outer.maximumX &&
         inner.minimumY >= outer.minimumY &&
         inner.maximumY <= outer.maximumY;
}

} // namespace competition
