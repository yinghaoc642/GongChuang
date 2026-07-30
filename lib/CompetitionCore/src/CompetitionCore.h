#ifndef GONGCHUANG_COMPETITION_CORE_H
#define GONGCHUANG_COMPETITION_CORE_H

#include <stddef.h>
#include <stdint.h>

namespace competition {

constexpr size_t TASK_CODE_LENGTH = 15U;

enum TaskCodeStatus : uint8_t {
  TASK_CODE_OK = 0U,
  TASK_CODE_NULL,
  TASK_CODE_WRONG_LENGTH,
  TASK_CODE_WRONG_SEPARATOR,
  TASK_CODE_INVALID_COLOR_GROUP,
  TASK_CODE_COLOR_SET_MISMATCH,
  TASK_CODE_INVALID_POSITION_GROUP
};

struct TaskPlan {
  uint8_t colors[2][3];
  uint8_t positions[2][3];

  TaskPlan();
  void clear();
};

TaskCodeStatus parseTaskCode(const char *code, TaskPlan &plan);

// Return the first-batch storage ring for a second-batch color, or 0 if absent.
uint8_t stackedRingForColor(
    const TaskPlan &plan, uint8_t secondBatchColor);

enum StartZone : uint8_t {
  START_ZONE_1 = 1U,
  START_ZONE_2 = 2U
};

struct Rectangle {
  float minimumX;
  float maximumX;
  float minimumY;
  float maximumY;

  Rectangle(
      float minX = 0.0f,
      float maxX = 0.0f,
      float minY = 0.0f,
      float maxY = 0.0f)
      : minimumX(minX),
        maximumX(maxX),
        minimumY(minY),
        maximumY(maxY) {}
};

Rectangle startZoneBounds(
    StartZone zone,
    float fieldSizeMm,
    float zoneSizeMm);

Rectangle axisAlignedFootprint(
    float centerX,
    float centerY,
    float footprintX,
    float footprintY,
    int16_t cardinalHeadingDegrees);

bool rectangleContains(
    const Rectangle &outer, const Rectangle &inner);

} // namespace competition

#endif
