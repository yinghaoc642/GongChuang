#include <CompetitionCore.h>

#include <assert.h>

using namespace competition;

namespace {

const uint8_t kPermutations[6][3] = {
    {1U, 2U, 3U},
    {1U, 3U, 2U},
    {2U, 1U, 3U},
    {2U, 3U, 1U},
    {3U, 1U, 2U},
    {3U, 2U, 1U}};

void buildCode(
    const uint8_t firstColors[3],
    const uint8_t firstPositions[3],
    const uint8_t secondColors[3],
    const uint8_t secondPositions[3],
    char code[16]) {
  for (uint8_t item = 0U; item < 3U; ++item) {
    code[item] = static_cast<char>('0' + firstColors[item]);
    code[4U + item] =
        static_cast<char>('0' + firstPositions[item]);
    code[8U + item] =
        static_cast<char>('0' + secondColors[item]);
    code[12U + item] =
        static_cast<char>('0' + secondPositions[item]);
  }
  code[3] = '+';
  code[7] = '+';
  code[11] = '+';
  code[15] = '\0';
}

void testAllLegalTaskCodes() {
  uint32_t tested = 0U;
  for (uint8_t excludedColor = 1U;
       excludedColor <= 4U;
       ++excludedColor) {
    uint8_t colorSet[3] = {0U, 0U, 0U};
    uint8_t index = 0U;
    for (uint8_t color = 1U; color <= 4U; ++color) {
      if (color != excludedColor) {
        colorSet[index++] = color;
      }
    }

    for (uint8_t firstColorOrder = 0U;
         firstColorOrder < 6U;
         ++firstColorOrder) {
      uint8_t firstColors[3];
      for (uint8_t item = 0U; item < 3U; ++item) {
        firstColors[item] =
            colorSet[kPermutations[firstColorOrder][item] - 1U];
      }

      for (uint8_t secondColorOrder = 0U;
           secondColorOrder < 6U;
           ++secondColorOrder) {
        uint8_t secondColors[3];
        for (uint8_t item = 0U; item < 3U; ++item) {
          secondColors[item] =
              colorSet[
                  kPermutations[secondColorOrder][item] - 1U];
        }

        for (uint8_t firstPositionOrder = 0U;
             firstPositionOrder < 6U;
             ++firstPositionOrder) {
          for (uint8_t secondPositionOrder = 0U;
               secondPositionOrder < 6U;
               ++secondPositionOrder) {
            char code[16];
            buildCode(
                firstColors,
                kPermutations[firstPositionOrder],
                secondColors,
                kPermutations[secondPositionOrder],
                code);
            TaskPlan plan;
            assert(parseTaskCode(code, plan) == TASK_CODE_OK);
            for (uint8_t item = 0U; item < 3U; ++item) {
              const uint8_t color = plan.colors[1][item];
              uint8_t expectedRing = 0U;
              for (uint8_t firstItem = 0U;
                   firstItem < 3U;
                   ++firstItem) {
                if (plan.colors[0][firstItem] == color) {
                  expectedRing =
                      plan.positions[0][firstItem];
                }
              }
              assert(
                  stackedRingForColor(plan, color) ==
                  expectedRing);
            }
            ++tested;
          }
        }
      }
    }
  }
  assert(tested == 5184U);
}

void testInvalidTaskCodes() {
  TaskPlan plan;
  assert(parseTaskCode(nullptr, plan) == TASK_CODE_NULL);
  assert(
      parseTaskCode("134+123+314+23", plan) ==
      TASK_CODE_WRONG_LENGTH);
  assert(
      parseTaskCode("134-123+314+231", plan) ==
      TASK_CODE_WRONG_SEPARATOR);
  assert(
      parseTaskCode("114+123+141+231", plan) ==
      TASK_CODE_INVALID_COLOR_GROUP);
  assert(
      parseTaskCode("134+123+124+231", plan) ==
      TASK_CODE_COLOR_SET_MISMATCH);
  assert(
      parseTaskCode("134+113+314+231", plan) ==
      TASK_CODE_INVALID_POSITION_GROUP);
}

void testStartZoneFootprints() {
  const Rectangle zone1 =
      startZoneBounds(START_ZONE_1, 2400.0f, 300.0f);
  const Rectangle zone2 =
      startZoneBounds(START_ZONE_2, 2400.0f, 300.0f);
  const Rectangle final1 = axisAlignedFootprint(
      2250.0f, 2250.0f, 230.0f, 300.0f, 180);
  const Rectangle final2 = axisAlignedFootprint(
      2250.0f, 150.0f, 230.0f, 300.0f, 180);
  assert(rectangleContains(zone1, final1));
  assert(rectangleContains(zone2, final2));

  const Rectangle oldFinal = axisAlignedFootprint(
      2255.0f, 2210.0f, 230.0f, 300.0f, 180);
  assert(!rectangleContains(zone1, oldFinal));
}

} // namespace

int main() {
  testAllLegalTaskCodes();
  testInvalidTaskCodes();
  testStartZoneFootprints();
  return 0;
}
