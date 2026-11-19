#pragma once

#include <Arduino.h>

namespace gongchuang {

// STM32 Arduino 3.x uses Uart for concrete RX/TX serial instances;
// HardwareSerial remains the abstract Arduino interface in that core.
#if defined(STM32_CORE_VERSION_MAJOR) && STM32_CORE_VERSION_MAJOR >= 3
using Stm32HardwareSerial = ::Uart;
#else
using Stm32HardwareSerial = ::HardwareSerial;
#endif

} // namespace gongchuang
