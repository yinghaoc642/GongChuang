/*

 * STM32 Battery Voltage Monitor - Arduino Framework

  * Hardware connections:

 * - Battery+ -> Voltage Divider -> A0 (PA0)

 * - Serial output via USB or UART pins

  * Important: Use voltage divider if battery voltage > 3.3V

 * Example: 12V battery needs 11:1 divider (10k + 1k resistors)

 */

// Configuration

#include <Arduino.h>

#define BATTERY_PIN PC1           // Analog pin for battery voltage
#define VREF 3.3                  // Reference voltage (3.3V for STM32)
#define VOLTAGE_DIVIDER_RATIO 11  // Adjust based on your voltage divider
#define BATTERY_LOW_THRESHOLD 3.2 // Low battery warning voltage
#define SAMPLE_COUNT 10           // Number of samples for averaging
#define READ_INTERVAL 1000        // Reading interval in milliseconds

/*UART debug*/

#define DEBUG_BAUDRATE 115200
#define DEBUG_RX PB12
#define DEBUG_TX PB13

#define TJCHMI_BAUDRATE 115200
#define TJCHMI_RX PB15
#define TJCHMI_TX PB14

HardwareSerial Serial_DEBUG(DEBUG_RX, DEBUG_TX);

//                              RX          TX
HardwareSerial Serial_TJCHMI(TJCHMI_RX, TJCHMI_TX);
// Variables
float batteryVoltage = 0.0;
float batteryPercentage = 0.0;



float readBatteryVoltage();
float readDirectVoltage();
float getBatteryPercentage(float voltage);
void displayBatteryInfo(float voltage, float percentage);
float getBatteryPercentageLeadAcid(float voltage);
void calibrateVoltage();
void printJsonData();
float readDirectVoltage();



void setup()

{
  // Initialize serial communication
  Serial_DEBUG.begin(115200);
  while (!Serial_DEBUG)
  {

    ; // Wait for serial port to connect (needed for some boards)

  }

  Serial_TJCHMI.begin(TJCHMI_BAUDRATE);
  delay(300);
  // Configure ADC resolution (12-bit for STM32)
  analogReadResolution(12);
  // Optional: Set ADC reference (if your board supports it)
  // analogReference(AR_DEFAULT);

  Serial_DEBUG.println("=== STM32 Battery Voltage Monitor ===");

  Serial_DEBUG.println("Starting battery monitoring...");

  Serial_DEBUG.println("Voltage | Percentage | Status");

  Serial_DEBUG.println("--------|------------|--------");

  while (Serial_TJCHMI.read() >= 0)

    ;                                           // 清空串口缓冲区

  Serial_TJCHMI.print("page main\xff\xff\xff"); // 发送命令让屏幕跳转到main页面

  delay(1000);

}



void loop()

{

  // Read battery voltage

  batteryVoltage = readBatteryVoltage();



  // Calculate battery percentage

  batteryPercentage = getBatteryPercentage(batteryVoltage);



  // Display results

  displayBatteryInfo(batteryVoltage, batteryPercentage);



  // Check for low battery

  if (batteryVoltage < BATTERY_LOW_THRESHOLD)

  {

    Serial_DEBUG.println("⚠  LOW BATTERY WARNING! ⚠");

  }

      //刷新屏幕显示

    char strTemp[20];

    sprintf(strTemp, "x1.val=%d\xff\xff\xff", (int)(batteryVoltage*100));

    //把字符串发送出去

    Serial_TJCHMI.print(strTemp);



  // Wait before next reading

  delay(READ_INTERVAL);

}



float readBatteryVoltage()

{

  float totalVoltage = 0.0;



  // Take multiple samples for better accuracy

  for (int i = 0; i < SAMPLE_COUNT; i++)

  {

    int adcValue = analogRead(BATTERY_PIN);



    // Convert ADC value to voltage

    float voltage = (adcValue * VREF) / 4096.0; // 12-bit ADC = 4096 levels



    // Apply voltage divider ratio

    voltage *= VOLTAGE_DIVIDER_RATIO;



    totalVoltage += voltage;

    delay(10); // Small delay between samples

  }



  return totalVoltage / SAMPLE_COUNT;

}



float getBatteryPercentage(float voltage)

{

  // Battery voltage curve (adjust these values for your battery type)



  // Li-ion battery curve (3.0V - 4.2V)

  float minVoltage = 3.0;

  float maxVoltage = 4.2;



  // For other battery types, adjust these values:

  // Lead-acid: 10.5V - 12.6V (for 12V battery)

  // NiMH: 0.9V - 1.4V (per cell)

  // Alkaline: 0.8V - 1.6V (per cell)



  if (voltage >= maxVoltage)

    return 100.0;

  if (voltage <= minVoltage)

    return 0.0;



  return ((voltage - minVoltage) / (maxVoltage - minVoltage)) * 100.0;

}



void displayBatteryInfo(float voltage, float percentage)

{

  // Format: "12.34V  |    85.2%   | Good"

  Serial_DEBUG.print(voltage, 2);

  Serial_DEBUG.print("V");



  // Add spacing for alignment

  if (voltage < 10.0)

    Serial_DEBUG.print(" ");



  Serial_DEBUG.print("  |  ");

  Serial_DEBUG.print(percentage, 1);

  Serial_DEBUG.print("%");



  // Add spacing for percentage alignment

  if (percentage < 100.0)

    Serial_DEBUG.print(" ");

  if (percentage < 10.0)

    Serial_DEBUG.print(" ");



  Serial_DEBUG.print("   | ");



  // Battery status

  if (percentage > 75)

  {

    Serial_DEBUG.println("Excellent");

  }

  else if (percentage > 50)

  {

    Serial_DEBUG.println("Good");

  }

  else if (percentage > 25)

  {

    Serial_DEBUG.println("Fair");

  }

  else if (percentage > 10)

  {

    Serial_DEBUG.println("Low");

  }

  else

  {

    Serial_DEBUG.println("Critical");

  }

}



// Alternative function for different battery chemistry

float getBatteryPercentageLeadAcid(float voltage)

{

  // 12V Lead-acid battery curve

  float minVoltage = 10.5; // Fully discharged

  float maxVoltage = 12.6; // Fully charged



  if (voltage >= maxVoltage)

    return 100.0;

  if (voltage <= minVoltage)

    return 0.0;



  return ((voltage - minVoltage) / (maxVoltage - minVoltage)) * 100.0;

}



// Function to calibrate voltage reading

void calibrateVoltage()

{

  Serial_DEBUG.println("=== Voltage Calibration Mode ===");

  Serial_DEBUG.println("Measure actual battery voltage with multimeter");

  Serial_DEBUG.println("Compare with readings below:");



  for (int i = 0; i < 10; i++)

  {

    float voltage = readBatteryVoltage();

    Serial_DEBUG.print("Reading ");

    Serial_DEBUG.print(i + 1);

    Serial_DEBUG.print(": ");

    Serial_DEBUG.print(voltage, 3);

    Serial_DEBUG.println("V");

    delay(1000);

  }



  Serial_DEBUG.println("Adjust VOLTAGE_DIVIDER_RATIO if readings don't match multimeter");

  Serial_DEBUG.println("=== End Calibration ===");

}



// Function for advanced monitoring with JSON output

void printJsonData()

{

  Serial_DEBUG.print("{");

  Serial_DEBUG.print("\"voltage\":");

  Serial_DEBUG.print(batteryVoltage, 2);

  Serial_DEBUG.print(",\"percentage\":");

  Serial_DEBUG.print(batteryPercentage, 1);

  Serial_DEBUG.print(",\"timestamp\":");

  Serial_DEBUG.print(millis());

  Serial_DEBUG.print(",\"status\":\"");



  if (batteryVoltage < BATTERY_LOW_THRESHOLD)

  {

    Serial_DEBUG.print("low");

  }

  else if (batteryPercentage > 75)

  {

    Serial_DEBUG.print("good");

  }

  else

  {

    Serial_DEBUG.print("fair");

  }



  Serial_DEBUG.println("\"}");

}



// Function to get voltage without divider (direct 3.3V max reading)

float readDirectVoltage()

{

  int adcValue = analogRead(BATTERY_PIN);

  return (adcValue * VREF) / 4096.0;

}