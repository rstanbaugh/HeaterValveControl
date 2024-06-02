#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Function definitions
bool i2CAddrTest(uint8_t addr);

// Pin Assignments
#define VGG_1 A0            // measuring Vgg from dimmer 1
#define VGG_2 A2            // measuring Vgg from dimmer 2
#define HEATER_MASTER 7     // digital pin to turn master relay on

// LCD Pins
// SDA (Serial Data Access) pin is A4
// SCL (Serial Clock) pin is A5

// Program controls / limits
#define MEASURMENT_INTERVAL 1000
#define VREF 4.1          // set by LM4040 chip
#define MAX_TEMP 160      // temperature shutoff threshold

#define ONE_WIRE_BUS 5    // Temperature data wire is connected to the Arduino digital pin 5

// Variables
int PWM_out_pin_1 = 3;    // assign D3 pin for PWM out signal; 490 Hz
int PWM_out_pin_2 = 9;    // assign D4 pin for PWM out signal; 490 Hz
int vgg_adc_1, vgg_adc_2;
float vgg_1, vgg_2;
byte pwm_out_level_1, pwm_out_level_2;
float temp_1;
String status = "NORMAL";


int intDimmerLevel_1, intDimmerLevel_2;   // levels of dimmers #1 & #2
String previousLine1 = "";                // LCD Previous line #1
String previousLine2 = "";                // LCD Previous Line #2

// Initialize the LCD with the numbers of the interface pins
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

// Function to pad strings to a fixed length
String padString(String str, int length) {
  while ((int)str.length() < length) {
    str += " ";
  }
  return str;
}

// Function to map dimmer output to PWM values
int dimmerToPWM(float vDimmer, int &intDimmerLevel) {
  // Dimmer output goes through 1/3 voltage divider, so output is 0 - 10 VDC ÷ 3
  // Dimmer voltages based on experimental measurement of dimmer output
  // Serial.print("vDimmer: ");
  // Serial.println(vDimmer);
  if (vDimmer > 3.25) {        // 100% heat - 7 LEDs lit
    intDimmerLevel = 7;
    return 255;
  }
  else if (vDimmer > 2.80) {   // 85% heat - 6 LEDs lit
    intDimmerLevel = 6;
    return 217;
  }
  else if (vDimmer > 2.30) {   // 70% heat - 5 LEDs lit
    intDimmerLevel = 5;
    return 179;
  }
  else if (vDimmer > 1.75) {   // 55% heat - 4 LEDs lit
    intDimmerLevel = 4;
    return 140;
  }
  else if (vDimmer > 1.35) {   // 40% heat - 3 LEDs lit
    intDimmerLevel = 3;
    return 102;
  }
  else if (vDimmer > 0.9) {    // 25% heat - 2 LEDs lit
    intDimmerLevel = 2;
    return 64;
  }
  else if (vDimmer > 0.15){
    intDimmerLevel = 1;         // Minimum heat - 1 LED lit
    return 0;
  }
  else{
    intDimmerLevel = 0;
    return 0;
  }
}

void setup() {
  // Serial setup
  Serial.begin(9600);
  while (!Serial);  // Wait for Serial monitor to open
  Serial.println("Serial 9600 baud running...");

  Wire.begin();

  // Start up temperature sensors
  sensors.begin();

  // PWM setup
  pinMode(PWM_out_pin_1, OUTPUT);    // Setup output for pin = PWM_1
  pinMode(PWM_out_pin_2, OUTPUT);    // Setup output for pin = PWM_2

  // LCD setup
  if (!i2CAddrTest(0x27)) {
    lcd = LiquidCrystal_I2C(0x3F, 16, 2); // Attempt to initialize LCD with a different address if 0x27 fails
  }

  lcd.init();              // Initialize the LCD
  lcd.backlight();         // Turn on backlight
  lcd.clear();             // Clear the screen
  Serial.println("LCD 1602 initialized...");

  // Initialize the MASTER HEATER RELAY control pin as an output
  pinMode(HEATER_MASTER, OUTPUT);
  
  // Set the MASTER HEATER RELAY control pin high to turn on the relay
  digitalWrite(HEATER_MASTER, HIGH);
  
  // Print a message to the serial monitor for confirmation
  Serial.println("Heater master relay turned on");
}

unsigned long previousMillis = 0;  // Will store last time LCD was updated

void loop() {
  unsigned long currentMillis = millis();   // Time tracking

  // Measurement clock
  if (currentMillis - previousMillis >= MEASURMENT_INTERVAL) {
    previousMillis = currentMillis;  // Save the last time you updated the display

    // Read analog values from ADC on A0 and A2
    vgg_adc_1 = analogRead(VGG_1);  // Read value from Dimmer #1 (0-10 VDC across 1/3 voltage divider)
    vgg_adc_2 = analogRead(VGG_2);  // Read value from Dimmer #2 (0-10 VDC across 1/3 voltage divider)

    // Convert ADC readings to voltages
    vgg_1 = map(vgg_adc_1, 0, 1023, 0, VREF * 1000) / 1000.0; // Convert 0-1023 to 0-5.0V
    vgg_2 = map(vgg_adc_2, 0, 1023, 0, VREF * 1000) / 1000.0; // Convert 0-1023 to 0-5.0V

    // Map Dimmer #1 & #2 to PWM to control gate voltage
    pwm_out_level_1 = dimmerToPWM(vgg_1, intDimmerLevel_1);
    pwm_out_level_2 = dimmerToPWM(vgg_2, intDimmerLevel_2);

    // Write to PWM pins
    analogWrite(PWM_out_pin_1, pwm_out_level_1);  // Signal to control Heater #1 (gate of MOSFET #1)
    analogWrite(PWM_out_pin_2, pwm_out_level_2);  // Signal to control Heater #2 (gate of MOSFET #2)

    // Call sensors.requestTemperatures() to issue a global temperature and requests to all devices on the bus
    sensors.requestTemperatures(); 
    // "byIndex": You can have more than one IC on the same bus. 0 refers to the first IC on the wire
    temp_1 = sensors.getTempFByIndex(0);

    // Check if the temperature exceeds the maximum limit
    temp_1= 100.0;
    if (temp_1 > MAX_TEMP) {
      digitalWrite(HEATER_MASTER, LOW);  // Turn off the master relay
      Serial.println("Temperature exceeded maximum limit! Heaters turned off.");
      status = "FAULT";
    }

    // Calculate percentages
    int pwm_percent_1 = round(pwm_out_level_1 * 100.0 / 255);
    int pwm_percent_2 = round(pwm_out_level_2 * 100.0 / 255);

    // Build strings for each line with padding
    char line1[17], line2[17];
    if (status=="NORMAL"){
      if (intDimmerLevel_1 == 0 && intDimmerLevel_2 == 0) {
        sprintf(line1, "2:OFF    1:OFF");
      } 
      else if (intDimmerLevel_1 == 0) {
        sprintf(line1, "2:%1d/%-3d  1:OFF   ", intDimmerLevel_2, pwm_percent_2);
      } 
      else if (intDimmerLevel_2 == 0) {
        sprintf(line1, "2:OFF    1:%1d/%-3d", intDimmerLevel_1, pwm_percent_1);
      } 
      else {
        sprintf(line1, "2:%1d/%-3d  1:%1d/%-3d", intDimmerLevel_2, pwm_percent_2, intDimmerLevel_1, pwm_percent_1);
      }
    }
    else{
      sprintf(line1, "2:OFF      1:OFF");
    }   
    sprintf(line2, "Temp:%-3.0d  %s", (int) temp_1, status.c_str());

    // Update the LCD only if the lines have changed
    if (String(line1) != previousLine1) {
      lcd.setCursor(0, 0);
      lcd.print(line1);
      previousLine1 = line1;
    }
    if (String(line2) != previousLine2) {
      lcd.setCursor(0, 1);
      lcd.print(line2);
      previousLine2 = line2;
    }

    // Repeat the LCD prints to the Serial port
    Serial.print("Vgg 1: ");
    Serial.print(vgg_1, 1);
    Serial.println(" V, ");
    Serial.print("Temp: ");
    Serial.print(temp_1, 1);
    Serial.println(" F");
    Serial.print("PWM: ");
    Serial.println(pwm_out_level_1);
  }
}

bool i2CAddrTest(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}
