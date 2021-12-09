#include <Arduino.h>
#include <esp_sleep.h>
#include "AiEsp32RotaryEncoder.h"
#include <SPI.h>

#include <TFT_eSPI.h> // Hardware-specific library

// WE WILL attach() THE BUTTON TO THE FOLLOWING PIN IN setup()
#define UP_BUTTON_PIN 35
#define DOWN_BUTTON_PIN 0

// Include the Bounce2 library found here :
// https://github.com/thomasfredericks/Bounce2
#include <Bounce2.h>

// INSTANTIATE A Button OBJECT FROM THE Bounce2 NAMESPACE
Bounce2::Button up_btn = Bounce2::Button();
Bounce2::Button down_btn = Bounce2::Button();

TFT_eSPI tft = TFT_eSPI(); // Invoke custom library

#define ROTARY_ENCODER_A_PIN 21
#define ROTARY_ENCODER_B_PIN 22
#define ROTARY_ENCODER_BUTTON_PIN 17
#define ROTARY_ENCODER_VCC_PIN 13

#define ROTARY_ENCODER_STEPS 4
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN, ROTARY_ENCODER_STEPS);

#define ADC_EN 14
#define ADC_PIN 34

int vref = 1100;

int first;

void IRAM_ATTR readEncoderISR()
{
  rotaryEncoder.readEncoder_ISR();
}

unsigned long longpress_time = 0;
unsigned long sleepreset_time = 0;
const unsigned long sleep_period = 180000;

RTC_DATA_ATTR long freq = 1000;
RTC_DATA_ATTR long duty = 50;
RTC_DATA_ATTR int mode = 0;
RTC_DATA_ATTR bool out = false;

const int ledChannel = 0;
const int ledPin = 12;
const int resolution = 10; //bit

void sleepreset()
{
  sleepreset_time = millis();
}

void deppsleep()
{
  Serial.println("Sleep.");
  longpress_time = 0;
  ledcDetachPin(ledPin);
  digitalWrite(TFT_BL, !TFT_BACKLIGHT_ON);
  digitalWrite(ROTARY_ENCODER_VCC_PIN, 0);
  digitalWrite(ADC_EN, 0);
  //esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);
  esp_sleep_enable_ext1_wakeup(GPIO_SEL_35, ESP_EXT1_WAKEUP_ALL_LOW);
  delay(1000);
  esp_deep_sleep_start();
}

void printFreq(long val, uint16_t color)
{
  long v = val;
  if (val == 0)
  {
    v = 1;
    freq = 1;
  }

  tft.setCursor(80, 35, 7);
  tft.fillRect(80, 35, tft.width() - 80, chr_hgt_f7s, TFT_BLACK);
  tft.setTextColor(color, TFT_BLACK);
  tft.printf("%ld", v);
}

void printDuty(long val, uint16_t color)
{
  tft.setCursor(80, 87, 7);
  tft.fillRect(80, 87, tft.width() - 80, chr_hgt_f7s, TFT_BLACK);
  tft.setTextColor(color, TFT_BLACK);
  tft.printf("%ld", val);
}

void setup(void)
{
  Serial.begin(115200);
  Serial.printf("Internal Total heap %d, internal Free Heap %d\n", ESP.getHeapSize(), ESP.getFreeHeap());
  Serial.printf("SPIRam Total heap %d, SPIRam Free Heap %d\n", ESP.getPsramSize(), ESP.getFreePsram());
  Serial.printf("ChipRevision %d, Cpu Freq %d MHz, SDK Version %s\n", ESP.getChipRevision(), ESP.getCpuFreqMHz(), ESP.getSdkVersion());
  Serial.printf("Flash Size %d, Flash Speed %d\n", ESP.getFlashChipSize(), ESP.getFlashChipSpeed());

  /*
  Serial.setDebugOutput(true);
  log_v("Verbose");
  log_d("Debug");
  log_i("Info");
  log_w("Warning"); 
  log_e("Error");
*/
  first = 1;

  up_btn.attach(UP_BUTTON_PIN, INPUT_PULLUP);     // USE EXTERNAL PULL-UP
  down_btn.attach(DOWN_BUTTON_PIN, INPUT_PULLUP); // USE EXTERNAL PULL-UP

  // DEBOUNCE INTERVAL IN MILLISECONDS
  up_btn.interval(50);
  down_btn.interval(50);

  // INDICATE THAT THE LOW STATE CORRESPONDS TO PHYSICALLY PRESSING THE BUTTON
  up_btn.setPressedState(LOW);
  down_btn.setPressedState(LOW);

  //we must initialize rotary encoder
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  //set boundaries and if values should cycle or not
  //in this example we will set possible values between 0 and 1000;
  rotaryEncoder.setBoundaries(0, 2000, false); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)

  /*Rotary acceleration introduced 25.2.2021.
   * in case range to select is huge, for example - select a value between 0 and 1000 and we want 785
   * without accelerateion you need long time to get to that number
   * Using acceleration, faster you turn, faster will the value raise.
   * For fine tuning slow down.
   */
  //rotaryEncoder.disableAcceleration(); //acceleration is now enabled by default - disable if you dont need it
  rotaryEncoder.setAcceleration(250); //or set the value - larger number = more accelearation; 0 or 1 means disabled acceleration

  tft.init();

  tft.setRotation(1);

  tft.fillScreen(TFT_BLACK);

  // Set "cursor" at top left corner of display (0,0) and select font 4
  tft.setCursor(10, 10, 4);

  // Set the font colour to be white with a black background
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  // We can now plot text on screen using the "print" class
  tft.print("OFF");

  tft.setCursor(00, 50, 4);
  tft.setTextColor(TFT_GREENYELLOW, TFT_BLACK);
  tft.print("Freq: ");

  tft.setCursor(00, 100, 4);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.print("Duty: ");

  //printFreq(freq, TFT_GREENYELLOW);
  //printDuty(duty, TFT_WHITE);
  /*
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars); //Check type of calibration value used to characterize ADC
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
  {
    Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
    vref = adc_chars.vref;
  }
  else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
  {
    Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
  }
  else
  {
    Serial.println("Default Vref: 1100mV");
  }

  Serial.println();
*/
  /*
    ADC_EN is the ADC detection enable port
    If the USB port is used for power supply, it is turned on by default.
    If it is powered by battery, it needs to be set to high level
    */
  pinMode(ADC_EN, OUTPUT);
  digitalWrite(ADC_EN, HIGH);
}

void pwm()
{
  ledcSetup(ledChannel, freq, resolution);
  ledcWrite(ledChannel, (1 << resolution) * duty / 100);
  ledcAttachPin(ledPin, ledChannel);
}

void showVoltage()
{
  static uint64_t timeStamp = 0;
  if (millis() - timeStamp > 1000)
  {
    timeStamp = millis();
    uint16_t v = analogRead(ADC_PIN);
    float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
    String voltage = "Voltage :" + String(battery_voltage) + "V";
    Serial.println(voltage);

    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(voltage, tft.width() / 2, tft.height() / 2);
  }
}

void loop()
{
  //Serial.print(".");

  up_btn.update();
  down_btn.update();

  if (up_btn.pressed())
  {
    Serial.println("B UP");
    //digitalWrite(TFT_BL, 1);
    out = true;
    tft.setCursor(10, 10, 4);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.print("ON   ");

    pwm();

    sleepreset();
  };

  if (down_btn.pressed())
  {
    Serial.println("B DOWN");
    //digitalWrite(TFT_BL, 0);
    out = false;
    tft.setCursor(10, 10, 4);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print("OFF");

    ledcDetachPin(ledPin);

    //showVoltage();
    longpress_time = 0;

    sleepreset();
  };

  if (down_btn.isPressed())
  {
    if (longpress_time == 0)
      longpress_time = millis();

    if (millis() - longpress_time > 2000)
    {
      deppsleep();
    }
  }

  if (rotaryEncoder.encoderChanged())
  {
    Serial.print("Value: ");
    Serial.println(rotaryEncoder.readEncoder());

    long val = rotaryEncoder.readEncoder();
    switch (mode)
    {
    case 0:
      /* частота */
      freq = val * 10;
      printFreq(freq, TFT_GREENYELLOW);
      if (out)
        pwm();
      break;
    case 1:
      /* частота */
      duty = val;
      printDuty(duty, TFT_GREEN);
      ledcWrite(ledChannel, (1 << resolution) * duty / 100);
      break;

    default:
      break;
    }

    sleepreset();
  }

  if (rotaryEncoder.isEncoderButtonClicked() || first)
  {
    if (!first)
    {
      mode = mode + 1;
      if (mode > 1)
        mode = 0;
    }

    switch (mode)
    {
    case 0:
      /* частота */
      rotaryEncoder.setBoundaries(0, 2000, false); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
      rotaryEncoder.setEncoderValue(freq / 10);
      rotaryEncoder.setAcceleration(250);
      printFreq(freq, TFT_GREENYELLOW);
      printDuty(duty, TFT_WHITE);
      break;
    case 1:
      /* заполнение */
      rotaryEncoder.setBoundaries(0, 100, false); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
      rotaryEncoder.setEncoderValue(duty);
      rotaryEncoder.setAcceleration(50);
      printFreq(freq, TFT_WHITE);
      printDuty(duty, TFT_GREEN);
      break;

    default:
      break;
    }
    sleepreset();
  }

  static unsigned long timeStamp = 0;
  if (millis() - timeStamp > 1000 || first)
  {
    timeStamp = millis();
    uint16_t v = analogRead(ADC_PIN);
    float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * 1.1;
    String voltage = " " + String(battery_voltage, 2) + "V ";

    uint32_t color = TFT_DARKGREEN;
    if (battery_voltage > 4.5)
    {
      color = TFT_DARKGREY;
    }
    if (battery_voltage < 3.6)
    {
      color = TFT_RED;
    }
    tft.setTextColor(TFT_WHITE, color);
    //tft.fillRect(140, 0, 20, 26, color);
    tft.drawString(voltage, 160, 3, 4);
    //tft.drawRect(160, 0, 240 - 160, 26, color);
  };

  if (millis() - sleepreset_time > sleep_period)
  {
    deppsleep();
  }

  first = 0;
  delay(50);
}
