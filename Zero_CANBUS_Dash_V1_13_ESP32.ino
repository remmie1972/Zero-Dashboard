/*  BNice example of receiving CAN messages from an Zero Mototrcycle using an ESP32 Feather by adafruit and a 3.5 inch TFT display
    hardware :
    ESP32 Feather (HUZZAH32) by adafruit : https://www.adafruit.com/product/3405
    3.5inch TFT Featherwing display by adafruit : https://www.adafruit.com/product/3651
    CANBUS Featherwing by SKPANG.co.uk : http://skpang.co.uk/catalog/canbus-featherwing-for-esp32-p-1556.html

    libraries required from github : https://github.com/nhatuan84/arduino-esp32-can-demo

    based on the example : https://github.com/skpang/esp32can_demo/blob/master/esp32can_demo.ino (mainly for the GPIO numbers for the Huzzah ESP 32 Feather)
    and example : https://github.com/nhatuan84/arduino-esp32-can-demo/tree/master/examples/esp32cansend
*/

bool demoMode = true;
int debug = 0;   // 2 is canid screen with all the small values of all canid's

#include <ESP32CAN.h>
#include <CAN_config.h>
#include <TFT_eSPI.h>
#include "ZeroLogo.h"
#include "Free_Fonts.h"

#define DISPLAYINTERVAL 100
#define TFT_DC 33                                                                   // Connect DC of display to pin 33. By using a feather and feathewing, all required connections are already made through the headers
#define TFT_CS 15                                                                   // Connect CS of display to pin 15

// Color definitions for TFT ------------------------------------
#define BLACK   0x0000
#define WHITE   0xFFFF
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define GREY    0x7BEF

#define TFT_HEIGHT 320
#define TFT_WIDTH 480
#define DEG2RAD 0.0174532925
#define FONT &FreeSans9pt7b
#define FSS9 &FreeSans9pt7b
#define GFXFF 1
#define GLCD  0

unsigned char canID181[8] = {0};
unsigned char canID188[8] = {0};
unsigned char canID192[8] = {0};
unsigned char canID206[8] = {0};
unsigned char canID240[8] = {0};
unsigned char canID281[8] = {0};
unsigned char canID288[8] = {0};
unsigned char canID292[8] = {0};
unsigned char canID306[8] = {0};
unsigned char canID308[8] = {0};
unsigned char canID340[8] = {0};
unsigned char canID381[8] = {0};
unsigned char canID388[8] = {0};
unsigned char canID406[8] = {0};
unsigned char canID408[8] = {0};
unsigned char canID440[8] = {0};
unsigned char canID481[8] = {0};
unsigned char canID488[8] = {0};
unsigned char canID501[8] = {0};
unsigned char canID506[8] = {0};
unsigned char canID508[8] = {0};
unsigned char canID540[8] = {0};
unsigned char canID588[8] = {0};
unsigned char canID608[8] = {0};
unsigned char canID701[8] = {0};
unsigned char canID1C0[8] = {0};
unsigned char canID2C0[8] = {0};
unsigned char canID3C0[8] = {0};

unsigned char len = 0;
int led = 13;
unsigned long previousDisplayMillis = 0;
unsigned long ZeroToHundredStart = 0;
unsigned long ZeroToHundredStop = 0;
bool ZeroToHundredBusy = 1;
bool Charging = false;
unsigned long ZeroToHundredTime = 0;
unsigned long NumberOfCanMessageSets = 0;
unsigned long ChargerVoltage = 0;
byte Page = -1;
float Speed = 1;
float previousSpeed = 0;
float Range = 0;
float SurplusRange = 0;
float TargetRange = 150;
float Trip1 = 0;
float Trip2 = 0;
float Vbatt = 0;
float StationaryVolt = 0;
float InternalResistance = 0.02;
float ODO = 0;
float StartODO = 0;
float AmpHours = 0;
float demoValue = 0;
float TripWhPerKm = 0;

int RPM = 0;
int displayUpdates = 4;
int ChargeMinutesToGo = 0;
int MotorRPM = 0;
int MotorTemp = 0;
int ErrorCode = 0;
int ChargeCycles = 0;
int CellBalance = 0;
int ThrottlePosition = 0;
int innerRingColor = 0;
int innerRingDarkColor = 0;
int outerRingColor = 0;
int padding = 0;
byte PowerBar = 0;
byte TorqueBar = 0;
int SOC = 0;
byte SOC2 = 0;
byte BatteryTemp = 0;
byte BatteryTemp2 = 0;
byte DashTimeHours = 0;
byte DashTimeMinutes = 0;
byte DashTimeSeconds = 0;
byte Hours = 0;
byte Minutes = 0;
int BatteryAmps = 0;
byte ControllerTemp = 0;
byte MotorTemperature = 0;
int Power = 0;
String Mode = "Custom";
String previousMode = "";
String ErrorText = "";
unsigned char canIDByte[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int upDown = 1;
CAN_frame_t rx_frame;

CAN_device_t CAN_cfg;
//Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC);                               // Set the pins for the TFT display
TFT_eSPI tft = TFT_eSPI();                   // Invoke custom library with default width and height

void setup() {
	Serial.begin(115200);
	pinMode(led, OUTPUT);
	digitalWrite(led, HIGH);
	delay(100);
	digitalWrite(led, LOW);
	CAN_cfg.speed = CAN_SPEED_500KBPS;
	CAN_cfg.tx_pin_id = GPIO_NUM_25;
	CAN_cfg.rx_pin_id = GPIO_NUM_26;
	CAN_cfg.rx_queue = xQueueCreate(10, sizeof(CAN_frame_t));
	//start CAN Module
	ESP32Can.CANInit();
	delay(100);
	tft.begin();
	tft.fillScreen(BLACK);
	tft.setRotation(1);
	//tft.setFont();
	tft.setTextSize(1);                                                             // Set the fontsize (see library for more fonts)
	tft.setTextWrap(0);                                                             // Set the text wrap of the TFT (0 = no text wrap)
	tft.setTextWrap(false, false);
	tft.setTextDatum(MC_DATUM);                     // text alignment middle center
	tft.setSwapBytes(true);

	tft.pushImage(240-200/2,160-182/2, 200, 182, ZeroLogo);
	delay(2000);

	tft.fillScreen(BLACK);
	tft.drawLine(TFT_WIDTH * 1.5 / 20, TFT_HEIGHT / 2, TFT_WIDTH * 4.25 / 20, TFT_HEIGHT / 2, WHITE);
	tft.drawLine(TFT_WIDTH * 1.5 / 20, 1 + TFT_HEIGHT / 2, TFT_WIDTH * 4.25 / 20, 1 + TFT_HEIGHT / 2, WHITE);
	tft.drawLine(TFT_WIDTH * (1 - 4.25 / 20), TFT_HEIGHT / 2, TFT_WIDTH * (1 - 1.5 / 20), TFT_HEIGHT / 2, WHITE);
	tft.drawLine(TFT_WIDTH * (1 - 4.25 / 20), 1 + TFT_HEIGHT / 2, TFT_WIDTH * (1 - 1.5 / 20), 1 + TFT_HEIGHT / 2, WHITE);
	// fillArc(xpos, ypos, start angle, nr of segments / degrees, radiusx, radiusy, width, color);

	fillArc(TFT_WIDTH / 2, TFT_HEIGHT / 2, 120, 150, TFT_WIDTH * 11.5 / 40, TFT_WIDTH * 11.5 / 40, 1, ((255 >> 3) << 11) | ((255 >> 2) << 5) | (255 >> 3)); // bottom half circle
	fillArc(TFT_WIDTH / 2, TFT_HEIGHT / 2, -60, 150, TFT_WIDTH * 11.5 / 40, TFT_WIDTH * 11.5 / 40, 1, ((255 >> 3) << 11) | ((255 >> 2) << 5) | (255 >> 3)); // top half circle
	tft.drawLine(240 + 138 * sin(34 * DEG2RAD), 160 - 138 * cos(34 * DEG2RAD), 240 + 150 * sin(34 * DEG2RAD), 160 - 150 * cos(34 * DEG2RAD), WHITE); // 100 kph marker

	tft.drawLine(TFT_WIDTH / 2 + sin(59 * DEG2RAD)*TFT_WIDTH * (171 + 10) / 480, TFT_HEIGHT / 2 - cos((59)*DEG2RAD)*TFT_WIDTH * 171 / 480, TFT_WIDTH / 2 + sin(59 * DEG2RAD)*TFT_WIDTH * (171 + 20) / 480, TFT_HEIGHT / 2 - cos((59)*DEG2RAD)*TFT_WIDTH * 171 / 480, WHITE);
	fillArc(4 + TFT_WIDTH / 2 + sin(59 * DEG2RAD)*TFT_WIDTH * (171 + 5) / 480, 5 + TFT_HEIGHT / 2 - cos((59)*DEG2RAD)*TFT_WIDTH * 171 / 480, -31 - 90, 31 + 90, 5, 5, 0.5, WHITE);
	fillArc(TFT_WIDTH / 2, TFT_HEIGHT / 2, 62, 19, TFT_WIDTH * 171 / 480, TFT_WIDTH * 171 / 480, 0.5, WHITE);
	tft.drawLine(TFT_WIDTH / 2 + sin((59 + 22)*DEG2RAD)*TFT_WIDTH * 171 / 480, TFT_HEIGHT / 2 - cos((59 + 22)*DEG2RAD)*TFT_WIDTH * 171 / 480, TFT_WIDTH / 2 + sin((59 + 22)*DEG2RAD)*TFT_WIDTH * (171 + 20) / 480, TFT_HEIGHT / 2 - cos((59 + 22)*DEG2RAD)*TFT_WIDTH * 171 / 480, WHITE);

	tft.drawLine(TFT_WIDTH / 2 + sin(99 * DEG2RAD)*TFT_WIDTH * 171 / 480, TFT_HEIGHT / 2 - cos((99)*DEG2RAD)*TFT_WIDTH * 171 / 480, TFT_WIDTH / 2 + sin(99 * DEG2RAD)*TFT_WIDTH * (171 + 20) / 480, TFT_HEIGHT / 2 - cos((99)*DEG2RAD)*TFT_WIDTH * 171 / 480, WHITE);
	fillArc(TFT_WIDTH / 2, TFT_HEIGHT / 2, 99, 22, TFT_WIDTH * 171 / 480, TFT_WIDTH * 171 / 480, 0.5, WHITE);
	tft.drawLine(TFT_WIDTH / 2 + sin((99 + 22)*DEG2RAD)*TFT_WIDTH * 171 / 480, TFT_HEIGHT / 2 - cos((99 + 22)*DEG2RAD)*TFT_WIDTH * 171 / 480, TFT_WIDTH / 2 + sin((99 + 22)*DEG2RAD)*TFT_WIDTH * (171 + 20) / 480, TFT_HEIGHT / 2 - cos((99 + 22)*DEG2RAD)*TFT_WIDTH * 171 / 480, WHITE);

	tft.drawLine(TFT_WIDTH / 2 + sin((-99 - 22)*DEG2RAD)*TFT_WIDTH * 171 / 480, TFT_HEIGHT / 2 - cos((-99 - 22)*DEG2RAD)*TFT_WIDTH * 171 / 480, TFT_WIDTH / 2 + sin((-99 - 22)*DEG2RAD)*TFT_WIDTH * (171 + 20) / 480, TFT_HEIGHT / 2 - cos((-99 - 22)*DEG2RAD)*TFT_WIDTH * 171 / 480, WHITE);
	fillArc(TFT_WIDTH / 2, TFT_HEIGHT / 2, -99 - 22, 22, TFT_WIDTH * 171 / 480, TFT_WIDTH * 171 / 480, 0.5, WHITE);
	tft.drawLine(TFT_WIDTH / 2 + sin((-99)*DEG2RAD)*TFT_WIDTH * 171 / 480, TFT_HEIGHT / 2 - cos((-99)*DEG2RAD)*TFT_WIDTH * 171 / 480, TFT_WIDTH / 2 + sin((-99)*DEG2RAD)*TFT_WIDTH * (171 + 20) / 480, TFT_HEIGHT / 2 - cos((-99)*DEG2RAD)*TFT_WIDTH * 171 / 480, WHITE);

	tft.drawLine(TFT_WIDTH / 2 + sin((-59 - 22)*DEG2RAD)*TFT_WIDTH * 171 / 480, TFT_HEIGHT / 2 - cos((-59 - 22)*DEG2RAD)*TFT_WIDTH * 171 / 480, TFT_WIDTH / 2 + sin((-59 - 22)*DEG2RAD)*TFT_WIDTH * (171 + 20) / 480, TFT_HEIGHT / 2 - cos((-59 - 22)*DEG2RAD)*TFT_WIDTH * 171 / 480, WHITE);
	fillArc(TFT_WIDTH / 2, TFT_HEIGHT / 2, -59 - 22, 22, TFT_WIDTH * 171 / 480, TFT_WIDTH * 171 / 480, 0.5, WHITE);
	tft.drawLine(TFT_WIDTH / 2 + sin(-59 * DEG2RAD)*TFT_WIDTH * 171 / 480, TFT_HEIGHT / 2 - cos((-59)*DEG2RAD)*TFT_WIDTH * 171 / 480, TFT_WIDTH / 2 + sin(-59 * DEG2RAD)*TFT_WIDTH * (171 + 20) / 480, TFT_HEIGHT / 2 - cos((-59)*DEG2RAD)*TFT_WIDTH * 171 / 480, WHITE);

}

void loop()
{
	if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {
		processCanMessage();
		demoMode = false;
	}
	if (millis() > (previousDisplayMillis + DISPLAYINTERVAL))
	{
		previousDisplayMillis = millis();
		if (debug ==2)
			showCanIDPage();
		else showZeroSRFDash();

	}
	CalcZeroToHundred();
}


void processCanMessage()
{
	unsigned char buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	digitalWrite(led, HIGH);
	unsigned long canId = rx_frame.MsgID;
	len = rx_frame.FIR.B.DLC;
	buf[0] = rx_frame.data.u8[0];
	buf[1] = rx_frame.data.u8[1];
	buf[2] = rx_frame.data.u8[2];
	buf[3] = rx_frame.data.u8[3];
	buf[4] = rx_frame.data.u8[4];
	buf[5] = rx_frame.data.u8[5];
	buf[6] = rx_frame.data.u8[6];
	buf[7] = rx_frame.data.u8[7];

	if (debug == true)
	{
		Serial.print(millis() * 0.001);
		Serial.print("\t");
		Serial.print("0");
		Serial.print(canId, HEX);
		Serial.print("\t");
		for (int i = 0; i < len; i++)
		{
			if ( buf[i] < 100)   Serial.print("0");
			if ( buf[i] < 10)    Serial.print("0");
			Serial.print(buf[i], DEC);
			Serial.print(" ");
		}
		Serial.println();
	}
	digitalWrite(led, LOW);


	if (debug == true)
	{
		if (canId == 0x0000)                                                                    // set the canID which to decode on page 4 and 5
		{
			for (int i = 0; i < 8; i++)
			{
				canIDByte[i] = buf[i];
			}
		}
		if (canId != 0x000)                                                                    // set the canID which to decode op the serial monitor with single, double and triple bytes
		{
			for (int i = 0; i < 8; i++)
			{
				if (buf[i] < 100)   Serial.print(" ");
				if (buf[i] < 10)    Serial.print(" ");
				Serial.print(buf[i]);
				Serial.print(" ");
			}

			Serial.print("\t\t");
			for (int i = 0; i < 7; i++)
			{
				if (((0.01 * buf[i] + (2.56 * buf[i + 1])) * 100) < 10000)   Serial.print (" ");
				if (((0.01 * buf[i] + (2.56 * buf[i + 1])) * 100) < 1000)     Serial.print (" ");
				if (((0.01 * buf[i] + (2.56 * buf[i + 1])) * 100) < 100)      Serial.print (" ");
				if (((0.01 * buf[i] + (2.56 * buf[i + 1])) * 100) < 10)       Serial.print (" ");
				Serial.print((0.01 * buf[i] + (2.56 * buf[i + 1])) * 100, 0);
				Serial.print(" ");
			}

			Serial.print("\t\t");
			for (int i = 0; i < 6; i++)
			{
				if ((0.001 * buf[i] + (0.256 * buf[i + 1]) + (65.535 * buf[i + 2])) * 1000 < 100000000)   Serial.print (" ");
				if ((0.001 * buf[i] + (0.256 * buf[i + 1]) + (65.535 * buf[i + 2])) * 1000 < 10000000)   Serial.print (" ");
				if ((0.001 * buf[i] + (0.256 * buf[i + 1]) + (65.535 * buf[i + 2])) * 1000 < 1000000)   Serial.print (" ");
				if ((0.001 * buf[i] + (0.256 * buf[i + 1]) + (65.535 * buf[i + 2])) * 1000 < 1000000)   Serial.print (" ");
				if ((0.001 * buf[i] + (0.256 * buf[i + 1]) + (65.535 * buf[i + 2])) * 1000 < 100000)   Serial.print (" ");
				if ((0.001 * buf[i] + (0.256 * buf[i + 1]) + (65.535 * buf[i + 2])) * 1000 < 10000)   Serial.print (" ");
				if ((0.001 * buf[i] + (0.256 * buf[i + 1]) + (65.535 * buf[i + 2])) * 1000 < 1000)   Serial.print (" ");
				if ((0.001 * buf[i] + (0.256 * buf[i + 1]) + (65.535 * buf[i + 2])) * 1000 < 100)   Serial.print (" ");
				if ((0.001 * buf[i] + (0.256 * buf[i + 1]) + (65.535 * buf[i + 2])) * 1000 < 10)   Serial.print (" ");

				Serial.print((0.001 * buf[i] + (0.256 * buf[i + 1]) + (65.535 * buf[i + 2])) * 1000, 0);
				Serial.print("  ");
			}
			Serial.println();

		}
	}
	// ***************************
	// canIDs with several values
	// ***************************
	if (canId == 0x0000)
	{
		// ?? byte 0
		// ?? byte 1
		// ?? byte 2
		// ?? byte 3
		// ?? byte 4
		// ?? byte 5
		// ?? byte 6
		// ?? byte 7
	}
	if (canId == 0x0181)
	{
		memcpy(canID181, buf, 8);
		// ?? byte 0 (0)
		// ?? byte 1 (0)
		// ?? byte 2 (0)
		// ?? byte 3 (0)
		// ?? byte 4 (byte 4..5 very eratic) look somethinh that balances between 0..40 and 65535..65524 so plus minus someting)
		// ?? byte 5
		// ?? byte 6 (byte 6..7 very eratic) look somethinh that balances between 0..40 and 65535..65524 so plus minus someting)
		// ?? byte 7
	}
	if (canId == 0x0188)
	{
		memcpy(canID188, buf, 8);
		SOC2 = buf[0];                                                                       // byte 0 (always 90)
		// ?? byte 1 (always 0)
		// ?? byte 2 (always 66, but 3 while charging in another session)
		ChargeCycles = buf[3] + 256 * buf[4];                                                   // byte 3 is number of charge cycles (probably byte 4 as well (highbyte)
		CellBalance = buf[5];                                                                   // byte 5 could be Cel Balance value
		// ?? byte 6 (always 0)
		// ?? byte 7 (always 4 maybe number of bricks ?)
	}
	if (canId == 0x0192)
	{
		memcpy(canID192, buf, 8);
		// ?? byte 0
		// ?? byte 1
		// ?? byte 2
		// ?? byte 3
		// ?? byte 4
		// ?? byte 5
		// ?? byte 6
		// ?? byte 7
	}
	if (canId == 0x01C0)
	{
		memcpy(canID1C0, buf, 8);
		// ?? byte 0 (fixed at 32)
		// ?? byte 1 (fixed at 0 unless dash buttons are pressed)
// 		if (bitRead(buf[1], 0) == 1) TopDisplayButtonPressed = true;
// 		else TopDisplayButtonPressed = false;
// 		if (bitRead(buf[1], 1) == 1) BottomDisplayButtonPressed = true;
// 		else BottomDisplayButtonPressed = false;
		DashTimeHours = buf[2];                                                              // byte 2 is Dash Time Hours
		DashTimeMinutes = buf[3];                                                            // byte 3 is Dash Time Minutes
		DashTimeSeconds = buf[4];                                                            // byte 4 is Dash Time Seconds
		NumberOfCanMessageSets = buf[5] + 258 * buf[6] + 256 * 256 * buf[7];                 // byte 5 - 7 is number of canbus message SETS received since power on ??
	}

	if (canId == 0x0206)                                                                // this canID only appears during charging !!
	{
		memcpy(canID206, buf, 8);
		// ?? byte 0 (second temperature sensor in the batterypack temperature ??)
		// ?? byte 1 (fixed at 2 during charging)
		ChargerVoltage = buf[2] * 0.001 + buf[3] * 0.256 + buf[4] * 65.535;              // byte 2-3-4 Charger voltage in mV (division by 1000 gets Volts)
		// ?? byte 5 (fixed at 1)
		// ?? byte 6 (fixed at 0)
		// ?? byte 7 (fixed at 0)
		Charging = true;
	}

	if (canId == 0x240)
	{
		memcpy(canID240, buf, 8);
		if (bitRead(buf[0], 2) == 1) Mode = "Sport ";        //A5                                        // CanID 0x0240 byte 0 = Mode
		if (bitRead(buf[0], 3) == 1) Mode = "Eco   ";       // A9
		if (bitRead(buf[0], 4) == 1) Mode = "Custom";    // B1
		// ?? byte 1 = always 0F : unknown
		Speed = (buf[2] * 0.01) + (buf[3] * 2.56);                                          // byte 2 and 3 is Speed in kph
		PowerBar = buf[4];                                                                  // ?? byte 4 is Power bar on dash ?? (It is power related)
		TorqueBar = buf[5];                                                                 // ?? byte 5 is Torque bar on dash ?? (It is Torque related)
		SOC = buf[6];                                                                       // byte 6 is State Of Charge on dash in %
		// no Byte 7
	}
	if (canId == 0x0281)
	{
		memcpy(canID281, buf, 8);
		MotorRPM = buf[0] + 256 * buf[1];                                                // ?? byte 0, 1 is motor RPM (same as canID 0x0340 byte 4 and 5)
		// ?? byte 2 (fixed at 0)
		// ?? byte 3 (fixed at 0)
		ThrottlePosition = buf[4] + 256 * buf[5];                                        // ?? byte 4, 5 is throttle position (around 125 is closed, aroud 1200 is fully open)
		MotorTemp = buf[6];                                                              // ?? byte 6 is motor temperature (second time) this time in 1 byte , not 2 as in canid 440
		// ?? byte 7 (fixed at 0)
	}
	if (canId == 0x0288)
	{
		memcpy(canID288, buf, 8);
		Power = buf[0] + 256 * buf[1];                                                      // byte 0, 1 Power in Watts
		// ?? byte 2 (fixed at 231) byte 2 .. 7 the same during charging several weeks later
		// ?? byte 3 (fixed at 0)
		// ?? byte 4 (fixed at 50)
		// ?? byte 5 (fixed at 114)
		// ?? byte 6 (fixed at 0)
		// ?? byte 7 (fixed at 18 not any temperature))
	}
	if (canId == 0x0292)
	{
		memcpy(canID292, buf, 8);
		// ?? byte 0 (176 during charging) (byte 0..2 give 116399 (battery cut-off???)
		// ?? byte 1 (198 during charging)
		// ?? byte 2 (1 during charging)
		// ?? byte 3 (0 during charging)
		// ?? byte 4 (255 during charging)
		// ?? byte 5 (255 during charging)
		// ?? byte 6 ( 0 during charging)
		// ?? byte 7 (0 during charging)
	}
	if (canId == 0x02C0)
	{
		memcpy(canID2C0, buf, 8);
		ODO = 0.1 * buf[0] + 25.6 * buf[1] + 6553.5 * buf[2];                               // byte 0, 1 and 2 are ODO in units of 100 mtrs (divided by 10 to get KM)
		// ?? byte 3 (fixed at 0)
		// ?? byte 4 (fixed at 0)
		// ?? byte 5 (fixed at 0)
		// ?? byte 6 (fixed at 16)
		// ?? byte 7 (fixed at 0)
	}
	if (canId == 0x0306)
	{
		memcpy(canID306, buf, 8);
		// ?? byte 0 (18/19 during charging) could be a temperature of sme sort
		// ?? byte 1 (176 during charging)
		// ?? byte 2 (198 during charging)
		// ?? byte 3 (1 during charging) (byte 1..3 give 116399)
		// ?? byte 4 (0 during charging)
		// ?? byte 5 (255 during charging)
		// ?? byte 6 (255 during charging)
		// ?? byte 7 0 during charging)
	}
	if (canId == 0x0308)
	{
		memcpy(canID308, buf, 8);
		// ?? byte 0
		// ?? byte 1
		// ?? byte 2
		// ?? byte 3
		// ?? byte 4
		// ?? byte 5
		// ?? byte 6
		// ?? byte 7
	}
	if (canId == 0x340)
	{
		memcpy(canID340, buf, 8);
		Trip1 = buf[0] * 0.01 + buf[1] * 2.56;                                              // byte 0, 1 is Trip 1
		// ?? byte 2 (always 0) byte 2,3 and 7 the same during charging several weeks later
		// ?? byte 3 (always 0)
		RPM =  buf[4] + 256 * buf[5];                                                       // byte 4, 5 is motor RPM
		ErrorCode = buf[6];                                                                 // byte 6 (error code on dash, 44 is killswitch, 45 is kickstand etc)
		// ?? byte 7 (always 0)
     if (ErrorCode == 47) Charging = true;
	}

	if (canId == 0x0381)
	{
		memcpy(canID381, buf, 8);
		// ?? byte 0 (95/96 during charging)
		// ?? byte 1 (6 during charging_
		ControllerTemp = buf[2];
		// ?? byte 3 (always 0 during charging)
		// ?? byte 4 (always 0 during charging)
		// ?? byte 6 (100..102 during charging)
		// ?? byte 7 (always 6 during charging)
	}


	if (canId == 0x0388)
	{
		memcpy(canID388, buf, 8);
		// ?? byte 0 (very random ?? 0-25 when speed = 0 , goes higher/lower when powering or regen)
		// ?? byte 1 (very reandom ?? 246-248 when speed = 0, goes higher/lower when powering or regen)
		// ?? byte 2 (always 15, later it was 14 when charging)
		Vbatt = 0.001 * buf[3] + 0.256 * buf[4] + 65.535 * buf[5];                          // byte 3, 4 and 5 are Vbatt in units of mV (divide by 1000 to get Volts)
		// ?? byte 6 (always zero, including charging)
		// ?? byte 7 (increases with torque/power and is negative with regen : could be power/torque/battery or motor amps, around 245,246 during charging
	}
	if (canId == 0x03C0)
	{
		memcpy(canID3C0, buf, 8);
		// ?? byte 0 (40 during charging)
		// ?? byte 1 (68 during charging)
		// ?? byte 2 (1 during charging)
		// ?? byte 3 (0)
		// ?? byte 4 (0)
		// ?? byte 5 (0)
		// ?? byte 6 (85 during charging)
		// ?? byte 7 (94 during charging)
	}
	if (canId == 0x0406)    // occasionally 18  16  39  41  29 156 241   2
	{
		memcpy(canID406, buf, 8);
		// ?? byte 0 (18/19 during charging)
		// ?? byte 1 (16 during charging)
		// ?? byte 2 (39 during charging)
		// ?? byte 3 1 during charging)
		// ?? byte 4 (0 during charging)
		// ?? byte 5 (255 during charging)
		// ?? byte 6 (255 during charging)
		// ?? byte 7 (0 during charging)
	}
	if (canId == 0x0408)
	{
		memcpy(canID408, buf, 8);
		// byte 0 (always 0 during charging)
		BatteryTemp = buf[1];                                                               // byte 1 is battery temp in celcius
		// byte 2 (always 20 during charging)
		BatteryAmps = buf[3] + 256 * buf[4];                                                // byte 3 battery amps, both when charging and
		// byte 5 (fixed at 131, 66 during charging)
		// byte 6 (fixed at 0)
		// byte 7 (fixed at 255)
	}

	if (canId == 0x0440)
	{
		memcpy(canID440, buf, 8);
		Trip2 = buf[0] * 0.01 + buf[1] * 2.56;                                              // byte 0, 1 is Trip 2 in units of 10 mtrs (divided by 100 to get KM)
		// byte 2 (fixed at 0)
		// byte 3 (fixed at 0)
		Range = buf[4] * 0.01 + buf[5] * 2.56;                                              // byte 4, 5 is range in units of 10 mtrs (divided by 100 to get KM)
		MotorTemperature = buf[6] * 0.01 + buf[7] * 2.56;                                   // byte 6, 7 is motortemperature in 0.01 degrees but only 2200, 2300, 2400 (divsion by 100 is immediate integer value)
	}
	if (canId == 0x0481)
	{
		memcpy(canID481, buf, 8);
		// ?? byte 0 (0 during charging)
		// ?? byte 1 (0)
		// ?? byte 2 (226)
		// ?? byte 3 (2)
		// ?? byte 4 (0)
		// ?? byte 5 (0)
		// ?? byte 6 (0)
		// ?? byte 7 (0)
	}
	if (canId == 0x0488)
	{
		memcpy(canID488, buf, 8);
		// ?? byte 0
		// ?? byte 1
		// ?? byte 2
		// ?? byte 3
		// ?? byte 4
		// ?? byte 5
		// ?? byte 6
		// ?? byte 7
	}
	if (canId == 0x0501)
	{
		memcpy(canID501, buf, 8);
		// ?? byte 0 (0 during charging)
		// ?? byte 1 (0)
		// ?? byte 2 (178)
		// ?? byte 3 (2)
		// ?? byte 4 (147)
		// ?? byte 5 (2)
		// ?? byte 6 (0)
		// ?? byte 7 (116)
	}
	if (canId == 0x0506)
	{
		memcpy(canID506, buf, 8);
		// ?? byte 0
		// ?? byte 1
		// ?? byte 2
		// ?? byte 3
		// ?? byte 4
		// ?? byte 5
		// ?? byte 6
		// ?? byte 7
	}
	if (canId == 0x0508)
	{
		memcpy(canID508, buf, 8);
		// ?? byte 0,1 ,2 are number of seconds used or maybe even lifetime possibly bytes 3 and on also included (at feb-2019 bytes are (xxx,223,114,92,10,0,75,0)
		// ?? byte 1 (204) on 21-3-2019_
		// ?? byte 2 (147)
		// ?? byte 3 (92)
		// ?? byte 4 (10)
		// ?? byte 5 (0)
		// ?? byte 6 (75)
		// ?? byte 7 (0)
	}
	if (canId == 0x0540)
	{
		memcpy(canID540, buf, 8);
		ChargeMinutesToGo = buf[0] + 256* buf[1];// ?? byte 0 minutes to go for charging (possibly also byte 1 for values > 256 minutes
		// ?? byte 2
		// ?? byte 3
		TripWhPerKm = buf[4] * 0.01 + buf[5] * 2.56;
		// ?? byte 5
		// ?? byte 6
		// ?? byte 7
	}
	if (canId == 0x0588)
	{
		memcpy(canID588, buf, 8);
		// ?? byte 0
		// ?? byte 1
		// ?? byte 2
		// ?? byte 3
		// ?? byte 4
		// ?? byte 5
		// ?? byte 6
		// ?? byte 7
	}
	if (canId == 0x0608)
	{
		memcpy(canID608, buf, 8);
		// ?? byte 0
		// ?? byte 1
		// ?? byte 2
		// ?? byte 3
		// ?? byte 4
		// ?? byte 5
		// ?? byte 6
		// ?? byte 7
	}
	if (canId == 0x0701)
	{
		memcpy(canID701, buf, 8);
		// ?? byte 0
		// ?? byte 1
		// ?? byte 2
		// ?? byte 3
		// ?? byte 4
		// ?? byte 5
		// ?? byte 6
		// ?? byte 7
	}

}

void showZeroSRFDash()                                                                    // full page with as much info as possible
{
	// fillArc(xpos, ypos, start angle, nr of segments / degrees, radiusx, radiusy, width, color);
	unsigned long displayStartMillis = micros();

	if (BatteryAmps > 32768)
	{
		BatteryAmps =  BatteryAmps - 65535;
	}
	AmpHours = AmpHours + ((float(BatteryAmps) * DISPLAYINTERVAL) / 3600000);

	int Hours = int(millis() / 3600000);
	int Minutes = (millis() % 3600000) / 60000;

	if (demoMode == true)
	{
		createDemoValues();
	}
	if (Charging == true)
	{
		Mode = "Charge";
		Speed = abs(BatteryAmps);
		padding = tft.textWidth("kph", 4);
		tft.setTextPadding(padding);
		tft.setTextColor(((128 >> 3) << 11) | ((128 >> 2) << 5) | (128 >> 3),BLACK);
		tft.drawString("Amps",240,220,4);
	}
	else
	{
		padding = tft.textWidth("Amps", 4);
		tft.setTextPadding(padding);
		tft.setTextColor(((128 >> 3) << 11) | ((128 >> 2) << 5) | (128 >> 3),BLACK);
		tft.drawString("kph",240,220,4);
	}

	if (Mode == "Eco   ")
	{
		outerRingColor = ((0 >> 3) << 11) | ((128 >> 2) << 5) | (0 >> 3);
		innerRingColor = ((0 >> 3) << 11) | ((255 >> 2) << 5) | (0 >> 3);
		innerRingDarkColor = ((0 >> 3) << 11) | ((64 >> 2) << 5) | (0 >> 3);
	}
	else if (Mode == "Sport ")
	{
		outerRingColor = ((128 >> 3) << 11) | ((0 >> 2) << 5) | (0 >> 3);
		innerRingColor = ((255 >> 3) << 11) | ((0 >> 2) << 5) | (0 >> 3);
		innerRingDarkColor = ((64 >> 3) << 11) | ((0 >> 2) << 5) | (0 >> 3);
	}
	else if (Mode == "Custom") // Mode would be Custom
	{
		outerRingColor = ((0 >> 3) << 11) | ((128 >> 2) << 5) | (128 >> 3);
		innerRingColor = ((0 >> 3) << 11) | ((255 >> 2) << 5) | (255 >> 3);
		innerRingDarkColor = ((0 >> 3) << 11) | ((64 >> 2) << 5) | (64 >> 3);
	}
	else// (Mode == "Charge")
	{
		outerRingColor = ((0 >> 3) << 11) | ((0 >> 2) << 5) | (128 >> 3);
		innerRingColor = ((0 >> 3) << 11) | ((0 >> 2) << 5) | (255 >> 3);
		innerRingDarkColor = ((0 >> 3) << 11) | ((0 >> 2) << 5) | (64 >> 3);
	}

	int smallArcLength = 19;
	int value = Speed * 270 / 160;
	value = constrain(value, 0, 270);
	int previousValue = previousSpeed * 270 / 160;  //
	previousValue = constrain(previousValue, 0, 270);
	if (previousMode != Mode)                       // redraw all stationary Text if mode has changed and also at startup !!
	{
		fillArc(240, 160, -135, 270, 132, 132, 12, outerRingColor); // redraw outer ring
		fillArc(240, 160, -135, 270, 120, 120, 30, innerRingDarkColor); // redraw inner Dark ring
		previousValue = 0;
		tft.setTextColor(innerRingColor, BLACK);
		tft.setTextSize(1);
		tft.setFreeFont(FONT);
		tft.setTextDatum(MC_DATUM);
		padding = tft.textWidth("Surplus", GFXFF);
		tft.setTextPadding(padding);
		tft.drawString("Stint"  ,  30, 5,GFXFF);
		tft.drawString("Temp"   ,  30,100,GFXFF);
		tft.drawString("Power"  ,  30,220,GFXFF);
		tft.drawString("Trip1"  ,  30,285,GFXFF);
		tft.drawString("Trip2"  , 120,285,GFXFF);
		tft.drawString("Mode"   , 240, 90,GFXFF);
		tft.drawString("Range"  , 240,250,GFXFF);
		tft.drawString("Surplus", 360,285,GFXFF);
		tft.drawString("Time"   , 450, 5,GFXFF);
		tft.drawString("SOC"    , 450,100,GFXFF);
		tft.drawString("Bat"    , 450,220,GFXFF);
		tft.drawString("ODO"    , 450,285,GFXFF);
	}
	previousMode = Mode;
	if (previousSpeed != Speed)
	{
		tft.setTextColor(WHITE, BLACK);
		tft.setTextPadding(53*3);
		tft.drawNumber(Speed,240,160,8);
		tft.setTextPadding(0);
	}
	previousSpeed = Speed;

	if (previousValue < value)
	{
		fillArc(240, 160, -135 + previousValue, value - previousValue, 120, 120, 30,  innerRingColor); // inner ring Color part (from -135 degrees to current value)
	}
	else if (previousValue > value)
	{
		fillArc(240, 160, -135 + value + 1, previousValue - value + 1 , 120, 120, 30, innerRingDarkColor); // inner Ring dark part (from current value to +135 degrees)
	}
	displayUpdates++;
	if (displayUpdates == 5)
	{
		displayUpdates = 0;
		float upperRightValue = SOC * 0.01;
		upperRightValue = 0.01 * constrain(upperRightValue * 100, 0, 100);
		fillArc(TFT_WIDTH / 2, TFT_HEIGHT / 2,  59 + 3 + smallArcLength * (1 - upperRightValue), smallArcLength * upperRightValue    , TFT_WIDTH * 180 / 480, TFT_WIDTH * 180 / 480, 3, innerRingColor); //upper right quadrant value
		fillArc(TFT_WIDTH / 2, TFT_HEIGHT / 2,  59 + 3                                   , smallArcLength * (1 - upperRightValue), TFT_WIDTH * 180 / 480, TFT_WIDTH * 180 / 480, 3, BLACK);

		float lowerRightValue = (Vbatt - 92) / 24;
		lowerRightValue = 0.01 * constrain(lowerRightValue * 100, 0, 100);
		fillArc(TFT_WIDTH / 2, TFT_HEIGHT / 2, 100 + smallArcLength * (1 - lowerRightValue)   , smallArcLength * lowerRightValue    , TFT_WIDTH * 180 / 480, TFT_WIDTH * 180 / 480, 3, innerRingColor);
		fillArc(TFT_WIDTH / 2, TFT_HEIGHT / 2, 100                                      , smallArcLength * (1 - lowerRightValue), TFT_WIDTH * 180 / 480, TFT_WIDTH * 180 / 480, 3, BLACK);

		//Power = 65535 - (1400/4.8); // for diagnostic purposes
		//Power = (40000/4.8); // for diagnostic purposes
		float PowerGraph = 0;
		if (Power > 32768)
		{
			PowerGraph = 4*(65535 - Power);
		}
		else
		{
			PowerGraph = Power;
		}
		float lowerLeftValue = PowerGraph * 0.0001;
		lowerLeftValue = 0.01 * constrain(lowerLeftValue * 100, 0, 100);
		fillArc(TFT_WIDTH / 2, TFT_HEIGHT / 2, -99 - 22 + 2                                , smallArcLength * lowerLeftValue     , TFT_WIDTH * 180 / 480, TFT_WIDTH * 180 / 480, 3, innerRingColor);
		fillArc(TFT_WIDTH / 2, TFT_HEIGHT / 2, -99 - 22 + 2 + smallArcLength * lowerLeftValue  , smallArcLength * (1 - lowerLeftValue) , TFT_WIDTH * 180 / 480, TFT_WIDTH * 180 / 480, 3, BLACK);

		float upperLeftValue = MotorTemp * 0.01;
		upperLeftValue = 0.01 * constrain(upperLeftValue * 100, 0, 100);
		fillArc(TFT_WIDTH / 2, TFT_HEIGHT / 2, -59 - 22                                  , smallArcLength * upperLeftValue     , TFT_WIDTH * 180 / 480, TFT_WIDTH * 180 / 480, 3, innerRingColor);
		fillArc(TFT_WIDTH / 2, TFT_HEIGHT / 2, -59 - 22  + smallArcLength * upperLeftValue  , smallArcLength * (1 - upperLeftValue) , TFT_WIDTH * 180 / 480, TFT_WIDTH * 180 / 480, 3, BLACK);

		tft.setTextColor(WHITE, BLACK);
		tft.setTextSize(1);
		tft.setFreeFont(FONT);
		tft.setTextDatum(MC_DATUM);
		padding = tft.textWidth("Custom", GFXFF); // get the width of the text in pixels
		tft.setTextPadding(padding);

		SurplusRange = constrain(Range - (TargetRange - Trip1), -200, 200);
		if (TorqueBar > 128)  TorqueBar = TorqueBar - 255;
		if (Power > 32768)    Power = Power - 65535;
		if (ODO > 0 and StartODO == 0) StartODO = ODO;
		StationaryVolt = Vbatt + BatteryAmps * InternalResistance;
		//if (BatteryAmps > 10) InternalResistance = (StationaryVolt - Vbatt)*1000/BatteryAmps;

		tft.drawString(String(ODO - StartODO,1)+"km"        , 30, 50,GFXFF);
		tft.drawString(String(MotorTemp)+"C"                , 30, 80,GFXFF);
		tft.drawString(String(ControllerTemp) +"C"          , 30,120,GFXFF);
        tft.drawString(String(TripWhPerKm,1)+"Wh"           , 30,148,GFXFF);
		tft.drawString(String(AmpHours,1)+"Ah"              , 30,172,GFXFF);
		if(abs(Power * 0.0048) < 10)
			tft.drawString(String(Power * 0.0048,1)+"kW"    , 30,240,GFXFF);
		else
			tft.drawString(String(Power * 0.0048,0)+"kW"    , 30,240,GFXFF);
		tft.drawString(String(TorqueBar) + "Nm"             , 30,260,GFXFF);

		tft.drawString(String(ZeroToHundredTime*0.001,2)    ,350, 30,GFXFF);
		tft.drawString(String(CellBalance) + "mV"           ,450,120,GFXFF);
		tft.drawString(String(BatteryTemp)+"C"              ,450,200,GFXFF);
		tft.drawString(String(StationaryVolt,1)                      ,450,240,GFXFF);
        
		//tft.drawString(String(StationaryVolt,1) ,380,260,GFXFF);
		tft.drawString(String(StationaryVolt/28,2)                   ,450,260,GFXFF);
		padding = tft.textWidth("Customs", GFXFF); // get the width of the text in pixels
		tft.setTextPadding(padding);
		tft.drawString(Mode                                 ,240,110,GFXFF);
		padding = tft.textWidth("23:58", 4); // get the width of the text in pixels


		tft.setTextPadding(padding);
		if(Minutes < 10)
			tft.drawString(String(Hours) +":0"+ String(Minutes)  , 30, 30,4);
		else
			tft.drawString(String(Hours) +":"+ String(Minutes)  , 30, 30,4);
		if(BatteryAmps > 124) tft.setTextColor(RED, BLACK);
		tft.drawString(String(BatteryAmps)+"A"                  , 30,202,4);
		tft.setTextColor(WHITE, BLACK);
		tft.drawString(String(Trip1,1)                          , 30,309,4);
		tft.drawString(String(Trip2,1)                          ,120,309,4);
		tft.drawString(String(Range,1)                          ,240,280,4);
		if(abs(SurplusRange) < 100)
			tft.drawString(String(SurplusRange,1)                   ,360,309,4);
		else
			tft.drawString(String(SurplusRange,0)                   ,360,309,4);
		if(DashTimeMinutes < 10)
			tft.drawString(String(DashTimeHours)+":0"+String(DashTimeMinutes)    ,450, 30,4);
		else
			tft.drawString(String(DashTimeHours)+":"+String(DashTimeMinutes)    ,450, 30,4);
		int Chargehours = ChargeMinutesToGo/60; //+":");//+ChargeMinutesToGo%60);

		if(Charging == true or demoMode == true)
		{
			if(ChargeMinutesToGo%60 < 10)
				tft.drawString(String(Chargehours)+":0"+String(ChargeMinutesToGo%60), 450,55,4);
			else
				tft.drawString(String(Chargehours)+":"+String(ChargeMinutesToGo%60), 450,55,4);
		}
		else tft.drawString(" ",450,50,4);
		tft.drawString(String(SOC) + "%"                        ,450, 80,4);
		tft.drawString(String(ODO,0)                            ,450,309,4);

		if (ErrorCode ==  0)        ErrorText = ":No error";
		else if (ErrorCode ==  1)   ErrorText = ":High Throttle";
		else if (ErrorCode ==  2)   ErrorText = ":Motor Temp Warning Stage 1";
		else if (ErrorCode ==  3)   ErrorText = ":Motor Temp Warning Stage 2";
		else if (ErrorCode ==  4)   ErrorText = ":Controller Temp Warning  1";
		else if (ErrorCode ==  5)   ErrorText = ":Controller Temp Warning  2";
		else if (ErrorCode ==  6)   ErrorText = ":BMS Throttle Enable Wire";
		else if (ErrorCode ==  7)   ErrorText = ":Low Battery Voltage";
		else if (ErrorCode ==  8)   ErrorText = ":High Battery Temp Status";
		else if (ErrorCode ==  9)   ErrorText = ":Low Battery Temp Status";
		else if (ErrorCode == 10)   ErrorText = ":Battery Temp Warning 1";
		else if (ErrorCode == 11)   ErrorText = ":Battery Temp Warning 2";
		else if (ErrorCode == 12)   ErrorText = ":Reserve Partition Warning";
		else if (ErrorCode == 13)   ErrorText = ":Reserve Switch Warning";
		else if (ErrorCode == 14)   ErrorText = ":Safety Override Active";
		else if (ErrorCode == 15)   ErrorText = ":Charger Attached,Not Charging";
		else if (ErrorCode == 16)   ErrorText = ":CIB Contactor Compromised";
		else if (ErrorCode == 17)   ErrorText = ":Charger Error";
		else if (ErrorCode == 18)   ErrorText = ":Battery Temp Sensor Fault";
		else if (ErrorCode == 19)   ErrorText = ":High Charge Current";
		else if (ErrorCode == 20)   ErrorText = ":BMS Low Isolation";
		else if (ErrorCode == 21)   ErrorText = ":Board Vpack Error";
		else if (ErrorCode == 22)   ErrorText = ":Board Temperature Error";
		else if (ErrorCode == 23)   ErrorText = ":POST Error";
		else if (ErrorCode == 24)   ErrorText = ":Startup Error";
		else if (ErrorCode == 25)   ErrorText = ":Contactor Open Warning";
		else if (ErrorCode == 26)   ErrorText = ":Contactor Welded Error";
		else if (ErrorCode == 27)   ErrorText = ":Precharge Error";
		else if (ErrorCode == 28)   ErrorText = ":BMS Isolation Fault";
		else if (ErrorCode == 29)   ErrorText = ":BMS Isolation Danger";
		else if (ErrorCode == 30)   ErrorText = ":BMS CAN Error";
		else if (ErrorCode == 31)   ErrorText = ":SEVCON CAN Error";
		else if (ErrorCode == 32)   ErrorText = ":Module Variance Too High";
		else if (ErrorCode == 33)   ErrorText = ":SEVCON Error Turn Off";
		else if (ErrorCode == 34)   ErrorText = ":Charge Error Turn Off";
		else if (ErrorCode == 35)   ErrorText = ":Loopback Error";
		else if (ErrorCode == 36)   ErrorText = ":Board 5V Error";
		else if (ErrorCode == 37)   ErrorText = ":Board 3.3V Error";
		else if (ErrorCode == 38)   ErrorText = ":Idle Turn Off";
		else if (ErrorCode == 39)   ErrorText = ":Throttle OutOfRange Disable";
		else if (ErrorCode == 40)   ErrorText = ":BMS Throttle EnWire Disable";
		else if (ErrorCode == 41)   ErrorText = ":Low BatteryVoltage Disable";
		else if (ErrorCode == 42)   ErrorText = ":High Battery Temp Disable";
		else if (ErrorCode == 43)   ErrorText = ":Low Battery Temp Disable";
		else if (ErrorCode == 44)   ErrorText = ":Kill Switch Disable";
		else if (ErrorCode == 45)   ErrorText = ":Kickstand Switch Disable";
		else if (ErrorCode == 46)   ErrorText = ":BMS Charger Conn. Disable";
		else if (ErrorCode == 47)   ErrorText = ":MBB Charger Conn. Disable";
		else if (ErrorCode == 48)   ErrorText = ":SEVCON Startup Disable";
		else if (ErrorCode == 49)   ErrorText = ":Contactor Open Disable";
		else if (ErrorCode == 50)   ErrorText = ":BMS Self-Test Error";
		else if (ErrorCode == 51)   ErrorText = ":BMS Self-Test Warning";
		else if (ErrorCode == 52)   ErrorText = ":Reserve Partition Disable";
		else if (ErrorCode == 53)   ErrorText = ":BMS Internal Disable";
		else if (ErrorCode == 54)   ErrorText = ":Internal Disable Error";
		else if (ErrorCode == 55)   ErrorText = ":Internal Fault Error";
		else if (ErrorCode == 56)   ErrorText = ":Monolith Not Connected";
		else if (ErrorCode == 57)   ErrorText = ":Module Did Not Connect";
		else if (ErrorCode == 58)   ErrorText = ":BMS Sensor Warning";
		else if (ErrorCode == 59)   ErrorText = ":BMS System Warning";
		else if (ErrorCode == 60)   ErrorText = ":Bike Configuration";
		else                        ErrorText = ":See Manual";
		padding = tft.textWidth("47:MBB Charger Conn. Disable      ", GFXFF); // get the width of the text in pixels
		tft.setTextPadding(padding);
		if(ErrorCode != 0) tft.drawString(ErrorCode + ErrorText                       ,240, 8,GFXFF);
		else tft.drawString(" "                       ,240, 8,GFXFF);
		tft.setTextPadding(0);
	}
	//tft.setTextSize(1);
	//tft.setCursor(240 - 3 * 3 * 1, 300);
	//tft.print(micros() - displayStartMillis);
	//delay(1000);
	//tft.drawLine(240,0,240,320,WHITE);
	//tft.drawLine(0,160,480,160,WHITE);
	//tft.print("  ");
}





// #########################################################################
// Draw an arc with a defined thickness
// #########################################################################

// x,y == coords of centre of arc
// start_angle = 0 - 359
// seg_count = number of 3 degree segments to draw (120 => 360 degree arc)
// rx = x axis radius
// yx = y axis radius
// w  = width (thickness) of arc in pixels
// colour = 16 bit colour value
// Note if rx and ry are the same an arc of a circle is drawn



int fillArc(int x, int y, int start_angle, int seg_count, int rx, int ry, int w, unsigned int colour)
{

	byte seg = 1; // Segments are 3 degrees wide = 120 segments for 360 degrees
	byte inc = 1; // Draw segments every 3 degrees, increase to 6 for segmented ring

	// Calculate first pair of coordinates for segment start
	float sx = cos((start_angle - 90) * DEG2RAD);
	float sy = sin((start_angle - 90) * DEG2RAD);
	uint16_t x0 = sx * (rx - w) + x;
	uint16_t y0 = sy * (ry - w) + y;
	uint16_t x1 = sx * rx + x;
	uint16_t y1 = sy * ry + y;

	// Draw colour blocks every inc degrees
	for (int i = start_angle; i < start_angle + seg * seg_count; i += inc) {

		// Calculate pair of coordinates for segment end
		float sx2 = cos((i + seg - 90) * DEG2RAD);
		float sy2 = sin((i + seg - 90) * DEG2RAD);
		int x2 = sx2 * (rx - w) + x;
		int y2 = sy2 * (ry - w) + y;
		int x3 = sx2 * rx + x;
		int y3 = sy2 * ry + y;


		if ((i == 0 or abs(i) == 34 or abs(i) == 68 or abs(i) == 101) and colour != 0xFFFF and rx < 150)        // do not draw (colored) lines when the speed is 20, 40, 60, 80 ,100 120 or 140. White lines are excluded by using coloour != 0xFFFF and outer arc are excluded by using rx < 150
		{
			tft.fillTriangle(x0, y0, x1, y1, x2, y2, 0x0000);          // Try drawTriangle instead of fillTriangle to save time (a lot of time), edit actually drawTriangle takes longer and yields worse results on a 480x320 screen
			tft.fillTriangle(x1, y1, x2, y2, x3, y3, 0x0000);
		}
		else
		{
			tft.fillTriangle(x0, y0, x1, y1, x2, y2, colour);
			tft.fillTriangle(x1, y1, x2, y2, x3, y3, colour);
		}
		// Copy segment end to sgement start for next segment
		x0 = x2;
		y0 = y2;
		x1 = x3;
		y1 = y3;
	}
}

void createDemoValues()
{
	demoValue = demoValue + upDown;
	if (demoValue >= 100 or demoValue <= 0)
	{
		upDown = upDown * -1;
		if (Mode == "Eco   ") Mode = "Sport ";
		else if (Mode == "Sport ") Mode = "Custom";
		else if (Mode == "Custom") Mode = "Charge";
		else  Mode = "Eco   ";
	}

	Speed = int(demoValue * 160 / 100);
	CellBalance = demoValue * 1.01;
	SOC = demoValue;
	BatteryTemp = demoValue * 60 / 100 ;
	Vbatt = demoValue * 20.1 / 100 + 96;
	Range = demoValue * 170 / 100;
	Hours = demoValue * 24  / 100;
	Minutes = demoValue * 59 / 160;
	DashTimeHours = demoValue * 24 / 100;
	DashTimeMinutes = demoValue * 59 / 100;
	Trip1 = demoValue * 1.1;
	Trip2 = demoValue * 13.1;
	Power = ( demoValue * 65535 / 100);
	BatteryAmps = -100 + 775 * demoValue / 100;
	ControllerTemp = demoValue * 60 / 100 ;
	MotorTemp = demoValue * 60 / 100 ;
	ErrorCode = int(demoValue);
	ODO = 8000 + demoValue * 1;
	TorqueBar = -20 + demoValue * 1.6;
    TripWhPerKm = demoValue;
    ChargeMinutesToGo = demoValue*9;
  //  Charging = true;

}

void   CalcZeroToHundred()
{
	if (Speed == 0)
	{
		ZeroToHundredStart = millis();
		ZeroToHundredBusy = 1;
	}
	if (Speed >= 100 and ZeroToHundredBusy == 1)
	{
		ZeroToHundredStop = millis();
		ZeroToHundredBusy = 0;
		ZeroToHundredTime = ZeroToHundredStop - ZeroToHundredStart;
	}
}

void showCanIDPage()
{
	unsigned long displayStartMillis = micros();

	int i = 20;
	tft.setCursor(0,0);
	tft.print("canid 0   1   2   3   4   5   6   7  -   0-1   1-2   2-3   3-4   4-5   5-6   6-7");
	memcpy(canIDByte, canID181, 8);
	showCanID(i+0,0x181);
	memcpy(canIDByte, canID188, 8);
	showCanID(i+8,0x188);
	memcpy(canIDByte, canID192, 8);
	showCanID(i+16,0x192);
	memcpy(canIDByte, canID206, 8);
	showCanID(i+24,0x206);
	memcpy(canIDByte, canID240, 8);
	showCanID(i+32,0x240);
	memcpy(canIDByte, canID281, 8);
	showCanID(i+40,0x281);
	memcpy(canIDByte, canID288, 8);
	showCanID(i+48,0x288);
	memcpy(canIDByte, canID292, 8);
	showCanID(i+56,0x292);
	memcpy(canIDByte, canID306, 8);
	showCanID(i+64,0x306);
	memcpy(canIDByte, canID308, 8);
	showCanID(i+72,0x308);
	memcpy(canIDByte, canID340, 8);
	showCanID(i+80,0x340);
	memcpy(canIDByte, canID381, 8);
	showCanID(i+88,0x381);
	memcpy(canIDByte, canID388, 8);
	showCanID(i+96,0x388);
	memcpy(canIDByte, canID406, 8);
	showCanID(i+104,0x406);
	memcpy(canIDByte, canID408, 8);
	showCanID(i+112,0x408);
	memcpy(canIDByte, canID440, 8);
	showCanID(i+120,0x440);
	memcpy(canIDByte, canID481, 8);
	showCanID(i+128,0x481);
	memcpy(canIDByte, canID488, 8);
	showCanID(i+136,0x488);
	memcpy(canIDByte, canID501, 8);
	showCanID(i+144,0x501);
	memcpy(canIDByte, canID506, 8);
	showCanID(i+152,0x506);
	memcpy(canIDByte, canID508, 8);
	showCanID(i+160,0x508);
	memcpy(canIDByte, canID540, 8);
	showCanID(i+168,0x540);
	memcpy(canIDByte, canID588, 8);
	showCanID(i+176,0x588);
	memcpy(canIDByte, canID608, 8);
	showCanID(i+184,0x608);
	memcpy(canIDByte, canID701, 8);
	showCanID(i+192,0x701);
	memcpy(canIDByte, canID1C0, 8);
	showCanID(i+200,0x1C0);
	memcpy(canIDByte, canID2C0, 8);
	showCanID(i+208,0x2C0);
	memcpy(canIDByte, canID3C0, 8);
	showCanID(i+216,0x3C0);

	tft.setCursor(0,250);
	tft.setTextSize(8);
	tft.print(Speed,0);
	tft.print("  ");
	tft.setTextSize(1);
	tft.print(micros() - displayStartMillis);


}

void showCanID(int ypos, int canid)
{
	tft.setCursor(0, ypos);
	tft.print("0");
	tft.print(canid,HEX);
	tft.print(" ");
	for (int i = 0; i < 8; i++)
	{
		if ( canIDByte[i] < 100)  tft.print(" ");
		if ( canIDByte[i] < 10)  tft.print(" ");
		if (canIDByte[i] == 468 or canIDByte[i] == 345 or canIDByte[i] == 139)
			tft.setTextColor(RED,BLACK);
		else tft.setTextColor(WHITE,BLACK);
		tft.print(canIDByte[i]);
		tft.print(" ");
	}
	tft.print("- ");
	for (int i = 0; i < 7; i++)
	{
		if ( (0.01 * canIDByte[i] + (2.56 * canIDByte[i + 1])) * 100 < 10000)  tft.print(" ");
		if ( (0.01 * canIDByte[i] + (2.56 * canIDByte[i + 1])) * 100 < 1000)  tft.print(" ");
		if ( (0.01 * canIDByte[i] + (2.56 * canIDByte[i + 1])) * 100 < 100)  tft.print(" ");
		if ( (0.01 * canIDByte[i] + (2.56 * canIDByte[i + 1])) * 100 < 10)  tft.print(" ");
		if ((0.01 * canIDByte[i] + (2.56 * canIDByte[i + 1])) * 100 == 468 or (0.01 * canIDByte[i] + (2.56 * canIDByte[i + 1])) * 100 == 46.80)
			tft.setTextColor(RED,BLACK);
		else tft.setTextColor(WHITE,BLACK);

		tft.print((0.01 * canIDByte[i] + (2.56 * canIDByte[i + 1])) * 100, 0);
		tft.print(" ");
	}
}
