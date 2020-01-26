//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2016-10-31 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------

#define NDEBUG
// define this to read the device id, serial and device type from bootloader section
#define USE_OTA_BOOTLOADER

//#define USE_HW_SERIAL
//#define __AVR_ATmega328PB__ 1

#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <AskSinPP.h>
#include <LowPower.h>
#include <sensors/Sht21.h> //https://github.com/manawyrm/SHT21
#include <MultiChannelDevice.h>
#include <Adafruit_BMP280.h>

// B0 == PIN 8
#define CONFIG_BUTTON_PIN 8


//-----------------------------------------------------------------------------------------

//Korrektur von Temperatur und Luftfeuchte
//Einstellbarer OFFSET für Temperatur -> gemessene Temp +/- Offset = Angezeigte Temp.
#define OFFSETtemp 0 //z.B -50 ≙ -5°C / 50 ≙ +5°C

//Einstellbarer OFFSET für Luftfeuchte -> gemessene Luftf. +/- Offset = Angezeigte Luftf.
#define OFFSEThumi 0 //z.B -10 ≙ -10%RF / 10 ≙ +10%RF

//-----------------------------------------------------------------------------------------

enum deviceTypes 
{
	DEVICE_TYPE_WDS40, // 0x003F
	DEVICE_TYPE_TEMPHUMID, // 0xFA02
	DEVICE_TYPE_TEMPHUMIDPRESSURE // 0xFA01
};
uint8_t deviceType = DEVICE_TYPE_TEMPHUMID;
uint8_t deviceModel[3];

// number of available peers per channel
#define PEERS_PER_CHANNEL 6

//seconds between sending messages
#define MSG_INTERVAL 180

// all library classes are placed in the namespace 'as'
using namespace as;

const struct DeviceInfo PROGMEM devinfo = {
	{0x23, 0x00, 0x0A},			// Device ID
	"TBSP23000A",
	//{0x00, 0x3d}				// Device Model Outdoor
	{0xFA, 0x02},				// Device Model Indoor
	0x11,                   	// Firmware Version
	as::DeviceType::THSensor,	// Device Type
	{0x01, 0x00}          		// Info Bytes
};

/**
	 Configure the used hardware
*/
typedef AvrSPI<10, 11, 12, 13> SPIType;
typedef Radio<SPIType, 2> RadioType;
typedef DualStatusLed<5,4> LedType;
typedef AskSin<LedType, BatterySensor, RadioType> Hal;
Hal hal;

Adafruit_BMP280 bmp; // I2C

class WeatherEventMsg : public Message {
	public:
		void init(uint8_t msgcnt, int16_t temp, uint8_t humidity, bool batlow, uint16_t batVoltage, uint16_t pressure) {
			uint8_t t1 = (temp >> 8) & 0x7f;
			uint8_t t2 = temp & 0xff;
			if ( batlow == true )
			{
				t1 |= 0x80; // set bat low bit
			}

			if (deviceType == DEVICE_TYPE_WDS40)
			{
				Message::init(11 + 1, msgcnt, 0x70, BIDI | WKMEUP, t1, t2);
				pload[0] = humidity;
			}
			if (deviceType == DEVICE_TYPE_TEMPHUMID)
			{
				Message::init(11 + 3, msgcnt, 0x70, BIDI | WKMEUP, t1, t2);
				pload[0] = humidity;
				pload[1] = (batVoltage >> 8) & 0xFF;
				pload[2] = batVoltage & 0xFF;
			}
			if (deviceType == DEVICE_TYPE_TEMPHUMIDPRESSURE)
			{
				Message::init(11 + 5, msgcnt, 0x70, BIDI | WKMEUP, t1, t2);
				pload[0] = humidity;
				pload[1] = (pressure >> 8) & 0xFF;
				pload[2] = pressure & 0xFF;
				pload[3] = (batVoltage >> 8) & 0xFF;
				pload[4] = batVoltage & 0xFF;
			}

		}
};

class WeatherChannel : public Channel<Hal, List1, EmptyList, List4, PEERS_PER_CHANNEL, List0>, public Alarm {

		WeatherEventMsg msg;
		int16_t         temp;
		uint8_t         humidity;
		Sht21<>         sht21; 
		uint16_t        millis;
		uint16_t        pressure;

	public:
		WeatherChannel () : Channel(), Alarm(5), temp(0), humidity(0), millis(0), pressure(0) {}
		virtual ~WeatherChannel () {}


		// here we do the measurement
		void measure () {
			DPRINT("Measure and sleep...\n");

			pinMode(A3, OUTPUT);
			digitalWrite(A3, HIGH);

			if (deviceType == DEVICE_TYPE_TEMPHUMIDPRESSURE)
			{
				if (!bmp.begin())
				{
					pressure = 65535;
				}
			}
		 
			LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);  
			sht21.measure();
			temp = sht21.temperature();
			humidity = sht21.humidity();

			if (deviceType == DEVICE_TYPE_TEMPHUMIDPRESSURE)
			{
				/* Default settings from datasheet. */
				bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
								Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
								Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
								Adafruit_BMP280::FILTER_X16,      /* Filtering. */
								Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

				pressure = (bmp.readPressure() / 10.0f);
			}

			pinMode(A3, INPUT);
			digitalWrite(A3, LOW);    

			DPRINT("T/H = " + String(temp+OFFSETtemp) + "/" + String(humidity+OFFSEThumi) + "\n");
		}

		virtual void trigger (__attribute__ ((unused)) AlarmClock& clock) {
			uint8_t msgcnt = device().nextcount();
			// reactivate for next measure
			tick = delayInterval();
			clock.add(*this);
			measure();

			msg.init(msgcnt, temp+OFFSETtemp, humidity+OFFSEThumi, device().battery().low(), device().battery().meter().value(), pressure);
			if (msgcnt % 20 == 1) device().sendPeerEvent(msg, *this); else device().broadcastEvent(msg, *this);
		}

		uint32_t delayInterval () {
			return seconds2ticks(MSG_INTERVAL);
		}
		void setup(Device<Hal, List0>* dev, uint8_t number, uint16_t addr) {
			Channel::setup(dev, number, addr);
			sht21.init();
			sysclock.add(*this);
		}

		uint8_t status () const {
			return 0;
		}

		uint8_t flags () const {
			return 0;
		}
};

typedef MultiChannelDevice<Hal, WeatherChannel, 1> WeatherType;
WeatherType sdev(devinfo, 0x20);
ConfigButton<WeatherType> cfgBtn(sdev);


void setup () 
{
	DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
	sdev.init(hal);
	sdev.getDeviceModel(deviceModel);

	// Autodetect device type from model ID in bootloader: 
	if (deviceModel[0] == 0xFA && deviceModel[1] == 0x01)
	{
		deviceType = DEVICE_TYPE_TEMPHUMIDPRESSURE;
	}
	if (deviceModel[0] == 0xFA && deviceModel[1] == 0x02)
	{
		deviceType = DEVICE_TYPE_TEMPHUMID;
	}
	if (deviceModel[0] == 0x00 && deviceModel[1] == 0x3F)
	{
		deviceType = DEVICE_TYPE_WDS40;
	}

	hal.initBattery(60UL * 60, 22, 19);
	buttonISR(cfgBtn, CONFIG_BUTTON_PIN);
	sdev.initDone();
}

void loop() {
	bool worked = hal.runready();
	bool poll = sdev.pollRadio();
	if ( worked == false && poll == false ) {
		hal.activity.savePower<Sleep<>>(hal);
	}
}
