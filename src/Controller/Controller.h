// controller.h

#ifndef _CONTROLLER_h
#define _CONTROLLER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#endif
#include "RF24.h"
typedef struct rx_values_t {
	uint8_t throttle;
	uint8_t yaw;
	uint8_t pitch;
	uint8_t roll;
	int8_t trim_yaw;
	int8_t trim_pitch;
	int8_t trim_roll;
	uint8_t flip;
	uint8_t highspeed;
	float P;
	float I;
	float D;
};
class Controller
{
private:
	RF24* mWireless;
	uint8_t mChannel;
	rf24_pa_dbm_e mPALevel;
	bool mWriting;
public:
	uint8_t receive(rx_values_t* rxValues);
	void init();
	uint8_t send(rx_values_t* rxValues);
	uint8_t sendGeneral(void* buf, uint8_t size);
	void print(rx_values_t* rxValues);
//constructors
	Controller(RF24* radio, uint8_t channel, bool isWriting);
	Controller(RF24* radio, uint8_t channel, rf24_pa_dbm_e PALevel, bool isWriting);
};
#endif
