//
//
//

#include "controller.h"
#include "RF24.h"
static byte addresses[][8] = { "1Node","2Node" };
Controller::Controller(RF24* radio, uint8_t channel, bool isWriting)
:Controller(radio, channel, RF24_PA_MIN, isWriting) {}
Controller::Controller(RF24* radio, uint8_t channel,rf24_pa_dbm_e PALevel, bool isWriting)
:mWireless(radio),mChannel(channel),mPALevel(PALevel),mWriting(isWriting){}

void Controller::init() {
	mWireless->begin();
	mWireless->setPALevel(RF24_PA_HIGH);
	mWireless->setPayloadSize(sizeof(rx_values_t));
	mWireless->setChannel(mChannel);
	// RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps
	mWireless->setDataRate(RF24_250KBPS);
	// RF24_CRC_8 or RF24_CRC_16 CRC
	//cyclic redundancy check (CRC) detects accidental changes to raw data
	mWireless->setCRCLength(RF24_CRC_16);
	if (mWriting) {
		mWireless->openWritingPipe(addresses[1]);
		mWireless->openReadingPipe(1, addresses[0]);
	}
	else {
		mWireless->openWritingPipe(addresses[0]);
		mWireless->openReadingPipe(1, addresses[1]);
	}
	mWireless->startListening();
}

uint8_t Controller::receive(rx_values_t* rxValues) {
	int i = 0;
	while (mWireless->available()) {
		mWireless->read(rxValues, sizeof(rx_values_t));
			i++;
	}
	return i;
}
uint8_t Controller::send(rx_values_t* rxValues) {
	mWireless->stopListening();
	Serial.println("sending data");
	bool isSent=mWireless->write(rxValues, sizeof(*rxValues));
	Serial.println("receiving data");
	mWireless->startListening();
	return isSent;
}

uint8_t Controller::sendGeneral(void* buf, uint8_t size) {
	mWireless->stopListening();
	bool isSent = mWireless->write(buf, size);
	mWireless->startListening();
	return isSent;
}

void Controller :: print(rx_values_t* rxValues){
	Serial.print(" :\t"); Serial.print(rxValues->throttle);
	Serial.print("\t"); Serial.print(rxValues->yaw);
	Serial.print("\t"); Serial.print(rxValues->pitch);
	Serial.print("\t"); Serial.print(rxValues->roll);
	Serial.print("\t"); Serial.print(rxValues->trim_yaw);
	Serial.print("\t"); Serial.print(rxValues->trim_pitch);
	Serial.print("\t"); Serial.print(rxValues->trim_roll);
	Serial.print("\t"); Serial.print(rxValues->highspeed);
	Serial.print("\t"); Serial.print(rxValues->flip);
	Serial.print("\t"); Serial.print(rxValues->P);
	Serial.print("\t"); Serial.print(rxValues->I);
	Serial.print("\t"); Serial.println(rxValues->D);
}
