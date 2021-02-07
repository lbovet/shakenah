/*
Extends the Serial class to encode SLIP over serial
*/

#ifndef SLIPEncodedStream_h
#define SLIPEncodedStream_h


#include "Arduino.h"
#include <Stream.h>

class SLIPEncodedStream: public Stream{

private:
	enum erstate {CHAR, FIRSTEOT, SECONDEOT, SLIPESC } rstate;

	Stream * serial;


public:
	SLIPEncodedStream(Stream &);

	int available();
	int read();
	int peek();
	void flush();

    //SLIP specific method which begins a transmitted packet
	void beginPacket();

	//SLIP specific method which ends a transmittedpacket
	void endPacket();
	// SLIP specific method which indicates that an EOT was received
	bool endofPacket();


	//overrides the Stream's write function to encode SLIP
	size_t write(uint8_t b);
    size_t write(const uint8_t *buffer, size_t size);

	//using Print::write;

};


#endif
