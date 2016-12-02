#include "cMsgT01.h"

cMsgT01::cMsgT01()
{
	//If required message member initialization can be done here

}

//Class destructor
cMsgT01::~cMsgT01()
{
}

//This function should be used just before reading dataBytes of message.
//Data Array members starting from 0 to message struct size are set after the function call
void cMsgT01::getPacket()
{
	//Sending Side
	memcpy(dataBytes, &message, sizeof(message));
}


//This function should be used just after a new dataPacket in order to set all the struct members
void cMsgT01::setPacket()
{
	//Receiving Side
	memcpy(&message, dataBytes, sizeof(message));
}