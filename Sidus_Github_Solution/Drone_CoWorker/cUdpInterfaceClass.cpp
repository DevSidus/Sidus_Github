#include "cUdpInterfaceClass.h"

UdpInterfaceClass::~UdpInterfaceClass()
{
}

UdpInterfaceClass::UdpInterfaceClass(char *_ssid, char * _pass, unsigned int _udpPort, char* _remoteIP)
{
	ssid = _ssid;
	password = _pass;
	udpPort = _udpPort;
	remoteIP.fromString(_remoteIP);
}

void UdpInterfaceClass::sendPacket(unsigned char * packetToSend, unsigned long packetSize)
{
	udp.beginPacket(remoteIP, udpPort);
	udp.write(packetToSend, packetSize);
	udp.endPacket();
}

bool UdpInterfaceClass::initConnection(int timeoutInMillis, bool _isAP)
{

	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);

	unsigned long startTime = millis();

	while ((millis()-startTime) < timeoutInMillis) {

		if (WiFi.waitForConnectResult() == WL_CONNECTED)
		{
			Serial.print("Wifi Ready, IP:");
			Serial.println(WiFi.localIP());
			Serial.print("Starting UDP at Port:");
			Serial.println(udpPort);
			
			if (udp.begin(udpPort) == 1)
			{
				//Send a welcome packet
				delay(200);
				udp.beginPacket(remoteIP, udpPort);
				udp.write("Welcome Message", 15);
				udp.endPacket();
				return true;
			}
			else
			{
				return false;
			}
			
		}
		else
		{
			delay(200);
		}
	}
	return false;	
}