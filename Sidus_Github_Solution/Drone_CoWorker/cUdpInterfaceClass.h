
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>

class UdpInterfaceClass
{
public:
	//Properties
	char *ssid;
	char *password;
	unsigned int udpPort;
	IPAddress remoteIP;
	IPAddress myIP;
	WiFiUDP udp;

	//Constructor and Destructor
	UdpInterfaceClass(char *_ssid, char * _pass, unsigned int _udpPort, char* _remoteIP);
	~UdpInterfaceClass();

	//Methods
	void sendPacket(unsigned char *, unsigned long);
	bool initConnection(int timeoutInMillis, bool isAP);
};

