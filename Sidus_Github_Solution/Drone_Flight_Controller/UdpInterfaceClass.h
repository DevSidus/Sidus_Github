#pragma once
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
	UdpInterfaceClass();
	~UdpInterfaceClass();
};

