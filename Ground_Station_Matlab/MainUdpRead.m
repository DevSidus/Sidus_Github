% Create the UDP
udpPort = udp('192.168.0.17',8080,'LocalPort', 8080);

% Create the udp message struct
udpMsg_struct = createUdpMsgStruct();

% Calculate the expected number of bytes in the udp message
expectedNumBytes = calculateExpectedNumBytesInUdpMsg(udpMsg_struct);

% Define the input buffer size
udpPort.InputBufferSize         = expectedNumBytes;

% Define the callback function
udpPort.BytesAvailableFcnCount  = expectedNumBytes;
udpPort.BytesAvailableFcnMode   = 'byte';
udpPort.BytesAvailableFcn       = @processUdpMsg; 
    
% Open the port
fopen(udpPort);

return

%% Close the port
fclose(udpPort);
delete(udpPort)
clear udpPort
clear all
clc