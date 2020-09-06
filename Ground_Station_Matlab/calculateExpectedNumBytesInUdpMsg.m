function expectedNumBytes = calculateExpectedNumBytesInUdpMsg(udpMsg_struct)

expectedNumBytes = 0;
for n = 1:size(udpMsg_struct,1)
    expectedNumBytes = expectedNumBytes + sizeof(udpMsg_struct{n,1});
end

