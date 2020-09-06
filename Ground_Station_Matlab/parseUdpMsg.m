function parseUdpMsg(udpMsg)

% Create the udp message struct
udpMsg_struct = createUdpMsgStruct();  

% Create the udp message
idxCnt = 1;
for n = 1:size(udpMsg_struct,1)
    dataType = udpMsg_struct{n,1};
    dataSize = sizeof(dataType);
    droneData.(udpMsg_struct{n,2}) = typecast(udpMsg(idxCnt:idxCnt+dataSize-1),dataType);
    idxCnt = idxCnt + dataSize;
end

processDroneData(droneData);