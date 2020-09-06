function processUdpMsg(src,~)
    % Read new bytes
    udpMsg = uint8(fread(src));
    
    % Parse the bytes
    parseUdpMsg(udpMsg);
end