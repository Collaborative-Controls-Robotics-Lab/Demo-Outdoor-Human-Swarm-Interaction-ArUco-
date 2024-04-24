clc
clear all
tcpipServer = tcpip('127.0.0.1',4000,'NetworkRole','Server');
while(1)
    data = membrane(1);
    fopen(tcpipServer);
    rawData = fread(tcpipServer,14,'char');
        for i=1:14 
            rawData(i)= char(rawData(i));
        end
fclose(tcpipServer);
end