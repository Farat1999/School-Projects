function receiveSignal = MyAWGNchannel(transmitSignal,noiseVariance)

receiveSignal = transmitSignal + noiseVariance*randn(size(transmitSignal));
