package flora.LoRaApp;

import inet.applications.contract.IApp;

simple WaterLeakLoRaApp like IApp
{
    parameters:
        @signal[LoRa_AppPacketSent](type=long);
        @statistic[LoRa_AppPacketSent](source=LoRa_AppPacketSent; record=count);

        volatile double timeToFirstReport @unit(s) = default(uniform(0s, 10s));
        volatile double reportInterval @unit(s) = default(10s);

        // Per-node leak rate. Total network lambda = leakLambda * numberOfNodes.
        volatile double leakLambda = default(0.004);
        
        double txGuardTime @unit(s) = default(20s);

        double initialLoRaTP @unit(dBm) = default(14dBm);
        double initialLoRaCF @unit(Hz) = default(868MHz);
        int initialLoRaSF = default(12);
        double initialLoRaBW @unit(Hz) = default(125kHz);
        int initialLoRaCR = default(4);
        bool initialUseHeader = default(true);

        bool evaluateADRinNode = default(false);
        int dataSize @unit(B) = default(10B);

    gates:
        input socketIn @labels(LoRaAppPacket/up);
        output socketOut @labels(LoRaAppPacket/down);
}