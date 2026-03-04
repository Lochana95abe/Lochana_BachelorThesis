package uav_mec_sim04;

simple CaptureUav4
{
    parameters:
        double taskStartTime @unit(s) = default(1s);
        double meanInterArrival @unit(s) = default(0.5s);      // 0.5 s period anchor [Ahybridfastinferenceapproachwithdistributedneuralnetworksfor.pdf, p.8]
        int imagesPerTask = default(5);                        // 5 images per task [Ahybridfastinferenceapproachwithdistributedneuralnetworksfor.pdf, p.8]
        int bytesPerImage = default(200000);                   // PLACEHOLDER
        double captureDelay @unit(s) = default(5ms);           // PLACEHOLDER

        double deadline @unit(s) = default(1s);                // inside 0.6 to 1.8 s band [Ahybridfastinferenceapproachwithdistributedneuralnetworksfor.pdf, p.8]
        int totalStages = default(6);

        double planReqTimeout @unit(s) = default(30ms);
        bool autonomyEnabled = default(true);

        double posX @unit(m) = default(100m);
        double posY @unit(m) = default(100m);
        double velX @unit(mps) = default(0mps);
        double velY @unit(mps) = default(0mps);

        double batteryWh = default(10.0);                      // PLACEHOLDER
        double eTxWhPerMB = default(0.0005);                   // PLACEHOLDER
        double eCmpWhPerStage = default(0.0002);               // PLACEHOLDER

        @display("i=block/airplane");
    gates:
        output dataOutU1;
        output dataOutU2;
        output dataOutE0;

        input  ctrlIn;
        output ctrlOut;
}

simple PeerUav4
{
    parameters:
        int nodeId = default(1);
        double computePerStage @unit(s) = default(10ms);       // PLACEHOLDER

        double statusPeriod @unit(s) = default(0.1s);
        double heartbeatTimeout @unit(s) = default(0.3s);

        double posX @unit(m) = default(250m);
        double posY @unit(m) = default(250m);
        double velX @unit(mps) = default(1mps);
        double velY @unit(mps) = default(0mps);

        bool enableFailures = default(false);
        double downStart @unit(s) = default(40s);
        double downEnd @unit(s) = default(60s);

        double batteryWh = default(10.0);                      // PLACEHOLDER
        double eTxWhPerMB = default(0.0005);                   // PLACEHOLDER
        double eCmpWhPerStage = default(0.0002);               // PLACEHOLDER
        double lowBatteryWh = default(1.0);                    // PLACEHOLDER

        @display("i=block/airplane");
    gates:
        input  dataIn[];
        output dataOutPeer;
        output dataOutE0;

        input  ctrlIn;
        output ctrlOut;
}

simple EdgeCoord4
{
    parameters:
        string policy = default("AEGIS");                      // AEGIS LOCAL EDGE STATIC NAIVE
        double plannerDelay @unit(s) = default(2ms);

        double cellSize @unit(m) = default(250m);
        int neighborCells = default(1);

        double guardBand @unit(s) = default(20ms);             // PLACEHOLDER
        double replanCooldown @unit(s) = default(0.3s);         // PLACEHOLDER
        double hysteresisGain = default(0.10);                  // PLACEHOLDER

        double baseRate @unit(bps) = default(94Mbps);          // uplink capacity anchor [Toward 5G Edge Computing for Enabling Autonomous Aerial Vehicles.pdf, p.13]
        double alpha = default(0.002);                          // PLACEHOLDER distance rate decay
        double baseLoss = default(0.001);                       // PLACEHOLDER
        double betaLoss = default(0.00001);                     // PLACEHOLDER

        double featureShrinkMid = default(0.25);                // 75 percent reduction after pool5 like point [A_Fine-Grained_End-to-End_Latency_Optimization_Framework_for_Wireless_Collaborative_Inference.pdf, p.2]

        double computePerStage @unit(s) = default(8ms);         // PLACEHOLDER

        double statusPeriod @unit(s) = default(0.1s);
        double heartbeatTimeout @unit(s) = default(0.3s);

        bool enableEdgeOutage = default(false);
        double edgeDownStart @unit(s) = default(70s);
        double edgeDownEnd @unit(s) = default(90s);

        double posX @unit(m) = default(500m);
        double posY @unit(m) = default(250m);

        double batteryWh = default(100.0);                      // PLACEHOLDER
        double eTxWhPerMB = default(0.0002);                    // PLACEHOLDER
        double eCmpWhPerStage = default(0.0001);                // PLACEHOLDER

        @display("i=block/server");
    gates:
        input  dataIn[];
        input  ctrlIn[];
        output ctrlOut[];
}

channel DataChan extends ned.DatarateChannel
{
    parameters:
        delay = 5ms;                // PLACEHOLDER propagation
        datarate = 94Mbps;          // anchor [Toward 5G Edge Computing for Enabling Autonomous Aerial Vehicles.pdf, p.13]
}

channel CtrlChan extends ned.DelayChannel
{
    parameters:
        delay = 1ms;                // PLACEHOLDER
}