package uav_mec_sim02;

simple CaptureUav2
{
    parameters:
        double taskStartTime @unit(s) = default(1s);
        double meanInterArrival @unit(s) = default(0.5s);
        int imagesPerTask = default(5);
        int bytesPerImage = default(200000);
        double captureDelay @unit(s) = default(5ms);

        double deadline @unit(s) = default(1s);
        int totalStages = default(4);

        double posX @unit(m) = default(100m);
        double posY @unit(m) = default(100m);

        @display("i=block/airplane");
    gates:
        output dataOutU1;
        output dataOutU2;
        output dataOutE0;

        input  ctrlIn;
        output ctrlOut;
}

simple PeerUav2
{
    parameters:
        int nodeId = default(1);
        double computePerStage @unit(s) = default(10ms);

        double posX @unit(m) = default(250m);
        double posY @unit(m) = default(250m);

        @display("i=block/airplane");
    gates:
        input  dataIn[];
        output dataOutPeer;
        output dataOutE0;

        input  ctrlIn;
        output ctrlOut;
}

simple EdgeNode2
{
    parameters:
        string policy = default("AEGIS");    // AEGIS LOCAL EDGE STATIC NAIVE
        double plannerDelay @unit(s) = default(2ms);

        double cellSize @unit(m) = default(250m);
        int neighborCells = default(1);

        double baseRate @unit(bps) = default(100Mbps); // placeholder
        double alpha = default(0.002);                 // placeholder

        double computePerStage @unit(s) = default(8ms); // placeholder

        double posX @unit(m) = default(500m);
        double posY @unit(m) = default(500m);

        @display("i=block/server");
    gates:
        input  dataIn[];
        input  ctrlIn[];
        output ctrlOut[];
}

channel DataChan extends ned.DatarateChannel
{
    parameters:
        delay = 5ms;
        datarate = 100Mbps;
}

channel CtrlChan extends ned.DelayChannel
{
    parameters:
        delay = 1ms;
}