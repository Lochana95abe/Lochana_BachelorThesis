package uav_mec_sim01;

simple EdgeNode
{
    parameters:
        double planTime @unit(s) = default(0.2s); // when edge broadcasts the plan
    gates:
        input dataIn;
        output ctrlOut[];   // vector output: allows multiple outgoing ctrl connections

}


simple PeerUav
{
    parameters:
        double computeDelay @unit(s) = default(20ms); // deterministic service time in Sim01
    gates:
    input dataIn;
    output dataOut;
    input ctrlIn;

}



channel DataChan extends ned.DatarateChannel
{
    parameters:
        delay = 5ms;        // propagation/processing delay on the link
        datarate = 100Mbps; // link capacity (transmission time depends on packet size)
}

simple CaptureUav
{
    parameters:
        double taskStartTime @unit(s) = default(1s);
    gates:
    output dataOut;
    input ctrlIn;

}

channel CtrlChan extends ned.DelayChannel
{
    parameters:
        delay = 1ms; // small control-plane delay
}


