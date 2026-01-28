package uav_mec_sim01;

network Sim01Network
{
    submodules:
        u0: CaptureUav;
        u1: PeerUav;
        u2: PeerUav;
        e0: EdgeNode;
    connections:
        // Data-plane chain
        u0.dataOut --> DataChan --> u1.dataIn;
        u1.dataOut --> DataChan --> u2.dataIn;
        u2.dataOut --> DataChan --> e0.dataIn;

        // Control-plane broadcast from edge to UAVs (added in 01.6)
        e0.ctrlOut++ --> CtrlChan --> u0.ctrlIn;
        e0.ctrlOut++ --> CtrlChan --> u1.ctrlIn;
        e0.ctrlOut++ --> CtrlChan --> u2.ctrlIn;

}
