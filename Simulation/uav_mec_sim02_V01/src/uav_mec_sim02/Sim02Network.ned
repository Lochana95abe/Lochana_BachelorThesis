package uav_mec_sim02;

network Sim02Network
{
    parameters:
        @display("bgb=1000,550");
    submodules:
        u0: CaptureUav2 { @display("p=120,280"); }
        u1: PeerUav2 { parameters: nodeId = 1; @display("p=360,180"); }
        u2: PeerUav2 { parameters: nodeId = 2; @display("p=360,380"); }
        e0: EdgeNode2 { @display("p=780,280"); }

    connections:
        // data plane from u0
        u0.dataOutU1 --> DataChan --> u1.dataIn++;
        u0.dataOutU2 --> DataChan --> u2.dataIn++;
        u0.dataOutE0 --> DataChan --> e0.dataIn++;

        // peer to peer
        u1.dataOutPeer --> DataChan --> u2.dataIn++;
        u2.dataOutPeer --> DataChan --> u1.dataIn++;

        // peer to edge
        u1.dataOutE0 --> DataChan --> e0.dataIn++;
        u2.dataOutE0 --> DataChan --> e0.dataIn++;

        // control uplink
        u0.ctrlOut --> CtrlChan --> e0.ctrlIn++;
        u1.ctrlOut --> CtrlChan --> e0.ctrlIn++;
        u2.ctrlOut --> CtrlChan --> e0.ctrlIn++;

        // control downlink broadcast
        e0.ctrlOut++ --> CtrlChan --> u0.ctrlIn;
        e0.ctrlOut++ --> CtrlChan --> u1.ctrlIn;
        e0.ctrlOut++ --> CtrlChan --> u2.ctrlIn;
}