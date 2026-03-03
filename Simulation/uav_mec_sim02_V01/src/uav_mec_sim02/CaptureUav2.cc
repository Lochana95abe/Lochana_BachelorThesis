#include <omnetpp.h>
#include <queue>
#include <string>

using namespace omnetpp;

static void setLongPar(cMessage *m, const char *name, long v) {
    if (!m->hasPar(name)) m->addPar(name) = v;
    else m->par(name).setLongValue(v);
}
static void setDoublePar(cMessage *m, const char *name, double v) {
    if (!m->hasPar(name)) m->addPar(name) = v;
    else m->par(name).setDoubleValue(v);
}

class CaptureUav2 : public cSimpleModule
{
  private:
    cMessage *genEvt = nullptr;
    cMessage *capDoneEvt = nullptr;

    long nextTaskId = 1;

    std::queue<cPacket*> pending;
    bool waitingPlan = false;

    // transmit queues per output gate
    cQueue txQ_u1;
    cQueue txQ_u2;
    cQueue txQ_e0;

    cMessage *txEvt_u1 = nullptr;
    cMessage *txEvt_u2 = nullptr;
    cMessage *txEvt_e0 = nullptr;

  protected:
    void initialize() override
    {
        genEvt = new cMessage("GEN");
        capDoneEvt = new cMessage("CAPDONE");

        txQ_u1.setName("txQ_u1");
        txQ_u2.setName("txQ_u2");
        txQ_e0.setName("txQ_e0");

        txEvt_u1 = new cMessage("TX_u1");
        txEvt_u2 = new cMessage("TX_u2");
        txEvt_e0 = new cMessage("TX_e0");

        scheduleAt(0.01, new cMessage("SENDHELLO"));
        scheduleAt(par("taskStartTime").doubleValue(), genEvt);
    }

    void sendHello()
    {
        auto *m = new cMessage("HELLO");
        setLongPar(m, "nodeId", 0);
        setDoublePar(m, "posX", par("posX").doubleValue());
        setDoublePar(m, "posY", par("posY").doubleValue());
        setDoublePar(m, "computePerStage", 0.0);
        send(m, "ctrlOut");
    }

    void flushTx(const char *gateName, cQueue &q, cMessage *evt)
    {
        if (q.isEmpty()) return;

        cGate *g = gate(gateName);
        simtime_t fin = g->getTransmissionFinishTime();

        if (simTime() < fin) {
            if (!evt->isScheduled()) scheduleAt(fin, evt);
            return;
        }

        auto *pkt = check_and_cast<cPacket*>(q.pop());
        send(pkt, gateName);

        if (!q.isEmpty()) {
            scheduleAt(g->getTransmissionFinishTime(), evt);
        }
    }

    void enqueueTx(cPacket *pkt, const char *gateName, cQueue &q, cMessage *evt)
    {
        cGate *g = gate(gateName);
        simtime_t fin = g->getTransmissionFinishTime();

        if (simTime() >= fin && q.isEmpty()) {
            send(pkt, gateName);
            return;
        }

        q.insert(pkt);

        if (!evt->isScheduled()) {
            scheduleAt(fin, evt);
        }
    }

    void createTask()
    {
        auto *pkt = new cPacket("TENSOR");

        long taskId = nextTaskId++;
        long totalStages = (long)par("totalStages").intValue();

        long imagesPerTask = (long)par("imagesPerTask").intValue();
        long bytesPerImage = (long)par("bytesPerImage").intValue();
        long bytes0 = imagesPerTask * bytesPerImage;

        pkt->setByteLength((int)bytes0);
        pkt->setTimestamp();

        pkt->addPar("taskId") = taskId;
        pkt->addPar("deadline") = par("deadline").doubleValue();
        pkt->addPar("totalStages") = totalStages;
        pkt->addPar("nextStage") = 1;
        pkt->addPar("bytes0") = bytes0;
        pkt->addPar("trace").setStringValue("U0");

        pkt->addPar("tCapStart") = SIMTIME_DBL(simTime());

        pending.push(pkt);

        scheduleAt(simTime() + par("captureDelay").doubleValue(), capDoneEvt);
    }

    void requestPlanForHead()
    {
        if (pending.empty()) return;
        if (waitingPlan) return;

        auto *pkt = pending.front();
        waitingPlan = true;

        auto *req = new cMessage("PLANREQ");
        setLongPar(req, "taskId", pkt->par("taskId").longValue());
        setDoublePar(req, "deadline", pkt->par("deadline").doubleValue());
        setLongPar(req, "totalStages", pkt->par("totalStages").longValue());
        setLongPar(req, "bytes0", pkt->par("bytes0").longValue());
        setDoublePar(req, "tPlanReq", SIMTIME_DBL(simTime()));
        send(req, "ctrlOut");
    }

    void startLocalCompute(cPacket *pkt)
    {
        long totalStages = pkt->par("totalStages").longValue();
        double perStage = 0.012; // placeholder seconds per stage
        pkt->addPar("t_cmpStart_u0") = SIMTIME_DBL(simTime());
        scheduleAt(simTime() + perStage * (double)totalStages, pkt);
    }

    void finishLocalCompute(cPacket *pkt)
    {
        pkt->addPar("t_cmpEnd_u0") = SIMTIME_DBL(simTime());

        pkt->setByteLength(1000);
        pkt->par("nextStage").setLongValue(pkt->par("totalStages").longValue() + 1);

        std::string tr = pkt->par("trace").stringValue();
        tr += "->u0local";
        pkt->par("trace").setStringValue(tr.c_str());

        pkt->addPar("t_send_u0") = SIMTIME_DBL(simTime());

        enqueueTx(pkt, "dataOutE0", txQ_e0, txEvt_e0);
    }

    void handlePlan(cMessage *plan)
    {
        long taskId = plan->par("taskId").longValue();
        long chainType = plan->par("chainType").longValue();
        delete plan;

        if (pending.empty()) { waitingPlan = false; return; }

        auto *pkt = pending.front();
        if (pkt->par("taskId").longValue() != taskId) { waitingPlan = false; return; }

        pending.pop();
        waitingPlan = false;

        pkt->addPar("chainType") = chainType;
        pkt->addPar("tPlanRecv") = SIMTIME_DBL(simTime());
        pkt->addPar("t_send_u0") = SIMTIME_DBL(simTime());

        if (chainType == 0) {
            pkt->addPar("localMode") = 1;
            startLocalCompute(pkt);
            requestPlanForHead();
            return;
        }

        if (chainType == 1) enqueueTx(pkt, "dataOutE0", txQ_e0, txEvt_e0);
        else if (chainType == 2 || chainType == 4) enqueueTx(pkt, "dataOutU1", txQ_u1, txEvt_u1);
        else if (chainType == 3 || chainType == 5) enqueueTx(pkt, "dataOutU2", txQ_u2, txEvt_u2);
        else enqueueTx(pkt, "dataOutE0", txQ_e0, txEvt_e0);

        requestPlanForHead();
    }

    void handleMessage(cMessage *msg) override
    {
        // tx events
        if (msg == txEvt_u1) { flushTx("dataOutU1", txQ_u1, txEvt_u1); return; }
        if (msg == txEvt_u2) { flushTx("dataOutU2", txQ_u2, txEvt_u2); return; }
        if (msg == txEvt_e0) { flushTx("dataOutE0", txQ_e0, txEvt_e0); return; }

        if (!strcmp(msg->getName(), "SENDHELLO")) {
            delete msg;
            sendHello();
            return;
        }

        if (msg->arrivedOn("ctrlIn")) {
            if (!strcmp(msg->getName(), "PLAN")) { handlePlan(msg); return; }
            delete msg;
            return;
        }

        if (msg == genEvt) {
            createTask();
            double meanIa = par("meanInterArrival").doubleValue();
            scheduleAt(simTime() + exponential(meanIa), genEvt);
            return;
        }

        if (msg == capDoneEvt) {
            requestPlanForHead();
            return;
        }

        if (auto *pkt = dynamic_cast<cPacket*>(msg)) {
            if (pkt->hasPar("localMode") && pkt->par("localMode").longValue() == 1) {
                finishLocalCompute(pkt);
                requestPlanForHead();
                return;
            }
        }

        delete msg;
    }

    void finish() override
    {
        cancelAndDelete(genEvt);
        cancelAndDelete(capDoneEvt);

        cancelEvent(txEvt_u1);
        cancelEvent(txEvt_u2);
        cancelEvent(txEvt_e0);
        delete txEvt_u1;
        delete txEvt_u2;
        delete txEvt_e0;

        while (!txQ_u1.isEmpty()) delete txQ_u1.pop();
        while (!txQ_u2.isEmpty()) delete txQ_u2.pop();
        while (!txQ_e0.isEmpty()) delete txQ_e0.pop();

        while (!pending.empty()) {
            delete pending.front();
            pending.pop();
        }
    }
};

Define_Module(CaptureUav2);
