#include <omnetpp.h>
#include <map>
#include <queue>
#include <string>
#include <algorithm>

using namespace omnetpp;

static void setLongPar(cMessage *m, const char *name, long v) {
    if (!m->hasPar(name)) m->addPar(name) = v;
    else m->par(name).setLongValue(v);
}
static void setDoublePar(cMessage *m, const char *name, double v) {
    if (!m->hasPar(name)) m->addPar(name) = v;
    else m->par(name).setDoubleValue(v);
}

class PeerUav2 : public cSimpleModule
{
  private:
    struct Range { long s; long e; };
    std::map<long, Range> planByTask;

    std::queue<cPacket*> q;
    bool busy = false;
    cPacket *current = nullptr;

    // transmit queues
    cQueue txQ_peer;
    cQueue txQ_e0;
    cMessage *txEvt_peer = nullptr;
    cMessage *txEvt_e0 = nullptr;

  protected:
    void initialize() override
    {
        txQ_peer.setName("txQ_peer");
        txQ_e0.setName("txQ_e0");
        txEvt_peer = new cMessage("TX_peer");
        txEvt_e0 = new cMessage("TX_e0");

        scheduleAt(0.02, new cMessage("SENDHELLO"));
    }

    void sendHello()
    {
        auto *m = new cMessage("HELLO");
        setLongPar(m, "nodeId", (long)par("nodeId").intValue());
        setDoublePar(m, "posX", par("posX").doubleValue());
        setDoublePar(m, "posY", par("posY").doubleValue());
        setDoublePar(m, "computePerStage", par("computePerStage").doubleValue());
        send(m, "ctrlOut");
    }

    void flushTx(const char *gateName, cQueue &qtx, cMessage *evt)
    {
        if (qtx.isEmpty()) return;

        cGate *g = gate(gateName);
        simtime_t fin = g->getTransmissionFinishTime();

        if (simTime() < fin) {
            if (!evt->isScheduled()) scheduleAt(fin, evt);
            return;
        }

        auto *pkt = check_and_cast<cPacket*>(qtx.pop());
        send(pkt, gateName);

        if (!qtx.isEmpty()) scheduleAt(g->getTransmissionFinishTime(), evt);
    }

    void enqueueTx(cPacket *pkt, const char *gateName, cQueue &qtx, cMessage *evt)
    {
        cGate *g = gate(gateName);
        simtime_t fin = g->getTransmissionFinishTime();

        if (simTime() >= fin && qtx.isEmpty()) {
            send(pkt, gateName);
            return;
        }

        qtx.insert(pkt);
        if (!evt->isScheduled()) scheduleAt(fin, evt);
    }

    void handlePlan(cMessage *plan)
    {
        long taskId = plan->par("taskId").longValue();

        long id = (long)par("nodeId").intValue();
        long s = 0, e = -1;
        if (id == 1) { s = plan->par("u1_s").longValue(); e = plan->par("u1_e").longValue(); }
        if (id == 2) { s = plan->par("u2_s").longValue(); e = plan->par("u2_e").longValue(); }

        planByTask[taskId] = Range{s,e};

        delete plan;
    }

    void sendToNext(cPacket *pkt)
    {
        long chainType = pkt->par("chainType").longValue();
        std::string me = getFullName(); // u1 or u2

        pkt->addPar((std::string("t_send_") + me).c_str()) = SIMTIME_DBL(simTime());

        std::string tr = pkt->par("trace").stringValue();
        tr += "->" + me;
        pkt->par("trace").setStringValue(tr.c_str());

        if (chainType == 2) {
            if (me == "u1") enqueueTx(pkt, "dataOutPeer", txQ_peer, txEvt_peer);
            else enqueueTx(pkt, "dataOutE0", txQ_e0, txEvt_e0);
            return;
        }
        if (chainType == 3) {
            if (me == "u2") enqueueTx(pkt, "dataOutPeer", txQ_peer, txEvt_peer);
            else enqueueTx(pkt, "dataOutE0", txQ_e0, txEvt_e0);
            return;
        }
        enqueueTx(pkt, "dataOutE0", txQ_e0, txEvt_e0);
    }

    void startNext()
    {
        if (busy) return;
        if (q.empty()) return;

        current = q.front();
        q.pop();

        long taskId = current->par("taskId").longValue();
        long nextStage = current->par("nextStage").longValue();

        Range r{0,-1};
        auto it = planByTask.find(taskId);
        if (it != planByTask.end()) r = it->second;

        bool canProcess = (r.e >= r.s) && (nextStage >= r.s) && (nextStage <= r.e);

        current->addPar((std::string("t_arr_") + getFullName()).c_str()) = SIMTIME_DBL(simTime());

        if (!canProcess) {
            sendToNext(current);
            current = nullptr;
            startNext();
            return;
        }

        busy = true;

        long stages = (r.e - r.s + 1);
        double t = par("computePerStage").doubleValue() * (double)stages;

        scheduleAt(simTime() + t, current);
    }

    void completeCompute(cPacket *pkt)
    {
        long taskId = pkt->par("taskId").longValue();
        Range r{0,-1};
        auto it = planByTask.find(taskId);
        if (it != planByTask.end()) r = it->second;

        if (r.e >= r.s) {
            pkt->par("nextStage").setLongValue(r.e + 1);
            int before = pkt->getByteLength();
            int after = std::max(1000, (int)(before * 0.8));
            pkt->setByteLength(after);
        }

        sendToNext(pkt);

        busy = false;
        current = nullptr;
        startNext();
    }

    void handleMessage(cMessage *msg) override
    {
        if (msg == txEvt_peer) { flushTx("dataOutPeer", txQ_peer, txEvt_peer); return; }
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

        if (msg->isSelfMessage()) {
            auto *pkt = check_and_cast<cPacket*>(msg);
            completeCompute(pkt);
            return;
        }

        auto *pkt = check_and_cast<cPacket*>(msg);
        q.push(pkt);
        startNext();
    }

    void finish() override
    {
        cancelEvent(txEvt_peer);
        cancelEvent(txEvt_e0);
        delete txEvt_peer;
        delete txEvt_e0;

        while (!txQ_peer.isEmpty()) delete txQ_peer.pop();
        while (!txQ_e0.isEmpty()) delete txQ_e0.pop();
    }
};

Define_Module(PeerUav2);
