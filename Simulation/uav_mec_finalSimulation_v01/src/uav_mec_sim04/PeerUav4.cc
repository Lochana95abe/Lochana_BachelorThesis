#include <omnetpp.h>
#include <map>
#include <queue>
#include <string>
#include <algorithm>
#include "Sim04Util.h"

using namespace omnetpp;

class PeerUav4 : public cSimpleModule
{
  private:
    cMessage *statusEvt = nullptr;
    cMessage *mobEvt = nullptr;

    struct Range { long s; long e; long planVer; };
    std::map<long, Range> rangeByTask;
    std::map<long, long> fallbackByTask;

    std::queue<cPacket*> q;
    bool busy = false;
    cPacket *current = nullptr;

    double posX = 0.0, posY = 0.0;
    double velX = 0.0, velY = 0.0;

    bool enableFailures = false;
    bool isDown = false;

    double batteryWh = 0.0;

  protected:
    void initialize() override
    {
        statusEvt = new cMessage("STATUS_TICK");
        mobEvt = new cMessage("MOB");

        posX = par("posX").doubleValue();
        posY = par("posY").doubleValue();
        velX = par("velX").doubleValue();
        velY = par("velY").doubleValue();

        enableFailures = par("enableFailures").boolValue();
        batteryWh = par("batteryWh").doubleValue();

        scheduleAt(0.02, new cMessage("SENDHELLO"));
        scheduleAt(simTime() + par("statusPeriod").doubleValue(), statusEvt);
        scheduleAt(simTime() + 0.1, mobEvt);
    }

    void sendHello()
    {
        auto *m = new cMessage("HELLO");
        setLongPar(m, "nodeId", (long)par("nodeId").intValue());
        setDoublePar(m, "posX", posX);
        setDoublePar(m, "posY", posY);
        setDoublePar(m, "computePerStage", par("computePerStage").doubleValue());
        setDoublePar(m, "batteryWh", batteryWh);
        send(m, "ctrlOut");
    }

    void tickMobility()
    {
        double dt = 0.1;
        posX += velX * dt;
        posY += velY * dt;
        scheduleAt(simTime() + dt, mobEvt);
    }

    void tickStatus()
    {
        if (enableFailures) {
            double ds = par("downStart").doubleValue();
            double de = par("downEnd").doubleValue();
            isDown = (simTime().dbl() >= ds && simTime().dbl() <= de);
        }

        auto *m = new cMessage("STATUS");
        setLongPar(m, "nodeId", (long)par("nodeId").intValue());
        setDoublePar(m, "posX", posX);
        setDoublePar(m, "posY", posY);
        setLongPar(m, "qLen", (long)q.size() + (busy ? 1 : 0));
        setDoublePar(m, "batteryWh", batteryWh);
        setLongPar(m, "isDown", isDown ? 1 : 0);
        send(m, "ctrlOut");

        scheduleAt(simTime() + par("statusPeriod").doubleValue(), statusEvt);
    }

    void handlePlan(cMessage *m)
    {
        long taskId = m->par("taskId").longValue();
        long planVer = m->par("planVer").longValue();

        long id = (long)par("nodeId").intValue();
        long s = 0, e = -1;

        if (id == 1) { s = m->par("u1_s").longValue(); e = m->par("u1_e").longValue(); }
        if (id == 2) { s = m->par("u2_s").longValue(); e = m->par("u2_e").longValue(); }

        rangeByTask[taskId] = Range{s,e,planVer};

        // store fallback chain for local decisions
        fallbackByTask[taskId] = m->par("chainFallback").longValue();

        delete m;
    }

    bool canProcess(long taskId, long nextStage) const
    {
        auto it = rangeByTask.find(taskId);
        if (it == rangeByTask.end()) return false;
        Range r = it->second;
        if (r.e < r.s) return false;
        return nextStage >= r.s && nextStage <= r.e;
    }

    void applyComputeAndForward(cPacket *pkt)
    {
        long taskId = pkt->par("taskId").longValue();
        long nextStage = pkt->par("nextStage").longValue();

        if (isDown) {
            delete pkt;
            return;
        }

        if (!canProcess(taskId, nextStage)) {
            forward(pkt);
            return;
        }

        long id = (long)par("nodeId").intValue();
        Range r = rangeByTask[taskId];
        long stages = r.e - r.s + 1;

        // queue already handled by serialized processing
        busy = true;
        current = pkt;

        pkt->addPar((std::string("t_cmpStart_u") + std::to_string(id)).c_str()) = SIMTIME_DBL(simTime());

        double t = par("computePerStage").doubleValue() * (double)stages;
        scheduleAt(simTime() + t, pkt);
    }

    void completeCompute(cPacket *pkt)
    {
        long taskId = pkt->par("taskId").longValue();
        long id = (long)par("nodeId").intValue();

        pkt->addPar((std::string("t_cmpEnd_u") + std::to_string(id)).c_str()) = SIMTIME_DBL(simTime());

        Range r = rangeByTask[taskId];

        // stage progress update
        pkt->par("nextStage").setLongValue(r.e + 1);

        // energy placeholder
        double eCmp = par("eCmpWhPerStage").doubleValue() * (double)(r.e - r.s + 1);
        batteryWh -= eCmp;
        pkt->addPar((std::string("e_u") + std::to_string(id) + "_cmpWh").c_str()) = eCmp;

        // feature shrink behavior: mid tensor becomes smaller, use 0.25 once (pool5 like) [A_Fine-Grained_End-to-End_Latency_Optimization_Framework_for_Wireless_Collaborative_Inference.pdf, p.2]
        long totalStages = pkt->par("totalStages").longValue();
        long mid = totalStages / 2;
        if (r.e >= mid) {
            int before = pkt->getByteLength();
            int after = std::max(1000, (int)(before * 0.25));
            pkt->setByteLength(after);
        } else {
            int before = pkt->getByteLength();
            int after = std::max(1000, (int)(before * 0.8));
            pkt->setByteLength(after);
        }

        forward(pkt);

        busy = false;
        current = nullptr;
        if (!q.empty()) {
            auto *n = q.front(); q.pop();
            applyComputeAndForward(n);
        }
    }

    void forward(cPacket *pkt)
    {
        long chain = pkt->par("chainType").longValue();
        long fallback = pkt->hasPar("chainFallback") ? pkt->par("chainFallback").longValue() : 0;

        std::string me = getFullName();

        // trace
        std::string tr = pkt->par("trace").stringValue();
        tr += "->" + me;
        pkt->par("trace").setStringValue(tr.c_str());

        // low battery guardrail: if battery low, skip peer to edge directly
        if (batteryWh <= par("lowBatteryWh").doubleValue()) {
            send(pkt, "dataOutE0");
            return;
        }

        // primary routing
        if (chain == 2) {
            if (me == "u1") { send(pkt, "dataOutPeer"); return; }
            send(pkt, "dataOutE0"); return;
        }
        if (chain == 3) {
            if (me == "u2") { send(pkt, "dataOutPeer"); return; }
            send(pkt, "dataOutE0"); return;
        }
        if (chain == 4 || chain == 5) {
            send(pkt, "dataOutE0"); return;
        }

        // fallback routing if unknown
        if (fallback == 2) {
            if (me == "u1") { send(pkt, "dataOutPeer"); return; }
            send(pkt, "dataOutE0"); return;
        }
        if (fallback == 3) {
            if (me == "u2") { send(pkt, "dataOutPeer"); return; }
            send(pkt, "dataOutE0"); return;
        }

        send(pkt, "dataOutE0");
    }

    void handleMessage(cMessage *msg) override
    {
        if (msg == mobEvt) { tickMobility(); return; }
        if (msg == statusEvt) { tickStatus(); return; }

        if (!strcmp(msg->getName(), "SENDHELLO")) {
            delete msg;
            sendHello();
            return;
        }

        if (msg->arrivedOn("ctrlIn")) {
            if (!strcmp(msg->getName(), "PLAN") || !strcmp(msg->getName(), "REPLAN")) {
                handlePlan(msg);
                return;
            }
            delete msg;
            return;
        }

        if (msg->isSelfMessage()) {
            auto *pkt = check_and_cast<cPacket*>(msg);
            completeCompute(pkt);
            return;
        }

        auto *pkt = check_and_cast<cPacket*>(msg);
        if (busy) q.push(pkt);
        else applyComputeAndForward(pkt);
    }

    void finish() override
    {
        recordScalar((std::string(getFullName()) + "_batteryWh_final").c_str(), batteryWh);
        cancelAndDelete(statusEvt);
        cancelAndDelete(mobEvt);
    }
};

Define_Module(PeerUav4);
