#include <omnetpp.h>
#include <queue>
#include <map>
#include <string>
#include <algorithm>
#include "Sim04Util.h"

using namespace omnetpp;

class CaptureUav4 : public cSimpleModule
{
  private:
    cMessage *genEvt = nullptr;
    cMessage *planTimeoutEvt = nullptr;
    cMessage *mobEvt = nullptr;

    cMessage *txEvt = nullptr;
    struct TxItem { cPacket *pkt; const char *gateName; };
    std::queue<TxItem> txq;

    long nextTaskId = 1;

    struct Plan {
        long chainPrimary = 1;
        long chainFallback = 0;
        long u1_s = 0, u1_e = -1;
        long u2_s = 0, u2_e = -1;
        long planVer = 0;
        double predPrimary = 0.0;
        double predFallback = 0.0;
    };

    std::queue<cPacket*> pending;
    std::map<long, Plan> planByTask;

    bool waitingPlan = false;
    long waitingTaskId = -1;

    double posX = 0.0, posY = 0.0;
    double velX = 0.0, velY = 0.0;

    double batteryWh = 0.0;

  protected:
    void initialize() override
    {
        genEvt = new cMessage("GEN");
        planTimeoutEvt = new cMessage("PLANTIMEOUT");
        mobEvt = new cMessage("MOB");
        txEvt = new cMessage("TX");

        posX = par("posX").doubleValue();
        posY = par("posY").doubleValue();
        velX = par("velX").doubleValue();
        velY = par("velY").doubleValue();
        batteryWh = par("batteryWh").doubleValue();

        scheduleAt(0.01, new cMessage("SENDHELLO"));
        scheduleAt(par("taskStartTime").doubleValue(), genEvt);
        scheduleAt(simTime() + 0.1, mobEvt);
    }

    simtime_t txFinishTime(const char *gateName)
    {
        cGate *g = gate(gateName);
        cChannel *ch = g->getChannel();
        auto *dc = dynamic_cast<cDatarateChannel*>(ch);
        if (!dc)
            return simTime(); // treat non datarate channels as free
        return dc->getTransmissionFinishTime();
    }

    void scheduleTxTick(simtime_t t)
    {
        if (txEvt->isScheduled())
            cancelEvent(txEvt);
        scheduleAt(t, txEvt);
    }

    void enqueueTx(cPacket *pkt, const char *gateName)
    {
        txq.push(TxItem{pkt, gateName});

        simtime_t eps = SimTime(1, SIMTIME_US);
        simtime_t tf = txFinishTime(gateName);

        if (tf > simTime())
            scheduleTxTick(tf + eps);
        else
            scheduleTxTick(simTime() + eps);
    }

    void tryTxOne()
    {
        if (txq.empty())
            return;

        TxItem it = txq.front();
        simtime_t eps = SimTime(1, SIMTIME_US);
        simtime_t tf = txFinishTime(it.gateName);

        if (tf > simTime()) {
            scheduleTxTick(tf + eps);
            return;
        }

        txq.pop();
        send(it.pkt, it.gateName);

        if (!txq.empty())
            scheduleTxTick(simTime() + eps);
    }

    void sendHello()
    {
        auto *m = new cMessage("HELLO");
        setLongPar(m, "nodeId", 0);
        setDoublePar(m, "posX", posX);
        setDoublePar(m, "posY", posY);
        setDoublePar(m, "computePerStage", 0.0);
        setDoublePar(m, "batteryWh", batteryWh);
        send(m, "ctrlOut");
    }

    void tickMobility()
    {
        double dt = 0.1;
        posX += velX * dt;
        posY += velY * dt;

        scheduleAt(simTime() + dt, mobEvt);

        auto *m = new cMessage("STATUS");
        setLongPar(m, "nodeId", 0);
        setDoublePar(m, "posX", posX);
        setDoublePar(m, "posY", posY);
        setLongPar(m, "qLen", (long)pending.size());
        setDoublePar(m, "batteryWh", batteryWh);
        send(m, "ctrlOut");
    }

    void createTaskAndStartCapture()
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

        pkt->addPar("selfKind") = 1; // 1 capture done, 2 local compute done
        scheduleAt(simTime() + par("captureDelay").doubleValue(), pkt);
    }

    void requestPlanForHead()
    {
        if (pending.empty()) return;
        if (waitingPlan) return;

        auto *pkt = pending.front();
        waitingPlan = true;
        waitingTaskId = pkt->par("taskId").longValue();

        auto *req = new cMessage("PLANREQ");
        setLongPar(req, "taskId", waitingTaskId);
        setDoublePar(req, "deadline", pkt->par("deadline").doubleValue());
        setLongPar(req, "totalStages", pkt->par("totalStages").longValue());
        setLongPar(req, "bytes0", pkt->par("bytes0").longValue());
        setDoublePar(req, "tPlanReq", SIMTIME_DBL(simTime()));
        send(req, "ctrlOut");

        if (planTimeoutEvt->isScheduled())
            cancelEvent(planTimeoutEvt);
        scheduleAt(simTime() + par("planReqTimeout").doubleValue(), planTimeoutEvt);
    }

    void autonomyStart(cPacket *pkt)
    {
        bool autonomyEnabled = par("autonomyEnabled").boolValue();
        pkt->addPar("t_send_u0") = SIMTIME_DBL(simTime());

        if (!autonomyEnabled) {
            pkt->addPar("chainType") = 1;
            enqueueTx(pkt, "dataOutE0");
            return;
        }

        pkt->addPar("chainType") = 4;
        enqueueTx(pkt, "dataOutU1");
    }

    void applyPlanToPacket(cPacket *pkt, const Plan &p)
    {
        pkt->addPar("chainType") = p.chainPrimary;
        pkt->addPar("chainFallback") = p.chainFallback;
        pkt->addPar("planVer") = p.planVer;

        pkt->addPar("u1_s") = p.u1_s;
        pkt->addPar("u1_e") = p.u1_e;
        pkt->addPar("u2_s") = p.u2_s;
        pkt->addPar("u2_e") = p.u2_e;

        pkt->addPar("predPrimary") = p.predPrimary;
        pkt->addPar("predFallback") = p.predFallback;

        pkt->addPar("tPlanRecv") = SIMTIME_DBL(simTime());
    }

    void sendFirstHop(cPacket *pkt)
    {
        long chain = pkt->par("chainType").longValue();

        if (chain == 0) {
            long totalStages = pkt->par("totalStages").longValue();
            double perStage = 0.012; // placeholder
            pkt->addPar("t_cmpStart_u0") = SIMTIME_DBL(simTime());
            pkt->par("selfKind").setLongValue(2);
            scheduleAt(simTime() + perStage * (double)totalStages, pkt);
            return;
        }

        pkt->addPar("t_send_u0") = SIMTIME_DBL(simTime());

        if (chain == 1) { enqueueTx(pkt, "dataOutE0"); return; }
        if (chain == 2 || chain == 4) { enqueueTx(pkt, "dataOutU1"); return; }
        if (chain == 3 || chain == 5) { enqueueTx(pkt, "dataOutU2"); return; }

        enqueueTx(pkt, "dataOutE0");
    }

    void finishLocalCompute(cPacket *pkt)
    {
        pkt->addPar("t_cmpEnd_u0") = SIMTIME_DBL(simTime());

        double eCmp = par("eCmpWhPerStage").doubleValue() * (double)pkt->par("totalStages").longValue();
        batteryWh -= eCmp;
        pkt->addPar("e_u0_cmpWh") = eCmp;

        pkt->setByteLength(1000);
        pkt->par("nextStage").setLongValue(pkt->par("totalStages").longValue() + 1);

        std::string tr = pkt->par("trace").stringValue();
        tr += "->u0local";
        pkt->par("trace").setStringValue(tr.c_str());

        pkt->addPar("t_send_u0_done") = SIMTIME_DBL(simTime());
        enqueueTx(pkt, "dataOutE0");
    }

    void handlePlanMsg(cMessage *m)
    {
        long taskId = m->par("taskId").longValue();

        Plan p;
        p.chainPrimary = m->par("chainPrimary").longValue();
        p.chainFallback = m->par("chainFallback").longValue();
        p.u1_s = m->par("u1_s").longValue();
        p.u1_e = m->par("u1_e").longValue();
        p.u2_s = m->par("u2_s").longValue();
        p.u2_e = m->par("u2_e").longValue();
        p.planVer = m->par("planVer").longValue();
        p.predPrimary = m->par("predPrimary").doubleValue();
        p.predFallback = m->par("predFallback").doubleValue();

        planByTask[taskId] = p;
        delete m;

        if (!waitingPlan) return;
        if (taskId != waitingTaskId) return;
        if (pending.empty()) { waitingPlan = false; return; }

        auto *pkt = pending.front();
        if (pkt->par("taskId").longValue() != taskId) { waitingPlan = false; return; }

        pending.pop();
        waitingPlan = false;

        if (planTimeoutEvt->isScheduled())
            cancelEvent(planTimeoutEvt);

        applyPlanToPacket(pkt, p);
        sendFirstHop(pkt);

        requestPlanForHead();
    }

    void handleCaptureDone(cPacket *pkt)
    {
        pkt->par("selfKind").setLongValue(0);
        pending.push(pkt);
        requestPlanForHead();
    }

    void handleMessage(cMessage *msg) override
    {
        if (msg == txEvt) { tryTxOne(); return; }
        if (msg == mobEvt) { tickMobility(); return; }

        if (!strcmp(msg->getName(), "SENDHELLO")) {
            delete msg;
            sendHello();
            return;
        }

        if (msg->arrivedOn("ctrlIn")) {
            if (!strcmp(msg->getName(), "PLAN") || !strcmp(msg->getName(), "REPLAN")) {
                handlePlanMsg(msg);
                return;
            }
            delete msg;
            return;
        }

        if (msg == genEvt) {
            createTaskAndStartCapture();
            double meanIa = par("meanInterArrival").doubleValue();
            scheduleAt(simTime() + exponential(meanIa), genEvt);
            return;
        }

        if (msg == planTimeoutEvt) {
            if (!pending.empty() && waitingPlan) {
                auto *pkt = pending.front();
                pending.pop();
                waitingPlan = false;
                autonomyStart(pkt);
                requestPlanForHead();
            }
            return;
        }

        if (msg->isSelfMessage()) {
            auto *pkt = check_and_cast<cPacket*>(msg);
            long kind = pkt->hasPar("selfKind") ? pkt->par("selfKind").longValue() : 0;

            if (kind == 1) { handleCaptureDone(pkt); return; }
            if (kind == 2) { finishLocalCompute(pkt); requestPlanForHead(); return; }

            delete pkt;
            return;
        }

        delete msg;
    }

    void finish() override
    {
        recordScalar("u0_batteryWh_final", batteryWh);

        cancelAndDelete(genEvt);
        cancelAndDelete(planTimeoutEvt);
        cancelAndDelete(mobEvt);
        cancelAndDelete(txEvt);

        while (!pending.empty()) { delete pending.front(); pending.pop(); }
        while (!txq.empty()) { delete txq.front().pkt; txq.pop(); }
    }
};

Define_Module(CaptureUav4);
