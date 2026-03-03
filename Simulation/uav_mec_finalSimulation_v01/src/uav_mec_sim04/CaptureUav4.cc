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
    cMessage *capDoneEvt = nullptr;
    cMessage *planTimeoutEvt = nullptr;
    cMessage *mobEvt = nullptr;

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
        capDoneEvt = new cMessage("CAPDONE");
        planTimeoutEvt = new cMessage("PLANTIMEOUT");
        mobEvt = new cMessage("MOB");

        posX = par("posX").doubleValue();
        posY = par("posY").doubleValue();
        velX = par("velX").doubleValue();
        velY = par("velY").doubleValue();

        batteryWh = par("batteryWh").doubleValue();

        scheduleAt(0.01, new cMessage("SENDHELLO"));
        scheduleAt(par("taskStartTime").doubleValue(), genEvt);
        scheduleAt(simTime() + 0.1, mobEvt);
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
        waitingTaskId = pkt->par("taskId").longValue();

        auto *req = new cMessage("PLANREQ");
        setLongPar(req, "taskId", waitingTaskId);
        setDoublePar(req, "deadline", pkt->par("deadline").doubleValue());
        setLongPar(req, "totalStages", pkt->par("totalStages").longValue());
        setLongPar(req, "bytes0", pkt->par("bytes0").longValue());
        setDoublePar(req, "tPlanReq", SIMTIME_DBL(simTime()));
        send(req, "ctrlOut");

        scheduleAt(simTime() + par("planReqTimeout").doubleValue(), planTimeoutEvt);
    }

    void autonomyStart(cPacket *pkt)
    {
        bool autonomyEnabled = par("autonomyEnabled").boolValue();
        if (!autonomyEnabled) {
            pkt->addPar("chainType") = 1; // edge only
            send(pkt, "dataOutE0");
            return;
        }

        // Simple autonomy ladder: prefer u1 then u2 then edge then local
        pkt->addPar("chainType") = 4;
        send(pkt, "dataOutU1");
    }

    void applyPlanToPacket(cPacket *pkt, const Plan &p)
    {
        pkt->addPar("chainType") = p.chainPrimary;
        pkt->addPar("chainFallback") = p.chainFallback;
        pkt->addPar("planVer") = p.planVer;

        pkt->addPar("u1_s") = p.u1_s; pkt->addPar("u1_e") = p.u1_e;
        pkt->addPar("u2_s") = p.u2_s; pkt->addPar("u2_e") = p.u2_e;

        pkt->addPar("predPrimary") = p.predPrimary;
        pkt->addPar("predFallback") = p.predFallback;

        pkt->addPar("tPlanRecv") = SIMTIME_DBL(simTime());
        pkt->addPar("t_send_u0") = SIMTIME_DBL(simTime());
    }

    void sendFirstHop(cPacket *pkt)
    {
        long chain = pkt->par("chainType").longValue();

        if (chain == 0) {
            // local only, modeled as local compute then send small result
            long totalStages = pkt->par("totalStages").longValue();
            double perStage = 0.012; // PLACEHOLDER
            pkt->addPar("t_cmpStart_u0") = SIMTIME_DBL(simTime());
            scheduleAt(simTime() + perStage * (double)totalStages, pkt);
            return;
        }

        if (chain == 1) { send(pkt, "dataOutE0"); return; }
        if (chain == 2 || chain == 4) { send(pkt, "dataOutU1"); return; }
        if (chain == 3 || chain == 5) { send(pkt, "dataOutU2"); return; }

        send(pkt, "dataOutE0");
    }

    void finishLocalCompute(cPacket *pkt)
    {
        pkt->addPar("t_cmpEnd_u0") = SIMTIME_DBL(simTime());

        // energy placeholder
        double eCmp = par("eCmpWhPerStage").doubleValue() * (double)pkt->par("totalStages").longValue();
        batteryWh -= eCmp;
        pkt->addPar("e_u0_cmpWh") = eCmp;

        pkt->setByteLength(1000);
        pkt->par("nextStage").setLongValue(pkt->par("totalStages").longValue() + 1);

        std::string tr = pkt->par("trace").stringValue();
        tr += "->u0local";
        pkt->par("trace").setStringValue(tr.c_str());

        pkt->addPar("t_send_u0_done") = SIMTIME_DBL(simTime());
        send(pkt, "dataOutE0");
    }

    void handlePlanMsg(cMessage *m)
    {
        long taskId = m->par("taskId").longValue();

        Plan p;
        p.chainPrimary = m->par("chainPrimary").longValue();
        p.chainFallback = m->par("chainFallback").longValue();
        p.u1_s = m->par("u1_s").longValue(); p.u1_e = m->par("u1_e").longValue();
        p.u2_s = m->par("u2_s").longValue(); p.u2_e = m->par("u2_e").longValue();
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

        cancelEvent(planTimeoutEvt);

        applyPlanToPacket(pkt, p);
        sendFirstHop(pkt);

        requestPlanForHead();
    }

    void handleMessage(cMessage *msg) override
    {
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
            createTask();
            double meanIa = par("meanInterArrival").doubleValue();
            scheduleAt(simTime() + exponential(meanIa), genEvt);
            return;
        }

        if (msg == capDoneEvt) {
            requestPlanForHead();
            return;
        }

        if (msg == planTimeoutEvt) {
            // edge unreachable for this task, autonomy
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
            // local compute done packet
            auto *pkt = check_and_cast<cPacket*>(msg);
            finishLocalCompute(pkt);
            requestPlanForHead();
            return;
        }

        delete msg;
    }

    void finish() override
    {
        recordScalar("u0_batteryWh_final", batteryWh);
        cancelAndDelete(genEvt);
        cancelAndDelete(capDoneEvt);
        cancelAndDelete(planTimeoutEvt);
        cancelAndDelete(mobEvt);

        while (!pending.empty()) { delete pending.front(); pending.pop(); }
    }
};

Define_Module(CaptureUav4);
