#include <omnetpp.h>
#include <map>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include "Sim04Util.h"

using namespace omnetpp;

struct NodeState4 {
    bool known = false;
    bool isDown = false;
    double x = 0.0;
    double y = 0.0;
    double computePerStage = 0.0;
    long qLen = 0;
    double batteryWh = 0.0;
    double lastSeen = -1.0;
};

class EdgeCoord4 : public cSimpleModule
{
  private:
    std::map<long, NodeState4> ns; // 0 u0, 1 u1, 2 u2

    cMessage *tickEvt = nullptr;

    struct TaskMeta {
        long planVer = 0;
        double lastReplan = -1.0;
        long replans = 0;
        double predPrimary = 0.0;
        double predFallback = 0.0;
        long chainPrimary = 1;
        long chainFallback = 0;
    };
    std::map<long, TaskMeta> tm;

    long reqTotal = 0;
    long doneTotal = 0;
    long deadlineMetTotal = 0;

    double batteryWh = 0.0;

  protected:
    void initialize() override
    {
        ns[0] = NodeState4{};
        ns[1] = NodeState4{};
        ns[2] = NodeState4{};
        tickEvt = new cMessage("TICK");
        batteryWh = par("batteryWh").doubleValue();

        scheduleAt(simTime() + par("statusPeriod").doubleValue(), tickEvt);
    }

    bool edgeIsDown() const
    {
        if (!par("enableEdgeOutage").boolValue()) return false;
        double s = par("edgeDownStart").doubleValue();
        double e = par("edgeDownEnd").doubleValue();
        return simTime().dbl() >= s && simTime().dbl() <= e;
    }

    double dist(long a, long b) const
    {
        double dx = ns.at(a).x - ns.at(b).x;
        double dy = ns.at(a).y - ns.at(b).y;
        return std::sqrt(dx*dx + dy*dy);
    }

    double distToEdge(long a) const
    {
        double dx = ns.at(a).x - par("posX").doubleValue();
        double dy = ns.at(a).y - par("posY").doubleValue();
        return std::sqrt(dx*dx + dy*dy);
    }

    int cellIndex(double v, double cellSize) const { return (int)std::floor(v / cellSize); }

    bool inLocality(long peerId) const
    {
        double cellSize = par("cellSize").doubleValue();
        int hop = par("neighborCells").intValue();
        int cx0 = cellIndex(ns.at(0).x, cellSize);
        int cy0 = cellIndex(ns.at(0).y, cellSize);
        int cxp = cellIndex(ns.at(peerId).x, cellSize);
        int cyp = cellIndex(ns.at(peerId).y, cellSize);
        return (std::abs(cxp - cx0) <= hop) && (std::abs(cyp - cy0) <= hop);
    }

    double rateEstimate(double d) const
    {
        double base = par("baseRate").doubleValue();
        double alpha = par("alpha").doubleValue();
        return base / (1.0 + alpha * d);
    }

    double lossEstimate(double d) const
    {
        double p0 = par("baseLoss").doubleValue();
        double b = par("betaLoss").doubleValue();
        return std::min(0.9, p0 + b * d * d);
    }

    double txTime(long bytes, double d) const
    {
        double bits = 8.0 * (double)bytes;
        return bits / rateEstimate(d);
    }

    double queueWait(long nodeId) const
    {
        return (double)ns.at(nodeId).qLen * ns.at(nodeId).computePerStage;
    }

    struct Plan {
        long chainPrimary = 1;
        long chainFallback = 0;
        long u1_s = 0, u1_e = -1;
        long u2_s = 0, u2_e = -1;
        double predPrimary = 1e18;
        double predFallback = 1e18;
    };

    double predLocal(long totalStages) const
    {
        double perStage = 0.012; // PLACEHOLDER
        return perStage * (double)totalStages;
    }

    double predEdgeOnly(long bytes0, long totalStages) const
    {
        double ttx = txTime(bytes0, distToEdge(0));
        double tcmp = par("computePerStage").doubleValue() * (double)totalStages;
        return ttx + tcmp + par("plannerDelay").doubleValue();
    }

    double predOnePeer(long peerId, long bytes0, long totalStages) const
    {
        double t01 = txTime(bytes0, dist(0, peerId));
        double tcmp = queueWait(peerId) + ns.at(peerId).computePerStage * (double)totalStages;
        long bytes1 = std::max(1000L, (long)(bytes0 * par("featureShrinkMid").doubleValue()));
        double t1e = txTime(bytes1, distToEdge(peerId));
        double tcmpE = 0.0; // edge can be terminal only for simplicity
        return t01 + tcmp + t1e + tcmpE + par("plannerDelay").doubleValue();
    }

    double predTwoPeers(long first, long second, long bytes0, long totalStages) const
    {
        long s1 = totalStages / 2;
        long s2 = totalStages - s1;

        double t01 = txTime(bytes0, dist(0, first));
        double c1 = queueWait(first) + ns.at(first).computePerStage * (double)s1;
        long bytes1 = std::max(1000L, (long)(bytes0 * 0.8));
        double t12 = txTime(bytes1, dist(first, second));
        double c2 = queueWait(second) + ns.at(second).computePerStage * (double)s2;
        long bytes2 = std::max(1000L, (long)(bytes1 * par("featureShrinkMid").doubleValue()));
        double t2e = txTime(bytes2, distToEdge(second));
        return t01 + c1 + t12 + c2 + t2e + par("plannerDelay").doubleValue();
    }

    Plan choosePlan(const std::string &policy, long bytes0, long totalStages, double deadline)
    {
        Plan p;

        if (policy == "LOCAL") {
            p.chainPrimary = 0;
            p.chainFallback = 1;
            p.predPrimary = predLocal(totalStages) + txTime(1000, distToEdge(0));
            p.predFallback = predEdgeOnly(bytes0, totalStages);
            return p;
        }
        if (policy == "EDGE") {
            p.chainPrimary = 1;
            p.chainFallback = 0;
            p.predPrimary = predEdgeOnly(bytes0, totalStages);
            return p;
        }
        if (policy == "STATIC") {
            p.chainPrimary = 2;
            p.chainFallback = 1;
            long s1 = totalStages / 2;
            p.u1_s = 1; p.u1_e = s1;
            p.u2_s = s1 + 1; p.u2_e = totalStages;
            p.predPrimary = predTwoPeers(1,2,bytes0,totalStages);
            p.predFallback = predEdgeOnly(bytes0, totalStages);
            return p;
        }
        if (policy == "NAIVE") {
            // nearest peer ignoring feasibility
            double d01 = dist(0,1);
            double d02 = dist(0,2);
            long pick = (d01 <= d02) ? 1 : 2;
            p.chainPrimary = (pick == 1) ? 4 : 5;
            p.chainFallback = 1;
            if (pick == 1) { p.u1_s = 1; p.u1_e = totalStages; }
            else { p.u2_s = 1; p.u2_e = totalStages; }
            p.predPrimary = predOnePeer(pick, bytes0, totalStages);
            p.predFallback = predEdgeOnly(bytes0, totalStages);
            return p;
        }

        // AEGIS full: locality shortlist, deadline feasibility, guard band, fallback, and node down checks
        std::vector<Plan> cand;

        // drop down peers
        bool ok1 = ns.at(1).known && !ns.at(1).isDown && inLocality(1);
        bool ok2 = ns.at(2).known && !ns.at(2).isDown && inLocality(2);

        if (ok1 && ok2) {
            Plan a; a.chainPrimary = 2; a.chainFallback = 3;
            long s1 = totalStages / 2;
            a.u1_s = 1; a.u1_e = s1;
            a.u2_s = s1 + 1; a.u2_e = totalStages;
            a.predPrimary = predTwoPeers(1,2,bytes0,totalStages);
            a.predFallback = predTwoPeers(2,1,bytes0,totalStages);
            cand.push_back(a);

            Plan b; b.chainPrimary = 3; b.chainFallback = 2;
            b.u2_s = 1; b.u2_e = s1;
            b.u1_s = s1 + 1; b.u1_e = totalStages;
            b.predPrimary = predTwoPeers(2,1,bytes0,totalStages);
            b.predFallback = predTwoPeers(1,2,bytes0,totalStages);
            cand.push_back(b);
        }
        if (ok1) {
            Plan a; a.chainPrimary = 4; a.chainFallback = 1;
            a.u1_s = 1; a.u1_e = totalStages;
            a.predPrimary = predOnePeer(1, bytes0, totalStages);
            a.predFallback = predEdgeOnly(bytes0, totalStages);
            cand.push_back(a);
        }
        if (ok2) {
            Plan a; a.chainPrimary = 5; a.chainFallback = 1;
            a.u2_s = 1; a.u2_e = totalStages;
            a.predPrimary = predOnePeer(2, bytes0, totalStages);
            a.predFallback = predEdgeOnly(bytes0, totalStages);
            cand.push_back(a);
        }

        // edge always available candidate
        Plan e; e.chainPrimary = 1; e.chainFallback = 0;
        e.predPrimary = predEdgeOnly(bytes0, totalStages);
        cand.push_back(e);

        double guard = par("guardBand").doubleValue();
        bool found = false;
        Plan best = cand.back();

        for (auto &c : cand) {
            if (c.predPrimary + guard <= deadline) {
                if (!found || c.predPrimary < best.predPrimary) { best = c; found = true; }
            }
        }
        if (found) return best;

        for (auto &c : cand) {
            if (c.predPrimary < best.predPrimary) best = c;
        }
        return best;
    }

    void broadcastPlan(long taskId, long planVer, const Plan &p, double deadline, bool isReplan)
    {
        int n = gateSize("ctrlOut");
        for (int i = 0; i < n; i++) {
            auto *m = new cMessage(isReplan ? "REPLAN" : "PLAN");
            setLongPar(m, "taskId", taskId);
            setLongPar(m, "planVer", planVer);
            setDoublePar(m, "deadline", deadline);

            setLongPar(m, "chainPrimary", p.chainPrimary);
            setLongPar(m, "chainFallback", p.chainFallback);

            setLongPar(m, "u1_s", p.u1_s); setLongPar(m, "u1_e", p.u1_e);
            setLongPar(m, "u2_s", p.u2_s); setLongPar(m, "u2_e", p.u2_e);

            setDoublePar(m, "predPrimary", p.predPrimary);
            setDoublePar(m, "predFallback", p.predFallback);

            send(m, "ctrlOut", i);
        }
    }

    void handleHello(cMessage *m)
    {
        long id = m->par("nodeId").longValue();
        ns[id].known = true;
        ns[id].x = m->par("posX").doubleValue();
        ns[id].y = m->par("posY").doubleValue();
        ns[id].computePerStage = m->par("computePerStage").doubleValue();
        ns[id].batteryWh = m->hasPar("batteryWh") ? m->par("batteryWh").doubleValue() : ns[id].batteryWh;
        ns[id].lastSeen = simTime().dbl();
        delete m;
    }

    void handleStatus(cMessage *m)
    {
        long id = m->par("nodeId").longValue();
        ns[id].known = true;
        ns[id].x = m->par("posX").doubleValue();
        ns[id].y = m->par("posY").doubleValue();
        ns[id].qLen = m->par("qLen").longValue();
        ns[id].batteryWh = m->par("batteryWh").doubleValue();
        ns[id].isDown = m->hasPar("isDown") ? (m->par("isDown").longValue() == 1) : false;
        ns[id].lastSeen = simTime().dbl();
        delete m;
    }

    void tickHealth()
    {
        double to = par("heartbeatTimeout").doubleValue();
        for (auto &kv : ns) {
            long id = kv.first;
            if (!kv.second.known) continue;
            if (kv.second.lastSeen < 0) continue;
            if (simTime().dbl() - kv.second.lastSeen > to) {
                kv.second.isDown = true;
            }
        }
    }

    bool allowReplan(long taskId, double newPred, double oldPred)
    {
        double cd = par("replanCooldown").doubleValue();
        double h = par("hysteresisGain").doubleValue();

        auto &x = tm[taskId];
        if (x.lastReplan >= 0 && (simTime().dbl() - x.lastReplan) < cd) return false;

        if (oldPred <= 0) return true;
        double improve = (oldPred - newPred) / oldPred;
        return improve >= h;
    }

    void handlePlanReq(cMessage *m)
    {
        reqTotal++;

        if (edgeIsDown()) { delete m; return; }

        long taskId = m->par("taskId").longValue();
        long totalStages = m->par("totalStages").longValue();
        long bytes0 = m->par("bytes0").longValue();
        double deadline = m->par("deadline").doubleValue();

        std::string policy = par("policy").stringValue();
        Plan p = choosePlan(policy, bytes0, totalStages, deadline);

        auto &x = tm[taskId];
        x.planVer += 1;
        x.chainPrimary = p.chainPrimary;
        x.chainFallback = p.chainFallback;
        x.predPrimary = p.predPrimary;
        x.predFallback = p.predFallback;

        broadcastPlan(taskId, x.planVer, p, deadline, false);

        delete m;
    }

    void handleDataArrival(cPacket *pkt)
    {
        if (edgeIsDown()) { delete pkt; return; }

        long taskId = pkt->par("taskId").longValue();
        long totalStages = pkt->par("totalStages").longValue();
        long nextStage = pkt->par("nextStage").longValue();
        double deadline = pkt->par("deadline").doubleValue();

        // compute remaining stages
        if (nextStage <= totalStages) {
            long rem = totalStages - nextStage + 1;
            double t = par("computePerStage").doubleValue() * (double)rem;

            // energy placeholder
            double eCmp = par("eCmpWhPerStage").doubleValue() * (double)rem;
            batteryWh -= eCmp;
            pkt->addPar("e_e0_cmpWh") = eCmp;

            scheduleAt(simTime() + t, pkt);
            return;
        }

        completeTask(pkt, deadline);
    }

    void completeTask(cPacket *pkt, double deadline)
    {
        doneTotal++;

        simtime_t L = simTime() - pkt->getTimestamp();
        bool met = (L.dbl() <= deadline);
        if (met) deadlineMetTotal++;

        // metrics
        recordScalar("task_latency_s", L.dbl());
        recordScalar("task_deadline_s", deadline);
        recordScalar("task_deadline_met", met ? 1.0 : 0.0);
        recordScalar("task_chainType", (double)pkt->par("chainType").longValue());

        // planning quality
        if (pkt->hasPar("predPrimary")) recordScalar("task_predPrimary_s", pkt->par("predPrimary").doubleValue());

        // end to end breakdown stamps if present
        if (pkt->hasPar("tCapStart")) recordScalar("t_capStart_s", pkt->par("tCapStart").doubleValue());
        if (pkt->hasPar("tPlanRecv")) recordScalar("t_planRecv_s", pkt->par("tPlanRecv").doubleValue());

        delete pkt;
    }

    void handleEdgeComputeDone(cPacket *pkt)
    {
        pkt->par("nextStage").setLongValue(pkt->par("totalStages").longValue() + 1);
        double deadline = pkt->par("deadline").doubleValue();
        completeTask(pkt, deadline);
    }

    void handleMessage(cMessage *msg) override
    {
        if (msg == tickEvt) {
            tickHealth();
            scheduleAt(simTime() + par("statusPeriod").doubleValue(), tickEvt);
            return;
        }

        if (msg->arrivedOn("ctrlIn")) {
            if (!strcmp(msg->getName(), "HELLO")) { handleHello(msg); return; }
            if (!strcmp(msg->getName(), "STATUS")) { handleStatus(msg); return; }
            if (!strcmp(msg->getName(), "PLANREQ")) { handlePlanReq(msg); return; }
            delete msg;
            return;
        }

        if (msg->isSelfMessage()) {
            auto *pkt = check_and_cast<cPacket*>(msg);
            handleEdgeComputeDone(pkt);
            return;
        }

        auto *pkt = check_and_cast<cPacket*>(msg);
        handleDataArrival(pkt);
    }

    void finish() override
    {
        recordScalar("req_total", (double)reqTotal);
        recordScalar("done_total", (double)doneTotal);
        recordScalar("deadline_met_total", (double)deadlineMetTotal);

        double succ = (reqTotal > 0) ? ((double)doneTotal / (double)reqTotal) : 0.0;
        double dsr = (doneTotal > 0) ? ((double)deadlineMetTotal / (double)doneTotal) : 0.0;
        double dmr = (doneTotal > 0) ? (1.0 - dsr) : 0.0;

        recordScalar("success_rate", succ);
        recordScalar("deadline_satisfaction_rate", dsr);
        recordScalar("deadline_miss_ratio", dmr);

        recordScalar("e0_batteryWh_final", batteryWh);

        cancelAndDelete(tickEvt);
    }
};

Define_Module(EdgeCoord4);
