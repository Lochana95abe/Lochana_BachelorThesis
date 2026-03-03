#include <omnetpp.h>
#include <map>
#include <vector>
#include <string>
#include <cmath>
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

struct NodeStatus {
    bool known = false;
    double x = 0;
    double y = 0;
    double computePerStage = 0;
};

class EdgeNode2 : public cSimpleModule
{
  private:
    std::map<long, NodeStatus> nodes;

    long totalReq = 0;
    long totalDone = 0;
    long deadlineMet = 0;

  protected:
    void initialize() override
    {
        nodes[0] = NodeStatus{};
        nodes[1] = NodeStatus{};
        nodes[2] = NodeStatus{};
    }

    double distXY(double ax,double ay,double bx,double by) const {
        double dx = ax - bx;
        double dy = ay - by;
        return std::sqrt(dx*dx + dy*dy);
    }

    double distNodes(long a, long b) const
    {
        return distXY(nodes.at(a).x, nodes.at(a).y, nodes.at(b).x, nodes.at(b).y);
    }

    int cellIndex(double v, double cellSize) const {
        return (int)std::floor(v / cellSize);
    }

    bool inLocality(long peerId) const
    {
        double cellSize = par("cellSize").doubleValue();
        int hop = par("neighborCells").intValue();

        const auto &u0 = nodes.at(0);
        const auto &p  = nodes.at(peerId);

        int cx0 = cellIndex(u0.x, cellSize);
        int cy0 = cellIndex(u0.y, cellSize);
        int cxp = cellIndex(p.x, cellSize);
        int cyp = cellIndex(p.y, cellSize);

        return (std::abs(cxp - cx0) <= hop) && (std::abs(cyp - cy0) <= hop);
    }

    double rateEstimate(double d) const
    {
        double base = par("baseRate").doubleValue();
        double alpha = par("alpha").doubleValue();
        return base / (1.0 + alpha * d);
    }

    double txTimeSeconds(long bytes, double d) const
    {
        double r = rateEstimate(d);
        double bits = 8.0 * (double)bytes;
        return bits / r;
    }

    struct Plan {
        long chainType;
        long u1_s, u1_e;
        long u2_s, u2_e;
        double pred;
    };

    Plan pickPlan(const std::string &policy, long bytes0, long totalStages, double deadline)
    {
        auto predEdgeOnly = [&]() {
            double d0e = distXY(nodes[0].x, nodes[0].y, par("posX").doubleValue(), par("posY").doubleValue());
            double tx = txTimeSeconds(bytes0, d0e);
            double cmp = par("computePerStage").doubleValue() * (double)totalStages;
            return tx + cmp + par("plannerDelay").doubleValue();
        };

        auto predOnePeer = [&](long peerId, long stagesOnPeer) {
            double d01 = distNodes(0, peerId);
            double d1e = distXY(nodes[peerId].x, nodes[peerId].y, par("posX").doubleValue(), par("posY").doubleValue());

            double tx1 = txTimeSeconds(bytes0, d01);
            double cmp = nodes[peerId].computePerStage * (double)stagesOnPeer;

            long bytes1 = std::max(1000L, (long)(bytes0 * 0.8));
            double tx2 = txTimeSeconds(bytes1, d1e);

            long rem = totalStages - stagesOnPeer;
            double cmpE = (rem > 0) ? (par("computePerStage").doubleValue() * (double)rem) : 0.0;

            return tx1 + cmp + tx2 + cmpE + par("plannerDelay").doubleValue();
        };

        auto predTwoPeers = [&](long first, long second, long s1, long s2) {
            double d0f = distNodes(0, first);
            double dfS = distNodes(first, second);
            double dSe = distXY(nodes[second].x, nodes[second].y, par("posX").doubleValue(), par("posY").doubleValue());

            double tx1 = txTimeSeconds(bytes0, d0f);
            double cmp1 = nodes[first].computePerStage * (double)s1;
            long bytes1 = std::max(1000L, (long)(bytes0 * 0.8));

            double tx2 = txTimeSeconds(bytes1, dfS);
            double cmp2 = nodes[second].computePerStage * (double)s2;
            long bytes2 = std::max(1000L, (long)(bytes1 * 0.8));

            double tx3 = txTimeSeconds(bytes2, dSe);

            long rem = totalStages - (s1 + s2);
            double cmpE = (rem > 0) ? (par("computePerStage").doubleValue() * (double)rem) : 0.0;

            return tx1 + cmp1 + tx2 + cmp2 + tx3 + cmpE + par("plannerDelay").doubleValue();
        };

        long s1 = totalStages / 2;
        long s2 = totalStages - s1;

        auto mk = [&](long ct,long u1s,long u1e,long u2s,long u2e,double pred) {
            Plan p; p.chainType=ct; p.u1_s=u1s; p.u1_e=u1e; p.u2_s=u2s; p.u2_e=u2e; p.pred=pred; return p;
        };

        if (policy == "LOCAL") return mk(0, 0, -1, 0, -1, par("plannerDelay").doubleValue());
        if (policy == "EDGE")  return mk(1, 0, -1, 0, -1, predEdgeOnly());
        if (policy == "STATIC") return mk(2, 1, s1, s1 + 1, totalStages, predTwoPeers(1,2,s1,s2));
        if (policy == "NAIVE") {
            double d01 = distNodes(0,1);
            double d02 = distNodes(0,2);
            long p = (d01 <= d02) ? 1 : 2;
            if (p == 1) return mk(4, 1, totalStages, 0, -1, predOnePeer(1,totalStages));
            return mk(5, 0, -1, 1, totalStages, predOnePeer(2,totalStages));
        }

        bool c1 = nodes[1].known && inLocality(1);
        bool c2 = nodes[2].known && inLocality(2);

        std::vector<Plan> cands;
        if (c1 && c2) {
            cands.push_back(mk(2, 1, s1, s1 + 1, totalStages, predTwoPeers(1,2,s1,s2)));
            cands.push_back(mk(3, s1 + 1, totalStages, 1, s1, predTwoPeers(2,1,s2,s1)));
        }
        if (c1) cands.push_back(mk(4, 1, totalStages, 0, -1, predOnePeer(1,totalStages)));
        if (c2) cands.push_back(mk(5, 0, -1, 1, totalStages, predOnePeer(2,totalStages)));
        cands.push_back(mk(1, 0, -1, 0, -1, predEdgeOnly()));

        bool found = false;
        Plan best = cands.back();
        for (auto &p : cands) {
            if (p.pred <= deadline) {
                if (!found || p.pred < best.pred) { best = p; found = true; }
            }
        }
        if (found) return best;

        for (auto &p : cands) if (p.pred < best.pred) best = p;
        return best;
    }

    void handleHello(cMessage *m)
    {
        long id = m->par("nodeId").longValue();
        nodes[id].known = true;
        nodes[id].x = m->par("posX").doubleValue();
        nodes[id].y = m->par("posY").doubleValue();
        nodes[id].computePerStage = m->par("computePerStage").doubleValue();
        delete m;
    }

    void broadcastPlan(long taskId, const Plan &p, double deadline)
    {
        int n = gateSize("ctrlOut");
        for (int i = 0; i < n; i++) {
            auto *m = new cMessage("PLAN");
            setLongPar(m, "taskId", taskId);
            setLongPar(m, "chainType", p.chainType);
            setDoublePar(m, "deadline", deadline);

            setLongPar(m, "u1_s", p.u1_s);
            setLongPar(m, "u1_e", p.u1_e);
            setLongPar(m, "u2_s", p.u2_s);
            setLongPar(m, "u2_e", p.u2_e);

            setDoublePar(m, "pred", p.pred);
            send(m, "ctrlOut", i);
        }
    }

    void handlePlanReq(cMessage *m)
    {
        totalReq++;

        long taskId = m->par("taskId").longValue();
        long totalStages = m->par("totalStages").longValue();
        long bytes0 = m->par("bytes0").longValue();
        double deadline = m->par("deadline").doubleValue();

        std::string policy = par("policy").stringValue();
        Plan p = pickPlan(policy, bytes0, totalStages, deadline);

        auto *evt = new cMessage("PLANSEND");
        setLongPar(evt, "taskId", taskId);
        setDoublePar(evt, "deadline", deadline);
        setLongPar(evt, "chainType", p.chainType);
        setLongPar(evt, "u1_s", p.u1_s); setLongPar(evt, "u1_e", p.u1_e);
        setLongPar(evt, "u2_s", p.u2_s); setLongPar(evt, "u2_e", p.u2_e);
        setDoublePar(evt, "pred", p.pred);

        scheduleAt(simTime() + par("plannerDelay").doubleValue(), evt);
        delete m;
    }

    void handlePlanSend(cMessage *evt)
    {
        Plan p;
        p.chainType = evt->par("chainType").longValue();
        p.u1_s = evt->par("u1_s").longValue();
        p.u1_e = evt->par("u1_e").longValue();
        p.u2_s = evt->par("u2_s").longValue();
        p.u2_e = evt->par("u2_e").longValue();
        p.pred = evt->par("pred").doubleValue();

        long taskId = evt->par("taskId").longValue();
        double deadline = evt->par("deadline").doubleValue();

        broadcastPlan(taskId, p, deadline);
        delete evt;
    }

    void finishTask(cPacket *pkt)
    {
        totalDone++;

        double deadline = pkt->par("deadline").doubleValue();
        simtime_t L = simTime() - pkt->getTimestamp();
        bool met = (L.dbl() <= deadline);
        if (met) deadlineMet++;

        recordScalar("task_latency_s", L.dbl());
        recordScalar("task_deadline_s", deadline);
        recordScalar("task_deadline_met", met ? 1.0 : 0.0);
        recordScalar("task_chainType", (double)pkt->par("chainType").longValue());

        delete pkt;
    }

    void handleEdgeComputeDone(cPacket *pkt)
    {
        pkt->par("nextStage").setLongValue(pkt->par("totalStages").longValue() + 1);
        finishTask(pkt);
    }

    void handleData(cPacket *pkt)
    {
        long totalStages = pkt->par("totalStages").longValue();
        long nextStage = pkt->par("nextStage").longValue();

        if (nextStage <= totalStages) {
            long rem = totalStages - nextStage + 1;
            double t = par("computePerStage").doubleValue() * (double)rem;
            scheduleAt(simTime() + t, pkt);
            return;
        }

        finishTask(pkt);
    }

    void handleMessage(cMessage *msg) override
    {
        if (msg->isSelfMessage() && !strcmp(msg->getName(), "PLANSEND")) {
            handlePlanSend(msg);
            return;
        }

        if (msg->arrivedOn("ctrlIn")) {
            if (!strcmp(msg->getName(), "HELLO")) { handleHello(msg); return; }
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
        handleData(pkt);
    }

    void finish() override
    {
        recordScalar("req_total", (double)totalReq);
        recordScalar("done_total", (double)totalDone);
        recordScalar("deadline_met_total", (double)deadlineMet);

        double succ = (totalReq > 0) ? ((double)totalDone / (double)totalReq) : 0.0;
        double dsr = (totalDone > 0) ? ((double)deadlineMet / (double)totalDone) : 0.0;
        double dmr = (totalDone > 0) ? (1.0 - dsr) : 0.0;

        recordScalar("success_rate", succ);
        recordScalar("deadline_satisfaction_rate", dsr);
        recordScalar("deadline_miss_ratio", dmr);
    }
};

Define_Module(EdgeNode2);
