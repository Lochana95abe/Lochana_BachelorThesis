#include <omnetpp.h>

using namespace omnetpp;

static double getDoubleParOr(cMessage *msg, const char *name, double def)
{
    return msg->hasPar(name) ? msg->par(name).doubleValue() : def;
}

static long getLongParOr(cMessage *msg, const char *name, long def)
{
    return msg->hasPar(name) ? msg->par(name).longValue() : def;
}

class EdgeNode : public cSimpleModule
{
  protected:
    void initialize() override
    {
        scheduleAt(par("planTime"), new cMessage("PLAN"));
    }

    void handleMessage(cMessage *msg) override
    {
        // Self-message: broadcast control-plan
        if (msg->isSelfMessage()) {
            int n = gateSize("ctrlOut");

            EV_INFO << "E0 SHORTLIST (Sim01, fixed demo): candidates=[u1,u2], chosenChain=U0->U1->U2->E0\n";

            const char *planText = "PLAN:U1[1-2],U2[3-4]";
            EV_INFO << "E0 broadcasting CTRL '" << planText << "' at t=" << simTime()
                    << " to " << n << " UAV(s)\n";

            for (int i = 0; i < n; i++) {
                send(new cMessage(planText), "ctrlOut", i);
            }

            delete msg;
            return;
        }

        // Data-plane tensor arrives here
        auto *pkt = check_and_cast<cPacket *>(msg);

        // End-to-end observed delay
        simtime_t oneWay = simTime() - pkt->getTimestamp();

        // Pull timestamps stamped by nodes (seconds)
        double t_send_u0 = getDoubleParOr(pkt, "t_send_u0", SIMTIME_DBL(pkt->getTimestamp()));
        double t_arr_u1  = getDoubleParOr(pkt, "t_arr_u1",  -1.0);
        double t_send_u1 = getDoubleParOr(pkt, "t_send_u1", -1.0);
        double t_arr_u2  = getDoubleParOr(pkt, "t_arr_u2",  -1.0);
        double t_send_u2 = getDoubleParOr(pkt, "t_send_u2", -1.0);
        double t_arr_e0  = SIMTIME_DBL(simTime());

        // Compute breakdown (only if timestamps exist; else set 0)
        double tx01 = (t_arr_u1 >= 0) ? (t_arr_u1 - t_send_u0) : 0.0;
        double cmp1 = (t_arr_u1 >= 0 && t_send_u1 >= 0) ? (t_send_u1 - t_arr_u1) : 0.0;

        double tx12 = (t_send_u1 >= 0 && t_arr_u2 >= 0) ? (t_arr_u2 - t_send_u1) : 0.0;
        double cmp2 = (t_arr_u2 >= 0 && t_send_u2 >= 0) ? (t_send_u2 - t_arr_u2) : 0.0;

        double tx2e = (t_send_u2 >= 0) ? (t_arr_e0 - t_send_u2) : 0.0;

        double totalSum = tx01 + cmp1 + tx12 + cmp2 + tx2e;

        long totalStages = getLongParOr(pkt, "totalStages", -1);
        long nextStage   = getLongParOr(pkt, "nextStage", -1);
        const char *trace = pkt->hasPar("trace") ? pkt->par("trace").stringValue() : "(no-trace)";
        long bytes0 = getLongParOr(pkt, "bytes0", -1);

        EV_INFO << "E0 received " << pkt->getName()
                << " at t=" << simTime()
                << " | bytes0=" << bytes0
                << " | bytesFinal=" << pkt->getByteLength()
                << " | nextStage=" << nextStage << "/" << totalStages
                << " | trace=" << trace << "\n";

        EV_INFO << "E0 latency breakdown (seconds): "
                << "tx01=" << tx01 << ", cmp1=" << cmp1
                << ", tx12=" << tx12 << ", cmp2=" << cmp2
                << ", tx2e=" << tx2e
                << " | totalSum=" << totalSum
                << " | totalObserved=" << oneWay.dbl() << "\n";

        // Record scalars (results .sca)
        recordScalar("L_total_observed_s", oneWay.dbl());
        recordScalar("L_total_sum_s", totalSum);
        recordScalar("L_link_u0_u1_s", tx01);
        recordScalar("L_comp_u1_s", cmp1);
        recordScalar("L_link_u1_u2_s", tx12);
        recordScalar("L_comp_u2_s", cmp2);
        recordScalar("L_link_u2_e0_s", tx2e);
        recordScalar("bytes0", (double)bytes0);
        recordScalar("bytesFinal", (double)pkt->getByteLength());

        if (totalStages > 0 && nextStage == totalStages + 1) {
            EV_INFO << "E0 TASK DONE (Sim01): all stages processed.\n";
        } else {
            EV_WARN << "E0 TASK NOT DONE (Sim01): stage progress incomplete.\n";
        }

        delete pkt;
        endSimulation();
    }
};

Define_Module(EdgeNode);
