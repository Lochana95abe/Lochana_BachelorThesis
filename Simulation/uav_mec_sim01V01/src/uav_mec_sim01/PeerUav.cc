#include <omnetpp.h>
#include <string>
#include <algorithm>

using namespace omnetpp;

static void setDoublePar(cPacket *pkt, const char *name, double value)
{
    if (!pkt->hasPar(name)) pkt->addPar(name) = value;
    else pkt->par(name).setDoubleValue(value);
}

static void setLongPar(cPacket *pkt, const char *name, long value)
{
    if (!pkt->hasPar(name)) pkt->addPar(name) = value;
    else pkt->par(name).setLongValue(value);
}

static void setStringPar(cPacket *pkt, const char *name, const char *value)
{
    if (!pkt->hasPar(name)) pkt->addPar(name).setStringValue(value);
    else pkt->par(name).setStringValue(value);
}

static bool parseRange(const std::string& plan, const std::string& tag, int& outStart, int& outEnd)
{
    // plan: "PLAN:U1[1-2],U2[3-4]" ; tag: "U1[" or "U2["
    auto pos = plan.find(tag);
    if (pos == std::string::npos) return false;

    auto close = plan.find("]", pos);
    if (close == std::string::npos) return false;

    std::string range = plan.substr(pos + tag.size(), close - (pos + tag.size())); // "1-2"
    auto dash = range.find("-");
    if (dash == std::string::npos) return false;

    try {
        outStart = std::stoi(range.substr(0, dash));
        outEnd   = std::stoi(range.substr(dash + 1));
    } catch (...) {
        return false;
    }
    return true;
}

class PeerUav : public cSimpleModule
{
  private:
    int stageStart = -1;
    int stageEnd   = -1;

    // Returns "u1" or "u2" etc.
    std::string me() const { return getFullName(); }

    // Returns token used in plan string: "U1[" or "U2["
    std::string planTag() const
    {
        if (me() == "u1") return "U1[";
        if (me() == "u2") return "U2[";
        return "";
    }

    // Parameter names for timestamps
    std::string parArrName() const { return "t_arr_" + me(); }   // e.g., t_arr_u1
    std::string parSendName() const { return "t_send_" + me(); } // e.g., t_send_u1

  protected:
    void handleMessage(cMessage *msg) override
    {
        // -------- Control-plane: store stage assignment --------
        if (msg->arrivedOn("ctrlIn")) {
            std::string plan = msg->getName();
            std::string tag  = planTag();

            if (!tag.empty() && parseRange(plan, tag, stageStart, stageEnd)) {
                EV_INFO << me() << " received CTRL '" << plan << "' at t=" << simTime()
                        << " | assignedStages=" << stageStart << "-" << stageEnd << "\n";
            } else {
                stageStart = -1;
                stageEnd   = -1;
                EV_INFO << me() << " received CTRL '" << plan << "' at t=" << simTime()
                        << " | assignedStages=NONE\n";
            }

            delete msg;
            return;
        }

        // -------- Data-plane arrival (from previous hop) --------
        if (!msg->isSelfMessage()) {
            auto *pkt = check_and_cast<cPacket *>(msg);

            // Stamp arrival time at this node
            setDoublePar(pkt, parArrName().c_str(), SIMTIME_DBL(simTime()));

            int totalStages = (int)pkt->par("totalStages").longValue();
            int nextStage   = (int)pkt->par("nextStage").longValue();
            int beforeBytes = pkt->getByteLength();

            bool canProcess = (stageStart != -1) && (stageEnd != -1) &&
                              (nextStage >= stageStart) && (nextStage <= stageEnd);

            // Decide updates if we process here
            int newNextStage = nextStage;
            int afterBytes   = beforeBytes;

            if (canProcess) {
                newNextStage = stageEnd + 1;
                afterBytes = std::max(1000, (int)(beforeBytes * 0.8)); // placeholder shrink
            }

            // Store pending updates to apply after compute
            setLongPar(pkt, "pendingNextStage", newNextStage);
            setLongPar(pkt, "pendingBytes", afterBytes);
            setLongPar(pkt, "didProcessHere", canProcess ? 1 : 0);

            EV_INFO << me() << " received TENSOR at t=" << simTime()
                    << " | bytes=" << beforeBytes
                    << " | nextStage=" << nextStage << "/" << totalStages
                    << " | assigned=" << stageStart << "-" << stageEnd
                    << " | willProcess=" << (canProcess ? "YES" : "NO")
                    << " | computeDelay=" << par("computeDelay") << "\n";

            if (canProcess) {
                scheduleAt(simTime() + par("computeDelay").doubleValue(), pkt);
            } else {
                // No processing: forward immediately but still stamp send time
                setDoublePar(pkt, parSendName().c_str(), SIMTIME_DBL(simTime()));

                // Trace update
                std::string trace = pkt->par("trace").stringValue();
                trace += std::string("->") + me();
                setStringPar(pkt, "trace", trace.c_str());

                send(pkt, "dataOut");
            }
            return;
        }

        // -------- Self-message (compute done) --------
        auto *pkt = check_and_cast<cPacket *>(msg);

        int beforeBytes = pkt->getByteLength();
        int beforeNext  = (int)pkt->par("nextStage").longValue();

        int afterBytes = (int)pkt->par("pendingBytes").longValue();
        int afterNext  = (int)pkt->par("pendingNextStage").longValue();

        pkt->setByteLength(afterBytes);
        pkt->par("nextStage").setLongValue(afterNext);

        // Stamp send time (end of compute)
        setDoublePar(pkt, parSendName().c_str(), SIMTIME_DBL(simTime()));

        // Trace update
        std::string trace = pkt->par("trace").stringValue();
        trace += std::string("->") + me();
        setStringPar(pkt, "trace", trace.c_str());

        EV_INFO << me() << " done compute at t=" << simTime()
                << " | bytes " << beforeBytes << "->" << afterBytes
                << " | nextStage " << beforeNext << "->" << afterNext
                << " | forwarding\n";

        send(pkt, "dataOut");
    }
};

Define_Module(PeerUav);
