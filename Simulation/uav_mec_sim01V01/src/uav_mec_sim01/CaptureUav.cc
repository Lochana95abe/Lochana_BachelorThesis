#include <omnetpp.h>
#include <string>

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

class CaptureUav : public cSimpleModule
{
  protected:
    void initialize() override
    {
        scheduleAt(par("taskStartTime"), new cMessage("START"));
    }

    void handleMessage(cMessage *msg) override
    {
        // Control-plane plan arrives on ctrlIn (log only in Sim01)
        if (msg->arrivedOn("ctrlIn")) {
            EV_INFO << "U0 received CTRL '" << msg->getName()
                    << "' at t=" << simTime() << "\n";
            delete msg;
            return;
        }

        // Self-message triggers tensor send
        if (msg->isSelfMessage()) {
            delete msg;

            auto *pkt = new cPacket("TENSOR");
            pkt->setByteLength(1'000'000);     // initial tensor size (bytes)
            pkt->setTimestamp();               // reference time for end-to-end observed delay

            // Tensor progress metadata (Sim01 demo)
            setLongPar(pkt, "totalStages", 4);
            setLongPar(pkt, "nextStage", 1);

            // Trace path
            setStringPar(pkt, "trace", "U0");

            // Latency bookkeeping (store explicit send timestamp too)
            setDoublePar(pkt, "t_send_u0", SIMTIME_DBL(simTime()));
            setLongPar(pkt, "bytes0", pkt->getByteLength());

            EV_INFO << "U0 sending TENSOR bytes=" << pkt->getByteLength()
                    << " nextStage=" << (int)pkt->par("nextStage").longValue()
                    << "/" << (int)pkt->par("totalStages").longValue()
                    << " at t=" << simTime() << "\n";

            send(pkt, "dataOut");
            return;
        }

        EV_WARN << "U0 got unexpected message '" << msg->getName() << "'\n";
        delete msg;
    }
};

Define_Module(CaptureUav);
