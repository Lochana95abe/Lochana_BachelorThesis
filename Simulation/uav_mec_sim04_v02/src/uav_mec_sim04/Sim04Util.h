#pragma once
#include <omnetpp.h>
#include <string>

using namespace omnetpp;

static inline void setLongPar(cMessage *m, const char *name, long v) {
    if (!m->hasPar(name)) m->addPar(name) = v;
    else m->par(name).setLongValue(v);
}
static inline void setDoublePar(cMessage *m, const char *name, double v) {
    if (!m->hasPar(name)) m->addPar(name) = v;
    else m->par(name).setDoubleValue(v);
}
static inline void setStringPar(cMessage *m, const char *name, const char *v) {
    if (!m->hasPar(name)) m->addPar(name).setStringValue(v);
    else m->par(name).setStringValue(v);
}

static inline long getLongParOr(cMessage *m, const char *name, long def) {
    return m->hasPar(name) ? m->par(name).longValue() : def;
}
static inline double getDoubleParOr(cMessage *m, const char *name, double def) {
    return m->hasPar(name) ? m->par(name).doubleValue() : def;
}
static inline std::string getStringParOr(cMessage *m, const char *name, const char *def) {
    return m->hasPar(name) ? std::string(m->par(name).stringValue()) : std::string(def);
}
