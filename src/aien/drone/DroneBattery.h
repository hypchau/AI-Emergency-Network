#ifndef __AIEN_DRONEBATTERY_H
#define __AIEN_DRONEBATTERY_H

#include <omnetpp.h>

namespace aien {

using namespace omnetpp;

class DroneMobility;

/**
 * Battery of a relay drone, see DroneBattery.ned.
 */
class DroneBattery : public cSimpleModule
{
  protected:
    double capacityJ = 0;   // joules
    double energyJ = 0;     // joules left in the installed pack
    double flightPower = 0; // W
    double payloadPower = 0; // W
    simtime_t lastUpdate;
    bool empty = false;
    DroneMobility *mobility = nullptr;
    static simsignal_t residualRatioSignal;

  protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override { throw cRuntimeError("DroneBattery does not process messages"); }
    virtual void refreshDisplay() const override;

  public:
    /** Integrates consumption up to now; returns the remaining energy in joules. */
    double update();
    double getResidualRatio() { update(); return energyJ / capacityJ; }
    /** Last computed level, without integrating (safe in refreshDisplay). */
    double peekResidualRatio() const { return energyJ / capacityJ; }
    double getEnergy() { update(); return energyJ; }
    double getCapacity() const { return capacityJ; }
    double getAirbornePower() const { return flightPower + payloadPower; }
    /** Endurance of a full pack, in seconds. */
    double getFullEndurance() const { return capacityJ / getAirbornePower(); }
    bool isEmpty() { update(); return empty; }
    /** Installs a pack holding `ratio` of the nominal capacity; returns the ratio of the removed pack. */
    double swap(double ratio = 1.0);
    virtual std::string str() const override;
};

} // namespace aien

#endif
