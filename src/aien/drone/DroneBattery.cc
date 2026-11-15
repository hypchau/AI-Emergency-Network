#include "DroneBattery.h"

#include "../mobility/DroneMobility.h"

namespace aien {

Define_Module(DroneBattery);

simsignal_t DroneBattery::residualRatioSignal = cComponent::registerSignal("residualRatio");

void DroneBattery::initialize()
{
    capacityJ = par("capacity").doubleValueInUnit("Wh") * 3600.0;
    energyJ = capacityJ;
    flightPower = par("flightPower").doubleValueInUnit("W");
    payloadPower = par("payloadPower").doubleValueInUnit("W");
    mobility = check_and_cast<DroneMobility *>(getModuleByPath(par("mobilityModule")));
    lastUpdate = simTime();
    emit(residualRatioSignal, 1.0);
    WATCH(energyJ);
    WATCH(empty);
}

double DroneBattery::update()
{
    simtime_t now = simTime();
    if (now > lastUpdate) {
        // the phase is the one that prevailed since the previous query: the
        // controller queries every second, so the error is at most 1 s of flight
        if (mobility->isAirborne()) {
            energyJ -= (now - lastUpdate).dbl() * getAirbornePower();
            if (energyJ <= 0) {
                energyJ = 0;
                empty = true;
            }
            emit(residualRatioSignal, energyJ / capacityJ);
        }
        lastUpdate = now;
    }
    return energyJ;
}

double DroneBattery::swap(double ratio)
{
    update();
    double removed = energyJ / capacityJ;
    energyJ = capacityJ * ratio;
    empty = false;
    emit(residualRatioSignal, ratio);
    return removed;
}

std::string DroneBattery::str() const
{
    char buf[32];
    snprintf(buf, sizeof(buf), "%.0f%%", 100.0 * energyJ / capacityJ);
    return buf;
}

void DroneBattery::refreshDisplay() const
{
    getDisplayString().setTagArg("t", 0, str().c_str());
}

} // namespace aien
