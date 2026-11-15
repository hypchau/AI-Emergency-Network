#include "DroneMobility.h"

#include <cmath>

#include "../terrain/TerrainModel.h"

namespace aien {

Define_Module(DroneMobility);

const char *DroneMobility::phaseName(Phase p)
{
    switch (p) {
        case PARKED: return "PARKED";
        case CLIMB: return "CLIMB";
        case CRUISE: return "CRUISE";
        case HOVER: return "HOVER";
        case DESCEND: return "DESCEND";
        case CRASHED: return "CRASHED";
    }
    return "?";
}

void DroneMobility::initialize(int stage)
{
    MovingMobilityBase::initialize(stage);
    if (stage == inet::INITSTAGE_LOCAL) {
        terrain = check_and_cast<TerrainModel *>(getModuleByPath(par("terrainModule")));
        homeLat = par("homeLatitude").doubleValueInUnit("deg");
        homeLon = par("homeLongitude").doubleValueInUnit("deg");
        parkingHeight = par("parkingHeight").doubleValueInUnit("m");
        flightHeight = par("flightHeight").doubleValueInUnit("m");
        cruiseSpeed = par("cruiseSpeed").doubleValueInUnit("mps");
        climbRate = par("climbRate").doubleValueInUnit("mps");
        descentRate = par("descentRate").doubleValueInUnit("mps");
        stationary = true;
        WATCH(height);
        WATCH(returning);
    }
}

void DroneMobility::setInitialPosition()
{
    // parking spot: ring around the truck, one slot per drone index
    Coord truck = flat(terrain->toScene(homeLat, homeLon, 0));
    int index = getParentModule()->isVector() ? getParentModule()->getIndex() : 0;
    int count = getParentModule()->isVector() ? getParentModule()->getVectorSize() : 1;
    double radius = par("parkingRadius").doubleValueInUnit("m");
    double angle = 2 * M_PI * index / std::max(1, count);
    home = truck + Coord(radius * cos(angle), radius * sin(angle), 0);
    horizontal = home;
    target = home;
    height = parkingHeight;
    phase = PARKED;
    lastPosition = terrain->aboveScenePosition(horizontal, height);
    lastVelocity = Coord::ZERO;
    nextChange = -1;
}

void DroneMobility::wakeUp()
{
    moveAndUpdate();          // bring the state up to now before changing the plan
    stationary = false;
    lastUpdate = simTime();
    scheduleUpdate();
}

void DroneMobility::flyTo(const Coord& sceneTarget)
{
    if (phase == CRASHED)
        return;
    wakeUp();
    target = flat(sceneTarget);
    returning = false;
    phase = (phase == PARKED || phase == DESCEND) ? CLIMB : CRUISE;
    EV_INFO << "flyTo " << target << " (phase " << phaseName(phase) << ")" << endl;
}

void DroneMobility::returnHome()
{
    if (phase == CRASHED || phase == PARKED)
        return;
    wakeUp();
    target = home;
    returning = true;
    if (phase != CLIMB)
        phase = CRUISE;
    EV_INFO << "returnHome" << endl;
}

void DroneMobility::crash()
{
    wakeUp();
    phase = CRASHED;
    returning = false;
}

void DroneMobility::move()
{
    double dt = (simTime() - lastUpdate).dbl();
    if (dt <= 0)
        return;
    Coord previous = lastPosition;
    switch (phase) {
        case PARKED:
        case HOVER:
            stationary = true;
            break;
        case CLIMB:
            height += climbRate * dt;
            if (height >= flightHeight) {
                height = flightHeight;
                phase = CRUISE;
            }
            break;
        case CRUISE: {
            Coord delta = target - horizontal;
            delta.z = 0;
            double distance = delta.length();
            double step = cruiseSpeed * dt;
            if (step >= distance) {
                horizontal = target;
                phase = returning ? DESCEND : HOVER;
            }
            else
                horizontal += delta / distance * step;
            break;
        }
        case DESCEND:
            height -= descentRate * dt;
            if (height <= parkingHeight) {
                height = parkingHeight;
                phase = PARKED;
                returning = false;
                stationary = true;
            }
            break;
        case CRASHED:
            height -= 15 * dt;
            if (height <= 0) {
                height = 0;
                stationary = true;
            }
            break;
    }
    lastPosition = terrain->aboveScenePosition(horizontal, height);
    lastVelocity = stationary ? Coord::ZERO : (lastPosition - previous) / dt;
    nextChange = -1;
}

double DroneMobility::estimateOutboundTime(const Coord& sceneTarget) const
{
    double climb = (flightHeight - parkingHeight) / climbRate;
    return climb + flat(sceneTarget).distance(home) / cruiseSpeed;
}

double DroneMobility::estimateReturnTime(const Coord& scenePosition) const
{
    double descent = (flightHeight - parkingHeight) / descentRate;
    return flat(scenePosition).distance(home) / cruiseSpeed + descent;
}

} // namespace aien
