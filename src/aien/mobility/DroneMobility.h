#ifndef __AIEN_DRONEMOBILITY_H
#define __AIEN_DRONEMOBILITY_H

#include "inet/mobility/base/MovingMobilityBase.h"

namespace aien {

using namespace omnetpp;
using inet::Coord;

class TerrainModel;

/**
 * Commanded drone flight model with terrain following. See DroneMobility.ned.
 */
class DroneMobility : public inet::MovingMobilityBase
{
  public:
    enum Phase { PARKED, CLIMB, CRUISE, HOVER, DESCEND, CRASHED };
    static const char *phaseName(Phase p);

  protected:
    TerrainModel *terrain = nullptr;
    double homeLat = 0, homeLon = 0;
    double parkingHeight = 3, flightHeight = 100;
    double cruiseSpeed = 18, climbRate = 6, descentRate = 5;

    Phase phase = PARKED;
    bool returning = false;
    Coord horizontal;   // current horizontal scene position (z = 0)
    Coord target;       // horizontal target (z = 0)
    Coord home;         // horizontal parking spot (z = 0)
    double height = 0;  // current height above ground level

  protected:
    virtual void initialize(int stage) override;
    virtual void setInitialPosition() override;
    virtual void move() override;
    void updatePosition();
    void wakeUp();
    static Coord flat(const Coord& c) { return Coord(c.x, c.y, 0); }

  public:
    /** Command: take off (if parked) and fly to the horizontal scene position, then hover. */
    void flyTo(const Coord& sceneTarget);
    /** Command: fly back to the hive and land. */
    void returnHome();
    /** Failure injection: the drone falls to the ground and stays there. */
    void crash();

    Phase getPhase() { moveAndUpdate(); return phase; }
    bool isReturning() const { return returning; }
    bool isAirborne() { auto p = getPhase(); return p != PARKED && p != CRASHED; }
    const Coord& getTarget() const { return target; }
    const Coord& getHome() const { return home; }
    double getCruiseSpeed() const { return cruiseSpeed; }
    double getFlightHeight() const { return flightHeight; }
    virtual double getMaxSpeed() const override { return std::sqrt(cruiseSpeed * cruiseSpeed + climbRate * climbRate); }

    /** Time to go from the parked state to a hover above `sceneTarget` (climb + cruise). */
    double estimateOutboundTime(const Coord& sceneTarget) const;
    /** Time to come back from `scenePosition` and land (cruise + descent). */
    double estimateReturnTime(const Coord& scenePosition) const;
};

} // namespace aien

#endif
