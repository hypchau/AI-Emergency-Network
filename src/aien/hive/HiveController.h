#ifndef __AIEN_HIVECONTROLLER_H
#define __AIEN_HIVECONTROLLER_H

#include <vector>
#include <string>

#include "inet/common/INETDefs.h"
#include "inet/common/geometry/common/Coord.h"

namespace inet { class CanvasProjection; }

namespace aien {

using namespace omnetpp;
using inet::Coord;

class TerrainModel;
class DroneMobility;
class DroneBattery;

/**
 * Post planning, launch and battery rotation of the relay drones.
 * See HiveController.ned for the algorithms.
 */
class HiveController : public cSimpleModule
{
  public:
    enum DroneState { READY, OUTBOUND, ON_STATION, RETURNING, SWAPPING, FAILED };
    static const char *stateName(DroneState s);

  protected:
    struct Post {
        int id = 0;
        Coord position;          // scene position at flight height
        double outboundTime = 0; // climb + cruise from the hive
        double returnTime = 0;   // cruise + descent to the hive
        double stationTime = 0;  // useful time on post per sortie
        int dronesNeeded = 0;    // to hold it continuously
        int parent = -1;         // -1 = mother antenna, else index of the post it links to
        bool bridge = false;     // relay only (no phone of its own)
        std::vector<int> users;  // phones it covers (plan)
        int active = -1;         // drone on post
        int inbound = -1;        // drone flying to the post
    };

    struct Drone {
        cModule *node = nullptr;
        DroneMobility *mobility = nullptr;
        DroneBattery *battery = nullptr;
        DroneState state = READY;
        int post = -1;
        simtime_t swapDoneAt;
        int sorties = 0;
    };

    // parameters
    TerrainModel *terrain = nullptr;
    simtime_t tickInterval, startTime, launchInterval;
    double accessRange = 0, meshRange = 0, motherRange = 0, motherHeight = 0;
    bool useLineOfSight = true;
    double reserveRatio = 0, handoverMargin = 0, batterySwapTime = 0, fullChargeTime = 0, minStationTime = 0;
    double gridStep = 0, planningMargin = 0, minPostSpacing = 0;
    int maxPosts = 0;
    simtime_t failureTime;

    // scenario
    cModule *truck = nullptr;
    Coord motherPosition;            // tethered antenna above the truck
    std::vector<Drone> drones;
    std::vector<cModule *> users;
    std::vector<Coord> userPositions;
    std::vector<bool> userCovered;
    std::vector<bool> userCoverable; // some candidate post (or the mother) could cover it
    std::vector<Post> posts;
    std::vector<simtime_t> packReadyAt; // spare packs on the truck: time at which each is charged

    // timers
    cMessage *tickTimer = nullptr;
    cMessage *launchTimer = nullptr;
    cMessage *failureTimer = nullptr;
    size_t nextInitialLaunch = 0;

    // statistics
    simtime_t lastTick;
    double coverageIntegral = 0;
    double plannedCoverage = 0;
    simtime_t timeTo50 = -1, timeTo90 = -1, timeToPlanned = -1;
    long numSwaps = 0, numFailures = 0, numGaps = 0, numLaunches = 0;
    double lastCoverage = 0;
    int lastConnected = 0;

    static simsignal_t coverageSignal, coveredUsersSignal, dronesAirborneSignal, dronesOnStationSignal,
                       dronesConnectedSignal, postsHeldSignal, batterySwapsSignal;

    // canvas figures (2D view)
    cGroupFigure *planLayer = nullptr;

  protected:
    virtual int numInitStages() const override { return inet::NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void refreshDisplay() const override;
    virtual void finish() override;

    void collectNodes();
    void planPosts();
    void pruneUselessBridges();
    void drawPlan();
    void drawScale(cCanvas *canvas, const inet::CanvasProjection *projection);
    void launchNextInitial();
    void tick();
    void updateDrone(int i);
    bool launch(int postIndex);
    int pickReadyDrone() const;
    void startSwap(int i);
    void injectFailure();
    void failDrone(int i, const char *reason);
    void evaluateCoverage();

    bool linked(const Coord& a, const Coord& b, double range) const;
    double airbornePower() const;
    double usableEndurance() const;
    std::string droneLabel(int i) const;

  public:
    virtual ~HiveController();
    virtual std::string str() const override;
};

} // namespace aien

#endif
