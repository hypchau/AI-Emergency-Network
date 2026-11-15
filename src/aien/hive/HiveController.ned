package aien.hive;

//
// Brain of the hive truck. It stands in for the AI orchestrator of the team:
// the number of drones is given (1..20), the controller decides where to put
// them and keeps them there with a battery rotation.
//
// 1. Post planning (at start-up), greedy and cost-aware:
//    - candidate posts on a grid (gridStep) over the area, at flightHeight above
//      the real terrain;
//    - a candidate must be linked to the mother antenna or to an already chosen
//      post (meshRange + line of sight over the DEM);
//    - for each candidate, the number of drones needed to hold it continuously is
//          S = E(1-r) - t_out - t_back - handoverMargin   (useful time on post)
//          N = ceil((E(1-r) + batterySwapTime) / S)
//      (E = endurance of a full pack, r = reserveRatio);
//    - candidates are ranked by newly covered phones per drone; when no
//      candidate covers new phones, a "bridge" post extends the chain towards
//      the nearest uncovered phone;
//    - posts are added while the fleet (numDrones) can sustain them.
// 2. Launch: one drone per post, staggered by launchInterval ("bees leaving
//    the hive").
// 3. Rotation: when the drone on a post will soon have to come back (return
//    energy + reserve), a fresh drone takes off so that it arrives before the
//    old one leaves. Landed drones get a charged pack (batterySwapTime); the
//    discharged pack goes to the charger (fullChargeTime for 0->100 %).
// 4. Failure injection (optional): a drone on post falls; the post is refilled.
// 5. Metrics every tick: covered phones (phone within accessRange of an
//    airborne drone that is connected to the hive, or within motherRange of the
//    mother antenna, line of sight required), connected drones, etc.
//
simple HiveController
{
    parameters:
        @class(HiveController);
        @display("i=block/cogwheel");
        string terrainModule = default("terrain");
        string truckModule = default("^");       // the controller runs inside the truck
        string droneModule = default("drone");   // top-level vector of ReliefDrone
        string userModule = default("user");     // top-level vector of Victim

        double tickInterval @unit(s) = default(1s);
        double startTime @unit(s) = default(10s);
        double launchInterval @unit(s) = default(20s);

        // radio abstraction (keep consistent with the radio ranges in omnetpp.ini)
        double accessRange @unit(m) = default(2000m);  // airborne 4G cell radius (Nokia/Optus: ~2 km)
        double meshRange @unit(m) = default(5000m);    // drone-to-drone mesh radio (conservative in a valley)
        double motherRange @unit(m) = default(2500m);  // tethered mother antenna above the truck
        double motherHeight @unit(m) = default(100m);  // Orion 2.2 TE: 100 m tether; EON800: 120 m
        bool useLineOfSight = default(true);           // only meaningful with a DEM

        // operations
        double reserveRatio = default(0.20);              // energy kept for safety on landing
        double handoverMargin @unit(s) = default(60s);    // overlap between the old and the new drone on a post
        double batterySwapTime @unit(s) = default(300s);  // swap a TB100 on the truck
        double fullChargeTime @unit(s) = default(2700s);  // BS100 station, 220 V fast mode: 45 min for 0->100 %
        int sparePacks = default(-1);                     // charged packs on the truck at start, -1 = 2 per drone
        double minStationTime @unit(s) = default(120s);   // a post is not worth holding below this

        // planning
        double gridStep @unit(m) = default(300m);
        double planningMargin @unit(m) = default(1500m);  // grid extends this far beyond phones and truck
        double minPostSpacing @unit(m) = default(1200m);
        int maxPosts = default(20);

        // failure injection: at failureTime, one random drone on post crashes (-1s = never)
        double failureTime @unit(s) = default(-1s);

        @signal[coverage](type=double);
        @signal[coveredUsers](type=long);
        @signal[dronesAirborne](type=long);
        @signal[dronesOnStation](type=long);
        @signal[dronesConnected](type=long);
        @signal[postsHeld](type=long);
        @signal[batterySwaps](type=long);
        @statistic[coverage](title="ratio of phones covered"; unit=ratio; record=vector,timeavg,max,last; interpolationmode=sample-hold);
        @statistic[coveredUsers](title="phones covered"; record=vector,timeavg,max; interpolationmode=sample-hold);
        @statistic[dronesAirborne](title="drones airborne"; record=vector,timeavg,max; interpolationmode=sample-hold);
        @statistic[dronesOnStation](title="drones on post"; record=vector,timeavg; interpolationmode=sample-hold);
        @statistic[dronesConnected](title="airborne drones connected to the hive"; record=vector,timeavg; interpolationmode=sample-hold);
        @statistic[postsHeld](title="posts held"; record=vector,timeavg; interpolationmode=sample-hold);
        @statistic[batterySwaps](title="battery swaps"; record=last; interpolationmode=sample-hold);
}
