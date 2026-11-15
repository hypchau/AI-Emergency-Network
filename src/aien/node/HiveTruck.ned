package aien.node;

import aien.hive.HiveController;
import inet.node.aodv.AodvRouter;

//
// Off-road truck acting as the "hive": it carries the mother antenna (tethered
// drone at ~100 m, unlimited endurance), the satellite backhaul, the battery
// charging station, the parked relay drones and the controller that plans and
// rotates them. It is the destination of all the SOS traffic (UdpSink).
//
module HiveTruck extends AodvRouter
{
    parameters:
        @display("i=aien/truck");
        mobility.typename = default("aien.mobility.GroundMobility");
        mobility.scatterRadius = default(0m);
        mobility.heightAboveGround = default(3m);
        // the mother antenna flies ~100 m above the truck: its height is taken
        // into account by the hive controller (coverage and line of sight)
    submodules:
        hive: HiveController {
            @display("p=125,480");
        }
}
