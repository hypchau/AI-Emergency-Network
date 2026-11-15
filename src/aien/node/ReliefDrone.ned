package aien.node;

import aien.drone.DroneBattery;
import inet.node.aodv.AodvRouter;

//
// Relay drone: DJI Matrice 400 carrying an airborne 4G base station and a mesh
// radio. In this first version both radio roles are abstracted by a single
// unit-disk interface (see omnetpp.ini); the node routes with AODV, so it relays
// the traffic of the phones and of the other drones towards the hive.
//
module ReliefDrone extends AodvRouter
{
    parameters:
        @display("i=aien/drone_parked");
        mobility.typename = default("aien.mobility.DroneMobility");
    submodules:
        battery: DroneBattery {
            @display("p=125,480");
        }
}
