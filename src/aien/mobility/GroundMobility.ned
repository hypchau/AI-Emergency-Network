package aien.mobility;

import inet.mobility.base.MobilityBase;

//
// Stationary node placed on the real ground (DEM): a victim's or rescuer's
// phone, or the hive truck.
//
// The node is placed at (latitude, longitude), optionally scattered uniformly
// in a disc of radius scatterRadius, at heightAboveGround above the terrain.
//
simple GroundMobility extends MobilityBase
{
    parameters:
        @class(GroundMobility);
        string terrainModule = default("terrain");
        double latitude @unit(deg);
        double longitude @unit(deg);
        double scatterRadius @unit(m) = default(0m);
        double heightAboveGround @unit(m) = default(1.5m);
}
