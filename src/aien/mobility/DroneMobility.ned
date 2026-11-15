package aien.mobility;

import inet.mobility.base.MovingMobilityBase;

//
// Flight model of a relay drone (defaults: DJI Matrice 400).
//
// The drone is commanded by the hive controller:
//   - flyTo(post): vertical climb above the hive, then straight cruise at a
//     constant height above the real terrain (terrain following), then hover;
//   - returnHome(): cruise back to the hive and land for a battery swap.
//
// Positions are geographic (WGS84) and converted to scene coordinates through
// the coordinate system module, so the same model works on the 2D canvas and
// on the osgEarth 3D globe.
//
simple DroneMobility extends MovingMobilityBase
{
    parameters:
        @class(DroneMobility);
        string terrainModule = default("terrain");
        double homeLatitude @unit(deg);           // hive (truck) position
        double homeLongitude @unit(deg);
        double parkingRadius @unit(m) = default(15m); // drones are parked on a ring around the truck
        double parkingHeight @unit(m) = default(3m);  // height of the landing deck above the ground
        double flightHeight @unit(m) = default(100m); // height above ground level when flying / hovering
        double cruiseSpeed @unit(mps) = default(18mps); // M400: 25 m/s max, 17 m/s used by DJI for max range
        double climbRate @unit(mps) = default(6mps);    // M400: 10 m/s max ascent
        double descentRate @unit(mps) = default(5mps);  // M400: 8 m/s max descent
        updateInterval = default(0.5s);
        faceForward = default(false);
}
