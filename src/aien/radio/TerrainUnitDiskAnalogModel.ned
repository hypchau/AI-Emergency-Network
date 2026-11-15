package aien.radio;

import inet.physicallayer.wireless.unitdisk.UnitDiskAnalogModel;

//
// Unit-disk propagation adapted to the emergency network:
//  - links with a phone (victim or rescuer, node name starting with
//    phoneNodePrefix) are limited to phoneRange in BOTH directions (4G access
//    cell radius), while drone/hive links use the transmitter's
//    communicationRange (mesh radio). Links stay symmetric, which AODV needs.
//  - a link is blocked when the relief hides one end from the other (line of
//    sight tested on the same DEM as the hive controller).
// Signals that cannot be received still interfere up to the transmitter's
// interferenceRange (when not blocked by the relief).
//
module TerrainUnitDiskAnalogModel extends UnitDiskAnalogModel
{
    parameters:
        @class(TerrainUnitDiskAnalogModel);
        string terrainModule = default("terrain");
        string phoneNodePrefix = default("user");
        double phoneRange @unit(m) = default(2000m);
        bool useLineOfSight = default(true);
        double losCacheGrid @unit(m) = default(25m); // positions are rounded to this grid to cache line-of-sight results
}
