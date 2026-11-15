#include "GroundMobility.h"

#include <cmath>

#include "../terrain/TerrainModel.h"

namespace aien {

Define_Module(GroundMobility);

void GroundMobility::setInitialPosition()
{
    auto terrain = check_and_cast<TerrainModel *>(getModuleByPath(par("terrainModule")));
    double lat = par("latitude").doubleValueInUnit("deg");
    double lon = par("longitude").doubleValueInUnit("deg");
    double radius = par("scatterRadius").doubleValueInUnit("m");
    double height = par("heightAboveGround").doubleValueInUnit("m");
    Coord center = terrain->toScene(lat, lon, 0);
    center.z = 0;
    Coord p = center;
    if (radius > 0) {
        // uniform in a disc
        double r = radius * std::sqrt(uniform(0, 1));
        double a = uniform(0, 2 * M_PI);
        p += Coord(r * cos(a), r * sin(a), 0);
    }
    lastPosition = terrain->aboveScenePosition(p, height);
}

} // namespace aien
