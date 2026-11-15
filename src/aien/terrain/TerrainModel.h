#ifndef __AIEN_TERRAINMODEL_H
#define __AIEN_TERRAINMODEL_H

#include <vector>

#include "inet/common/INETDefs.h"
#include "inet/common/ModuleRefByPar.h"
#include "inet/common/geometry/common/GeographicCoordinateSystem.h"

namespace aien {

using namespace omnetpp;
using inet::Coord;
using inet::GeoCoord;

/**
 * Digital elevation model (ESRI ASCII grid, WGS84) + line-of-sight helper.
 * See TerrainModel.ned.
 */
class TerrainModel : public cSimpleModule
{
  protected:
    // grid (row 0 = northernmost row, as in the .asc format)
    int ncols = 0, nrows = 0;
    double west = 0, south = 0, cellsize = 0; // degrees
    double nodata = -9999;
    std::vector<float> grid;
    bool demLoaded = false;

    double flatElevation = 0;
    double losSampleStep = 60;
    double losClearance = 5;

    mutable inet::IGeographicCoordinateSystem *coordinateSystem = nullptr;

  protected:
    virtual int numInitStages() const override { return inet::NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void handleMessage(cMessage *msg) override { throw cRuntimeError("TerrainModel does not process messages"); }
    void loadAsciiGrid(const char *fileName);
    double cell(int row, int col) const;
    inet::IGeographicCoordinateSystem *getCoordinateSystem() const;

  public:
    bool hasDem() const { return demLoaded; }

    /** Ground altitude (m above sea level) at a geographic position (degrees). */
    double getElevation(double latitudeDeg, double longitudeDeg) const;

    /** Ground altitude below a scene position. */
    double getElevationBelow(const Coord& scenePosition) const;

    /** Scene coordinate of a geographic position given its altitude above sea level. */
    Coord toScene(double latitudeDeg, double longitudeDeg, double altitudeAsl) const;

    /** Geographic coordinate (degrees, metres) of a scene position. */
    void toGeo(const Coord& scenePosition, double& latitudeDeg, double& longitudeDeg, double& altitudeAsl) const;

    /** Scene position located `heightAgl` metres above the ground at (lat, lon). */
    Coord abovePosition(double latitudeDeg, double longitudeDeg, double heightAgl) const;

    /** Scene position located `heightAgl` metres above the ground below the horizontal position of p. */
    Coord aboveScenePosition(const Coord& p, double heightAgl) const;

    /** True if the straight segment a-b stays above the terrain (plus clearance). */
    bool hasLineOfSight(const Coord& a, const Coord& b) const;
};

} // namespace aien

#endif
