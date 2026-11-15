#include "TerrainModel.h"

#include <cmath>
#include <fstream>
#include <sstream>
#include <algorithm>

namespace aien {

Define_Module(TerrainModel);

using namespace inet::units::values;

void TerrainModel::initialize(int stage)
{
    if (stage == inet::INITSTAGE_LOCAL) {
        flatElevation = par("flatElevation").doubleValueInUnit("m");
        losSampleStep = par("losSampleStep").doubleValueInUnit("m");
        losClearance = par("losClearance").doubleValueInUnit("m");
        const char *demFile = par("demFile");
        if (*demFile)
            loadAsciiGrid(demFile);
        else
            EV_WARN << "No DEM file given: flat ground at " << flatElevation << " m" << endl;
        const char *mapImage = par("mapImage");
        if (*mapImage && hasGUI()) {
            cDisplayString& ds = getSystemModule()->getDisplayString();
            ds.setTagArg("bgi", 0, mapImage);
            ds.setTagArg("bgi", 1, "s");
        }
        WATCH(demLoaded);
        WATCH(ncols);
        WATCH(nrows);
    }
}

void TerrainModel::loadAsciiGrid(const char *fileName)
{
    std::ifstream in(fileName);
    if (!in)
        throw cRuntimeError("Cannot open DEM file '%s'. Run tools/fetch_terrain.py first (see README), "
                            "or set **.terrain.demFile = \"\" to simulate on flat ground.", fileName);
    // header: ncols, nrows, xllcorner|xllcenter, yllcorner|yllcenter, cellsize, [NODATA_value]
    bool center = false;
    double xll = 0, yll = 0;
    std::string key;
    int headerItems = 0;
    while (headerItems < 6 && in >> key) {
        std::transform(key.begin(), key.end(), key.begin(), ::tolower);
        if (key == "ncols") { in >> ncols; headerItems++; }
        else if (key == "nrows") { in >> nrows; headerItems++; }
        else if (key == "xllcorner") { in >> xll; headerItems++; }
        else if (key == "yllcorner") { in >> yll; headerItems++; }
        else if (key == "xllcenter") { in >> xll; center = true; headerItems++; }
        else if (key == "yllcenter") { in >> yll; center = true; headerItems++; }
        else if (key == "cellsize") { in >> cellsize; headerItems++; }
        else if (key == "nodata_value") { in >> nodata; headerItems++; }
        else {
            // first data value reached (header without NODATA_value)
            grid.push_back((float)std::stod(key));
            break;
        }
    }
    if (ncols <= 0 || nrows <= 0 || cellsize <= 0)
        throw cRuntimeError("Invalid ESRI ASCII grid header in '%s'", fileName);
    // west/south = outer edges of the grid; cell centres are half a cell inside
    west = center ? xll - cellsize / 2 : xll;
    south = center ? yll - cellsize / 2 : yll;
    grid.reserve((size_t)ncols * nrows);
    double v;
    while ((int)grid.size() < ncols * nrows && in >> v)
        grid.push_back((float)v);
    if ((int)grid.size() != ncols * nrows)
        throw cRuntimeError("DEM '%s' truncated: %d values instead of %d", fileName, (int)grid.size(), ncols * nrows);
    demLoaded = true;
    float mn = 1e9, mx = -1e9;
    for (float f : grid)
        if (f != nodata) { mn = std::min(mn, f); mx = std::max(mx, f); }
    EV_INFO << "DEM loaded from " << fileName << ": " << ncols << "x" << nrows << " cells of " << cellsize
            << " deg, lon " << west << ".." << west + ncols * cellsize << ", lat " << south << ".." << south + nrows * cellsize
            << ", elevation " << mn << ".." << mx << " m" << endl;
}

double TerrainModel::cell(int row, int col) const
{
    row = std::clamp(row, 0, nrows - 1);
    col = std::clamp(col, 0, ncols - 1);
    float v = grid[(size_t)row * ncols + col];
    return v == nodata ? flatElevation : v;
}

double TerrainModel::getElevation(double lat, double lon) const
{
    if (!demLoaded)
        return flatElevation;
    // fractional position of the point relative to cell centres
    double fx = (lon - west) / cellsize - 0.5;                 // column, increasing eastwards
    double fy = (south + nrows * cellsize - lat) / cellsize - 0.5; // row, increasing southwards
    int c0 = (int)std::floor(fx), r0 = (int)std::floor(fy);
    double tx = fx - c0, ty = fy - r0;
    double v00 = cell(r0, c0), v01 = cell(r0, c0 + 1);
    double v10 = cell(r0 + 1, c0), v11 = cell(r0 + 1, c0 + 1);
    return (1 - ty) * ((1 - tx) * v00 + tx * v01) + ty * ((1 - tx) * v10 + tx * v11);
}

inet::IGeographicCoordinateSystem *TerrainModel::getCoordinateSystem() const
{
    if (coordinateSystem == nullptr) {
        const char *path = par("coordinateSystemModule");
        cModule *module = getModuleByPath(path);
        if (module == nullptr)
            module = getSimulation()->getSystemModule()->getSubmodule(path);
        coordinateSystem = dynamic_cast<inet::IGeographicCoordinateSystem *>(module);
        if (coordinateSystem == nullptr)
            throw cRuntimeError("Geographic coordinate system module '%s' not found", path);
    }
    return coordinateSystem;
}

Coord TerrainModel::toScene(double lat, double lon, double alt) const
{
    return getCoordinateSystem()->computeSceneCoordinate(GeoCoord(deg(lat), deg(lon), m(alt)));
}

void TerrainModel::toGeo(const Coord& p, double& lat, double& lon, double& alt) const
{
    GeoCoord g = getCoordinateSystem()->computeGeographicCoordinate(p);
    lat = g.latitude.get();
    lon = g.longitude.get();
    alt = g.altitude.get();
}

double TerrainModel::getElevationBelow(const Coord& p) const
{
    double lat, lon, alt;
    toGeo(p, lat, lon, alt);
    return getElevation(lat, lon);
}

Coord TerrainModel::abovePosition(double lat, double lon, double heightAgl) const
{
    return toScene(lat, lon, getElevation(lat, lon) + heightAgl);
}

Coord TerrainModel::aboveScenePosition(const Coord& p, double heightAgl) const
{
    double lat, lon, alt;
    toGeo(p, lat, lon, alt);
    return toScene(lat, lon, getElevation(lat, lon) + heightAgl);
}

bool TerrainModel::hasLineOfSight(const Coord& a, const Coord& b) const
{
    if (!demLoaded)
        return true;
    double length = a.distance(b);
    int n = std::max(2, (int)std::ceil(length / losSampleStep));
    for (int i = 1; i < n; i++) {
        Coord p = a + (b - a) * ((double)i / n);
        double lat, lon, alt;
        toGeo(p, lat, lon, alt);
        if (alt < getElevation(lat, lon) + losClearance)
            return false;
    }
    return true;
}

} // namespace aien
