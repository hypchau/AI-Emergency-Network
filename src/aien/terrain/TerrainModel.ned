package aien.terrain;

//
// Digital elevation model of the simulated area.
//
// Reads an ESRI ASCII grid (.asc, WGS84 lat/lon, elevations in metres above
// sea level). The same file is used by osgEarth to draw the 3D relief, so the
// drones fly exactly over the terrain that is displayed.
// Generate it with tools/fetch_terrain.py (real data, AWS Terrain Tiles).
//
// If demFile is empty the ground is flat at flatElevation.
//
// Also provides a line-of-sight test (sampling the DEM along the segment),
// used by the hive controller to plan posts and to evaluate coverage.
//
simple TerrainModel
{
    parameters:
        @display("i=block/table2;is=s");
        string demFile = default("");            // ESRI ASCII grid, path relative to the working directory
        double flatElevation @unit(m) = default(0m); // ground altitude when no DEM is loaded
        double losSampleStep @unit(m) = default(60m); // sampling step of the line-of-sight test
        double losClearance @unit(m) = default(5m);   // margin above the terrain required for line of sight
        string coordinateSystemModule = default("coordinateSystem");
        string mapImage = default("");  // 2D background (image name relative to the image path, e.g. "maps/rasuwa"), stretched over the network's bgb area
}
