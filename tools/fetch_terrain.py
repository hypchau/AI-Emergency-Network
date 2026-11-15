#!/usr/bin/env python3
"""
Download the real relief and map of the simulated area (run once, on a PC
with Internet access):

    pip install pillow
    python tools/fetch_terrain.py

Outputs
  simulations/terrain/rasuwa_dem.asc  ESRI ASCII elevation grid, WGS84, metres
  simulations/terrain/rasuwa_dem.prj  its coordinate reference system
  images/maps/rasuwa.png              OpenStreetMap background for the 2D view,
                                      stretched exactly over the simulation scene

Data sources
  Elevation: AWS Open Data "Terrain Tiles" (Mapzen/Tilezen, terrarium encoding),
             mostly SRTM 30 m in this region. https://registry.opendata.aws/terrain-tiles/
  Map:       OpenStreetMap standard tiles, (c) OpenStreetMap contributors (ODbL).
             Respect the tile usage policy: this script downloads ~80 tiles once,
             with a delay, and caches them in tools/cache/.

The scene bounds below MUST match omnetpp.ini (coordinateSystem.sceneLatitude/
Longitude = north-west corner) and EmergencyNetwork.ned (sceneWidth/Height).
"""
import argparse
import math
import os
import sys
import time
import urllib.request

try:
    from PIL import Image, ImageDraw
except ImportError:
    sys.exit("Pillow is required:  pip install pillow")

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
CACHE = os.path.join(HERE, "cache")

# Simulation scene (keep in sync with omnetpp.ini / EmergencyNetwork.ned)
NORTH, SOUTH = 28.30, 28.09
WEST, EAST = 85.27, 85.40
# The DEM extends a little beyond the scene so that 3D edges look right
DEM_MARGIN = 0.02  # degrees
DEM_CELL = 0.0003  # degrees (~33 m north-south, ~29 m east-west)
DEM_ZOOM = 13      # terrarium zoom level (~19 m/pixel here)
MAP_ZOOM = 14      # OSM zoom level for the 2D background
MAP_WIDTH = 1600   # pixels of the 2D background image

TERRARIUM_URL = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png"
OSM_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
USER_AGENT = "AIEmergencyNetwork-PFE/1.0 (student project, OMNeT++ simulation; one-off download)"

WGS84_PRJ = ('GEOGCS["WGS 84",DATUM["WGS_1984",SPHEROID["WGS 84",6378137,298.257223563]],'
             'PRIMEM["Greenwich",0],UNIT["degree",0.0174532925199433]]')


# ----------------------------------------------------------------- tiles ----
def mercator_pixel(lat, lon, zoom):
    """Global pixel coordinates (float) of a WGS84 point in Web Mercator at `zoom`."""
    n = 256 * (2 ** zoom)
    x = (lon + 180.0) / 360.0 * n
    lat_r = math.radians(lat)
    y = (1.0 - math.log(math.tan(lat_r) + 1.0 / math.cos(lat_r)) / math.pi) / 2.0 * n
    return x, y


def fetch_tile(url_template, kind, z, x, y, delay):
    path = os.path.join(CACHE, kind, str(z), str(x), "%d.png" % y)
    if not os.path.exists(path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        url = url_template.format(z=z, x=x, y=y)
        req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
        with urllib.request.urlopen(req, timeout=30) as r:
            data = r.read()
        with open(path, "wb") as f:
            f.write(data)
        time.sleep(delay)
    return Image.open(path).convert("RGB")


class Mosaic:
    """The tiles covering a lat/lon box at one zoom level, as one image."""

    def __init__(self, url, kind, zoom, north, south, west, east, delay):
        x0, y0 = mercator_pixel(north, west, zoom)
        x1, y1 = mercator_pixel(south, east, zoom)
        self.tx0, self.ty0 = int(x0 // 256), int(y0 // 256)
        tx1, ty1 = int(x1 // 256), int(y1 // 256)
        self.zoom = zoom
        cols, rows = tx1 - self.tx0 + 1, ty1 - self.ty0 + 1
        print("  %s: %d x %d tiles at zoom %d" % (kind, cols, rows, zoom))
        self.image = Image.new("RGB", (cols * 256, rows * 256))
        for i in range(cols):
            for j in range(rows):
                tile = fetch_tile(url, kind, zoom, self.tx0 + i, self.ty0 + j, delay)
                self.image.paste(tile, (i * 256, j * 256))
        self.pixels = self.image.load()
        self.w, self.h = self.image.size

    def sample(self, lat, lon):
        """Bilinear sample (r, g, b floats) at a WGS84 point."""
        gx, gy = mercator_pixel(lat, lon, self.zoom)
        fx = gx - self.tx0 * 256 - 0.5
        fy = gy - self.ty0 * 256 - 0.5
        x0, y0 = int(math.floor(fx)), int(math.floor(fy))
        tx, ty = fx - x0, fy - y0
        x0 = min(max(x0, 0), self.w - 2)
        y0 = min(max(y0, 0), self.h - 2)
        p = self.pixels
        out = []
        for k in range(3):
            a = p[x0, y0][k] * (1 - tx) + p[x0 + 1, y0][k] * tx
            b = p[x0, y0 + 1][k] * (1 - tx) + p[x0 + 1, y0 + 1][k] * tx
            out.append(a * (1 - ty) + b * ty)
        return out


def terrarium_height(rgb):
    r, g, b = rgb
    return r * 256.0 + g + b / 256.0 - 32768.0


# ------------------------------------------------------------------- DEM ----
def make_dem(delay):
    north, south = NORTH + DEM_MARGIN, SOUTH - DEM_MARGIN
    west, east = WEST - DEM_MARGIN, EAST + DEM_MARGIN
    print("Elevation (AWS Terrain Tiles)...")
    # terrarium values must not be interpolated per channel: sample the 4
    # neighbours, decode, then interpolate the heights
    mosaic = Mosaic(TERRARIUM_URL, "terrarium", DEM_ZOOM, north, south, west, east, delay)
    ncols = int(round((east - west) / DEM_CELL))
    nrows = int(round((north - south) / DEM_CELL))
    p = mosaic.pixels
    rows = []
    zmin, zmax = 1e9, -1e9
    for r in range(nrows):
        lat = north - (r + 0.5) * DEM_CELL
        values = []
        for c in range(ncols):
            lon = west + (c + 0.5) * DEM_CELL
            gx, gy = mercator_pixel(lat, lon, DEM_ZOOM)
            fx = gx - mosaic.tx0 * 256 - 0.5
            fy = gy - mosaic.ty0 * 256 - 0.5
            x0, y0 = int(math.floor(fx)), int(math.floor(fy))
            tx, ty = fx - x0, fy - y0
            x0 = min(max(x0, 0), mosaic.w - 2)
            y0 = min(max(y0, 0), mosaic.h - 2)
            h00 = terrarium_height(p[x0, y0])
            h01 = terrarium_height(p[x0 + 1, y0])
            h10 = terrarium_height(p[x0, y0 + 1])
            h11 = terrarium_height(p[x0 + 1, y0 + 1])
            z = (h00 * (1 - tx) + h01 * tx) * (1 - ty) + (h10 * (1 - tx) + h11 * tx) * ty
            zmin, zmax = min(zmin, z), max(zmax, z)
            values.append("%.1f" % z)
        rows.append(" ".join(values))
    out_dir = os.path.join(ROOT, "simulations", "terrain")
    os.makedirs(out_dir, exist_ok=True)
    asc = os.path.join(out_dir, "rasuwa_dem.asc")
    with open(asc, "w") as f:
        f.write("ncols %d\nnrows %d\nxllcorner %.6f\nyllcorner %.6f\ncellsize %.6f\nNODATA_value -9999\n"
                % (ncols, nrows, west, south, DEM_CELL))
        f.write("\n".join(rows))
        f.write("\n")
    with open(os.path.join(out_dir, "rasuwa_dem.prj"), "w") as f:
        f.write(WGS84_PRJ)
    print("  -> %s (%d x %d cells, %.0f..%.0f m)" % (os.path.relpath(asc, ROOT), ncols, nrows, zmin, zmax))


# ------------------------------------------------------------------- map ----
def make_map(delay):
    print("2D background (OpenStreetMap)...")
    mosaic = Mosaic(OSM_URL, "osm", MAP_ZOOM, NORTH, SOUTH, WEST, EAST, delay)
    # Same projection as INET's SimpleGeographicCoordinateSystem: x and y are
    # linear in longitude and latitude, so the image can simply be stretched
    # over the scene rectangle.
    lat0 = math.radians(NORTH)
    width_m = (EAST - WEST) * math.cos(lat0) * 111320
    height_m = (NORTH - SOUTH) * 111320
    w = MAP_WIDTH
    h = int(round(w * height_m / width_m))
    img = Image.new("RGB", (w, h))
    px = img.load()
    for j in range(h):
        lat = NORTH - (j + 0.5) / h * (NORTH - SOUTH)
        for i in range(w):
            lon = WEST + (i + 0.5) / w * (EAST - WEST)
            r, g, b = mosaic.sample(lat, lon)
            px[i, j] = (int(r), int(g), int(b))
    d = ImageDraw.Draw(img)
    text = "(c) OpenStreetMap contributors"
    d.rectangle([w - 230, h - 22, w, h], fill=(255, 255, 255))
    d.text((w - 224, h - 18), text, fill=(60, 60, 60))
    out_dir = os.path.join(ROOT, "images", "maps")
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, "rasuwa.png")
    img.save(out)
    print("  -> %s (%d x %d px for %.0f x %.0f m)" % (os.path.relpath(out, ROOT), w, h, width_m, height_m))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--skip-dem", action="store_true", help="do not build the elevation grid")
    ap.add_argument("--skip-map", action="store_true", help="do not build the 2D background")
    ap.add_argument("--delay", type=float, default=0.2, help="pause between tile downloads (s)")
    args = ap.parse_args()
    if not args.skip_dem:
        make_dem(args.delay)
    if not args.skip_map:
        make_map(args.delay)
    print("Done. Run the Rasuwa2D or Rasuwa3D configuration.")
