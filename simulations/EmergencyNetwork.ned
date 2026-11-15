package aien.simulations;

import aien.node.HiveTruck;
import aien.node.ReliefDrone;
import aien.node.Victim;
import aien.terrain.TerrainModel;
import inet.common.geometry.common.IGeographicCoordinateSystem;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.physicallayer.wireless.common.contract.packetlevel.IRadioMedium;
import inet.visualizer.common.IntegratedVisualizer;

//
// AI Emergency Network: a hive truck deploys numDrones relay drones over a
// disaster area to restore phone coverage for numUsers victims and rescuers.
//
// The scene is geographic: every position is given in latitude/longitude and
// placed on the real terrain (TerrainModel). Scene origin = north-west corner
// of the area (coordinateSystem.sceneLatitude/Longitude), x towards the east,
// y towards the south, in metres.
//
// 2D view: the background image (terrain.mapImage) is an OpenStreetMap
// rendering of the area made by tools/fetch_terrain.py, stretched over
// sceneWidth x sceneHeight.
// 3D view: osgEarth globe with the real relief (see the Rasuwa3D configuration).
//
network EmergencyNetwork
{
    parameters:
        int numDrones = default(10);       // fleet size, to be chosen by the AI later (1..20)
        int numUsers = default(50);
        double hiveLatitude @unit(deg);
        double hiveLongitude @unit(deg);
        double sceneWidth = default(12742);  // metres, must match the map image extent
        double sceneHeight = default(23377); // metres
        @display("bgb=$sceneWidth,$sceneHeight,#E8EEE4;bgu=m");
    submodules:
        // the visualizer must come first: in 3D it loads the earth map used by the coordinate system
        visualizer: IntegratedVisualizer {
            @display("p=300,300;is=vl");
        }
        coordinateSystem: <default(firstAvailableOrEmpty("SimpleGeographicCoordinateSystem"))> like IGeographicCoordinateSystem {
            @display("p=300,900;is=vl");
        }
        terrain: TerrainModel {
            @display("p=300,1500;is=vl");
        }
        configurator: Ipv4NetworkConfigurator {
            @display("p=300,2100;is=vl");
        }
        radioMedium: <default("UnitDiskRadioMedium")> like IRadioMedium {
            @display("p=300,2700;is=vl");
        }
        truck: HiveTruck {
            mobility.latitude = parent.hiveLatitude;
            mobility.longitude = parent.hiveLongitude;
            @display("is=vl");
        }
        drone[numDrones]: ReliefDrone {
            mobility.homeLatitude = parent.hiveLatitude;
            mobility.homeLongitude = parent.hiveLongitude;
            @display("is=l");
        }
        user[numUsers]: Victim {
            @display("is=n");
        }
}
