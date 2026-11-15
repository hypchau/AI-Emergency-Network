//
// AI Emergency Network - drone relay network restoring 4G coverage after a
// natural disaster (PFE, Setelia).
//
// Package layout:
//   aien.terrain   real elevation model (DEM) + line-of-sight
//   aien.mobility  drone flight model (terrain following) and ground nodes
//   aien.drone     drone battery model (DJI Matrice 400 + TB100)
//   aien.hive      the truck "hive": post planning, launch and battery rotation
//   aien.node      network nodes (relay drone, victim/rescuer phone, hive truck)
//
@license(LGPL);
@namespace(aien);
package aien;
