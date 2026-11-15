# AI Emergency Network

Simulation OMNeT++/INET d'un réseau de drones relais qui rétablit la couverture téléphonique (4G) après une catastrophe naturelle. Le projet est un PFE d'école d'ingénieur, proposé par Setelia.

Un camion tout-terrain sert de **ruche**. Il porte l'antenne mère (un drone captif à 100 m), la liaison satellite et l'atelier de batteries. Il lâche N drones DJI Matrice 400, chacun équipé d'une station de base 4G aéroportée. Les drones forment une chaîne maillée jusqu'aux victimes et se relaient pour tenir leurs postes malgré leur autonomie limitée.

Le scénario est réel : la **crue éclair de la Bhotekoshi** (district de Rasuwa, Népal) du 26 août 2026, qui a coupé la plupart des antennes du corridor Timure–Syabrubesi.

---

## Ce que montre la simulation

| Élément | Représentation |
| --- | --- |
| Camion-ruche (Syabrubesi) | icône camion ; le bandeau affiche couverture, postes tenus, drones reliés, batteries chargées |
| Drones relais | quadricoptères, couleur selon l'état : gris = prêt, bleu = aller, vert = sur poste, orange = retour, violet = échange de batterie, rouge = panne ; texte `ÉTAT Pn batterie%` |
| Téléphones (victimes, secours) | rouge avec « ! » = sans réseau, vert = couvert |
| Postes planifiés | losanges `P1, P2…` avec leur distance au camion ; pointillés = chaîne de liaison prévue |
| Couverture des drones | cercle de 2 km qui suit chaque drone déployé (pas la réserve) : vert = sur poste, bleu = aller, orange = retour, gris pointillé = coupé du camion |
| Repères | quadrillage de 1 km, échelle de 5 km, flèche du nord |
| Antenne mère | disque pointillé orange (2,5 km) autour du camion |
| Trafic SOS | flèches bleues des téléphones vers le camion, à travers les drones |

Le fond de carte et le relief sont réels (OpenStreetMap et SRTM). Les drones volent à 100 m au-dessus du terrain, et une crête masque les liaisons radio.

---

## Organisation du dépôt

```
src/aien/
  terrain/   TerrainModel        relief (grille ESRI ASCII) + test de visibilité
  mobility/  DroneMobility       vol commandé (montée, croisière en suivi de terrain, vol stationnaire, retour)
             GroundMobility      nœuds au sol posés sur le relief
  drone/     DroneBattery        TB100 977 Wh, vol 1990 W + charge utile 180 W  -> 27 min
  hive/      HiveController      planification des postes, décollages, relève, pannes, KPI
  radio/     TerrainUnitDiskAnalogModel  portée 2 km pour les téléphones, 5 km entre drones, relief bloquant
  node/      ReliefDrone, Victim, HiveTruck
simulations/
  EmergencyNetwork.ned           le réseau
  omnetpp.ini                    scénario Bhotekoshi + configurations
  rasuwa.earth                   carte 3D osgEarth
  run                            lanceur en ligne de commande
tools/
  fetch_terrain.py               télécharge le relief réel et le fond de carte (une fois)
  make_icons.py                  régénère les icônes
  analyse_results.py             courbes KPI (couverture, SOS livrés…)
images/aien/                     icônes
```

---

## Installation avec opp_env (Windows via WSL, Linux, macOS)

Prérequis : un workspace opp_env contenant **OMNeT++ 6.3.0** et **INET 4.5.4**, par exemple obtenu avec `opp_env install inet-4.5.4`. Le workspace contient alors les dossiers `omnetpp-6.3.0/` et `inet-4.5.4/`.

1. Dans le terminal WSL, clone ce dépôt **dans le workspace**, à côté de `inet-4.5.4/` :
   ```
   cd ~/mon_workspace                       # ton workspace opp_env
   git clone -b simulation-v1 https://github.com/hypchau/AI-Emergency-Network.git
   ```
2. Entre dans l'environnement : `opp_env shell`. OMNeT++ et INET ne sont utilisables que dans ce shell.
3. Au choix :
   - **IDE** : tape `omnetpp`. L'IDE s'ouvre sur le workspace. Si le projet `AI-Emergency-Network` n'apparaît pas, fais *File → Import → General → Existing Projects into Workspace*. Vérifie ensuite que le projet `inet` est coché dans *Project → Properties → Project References*, puis lance *Build Project*.
   - **Ligne de commande** :
     ```
     cd AI-Emergency-Network
     make makefiles          # INET est trouvé via $INET_ROOT, défini par opp_env
     make MODE=release
     cd simulations
     ./run -c QuickFlat      # ouvre Qtenv
     ```
4. Pour un premier test sans rien télécharger, prends **QuickFlat** : sol plat, pas de carte.

### Installation classique (OMNeT++ pour Windows, sans opp_env)

Importe le dossier dans le workspace où se trouve le projet INET (*File → Import → Existing Projects into Workspace*). Coche `inet` dans *Project References*, puis lance *Build Project*.

### Relief et carte réels (à faire une fois)

Le relief et la carte ne sont pas versionnés. Il faut les générer :

```
pip install pillow
python tools/fetch_terrain.py
```

Le script télécharge environ 40 tuiles d'altitude (AWS Terrain Tiles) et environ 80 tuiles OpenStreetMap, avec une pause entre deux téléchargements. Il écrit :

- `simulations/terrain/rasuwa_dem.asc` et `.prj` : le relief, utilisé à la fois par la simulation et par la vue 3D ;
- `images/maps/rasuwa.png` : le fond de carte 2D, calé exactement sur la scène.

Les configurations **Rasuwa2D**, **Rasuwa3D**, **Failure2D** et **FleetSweep** en ont besoin.

### Activer la 3D (configuration Rasuwa3D)

OMNeT++ doit être compilé avec OpenSceneGraph et osgEarth, ce qui n'est pas le cas par défaut. Avec opp_env, regarde les options proposées par `opp_env info omnetpp-6.3.0`. Avec une installation classique, mets `WITH_OSG=yes` et `WITH_OSGEARTH=yes` dans `configure.user`, puis relance `./configure` et `make`. Recompile ensuite INET, puis ce projet. Lance enfin **Rasuwa3D** et ouvre la vue 3D de Qtenv.

La carte est chargée depuis Internet et mise en cache dans `simulations/osgearth_cache/`.

> La vue 3D n'a pas pu être testée dans l'environnement où ce code a été écrit, faute d'osgEarth. Toute la logique de simulation (planification, vols, relève, radio, KPI) a été testée en 2D et en ligne de commande. En 3D, les icônes ne changent pas de couleur, car le visualiseur OSG d'INET ne les relit pas. L'état des drones s'affiche dans leur étiquette (batterie) et dans le bandeau du camion.

### Campagne de simulations

```
cd simulations
./run -u Cmdenv -c FleetSweep         # 1..20 drones x 3 graines, sans interface
python3 ../tools/analyse_results.py   # courbes dans plots/
```

---

## Configurations

| Configuration | Usage |
| --- | --- |
| `QuickFlat` | Vérification rapide, aucun téléchargement nécessaire |
| `Rasuwa2D` | Démonstration 2D : relief réel, fond OpenStreetMap |
| `Rasuwa3D` | Démonstration 3D sur le globe osgEarth |
| `Failure2D` | Un drone en poste tombe en panne à 30 min ; son poste est réarmé depuis la ruche |
| `FleetSweep` | Étude paramétrique : de 1 à 20 drones, 3 répétitions, en Cmdenv |

Le **nombre de drones** se règle par `*.numDrones = 10` (de 1 à 20). C'est le réglage que l'algorithme IA de l'équipe remplacera.

---

## Le contrôleur de la ruche (`HiveController`)

1. **Planification des postes**, au démarrage, par un algorithme glouton qui tient compte du coût :
   - les postes candidats forment une grille de 300 m, à 100 m au-dessus du relief ;
   - un candidat doit être relié à l'antenne mère ou à un poste déjà choisi : 5 km au plus, en ligne de vue sur le relief ;
   - pour chaque candidat, on calcule le temps utile sur poste et le nombre de drones nécessaires pour le tenir en continu :
     `S = E(1-r) - t_aller - t_retour - marge`  et  `N = ceil((E(1-r) + t_échange) / S)` ;
   - le contrôleur retient le meilleur rapport « téléphones nouvellement couverts par drone ». À défaut, il pose un poste relais vers le téléphone non couvert le plus proche, seulement si la flotte peut aussi tenir un poste au-delà ;
   - il ajoute des postes tant que la flotte peut les tenir.
2. **Décollage échelonné**, un drone toutes les 20 s, comme des abeilles qui quittent la ruche.
3. **Relève** : le remplaçant décolle pour arriver une minute avant que le drone en poste ne doive rentrer. Au retour, le drone reçoit une batterie chargée en 5 min, et la batterie vide part en charge (45 min de 0 à 100 %).
4. **Pannes** (configuration `Failure2D`) : le drone tombe et coupe sa radio, et le poste est réarmé.
5. **Indicateurs**, chaque seconde : téléphones couverts, drones reliés à la ruche, postes tenus, échanges de batteries, trous de couverture.

---

## Hypothèses (toutes réglables dans `omnetpp.ini`)

| Hypothèse | Valeur | Source / statut |
| --- | --- | --- |
| Drone | DJI Matrice 400, 18 m/s en croisière, 100 m au-dessus du sol | fiche DJI |
| Batterie | TB100, 977 Wh | fiche DJI |
| Autonomie avec la station 4G de 5,5 kg | 27 min (1990 W + 180 W) | **estimation** (loi d'échelle sur les 53 min de vol stationnaire annoncées par DJI), à valider |
| Réserve au retour | 20 % | choix de sécurité |
| Rayon de la cellule 4G | 2 km | essai Nokia/Optus |
| Portée du maillage | 5 km | prudente en vallée |
| Antenne mère | 2,5 km, à 100 m | drone captif (Elistair Orion 2.2 TE) |
| Échange / recharge de batterie | 5 min / 45 min | recharge : DJI BS100 |
| Positions des villages | Syabrubesi, Timure, Rasuwagadhi, Dhunche | coordonnées approximatives (Wikipedia/GeoNames) ; la position « mi-vallée » est une estimation |

## Limites connues (version 1)

- La radio est un modèle à **disque unitaire** à une seule interface (AODV d'INET ne gère qu'une interface). Les deux rôles, accès 4G et maillage, sont abstraits par des portées. La vraie couche 4G/5G (Simu5G) viendra à l'étape suivante.
- Le drone suit le relief à hauteur constante, sans modèle de vent. La consommation est la même en croisière et en stationnaire.
- Les téléphones sont immobiles.

## Pistes suivantes

- Accès 4G/5G réaliste avec **Simu5G 1.5.0** (eNB/gNB sur les drones) ;
- interface avec l'**IA** de l'équipe : nombre de drones et positions des postes lus dans un fichier ou fournis par un script Python ;
- deuxième camion-ruche pour mieux couvrir Timure et Rasuwagadhi, à plus de 10 km de Syabrubesi : à cette distance, le transit consomme presque toute l'autonomie du M400 ;
- victimes mobiles, trafic voix.

## Licences et attributions

- Fond de carte : © contributeurs OpenStreetMap (ODbL).
- Relief : AWS Open Data Terrain Tiles (Mapzen/Tilezen, données SRTM).
- OMNeT++ est sous Academic Public License : un usage commercial relève d'OMNEST.
- Les icônes sont générées par `tools/make_icons.py`.
