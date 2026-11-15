package aien.drone;

//
// Energy model of a relay drone: DJI Matrice 400 with one TB100 battery
// (977 Wh) carrying a ~5.5 kg airborne 4G base station.
//
// Power while airborne = flightPower (propulsion with the payload) + payloadPower
// (the base station). With the defaults: 977 Wh / (1990 W + 180 W) = 27 min,
// the endurance estimated in the market study (scaling of DJI's 53 min hover
// at 10.67 kg to ~15.2 kg take-off weight). Change flightPower to test other
// hypotheses.
//
// The battery is integrated lazily (on every query) from the mobility state:
// it only drains while the drone is airborne.
//
simple DroneBattery
{
    parameters:
        @class(DroneBattery);
        @display("i=block/plug;is=s");
        double capacity @unit(Wh) = default(977Wh);     // DJI TB100
        double flightPower @unit(W) = default(1990W);   // propulsion, M400 with ~5.5 kg payload (estimate)
        double payloadPower @unit(W) = default(180W);   // airborne 4G base station (Kingtone ALBS-3000 nominal)
        string mobilityModule = default("^.mobility");
        @signal[residualRatio](type=double);
        @statistic[residualRatio](title="battery level"; unit=ratio; record=vector?,min?; interpolationmode=linear);
}
