package aien.node;

import inet.node.aodv.AodvRouter;

//
// A phone on the ground (victim or rescuer) inside the disaster area. It sends
// periodic "SOS" UDP messages to the hive; they are delivered only if a relay
// drone (or the mother antenna) covers it and is connected to the hive.
// The icon turns from red to green when the phone is covered.
//
module Victim extends AodvRouter
{
    parameters:
        @display("i=aien/victim_sos");
        mobility.typename = default("aien.mobility.GroundMobility");
}
