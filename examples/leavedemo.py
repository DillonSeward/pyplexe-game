#!/usr/bin/env python
import os
import random
import sys
from typing import List

from maneuvers import LeaveManeuver, Maneuver
from utils import (
    communicate,
    init_simulation,
    init_topology,
    running,
    start_sumo,
)

if "SUMO_HOME" in os.environ:
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci

from plexe import GEAR, RPM, Plexe

# maneuver actors
LEADER = "v.0"
N_VEHICLES = 8
LEAVE_POSITION = N_VEHICLES // 2
FRONT_LEAVE = "v.%d" % (LEAVE_POSITION - 1)
BEHIND_LEAVE = "v.%d" % LEAVE_POSITION
LEAVER = "v.%d" % LEAVE_POSITION


def main(demo_mode, real_engine, setter=None):
    # used to randomly color the vehicles
    random.seed(1)
    start_sumo("cfg/freeway.sumo.cfg", False)
    plexe = Plexe()
    step = 0
    topology = init_topology([N_VEHICLES])

    leave_pos = 4
    leaver = topology.platoons[0].vehicles[leave_pos]
    mans: List[Maneuver] = []

    while running(demo_mode, step, 6000):
        # when reaching 60 seconds, reset the simulation when in demo_mode
        if demo_mode and step == 6000:
            start_sumo("cfg/freeway.sumo.cfg", True)
            step = 0
            random.seed(1)

        traci.simulationStep()
        if len(mans) > 0:
            for man in [m for m in mans if not m.completed()]:
                man.update(plexe, topology)

        if step == 0:
            # create vehicles and track the joiner
            init_simulation(plexe, topology, real_engine)
            traci.gui.trackVehicle("View #0", LEAVER)
            traci.gui.setZoom("View #0", 20000)
        if step % 10 == 1:
            # simulate vehicle communication every 100 ms
            communicate(plexe, topology)
        if step == 100:
            mans.append(LeaveManeuver(leaver, 1, topology))

        if real_engine and setter is not None:
            # if we are running with the dashboard, update its values
            tracked_id = traci.gui.getTrackedVehicle("View #0")
            if tracked_id != "":
                ed = plexe.get_engine_data(tracked_id)
                vd = plexe.get_vehicle_data(tracked_id)
                setter(ed[RPM], ed[GEAR], vd.speed, vd.acceleration)

        step += 1

    traci.close()


if __name__ == "__main__":
    main(True, False)
