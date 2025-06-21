#!/usr/bin/env python
import os
import sys
import random

from utils import (
    communicate,
    start_sumo,
    running,
    init_simulation,
    init_topology,
)

if "SUMO_HOME" in os.environ:
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
from typing import List

from maneuvers import LeaveManeuver, Maneuver, JoinManeuver
from plexe import Plexe, RPM, GEAR, SPEED, ACCELERATION

N_VEHICLES = 16
SIMULATION_END_STEP = 6000


def main(demo_mode, real_engine, setter=None):
    random.seed(1)
    start_sumo("cfg/freeway.sumo.cfg", False)
    plexe = Plexe()
    step = 0
    topology = init_topology([N_VEHICLES // 2, N_VEHICLES // 2])
    leaver = topology.platoons[0].vehicles[4]
    joiner = topology.platoons[1].vehicles[2]
    mans: List[Maneuver] = []
    complete = False

    while running(demo_mode, step, SIMULATION_END_STEP):
        traci.simulationStep()
        if len(mans) > 0:
            # for man in mans:
            #     if man.completed() and not complete:
            #         complete = True
            #     else:
            #         man.update(plexe, topology)

            for man in [m for m in mans if not m.completed()]:
                man.update(plexe, topology)

        if step == 0:
            init_simulation(plexe, topology, real_engine)
            traci.gui.trackVehicle("View #0", "v.0")
            traci.gui.setZoom("View #0", 20000)

        if step % 10 == 1:
            communicate(plexe, topology)

        if step == 100:
            print("[main] Step 100: Vehicle v.4 will leave its platoon")
            mans.append(LeaveManeuver(leaver, 1, topology))

        if step == 2000:
            # if complete:
            mans.append(JoinManeuver(leaver, 1, 2, topology))
            print("[main] Step 1500: Vehicle v.10 will join platoon 0 at index 2")
            # if topology.inPlatoon(joiner.id):
            # mans.append(LeaveManeuver(joiner, 4, topology))

            # mans.append(JoinManeuver(joiner, 0, 4, topology))

        if real_engine and setter is not None:
            tracked_id = traci.gui.getTrackedVehicle("View #0")
            if tracked_id != "":
                ed = plexe.get_engine_data(tracked_id)
                vd = plexe.get_vehicle_data(tracked_id)
                setter(ed[RPM], ed[GEAR], vd.speed, vd.acceleration)

        step += 1

    traci.close()


if __name__ == "__main__":
    main(True, False)
