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
    joinPlatoon,
    leavePlatoon,
    update_maneuvers,
)

if "SUMO_HOME" in os.environ:
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
from plexe import Plexe, ACC, CACC, FAKED_CACC, RPM, GEAR, ACCELERATION, SPEED

N_VEHICLES = 16


def main(demo_mode, real_engine, setter=None):
    # used to randomly color the vehicles
    random.seed(1)
    start_sumo("cfg/freeway.sumo.cfg", False)
    plexe = Plexe()
    step = 0
    topology = init_topology([N_VEHICLES // 2, N_VEHICLES // 2])
    print(topology)

    while running(demo_mode, step, 6000):
        traci.simulationStep()

        if step == 0:
            print("--------------STEP 0----------------")
            init_simulation(plexe, topology, real_engine)
            traci.gui.trackVehicle("View #0", "v.0")
            traci.gui.setZoom("View #0", 20000)

        if step % 10 == 1:
            # simulate vehicle communication every 100 ms
            update_maneuvers(topology, plexe)
            communicate(plexe, topology)
        if step == 100:
            print("--------------STEP 100----------------")
            _, vehicle = topology.get_vehicle("v.4")
            leavePlatoon(vehicle, topology, plexe)
        if step == 1500:
            print("--------------STEP 1500----------------")
            _, vehicle = topology.get_vehicle("v.10")
            target_pl = 0
            joinPlatoon(vehicle, target_pl, topology, plexe)

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
