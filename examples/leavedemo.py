#!/usr/bin/env python
import os
import sys
import random

from utils import (
    communicate,
    get_distance,
    start_sumo,
    running,
    init_simulation,
    init_topology,
    open_gap,
)

if "SUMO_HOME" in os.environ:
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
from plexe import Plexe, ACC, CACC, FAKED_CACC, RPM, GEAR, ACCELERATION, SPEED

# vehicle length
LENGTH = 4
# inter-vehicle distance
DISTANCE = 5
# inter-vehicle distance when leaving space for joining
JOIN_DISTANCE = DISTANCE * 2
# cruising speed
SPEED = 120 / 3.6

# maneuver states:
IN_PLATOON = 0
OPENING_GAP = 1
LEAVING = 2
COMPLETED = 3

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
    state = IN_PLATOON
    topology = init_topology([N_VEHICLES])

    leave_pos = 4
    leaver = topology.platoons[0][1][leave_pos]

    while running(demo_mode, step, 6000):
        # when reaching 60 seconds, reset the simulation when in demo_mode
        if demo_mode and step == 6000:
            start_sumo("cfg/freeway.sumo.cfg", True)
            step = 0
            state = IN_PLATOON
            random.seed(1)

        traci.simulationStep()

        if step == 0:
            # create vehicles and track the joiner
            init_simulation(plexe, topology, real_engine)
            traci.gui.trackVehicle("View #0", LEAVER)
            traci.gui.setZoom("View #0", 20000)
        if step % 10 == 1:
            # simulate vehicle communication every 100 ms
            communicate(plexe, topology)
        if state == IN_PLATOON and step == 100:
            # at 1 second, let the joiner get closer to the platoon
            topology = open_gap(plexe, leaver.back, leaver.id, JOIN_DISTANCE, topology)
            topology = open_gap(plexe, leaver.id, leaver.front, JOIN_DISTANCE, topology)
            state = OPENING_GAP

        if state == OPENING_GAP:
            plexe.set_active_controller(leaver.id, ACC)
            traci.vehicle.setSpeed(leaver.id, SPEED)
            plexe.set_fixed_lane(leaver.id, 1, safe=False)
            state = LEAVING

        if state == LEAVING:
            if get_distance(plexe, leaver.back, leaver.front) > (JOIN_DISTANCE + 1):
                _, back = topology.get_vehicle(leaver.back)
                _, front = topology.get_vehicle(leaver.back)
                back.front = front.id
                front.back = back.id

                # swithing back control scheme
                plexe.set_active_controller(back.id, CACC)
                plexe.set_active_controller(front.id, CACC)
                plexe.set_path_cacc_parameters(back.id, DISTANCE)
                plexe.set_path_cacc_parameters(front.id, DISTANCE)

                state = COMPLETED

        if state == COMPLETED:
            topology.reset_leaders()

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
