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
    add_platooning_vehicle,
    Topology,
    Vehicle,
    get_in_position,
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
GOING_TO_POSITION = 0
OPENING_GAP = 1
WAITING = 2
COMPLETED = 3

LEADER = "v.0"
N_VEHICLES = 8
JOIN_POSITION = N_VEHICLES // 2
FRONT_JOIN = "v.%d" % (JOIN_POSITION - 1)
BEHIND_JOIN = "v.%d" % JOIN_POSITION
JOINER = "v.%d" % N_VEHICLES


def main(demo_mode, real_engine, setter=None):
    # used to randomly color the vehicles
    random.seed(1)
    start_sumo("cfg/freeway.sumo.cfg", False)
    plexe = Plexe()
    step = 0
    state = WAITING
    topology = init_topology([N_VEHICLES])
    print(topology)

    join_pos = N_VEHICLES // 2
    joiner = topology.platoons[0][1][join_pos]
    print("GOT JOINER: ", joiner.id)

    while running(demo_mode, step, 6000):
        # when reaching 60 seconds, reset the simulation when in demo_mode
        if demo_mode and step == 6000:
            start_sumo("cfg/freeway.sumo.cfg", True)
            step = 0
            state = WAITING
            random.seed(1)

        traci.simulationStep()

        if step == 0:
            # create vehicles and track the joiner
            init_simulation(plexe, topology, DISTANCE, LENGTH, SPEED, real_engine)
            vid = JOINER
            add_platooning_vehicle(plexe, vid, 10, 1, SPEED, DISTANCE, real_engine)
            joiner_vehicle = Vehicle(JOINER, front=FRONT_JOIN)
            joiner_platoon_id = topology.create_platoon(JOINER)
            topology.add_vehicle(joiner_platoon_id, joiner_vehicle)
            plexe.set_fixed_lane(vid, 1, safe=False)
            traci.vehicle.setSpeedMode(vid, 0)
            plexe.set_active_controller(vid, ACC)
            plexe.set_path_cacc_parameters(vid, distance=JOIN_DISTANCE)
            traci.gui.trackVehicle("View #0", joiner.id)
            traci.gui.setZoom("View #0", 20000)

        if step % 10 == 1:
            # simulate vehicle communication every 100 ms
            communicate(plexe, topology)

        if step == 100:
            # at 1 second, let the joiner get closer to the platoon
            topology = get_in_position(plexe, JOINER, FRONT_JOIN, topology)
            state = GOING_TO_POSITION

        if state == GOING_TO_POSITION and step > 0:
            # when the distance of the joiner is small enough, let the others
            # open a gap to let the joiner enter the platoon
            if get_distance(plexe, JOINER, FRONT_JOIN) < JOIN_DISTANCE + 1:
                state = OPENING_GAP
                topology = open_gap(
                    plexe, BEHIND_JOIN, JOINER, JOIN_DISTANCE, topology, N_VEHICLES
                )
        if state == OPENING_GAP:
            # when the gap is large enough, complete the maneuver
            if get_distance(plexe, BEHIND_JOIN, FRONT_JOIN) > 2 * JOIN_DISTANCE + 2:
                state = COMPLETED
                plexe.set_fixed_lane(JOINER, 0, safe=False)
                plexe.set_active_controller(JOINER, CACC)
                plexe.set_path_cacc_parameters(JOINER, distance=DISTANCE)
                plexe.set_active_controller(BEHIND_JOIN, CACC)
                plexe.set_path_cacc_parameters(BEHIND_JOIN, distance=DISTANCE)
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
