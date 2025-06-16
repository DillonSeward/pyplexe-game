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
    reconfigure_platoons,
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
IN_PLATOON = 0
OPENING_GAP = 1
LEAVING = 2
JOINING = 3
GOING_TO_POSITION = 4
MERGING = 5
COMPLETED = 6

# maneuver actors
LEADER = "v.0"
N_VEHICLES = 20
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
    topology = init_topology([8, 8])
    leave_pos = 4
    leaver = topology.platoons[0][1][leave_pos]

    while running(demo_mode, step, 6000):
        # when reaching 60 seconds, reset the simulation when in demo_mode
        if demo_mode and step == 6000:
            start_sumo("cfg/freeway.sumo.cfg", True)
            step = 0
            state = IN_PLATOON
            print(state)
            random.seed(1)

        traci.simulationStep()

        if step == 0:
            # create vehicles and track the joiner
            init_simulation(plexe, topology, DISTANCE, LENGTH, SPEED, real_engine)
            traci.gui.trackVehicle("View #0", leaver.id)
            traci.gui.setZoom("View #0", 20000)

        if step % 10 == 1:
            # simulate vehicle communication every 100 ms
            communicate(plexe, topology)

        if state == IN_PLATOON and step == 100:
            # at 1 second, let the joiner get closer to the platoon
            # topology = get_in_position(plexe, leaver, leaver.front, topology)
            topology = open_gap(
                plexe, leaver.back, leaver.front, JOIN_DISTANCE, topology, N_VEHICLES
            )
            reconfigure_platoons(plexe, topology)
            state = OPENING_GAP
            print(state)
        if state == OPENING_GAP:
            plexe.set_active_controller(leaver.id, ACC)
            traci.vehicle.setSpeed(leaver.id, SPEED)
            plexe.set_fixed_lane(leaver.id, 1, safe=False)
            state = LEAVING
            print(state)

        if state == LEAVING:
            try:
                _, back = topology.get_vehicle(leaver.back)
                _, front = topology.get_vehicle(leaver.front)
            except ValueError:
                state = COMPLETED
                print(state)
                continue

            if get_distance(plexe, back.id, front.id) > (JOIN_DISTANCE + 1):
                # safely re-link and reconfigure
                if back.front == leaver.id:
                    back.front = front.id
                if front.back == leaver.id:
                    front.back = back.id

                plexe.set_active_controller(back.id, CACC)
                plexe.set_active_controller(front.id, CACC)
                plexe.set_path_cacc_parameters(back.id, DISTANCE)
                plexe.set_path_cacc_parameters(front.id, DISTANCE)

                state = JOINING
                print(state)

        if state == JOINING and step > 3000:
            # define mid-point join target in platoon 1
            platoon1 = topology.platoons[1][1]
            join_index = 4  # e.g., insert between v.11 and v.12
            front = platoon1[join_index - 1]
            back = platoon1[join_index]
            topology.platoons[1][1].insert(join_index, leaver)

            # Set joiner link relationships
            leaver.front = front.id
            leaver.back = back.id
            front.back = leaver.id
            back.front = leaver.id

            topology = get_in_position(plexe, leaver.id, front.id, SPEED, topology)
            topology = open_gap(
                plexe, front.id, back.id, JOIN_DISTANCE, topology, N_VEHICLES
            )

            # reconfigure_platoons(plexe, topology)

            state = GOING_TO_POSITION
            print(state)

        if state == GOING_TO_POSITION:
            if get_distance(plexe, leaver.id, front.id) < JOIN_DISTANCE + 1:
                state = MERGING

        if state == MERGING:
            # when the gap is large enough, complete the maneuver
            if get_distance(plexe, leaver.back, leaver.front) > 2 * JOIN_DISTANCE + 2:
                state = COMPLETED
                plexe.set_fixed_lane(leaver.id, 2, safe=False)
                plexe.set_active_controller(leaver.id, CACC)
                plexe.set_path_cacc_parameters(leaver.id, distance=DISTANCE)
                plexe.set_active_controller(back.id, CACC)
                plexe.set_path_cacc_parameters(back.id, distance=DISTANCE)
                topology.reset_leaders()
                # reconfigure_platoons(plexe, topology)

        if state == COMPLETED:
            topology.reset_leaders()
            # reconfigure_platoons(plexe, topology)
            topology.platoons = {k: v for k, v in topology.platoons.items() if v[1]}

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
