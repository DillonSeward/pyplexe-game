#!/usr/bin/env python
#
# Copyright (c) 2018-2022 Michele Segata <segata@ccs-labs.org>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see http://www.gnu.org/licenses/.
#

import os
import sys
import random
from typing import Dict, Tuple, List, Optional
from utils import add_platooning_vehicle, communicate, get_distance, \
    start_sumo, running, Topology, Vehicle

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
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
N_VEHICLES = 20
LEAVE_POSITION = N_VEHICLES / 2
FRONT_LEAVE = "v.%d" % (LEAVE_POSITION - 1)
BEHIND_LEAVE = "v.%d" % LEAVE_POSITION
LEAVER = "v.%d" % LEAVE_POSITION


# Maybe something like this needs to be moved to Topology class??
def init_topology(n_vehicles_per_platoon: List[int]) -> Topology:
    topology = Topology()
    vehicles_count = 0
    n_platoons = n_vehicles_per_platoon.__len__()
    
    for platoon_id in range(n_platoons):
        n = n_vehicles_per_platoon[platoon_id]
        for i in range(n):
            vid = "v.%d" % vehicles_count
            vehicle = Vehicle(vid)
            vehicles_count += 1
            if i == 0:
                topology.create_platoon(vid)
            if i > 0:
                front = "v.%d" % (i - 1)
                print("FRONT: ", front)
                vehicle.front = front
            if i < n - 1:
                back = "v.%d" % (i + 1)
                print("BACK: ", back)
                vehicle.back = back

            topology.add_vehicle(platoon_id, vehicle)

    return topology

def init_simulation(plexe, topology: Topology, real_engine = False):
    for platoon_id, (leader, vehicles) in topology.platoons.items():
        for i, v in enumerate(vehicles):
            add_platooning_vehicle(plexe, v.id, i * (DISTANCE + LENGTH) +
                                   50, 0, SPEED, DISTANCE, real_engine)
            # SECOND ARGUMENT IS LANE NUMBER
            # MAY NOT BE BEST TO HAVE THIS BE PLATOON ID
            plexe.set_fixed_lane(v.id, platoon_id, safe=False)
            traci.vehicle.setSpeedMode(v.id, 0)
            if v.id == leader:
                plexe.set_active_controller(v.id, ACC)
            else:
                plexe.set_active_controller(v.id, CACC)


def get_in_position(plexe, jid, fid, topology):
    """
    Makes the joining vehicle get close to the join position. This is done by
    changing the topology and setting the leader and the front vehicle for
    the joiner. In addition, we increase the cruising speed and we switch to
    the "fake" CACC, which uses a given GPS distance instead of the radar
    distance to compute the control action
    :param plexe: API instance
    :param jid: id of the joiner
    :param fid: id of the vehicle that will become the predecessor of the joiner
    :param topology: the current platoon topology
    :return: the modified topology
    """
    topology[jid] = {"leader": LEADER, "front": fid}
    plexe.set_cc_desired_speed(jid, SPEED + 15)
    plexe.set_active_controller(jid, FAKED_CACC)
    return topology


def open_gap(plexe, vid, jid, topology: Topology, n) -> Topology:
    """
    Makes the vehicle that will be behind the joiner open a gap to let the
    joiner in. This is done by creating a temporary platoon, i.e., setting
    the leader of all vehicles behind to the one that opens the gap and then
    setting the front vehicle of the latter to be the joiner. To properly
    open the gap, the vehicle leaving space switches to the "fake" CACC,
    to consider the GPS distance to the joiner
    :param plexe: API instance
    :param vid: vehicle that should open the gap
    :param jid: id of the joiner
    :param topology: the current platoon topology
    :param n: total number of vehicles currently in the platoon
    :return: the modified topology
    """
    topology.create_sub_platoon(vid)
    # index = int(vid.split(".")[1])
    # sub_platoon = topology.create_platoon(vid)
    # for i in range(index + 1, n):
    #     sub_vid = "v.%d" % i
    #     v = topology.remove_vehicle(sub_vid)[1]
    #     topology.platoons[sub_platoon][1].append(v)
        # temporarily change the leader
        # BAD!!
        # topology.get_vehicle("v.%d" % i)
        # topology.platoons[0][0] = vid

    # the front vehicle if the vehicle opening the gap is the joiner
    print ("CREATED SUBPLATOON: ", topology)
    topology.get_vehicle(vid)[1].front = jid
    # v = topology.get_vehicle(vid)[1]
    # v.front = jid
    # topology[0][vid]["front"] = jid
    plexe.set_active_controller(vid, FAKED_CACC)
    plexe.set_path_cacc_parameters(vid, distance=JOIN_DISTANCE)
    return topology



def reset_leader(vid, topology: Topology, n) -> Topology:
    """
    After the maneuver is completed, the vehicles behind the one that opened
    the gap, reset the leader to the initial one
    :param vid: id of the vehicle that let the joiner in
    :param topology: the current platoon topology
    :param n: total number of vehicles in the platoon (before the joiner)
    :return: the modified topology
    """
    index = int(vid.split(".")[1])
    for i in range(index + 1, n):
        # restore the real leader
        topology.get_vehicle("v.%d" % i).leader = LEADER
        # topology[0]["v.%d" % i]["leader"] = LEADER
    return topology


def main(demo_mode, real_engine, setter=None):
    # used to randomly color the vehicles
    random.seed(1)
    start_sumo("cfg/freeway.sumo.cfg", False)
    plexe = Plexe()
    step = 0
    state = IN_PLATOON
    topology = init_topology([N_VEHICLES])
    print(topology)
    
    leader = topology.platoons[0][0]
    leave_pos = N_VEHICLES // 2
    leaver = topology.platoons[0][1][leave_pos]
    print("GOT LEAVER: ", leaver.id)

    
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
            traci.gui.trackVehicle("View #0", leaver.id)
            traci.gui.setZoom("View #0", 20000)
        if step % 10 == 1:
            # simulate vehicle communication every 100 ms
            communicate(plexe, topology)
        if state == IN_PLATOON and step == 100:
            # at 1 second, let the joiner get closer to the platoon
            # topology = get_in_position(plexe, leaver, leaver.front, topology)
            topology = open_gap(plexe, leaver.back, leaver, topology, N_VEHICLES)
            topology = open_gap(plexe, leaver, leaver.front, topology, N_VEHICLES)
            state = OPENING_GAP
        if state == OPENING_GAP:
                plexe.SET_ACTIVE_CONTROLLER(leaver, ACC)
                plexe.set_fixed_lane(leaver, 1, safe=False)
                state = LEAVING
        if state == LEAVING:
            if get_distance(plexe, leaver.back, leaver.front) < JOIN_DISTANCE + 1:
                state == COMPLETED
            
       
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
