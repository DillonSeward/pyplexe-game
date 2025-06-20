from utils import (
    get_distance,
    open_gap,
    Topology,
    Vehicle,
    DISTANCE,
    SPEED,
    JOIN_DISTANCE,
)
import traci
from enum import Enum
from abc import abstractmethod
from plexe import CACC, ACC, FAKED_CACC


class Maneuver:
    def __init__(self, v: Vehicle):
        self.vehicle = v

    @abstractmethod
    def update(self, plexe, topology: Topology):
        pass

    @abstractmethod
    def completed(self) -> bool:
        pass


class LeaveManeuver(Maneuver):
    class State(Enum):
        IN_PLATOON = 0
        OPENING_GAP = 1
        LEAVING = 2
        FINISH = 3
        COMPLETED = 4

        def __str__(self):
            match self:
                case LeaveManeuver.State.IN_PLATOON:
                    return "IN_PLATOON"
                case LeaveManeuver.State.OPENING_GAP:
                    return "OPENING_GAP"
                case LeaveManeuver.State.LEAVING:
                    return "LEAVING"
                case LeaveManeuver.State.FINISH:
                    return "FINISH"
                case LeaveManeuver.State.COMPLETED:
                    return "COMPLETED"

    def __init__(self, vehicle: Vehicle, target_lane: int, topology: Topology):
        super().__init__(vehicle)
        self.target_lane = target_lane
        self.state = LeaveManeuver.State.IN_PLATOON

    def completed(self) -> bool:
        return self.state == LeaveManeuver.State.COMPLETED

    def update(self, plexe, topology: Topology):
        print("LEAVE STATE: ", self.state)
        match self.state:
            case LeaveManeuver.State.IN_PLATOON:
                topology = open_gap(
                    plexe, self.vehicle.back, self.vehicle.id, JOIN_DISTANCE, topology
                )
                topology = open_gap(
                    plexe, self.vehicle.id, self.vehicle.front, JOIN_DISTANCE, topology
                )
                self.state = LeaveManeuver.State.OPENING_GAP
            case LeaveManeuver.State.OPENING_GAP:
                plexe.set_active_controller(self.vehicle.id, ACC)
                traci.vehicle.setSpeed(self.vehicle.id, SPEED)
                plexe.set_fixed_lane(self.vehicle.id, 1, safe=False)
                self.state = LeaveManeuver.State.LEAVING
            case LeaveManeuver.State.LEAVING:
                if get_distance(plexe, self.vehicle.back, self.vehicle.front) > (
                    JOIN_DISTANCE + 1
                ):
                    _, back = topology.get_vehicle(self.vehicle.back)
                    _, front = topology.get_vehicle(self.vehicle.back)
                    back.front = front.id
                    front.back = back.id

                    # swithing back control scheme
                    plexe.set_active_controller(back.id, CACC)
                    plexe.set_active_controller(front.id, CACC)
                    plexe.set_path_cacc_parameters(back.id, DISTANCE)
                    plexe.set_path_cacc_parameters(front.id, DISTANCE)

                self.state = LeaveManeuver.State.FINISH
            case LeaveManeuver.State.FINISH:
                topology.reset_leaders()
                self.state = LeaveManeuver.State.COMPLETED
            case LeaveManeuver.State.COMPLETED:
                pass


class JoinManeuver(Maneuver):
    class State(Enum):
        WAITING = 0
        OPENING_GAP = 1
        GOING_TO_POSITION = 2
        FINISH = 3
        COMPLETED = 4

        def __str__(self):
            match self:
                case JoinManeuver.State.WAITING:
                    return "WAITING"
                case JoinManeuver.State.OPENING_GAP:
                    return "OPENING_GAP"
                case JoinManeuver.State.GOING_TO_POSITION:
                    return "GOING_TO_POSITION"
                case JoinManeuver.State.FINISH:
                    return "FINISH"
                case JoinManeuver.State.COMPLETED:
                    return "COMPLETED"

    def __init__(
        self, vehicle: Vehicle, target_platoon: int, join_idx: int, topology: Topology
    ):
        super().__init__(vehicle)
        self.target_platoon = target_platoon
        self.join_idx = join_idx
        self.state = JoinManeuver.State.WAITING
        in_platoon, _ = topology.get_vehicle(self.vehicle.id)
        assert in_platoon != self.target_platoon
        platoon_vehicles = topology.platoons[self.target_platoon].vehicles

        if (front_join := platoon_vehicles[self.join_idx]) is None:
            raise ValueError("platoon has no vehicle at ", self.join_idx)

        self.front_join = front_join
        print("front", front_join)
        _, self.back_join = topology.get_vehicle(front_join.back)
        print("back", self.back_join)

    def completed(self) -> bool:
        return self.state == JoinManeuver.State.COMPLETED

    def update(self, plexe, topology: Topology):
        print("JOIN STATE: ", self.state)
        match self.state:
            case JoinManeuver.State.WAITING:
                # at 1 second, let the joiner get closer to the platoon
                topology = JoinManeuver.get_in_position(
                    plexe, self.vehicle.id, self.front_join.id, topology
                )
                self.state = JoinManeuver.State.GOING_TO_POSITION

            case JoinManeuver.State.GOING_TO_POSITION:
                # when the distance of the joiner is small enough, let the others
                # open a gap to let the joiner enter the platoon
                if (
                    get_distance(plexe, self.vehicle.id, self.front_join.id)
                    < JOIN_DISTANCE + 1
                ):
                    self.state = JoinManeuver.State.OPENING_GAP
                    topology = open_gap(
                        plexe,
                        self.back_join.id,
                        self.vehicle.id,
                        JOIN_DISTANCE,
                        topology,
                    )
            case JoinManeuver.State.OPENING_GAP:
                # when the gap is large enough, complete the maneuver
                if (
                    get_distance(plexe, self.back_join.id, self.front_join.id)
                    > 2 * JOIN_DISTANCE - 2
                ):
                    print("GAP OPENED ENOUGH")
                    leader = topology.platoons[self.target_platoon].leader_id
                    lane = traci.vehicle.getLaneIndex(leader)
                    plexe.set_fixed_lane(self.vehicle.id, lane, safe=False)
                    plexe.set_active_controller(self.vehicle.id, CACC)
                    plexe.set_path_cacc_parameters(self.vehicle.id, distance=DISTANCE)
                    plexe.set_active_controller(self.back_join.id, CACC)
                    plexe.set_path_cacc_parameters(self.back_join.id, distance=DISTANCE)
                    self.state = JoinManeuver.State.FINISH

            case JoinManeuver.State.FINISH:
                topology.reset_leaders()
                self.state = JoinManeuver.State.COMPLETED

            case JoinManeuver.State.COMPLETED:
                pass

    def get_in_position(plexe, jid: str, fid: str, topology: Topology):
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
        _, joiner = topology.get_vehicle(jid)
        joiner.front = fid
        joiner_lane = traci.vehicle.getLaneIndex(jid)
        platoon_lane = traci.vehicle.getLaneIndex(fid)

        target_lane = (
            platoon_lane + 1 if joiner_lane > platoon_lane else platoon_lane - 1
        )
        plexe.set_fixed_lane(jid, target_lane, safe=False)
        # grab lane of platoon and lane of joiner
        # if joiner is below
        # set lane of joiner to one below platoon
        # if joiner is above
        # set lane of joiner to one above plaroon
        # traci.vehicle.setSpeedMode(joiner.id, 0)
        # traci.vehicle.deactivateGapControl
        traci.vehicle.setSpeed(joiner.id, SPEED + 45)
        # plexe.set_active_controller(joiner.id, CACC)
        plexe.set_active_controller(joiner.id, FAKED_CACC)
        plexe.set_cc_desired_speed(joiner.id, SPEED + 55)
        return topology
