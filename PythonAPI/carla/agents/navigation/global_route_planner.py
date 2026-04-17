# Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""This module provides GlobalRoutePlanner implementation."""

from __future__ import annotations

import contextlib
import math
from typing import TYPE_CHECKING, TypedDict

import networkx as nx
import numpy as np

import carla
from agents.navigation.local_planner import RoadOption

if TYPE_CHECKING:
    try:
        from typing import NotRequired
    except ImportError:
        from typing_extensions import NotRequired


class _TopologyDict(TypedDict):
    """TypedDict for topology segment data."""

    entry: carla.Waypoint
    exit: carla.Waypoint
    entryxyz: tuple[float, float, float]
    exitxyz: tuple[float, float, float]
    path: list[carla.Waypoint]


class _EdgeDict(TypedDict, total=False):
    """TypedDict for graph edge data."""

    length: int
    path: list[carla.Waypoint]
    entry_waypoint: carla.Waypoint
    exit_waypoint: carla.Waypoint
    entry_vector: np.ndarray | None
    exit_vector: np.ndarray | None
    net_vector: list[float] | None
    intersection: bool
    road_option: RoadOption
    change_waypoint: NotRequired[carla.Waypoint]


# Constants
_COORDINATE_DECIMAL_PLACES: int = 0
_WAYPOINT_AHEAD_COUNT: int = 5
_DESTINATION_DISTANCE_MULTIPLIER: float = 2.0
_ZERO_COST_LENGTH: int = 0
_ROUTE_END_MARGIN: int = 2
_NEGATIVE_NODE_MULTIPLIER: int = -1
_NO_CLOSEST_WAYPOINT: int = -1
_TURN_DECISION_THRESHOLD_DEGREES: float = 35.0
_ASTAR_WEIGHT_LENGTH: str = "length"
_CLIP_ANGLE_MIN: float = -1.0
_CLIP_ANGLE_MAX: float = 1.0
_ZERO_CROSS_VALUE: float = 0.0
_UNINITIALIZED_NODE_ID: int = -1


class GlobalRoutePlanner:
    """This class provides a very high level route plan."""

    _sampling_resolution: float
    _wmap: carla.Map
    _topology: list[_TopologyDict]
    _graph: nx.DiGraph
    _id_map: dict[tuple[float, float, float], int]
    _road_id_to_edge: dict[int, dict[int, dict[int, tuple[int, int]]]]
    _intersection_end_node: int
    _previous_decision: RoadOption

    def __init__(self, wmap: carla.Map, sampling_resolution: float) -> None:
        """Initialize the GlobalRoutePlanner.

        Args:
            wmap: carla.Map instance
            sampling_resolution: distance between sampled waypoints in meters
        """
        self._sampling_resolution = sampling_resolution
        self._wmap = wmap
        self._topology = []
        self._graph = nx.DiGraph()
        self._id_map = {}
        self._road_id_to_edge = {}

        self._intersection_end_node = _UNINITIALIZED_NODE_ID
        self._previous_decision = RoadOption.VOID

        # Build the graph
        self._build_topology()
        self._build_graph()
        self._find_loose_ends()
        self._lane_change_link()

    def trace_route(
        self,
        origin: carla.Location,
        destination: carla.Location,
    ) -> list[tuple[carla.Waypoint, RoadOption]]:
        """Return list of (carla.Waypoint, RoadOption) from origin to destination.

        Args:
            origin: starting location
            destination: ending location

        Returns:
            list of (waypoint, road option) tuples
        """
        route_trace: list[tuple[carla.Waypoint, RoadOption]] = []
        route = self._path_search(origin, destination)
        current_waypoint = self._wmap.get_waypoint(origin)
        destination_waypoint = self._wmap.get_waypoint(destination)

        for i in range(len(route) - 1):
            road_option = self._turn_decision(i, route)
            edge: _EdgeDict = self._graph.edges[route[i], route[i + 1]]
            path: list[carla.Waypoint] = []

            if edge["road_option"] not in (RoadOption.LANEFOLLOW, RoadOption.VOID):
                route_trace.append((current_waypoint, road_option))
                exit_wp = edge["exit_waypoint"]
                n1, n2 = self._road_id_to_edge[exit_wp.road_id][exit_wp.section_id][exit_wp.lane_id]
                next_edge: _EdgeDict = self._graph.edges[n1, n2]
                if next_edge["path"]:
                    closest_index = self._find_closest_in_list(current_waypoint, next_edge["path"])
                    closest_index = min(len(next_edge["path"]) - 1, closest_index + _WAYPOINT_AHEAD_COUNT)
                    current_waypoint = next_edge["path"][closest_index]
                else:
                    current_waypoint = next_edge["exit_waypoint"]
                route_trace.append((current_waypoint, road_option))

            else:
                path = path + [edge["entry_waypoint"]] + edge["path"] + [edge["exit_waypoint"]]
                closest_index = self._find_closest_in_list(current_waypoint, path)
                for waypoint in path[closest_index:]:
                    current_waypoint = waypoint
                    route_trace.append((current_waypoint, road_option))
                    if (
                        len(route) - i <= _ROUTE_END_MARGIN
                        and waypoint.transform.location.distance(destination)
                        < _DESTINATION_DISTANCE_MULTIPLIER * self._sampling_resolution
                    ):
                        break
                    if (
                        len(route) - i <= _ROUTE_END_MARGIN
                        and current_waypoint.road_id == destination_waypoint.road_id
                        and current_waypoint.section_id == destination_waypoint.section_id
                        and current_waypoint.lane_id == destination_waypoint.lane_id
                    ):
                        destination_index = self._find_closest_in_list(destination_waypoint, path)
                        if closest_index > destination_index:
                            break

        return route_trace

    def _build_topology(self) -> None:
        """Retrieve topology from the server and process into list of dictionary objects."""
        self._topology = []
        # Retrieving waypoints to construct a detailed topology
        for segment in self._wmap.get_topology():
            wp1, wp2 = segment[0], segment[1]
            l1, l2 = wp1.transform.location, wp2.transform.location
            # Rounding off to avoid floating point imprecision
            x1, y1, z1, x2, y2, z2 = np.round(
                [l1.x, l1.y, l1.z, l2.x, l2.y, l2.z],
                _COORDINATE_DECIMAL_PLACES,
            )
            wp1.transform.location, wp2.transform.location = l1, l2
            seg_dict: _TopologyDict = {
                "entry": wp1,
                "exit": wp2,
                "entryxyz": (x1, y1, z1),
                "exitxyz": (x2, y2, z2),
                "path": [],
            }
            endloc = wp2.transform.location
            if wp1.transform.location.distance(endloc) > self._sampling_resolution:
                w = wp1.next(self._sampling_resolution)[0]
                while w.transform.location.distance(endloc) > self._sampling_resolution:
                    seg_dict["path"].append(w)
                    next_ws = w.next(self._sampling_resolution)
                    if not next_ws:
                        break
                    w = next_ws[0]
            else:
                next_wps = wp1.next(self._sampling_resolution)
                if not next_wps:
                    continue
                seg_dict["path"].append(next_wps[0])
            self._topology.append(seg_dict)

    def _build_graph(self) -> None:
        """Build a networkx graph representation of topology."""
        self._graph = nx.DiGraph()
        self._id_map = {}
        self._road_id_to_edge = {}

        for segment in self._topology:
            entry_xyz, exit_xyz = segment["entryxyz"], segment["exitxyz"]
            path = segment["path"]
            entry_wp, exit_wp = segment["entry"], segment["exit"]
            intersection = entry_wp.is_junction
            road_id, section_id, lane_id = entry_wp.road_id, entry_wp.section_id, entry_wp.lane_id

            for vertex in entry_xyz, exit_xyz:
                # Adding unique nodes and populating id_map
                if vertex not in self._id_map:
                    new_id = len(self._id_map)
                    self._id_map[vertex] = new_id
                    self._graph.add_node(new_id, vertex=vertex)
            n1 = self._id_map[entry_xyz]
            n2 = self._id_map[exit_xyz]
            if road_id not in self._road_id_to_edge:
                self._road_id_to_edge[road_id] = {}
            if section_id not in self._road_id_to_edge[road_id]:
                self._road_id_to_edge[road_id][section_id] = {}
            self._road_id_to_edge[road_id][section_id][lane_id] = (n1, n2)

            entry_carla_vector = entry_wp.transform.rotation.get_forward_vector()
            exit_carla_vector = exit_wp.transform.rotation.get_forward_vector()
            net_carla_vector = (exit_wp.transform.location - entry_wp.transform.location).make_unit_vector()

            # Adding edge with attributes
            self._graph.add_edge(
                n1,
                n2,
                length=len(path) + 1,
                path=path,
                entry_waypoint=entry_wp,
                exit_waypoint=exit_wp,
                entry_vector=np.array([entry_carla_vector.x, entry_carla_vector.y, entry_carla_vector.z]),
                exit_vector=np.array([exit_carla_vector.x, exit_carla_vector.y, exit_carla_vector.z]),
                net_vector=[net_carla_vector.x, net_carla_vector.y, net_carla_vector.z],
                intersection=intersection,
                road_option=RoadOption.LANEFOLLOW,
            )

    def _find_loose_ends(self) -> None:
        """Find road segments that have an unconnected end and add them to the graph."""
        count_loose_ends = 0
        hop_resolution = self._sampling_resolution
        for segment in self._topology:
            end_wp = segment["exit"]
            exit_xyz = segment["exitxyz"]
            road_id, section_id, lane_id = end_wp.road_id, end_wp.section_id, end_wp.lane_id
            if (
                road_id in self._road_id_to_edge
                and section_id in self._road_id_to_edge[road_id]
                and lane_id in self._road_id_to_edge[road_id][section_id]
            ):
                pass
            else:
                count_loose_ends += 1
                if road_id not in self._road_id_to_edge:
                    self._road_id_to_edge[road_id] = {}
                if section_id not in self._road_id_to_edge[road_id]:
                    self._road_id_to_edge[road_id][section_id] = {}
                n1 = self._id_map[exit_xyz]
                n2 = _NEGATIVE_NODE_MULTIPLIER * count_loose_ends
                self._road_id_to_edge[road_id][section_id][lane_id] = (n1, n2)
                next_wp = end_wp.next(hop_resolution)
                path: list[carla.Waypoint] = []
                while (
                    next_wp
                    and next_wp[0].road_id == road_id
                    and next_wp[0].section_id == section_id
                    and next_wp[0].lane_id == lane_id
                ):
                    path.append(next_wp[0])
                    next_wp = next_wp[0].next(hop_resolution)
                if path:
                    n2_xyz = (
                        path[-1].transform.location.x,
                        path[-1].transform.location.y,
                        path[-1].transform.location.z,
                    )
                    self._graph.add_node(n2, vertex=n2_xyz)
                    self._graph.add_edge(
                        n1,
                        n2,
                        length=len(path) + 1,
                        path=path,
                        entry_waypoint=end_wp,
                        exit_waypoint=path[-1],
                        entry_vector=None,
                        exit_vector=None,
                        net_vector=None,
                        intersection=end_wp.is_junction,
                        road_option=RoadOption.LANEFOLLOW,
                    )

    def _lane_change_link(self) -> None:
        """Place zero cost links in the topology graph representing availability of lane changes."""
        left_found = False
        right_found = False

        for segment in self._topology:
            left_found = False
            right_found = False

            for waypoint in segment["path"]:
                if not segment["entry"].is_junction:
                    if (
                        waypoint.right_lane_marking
                        and waypoint.right_lane_marking.lane_change & carla.LaneChange.Right
                        and not right_found
                    ):
                        next_waypoint = waypoint.get_right_lane()
                        if (
                            next_waypoint is not None
                            and next_waypoint.lane_type == carla.LaneType.Driving
                            and waypoint.road_id == next_waypoint.road_id
                        ):
                            next_segment = self._localize(next_waypoint.transform.location)
                            if next_segment is not None:
                                self._graph.add_edge(
                                    self._id_map[segment["entryxyz"]],
                                    next_segment[0],
                                    entry_waypoint=waypoint,
                                    exit_waypoint=next_waypoint,
                                    intersection=False,
                                    exit_vector=None,
                                    path=[],
                                    length=_ZERO_COST_LENGTH,
                                    road_option=RoadOption.CHANGELANERIGHT,
                                    change_waypoint=next_waypoint,
                                )
                                right_found = True
                    if (
                        waypoint.left_lane_marking
                        and waypoint.left_lane_marking.lane_change & carla.LaneChange.Left
                        and not left_found
                    ):
                        next_waypoint = waypoint.get_left_lane()
                        if (
                            next_waypoint is not None
                            and next_waypoint.lane_type == carla.LaneType.Driving
                            and waypoint.road_id == next_waypoint.road_id
                        ):
                            next_segment = self._localize(next_waypoint.transform.location)
                            if next_segment is not None:
                                self._graph.add_edge(
                                    self._id_map[segment["entryxyz"]],
                                    next_segment[0],
                                    entry_waypoint=waypoint,
                                    exit_waypoint=next_waypoint,
                                    intersection=False,
                                    exit_vector=None,
                                    path=[],
                                    length=_ZERO_COST_LENGTH,
                                    road_option=RoadOption.CHANGELANELEFT,
                                    change_waypoint=next_waypoint,
                                )
                                left_found = True
                if left_found and right_found:
                    break

    def _localize(self, location: carla.Location) -> tuple[int, int] | None:
        """Find the road segment that a given location is part of.

        Args:
            location: carla.Location to localize

        Returns:
            tuple of node ids or None if not found
        """
        waypoint = self._wmap.get_waypoint(location)
        edge: tuple[int, int] | None = None
        with contextlib.suppress(KeyError):
            edge = self._road_id_to_edge[waypoint.road_id][waypoint.section_id][waypoint.lane_id]
        return edge

    def _distance_heuristic(self, n1: int, n2: int) -> float:
        """Calculate distance heuristic for path searching.

        Args:
            n1: first node id
            n2: second node id

        Returns:
            Euclidean distance between nodes
        """
        l1 = np.array(self._graph.nodes[n1]["vertex"])
        l2 = np.array(self._graph.nodes[n2]["vertex"])
        return float(np.linalg.norm(l1 - l2))

    def _path_search(self, origin: carla.Location, destination: carla.Location) -> list[int]:
        """Find the shortest path using A* search with distance heuristic.

        Args:
            origin: start position
            destination: end position

        Returns:
            path as list of node ids
        """
        start = self._localize(origin)
        end = self._localize(destination)

        route = nx.astar_path(
            self._graph,
            source=start[0],
            target=end[0],
            heuristic=self._distance_heuristic,
            weight=_ASTAR_WEIGHT_LENGTH,
        )
        route.append(end[1])
        return route

    def _successive_last_intersection_edge(
        self,
        index: int,
        route: list[int],
    ) -> tuple[int | None, _EdgeDict | None]:
        """Return the last successive intersection edge from a starting index.

        Args:
            index: starting index on the route
            route: list of node ids

        Returns:
            tuple of (last_node, last_intersection_edge)
        """
        last_intersection_edge: _EdgeDict | None = None
        last_node: int | None = None
        for node1, node2 in [(route[i], route[i + 1]) for i in range(index, len(route) - 1)]:
            candidate_edge: _EdgeDict = self._graph.edges[node1, node2]
            if node1 == route[index]:
                last_intersection_edge = candidate_edge
            if candidate_edge["road_option"] == RoadOption.LANEFOLLOW and candidate_edge["intersection"]:
                last_intersection_edge = candidate_edge
                last_node = node2
            else:
                break

        return last_node, last_intersection_edge

    def _turn_decision(
        self,
        index: int,
        route: list[int],
        threshold: float = math.radians(_TURN_DECISION_THRESHOLD_DEGREES),
    ) -> RoadOption:
        """Return the turn decision (RoadOption) for pair of edges around current index.

        Args:
            index: current index in route
            route: list of node ids
            threshold: angle threshold in radians

        Returns:
            RoadOption representing the turn decision
        """
        decision: RoadOption | None = None
        previous_node = route[index - 1]
        current_node = route[index]
        next_node = route[index + 1]
        next_edge: _EdgeDict = self._graph.edges[current_node, next_node]
        if index > 0:
            if (
                self._previous_decision != RoadOption.VOID
                and self._intersection_end_node > 0
                and self._intersection_end_node != previous_node
                and next_edge["road_option"] == RoadOption.LANEFOLLOW
                and next_edge["intersection"]
            ):
                decision = self._previous_decision
            else:
                self._intersection_end_node = _UNINITIALIZED_NODE_ID
                current_edge: _EdgeDict = self._graph.edges[previous_node, current_node]
                calculate_turn = (
                    current_edge["road_option"] == RoadOption.LANEFOLLOW
                    and not current_edge["intersection"]
                    and next_edge["road_option"] == RoadOption.LANEFOLLOW
                    and next_edge["intersection"]
                )
                if calculate_turn:
                    last_node, tail_edge = self._successive_last_intersection_edge(index, route)
                    self._intersection_end_node = last_node
                    if tail_edge is not None:
                        next_edge = tail_edge
                    cv, nv = current_edge["exit_vector"], next_edge["exit_vector"]
                    if cv is None or nv is None:
                        return next_edge["road_option"]
                    cross_list: list[float] = []
                    for neighbor in self._graph.successors(current_node):
                        select_edge: _EdgeDict = self._graph.edges[current_node, neighbor]
                        if select_edge["road_option"] == RoadOption.LANEFOLLOW and neighbor != route[index + 1]:
                            sv = select_edge["net_vector"]
                            cross_list.append(np.cross(cv, sv)[2])  # type: ignore[arg-type]
                    next_cross = np.cross(cv, nv)[2]  # type: ignore[arg-type]
                    deviation = math.acos(
                        np.clip(
                            np.dot(cv, nv) / (np.linalg.norm(cv) * np.linalg.norm(nv)),
                            _CLIP_ANGLE_MIN,
                            _CLIP_ANGLE_MAX,
                        ),
                    )
                    if not cross_list:
                        cross_list.append(_ZERO_CROSS_VALUE)
                    decision = self._classify_turn(
                        deviation, threshold, next_cross, cross_list,
                    )
                else:
                    decision = next_edge["road_option"]

        else:
            decision = next_edge["road_option"]

        self._previous_decision = decision
        return decision

    @staticmethod
    def _classify_turn(
        deviation: float,
        threshold: float,
        next_cross: float,
        cross_list: list[float],
    ) -> RoadOption:
        """Classify a turn based on deviation angle and cross-product values.

        Args:
            deviation: angle deviation in radians
            threshold: angle threshold in radians
            next_cross: cross product of current and next vectors
            cross_list: list of cross products for neighboring edges

        Returns:
            RoadOption for the classified turn
        """
        if deviation < threshold:
            return RoadOption.STRAIGHT
        if cross_list and next_cross < min(cross_list):
            return RoadOption.LEFT
        if cross_list and next_cross > max(cross_list):
            return RoadOption.RIGHT
        if next_cross < _ZERO_CROSS_VALUE:
            return RoadOption.LEFT
        if next_cross > _ZERO_CROSS_VALUE:
            return RoadOption.RIGHT
        return RoadOption.STRAIGHT

    def _find_closest_in_list(
        self,
        current_waypoint: carla.Waypoint,
        waypoint_list: list[carla.Waypoint],
    ) -> int:
        """Find the index of the closest waypoint in a list.

        Args:
            current_waypoint: reference waypoint
            waypoint_list: list of waypoints to search

        Returns:
            index of closest waypoint
        """
        min_distance = math.inf
        closest_index = _NO_CLOSEST_WAYPOINT
        for i, waypoint in enumerate(waypoint_list):
            distance = waypoint.transform.location.distance(current_waypoint.transform.location)
            if distance < min_distance:
                min_distance = distance
                closest_index = i

        return closest_index
