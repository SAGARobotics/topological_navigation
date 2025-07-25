#!/usr/bin/env python
"""
Created on Tue Sep 29 16:06:36 2020

@author: Adam Binch (abinch@sagarobotics.com)
"""
#########################################################################################################
from __future__ import division
import rospy, tf, tf2_ros, math
import yaml, json
import re, uuid, copy, os
import multiprocessing, rospkg
from datetime import datetime, timezone
from numpy import round, arctan2, mean

from topological_navigation_msgs.msg import *
import topological_navigation_msgs.srv
import std_msgs.msg

from std_srvs.srv import Trigger, TriggerResponse
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped, Pose

from rospy_message_converter import message_converter
from topological_navigation.tmap_utils import get_node_from_tmap2, get_node_names_from_edge_id_2, get_edge_from_id_tmap2


def pose_dist(pose1, pose2):
    return math.sqrt((pose1["position"]["x"] - pose2["position"]["x"])**2 + (pose1["position"]["y"] - pose2["position"]["y"])**2)

class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True
#########################################################################################################


#########################################################################################################
class map_manager_2(object):


    def __init__(self, advertise_srvs=True):

        self.advertise_srvs = advertise_srvs
        self.advertised = False

        self.cache_maps = rospy.get_param("~cache_topological_maps", False)
        self.auto_write = rospy.get_param("~auto_write_topological_maps", False)

        rospy.loginfo("cache_topological_maps: {}".format(self.cache_maps))
        rospy.loginfo("auto_write_topological_maps: {}".format(self.auto_write))

        self.cache_dir = os.path.join(os.path.expanduser("~"), ".ros", "topological_maps")
        if not os.path.exists(self.cache_dir):
            os.mkdir(self.cache_dir)

        self.goal_mappings = {}
        mb_goal_f = "{}/config/{}".format(rospkg.RosPack().get_path("topological_navigation"), "move_base_goal.yaml")
        with open(mb_goal_f, "r") as f:
            self.move_base_goal = yaml.safe_load(f)["topological_navigation/move_base_goal"]


    def init_map(self, name="new_map", metric_map="map_2d", pointset="new_map", transformation="default", filename="", load=True):

        self.name = name
        self.metric_map = metric_map
        self.pointset = pointset
        self.last_nodes = []

        if transformation == "default":
            self.transformation = {}
            self.transformation["rotation"] = {}
            self.transformation["rotation"]["w"] = 1.0
            self.transformation["rotation"]["x"] = 0.0
            self.transformation["rotation"]["y"] = 0.0
            self.transformation["rotation"]["z"] = 0.0
            self.transformation["translation"] = {}
            self.transformation["translation"]["x"] = 0.0
            self.transformation["translation"]["y"] = 0.0
            self.transformation["translation"]["z"] = 0.0
            self.transformation["child"] = "topo_map"
            self.transformation["parent"] = "map"
        else:
            self.transformation = transformation

        self.filename = filename
        if not self.filename:
            self.filename = os.path.join(self.cache_dir, self.name + ".tmap2")

        self.loaded = False
        self.load = load
        if self.load:
            self.load_map(self.filename)
        else:
            self.tmap2 = {}
            self.tmap2["name"] = self.name
            self.tmap2["metric_map"] = self.metric_map
            self.tmap2["pointset"] = self.pointset
            self.tmap2["transformation"] = self.transformation
            self.tmap2["nodes"] = []
            rospy.set_param('topological_map2_name', self.pointset)
            rospy.set_param('topological_map2_filename', os.path.split(self.filename)[1])
            rospy.set_param('topological_map2_path', os.path.split(self.filename)[0])

        self.set_meta()

        self.pub_simple_map = rospy.get_param("~publish_simple_topo_map", True)

        self.map_pub = rospy.Publisher('/topological_map_2', std_msgs.msg.String, latch=True, queue_size=1)
        self.publish_topological_map()
        self.names = self.create_list_of_nodes()

        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.broadcast_transform()

        self.convert_to_legacy = rospy.get_param("~convert_to_legacy", False)
        if self.tmap2 and self.convert_to_legacy and not self.pub_simple_map:
            self.points_pub = rospy.Publisher('/topological_map', topological_navigation_msgs.msg.TopologicalMap, latch=True, queue_size=1)
            self.tmap2_to_tmap()
            self.points_pub.publish(self.points)

        if self.advertise_srvs and not self.advertised:
            self.advertise()


    def set_meta(self):

        if "meta" not in self.tmap2:
            self.tmap2["meta"] = {}

        if "_other" not in self.tmap2["meta"]:
            self.tmap2["meta"]["_other"] = {}

        if "last_updated" not in self.tmap2["meta"]:
            self.tmap2["meta"]["last_updated"] = self.get_time()

        if "version" not in self.tmap2["meta"]:
            self.tmap2["meta"]["version"] = rospy.get_param("~default_version", "0.1")

        self.insert_datum(rospy.get_param('topological_map2_path'))


    def insert_datum(self, path_to_datum):

        if "datum_latitude" not in self.tmap2["meta"] or "datum_longitude" not in self.tmap2["meta"]:
            datum_latitude, datum_longitude = self.get_datum(path_to_datum)

            if datum_latitude and datum_longitude:
                self.tmap2["meta"]["datum_latitude"] = datum_latitude
                self.tmap2["meta"]["datum_longitude"] = datum_longitude


    def get_datum(self, path):

        try:
            with open(path + "/datum.yaml", "r") as f:
                datum = yaml.safe_load(f)
                datum_latitude = datum["datum_latitude"]
                datum_longitude = datum["datum_longitude"]
        except Exception as err:
            rospy.logwarn("Could not get datum from topological_map2_path {}. Exception is {}".format(path, err))
            datum_latitude = None
            datum_longitude = None

        return datum_latitude, datum_longitude


    def advertise(self):

        # Services that retrieve information from the map
        self.get_map_srv=rospy.Service('/topological_map_manager2/get_topological_map', Trigger, self.get_topological_map_cb)
        self.get_tagged_srv=rospy.Service('/topological_map_manager2/get_tagged_nodes', topological_navigation_msgs.srv.GetTaggedNodes, self.get_tagged_cb)
        self.get_tag_srv=rospy.Service('/topological_map_manager2/get_tags', topological_navigation_msgs.srv.GetTags, self.get_tags_cb)
        self.get_node_tag_srv=rospy.Service('/topological_map_manager2/get_node_tags', topological_navigation_msgs.srv.GetNodeTags, self.get_node_tags_cb)
        self.get_node_edges_srv=rospy.Service('/topological_map_manager2/get_edges_between_nodes', topological_navigation_msgs.srv.GetEdgesBetweenNodes, self.get_edges_between_cb)
        self.get_version_srv=rospy.Service('/topological_map_manager2/get_version', Trigger, self.get_version_cb)

        # Services that modify the map
        self.write_map_srv=rospy.Service('/topological_map_manager2/write_topological_map', topological_navigation_msgs.srv.WriteTopologicalMap, self.write_topological_map_cb)
        self.switch_map_srv=rospy.Service('/topological_map_manager2/switch_topological_map', topological_navigation_msgs.srv.WriteTopologicalMap, self.switch_topological_map_cb)
        self.reload_map_srv=rospy.Service('/topological_map_manager2/reload_topological_map', Trigger, self.reload_topological_map_cb)
        self.add_node_srv=rospy.Service('/topological_map_manager2/add_topological_node', topological_navigation_msgs.srv.AddNode, self.add_topological_node_cb)
        self.remove_node_srv=rospy.Service('/topological_map_manager2/remove_topological_node', topological_navigation_msgs.srv.RmvNode, self.remove_node_cb)
        self.add_edges_srv=rospy.Service('/topological_map_manager2/add_edges_between_nodes', topological_navigation_msgs.srv.AddEdge, self.add_edge_cb)
        self.remove_edge_srv=rospy.Service('/topological_map_manager2/remove_edge', topological_navigation_msgs.srv.AddEdge, self.remove_edge_cb)
        self.add_content_to_node_srv=rospy.Service('/topological_map_manager2/add_content_to_node', topological_navigation_msgs.srv.AddContent, self.add_content_cb)
        self.update_node_name_srv = rospy.Service("/topological_map_manager2/update_node_name", topological_navigation_msgs.srv.UpdateNodeName, self.update_node_name_cb)
        self.update_node_waypoint_srv = rospy.Service("/topological_map_manager2/update_node_pose", topological_navigation_msgs.srv.AddNode, self.update_node_waypoint_cb)
        self.update_node_tolerance_srv = rospy.Service("/topological_map_manager2/update_node_tolerance", topological_navigation_msgs.srv.UpdateNodeTolerance, self.update_node_tolerance_cb)
        self.modify_tag_srv=rospy.Service('/topological_map_manager2/modify_node_tags', topological_navigation_msgs.srv.ModifyTag, self.modify_tag_cb)
        self.add_tag_srv=rospy.Service('/topological_map_manager2/add_tag_to_node', topological_navigation_msgs.srv.AddTag, self.add_tag_cb)
        self.rm_tag_srv=rospy.Service('/topological_map_manager2/rm_tag_from_node', topological_navigation_msgs.srv.AddTag, self.rm_tag_cb)
        self.add_param_to_edge_config_srv=rospy.Service('/topological_map_manager2/add_param_to_edge_config', topological_navigation_msgs.srv.UpdateEdgeConfig, self.add_param_to_edge_config_cb)
        self.rm_param_from_edge_config_srv=rospy.Service('/topological_map_manager2/rm_param_from_edge_config', topological_navigation_msgs.srv.UpdateEdgeConfig, self.rm_param_from_edge_config_cb)
        self.rm_param_from_topological_map_srv=rospy.Service('/topological_map_manager2/rm_param_from_topological_map', topological_navigation_msgs.srv.UpdateEdgeConfig, self.rm_param_from_topological_map_cb)
        self.update_node_restrictions_srv=rospy.Service('/topological_map_manager2/update_node_restrictions', topological_navigation_msgs.srv.UpdateRestrictions, self.update_node_restrictions_cb)
        self.update_edge_restrictions_srv=rospy.Service('/topological_map_manager2/update_edge_restrictions', topological_navigation_msgs.srv.UpdateRestrictions, self.update_edge_restrictions_cb)
        self.update_edge_srv=rospy.Service('/topological_map_manager2/update_edge', topological_navigation_msgs.srv.UpdateEdge, self.update_edge_cb)
        self.update_action_srv=rospy.Service('/topological_map_manager2/update_action', topological_navigation_msgs.srv.UpdateAction, self.update_action_cb)
        self.add_datum_srv=rospy.Service('/topological_map_manager2/add_datum', topological_navigation_msgs.srv.AddDatum, self.add_datum_cb)
        self.update_fail_policy_srv=rospy.Service('/topological_map_manager2/update_fail_policy', topological_navigation_msgs.srv.UpdateFailPolicy, self.update_fail_policy_cb)
        self.set_influence_zone_srv=rospy.Service('/topological_map_manager2/set_node_influence_zone', topological_navigation_msgs.srv.SetInfluenceZone, self.set_influence_zone_cb)
        self.clear_nodes_srv=rospy.Service('/topological_map_manager2/clear_topological_nodes', Empty, self.clear_nodes_cb)
        self.set_version_srv=rospy.Service('/topological_map_manager2/set_version', topological_navigation_msgs.srv.SetVersion, self.set_version_cb)
        self.insert_node_srv=rospy.Service('/topological_map_manager2/insert_topological_node', topological_navigation_msgs.srv.InsertNode, self.insert_node_cb)

        # Services for modifying the map quickly
        self.add_nodes_srv=rospy.Service('/topological_map_manager2/add_topological_node_multi', topological_navigation_msgs.srv.AddNodeArray, self.add_topological_nodes_cb)
        self.update_node_waypoints_srv = rospy.Service("/topological_map_manager2/update_node_pose_multi", topological_navigation_msgs.srv.AddNodeArray, self.update_node_waypoints_cb)
        self.add_edges_srv=rospy.Service('/topological_map_manager2/add_edges_between_nodes_multi', topological_navigation_msgs.srv.AddEdgeArray, self.add_edges_cb)
        self.add_params_to_edges_srv=rospy.Service('/topological_map_manager2/add_param_to_edge_config_multi', topological_navigation_msgs.srv.UpdateEdgeConfigArray, self.add_params_to_edges_cb)
        self.set_influence_zones_srv=rospy.Service('/topological_map_manager2/set_node_influence_zone_multi', topological_navigation_msgs.srv.SetInfluenceZoneArray, self.set_influence_zones_cb)

        self.advertised = True


    def get_time(self):
        return datetime.now(timezone.utc).isoformat()


    def load_map(self, filename):

        def loader(filename, transporter):
            try:
                with open(filename, "r") as f:
                    transporter["tmap2"] = yaml.safe_load(f)
            except Exception as e:
                rospy.logerr(e)
                transporter["tmap2"] = {}


        self.loaded = False
        rospy.loginfo("Loading Topological Map {} ...".format(filename))

        transporter = multiprocessing.Manager().dict()
        p = multiprocessing.Process(target=loader, args=(filename, transporter))
        p.start()
        p.join()

        self.tmap2 = transporter["tmap2"]
        if not self.tmap2:
            return

        e1 = "Loaded map is {} and should be {}."
        e2 = " You may be attemting to load a legacy map using topological_navigation/map_manager2.py." \
                " In that case please use topological_navigation/map_manager.py instead."

        map_type = type(self.tmap2)
        if map_type is list:
            rospy.logerr((e1+e2).format(map_type, dict))
            self.tmap2 = {}
            return
        elif map_type is not dict:
            rospy.logerr(e1.format(map_type, dict))
            self.tmap2 = {}
            return

        self.loaded = True

        self.name = self.tmap2["name"]
        self.metric_map = self.tmap2["metric_map"]
        self.pointset = self.tmap2["pointset"]
        self.transformation = self.tmap2["transformation"]

        self.names = self.create_list_of_nodes()

        rospy.set_param('topological_map2_name', self.pointset)
        rospy.set_param('topological_map2_filename', os.path.split(self.filename)[1])
        rospy.set_param('topological_map2_path', os.path.split(self.filename)[0])

        rospy.loginfo("Done")

        self.map_check()

        if self.cache_maps:
            rospy.loginfo("Caching the map...")
            self.write_topological_map(os.path.join(self.cache_dir, os.path.basename(self.filename)))


    def publish_topological_map(self):

        tmap_to_pub = copy.deepcopy(self.tmap2)

        if self.pub_simple_map:
            for node in tmap_to_pub["nodes"]:
                if "map" in node["meta"]:
                    del node["meta"]["map"]
                if "pointset" in node["meta"]:
                    del node["meta"]["pointset"]
                if "restrictions_planning" in node["node"]:
                    del node["node"]["restrictions_planning"]
                if "restrictions_runtime" in node["node"]:
                    del node["node"]["restrictions_runtime"]

                for edge in node["node"]["edges"]:
                    if "fail_policy" in edge:
                        del edge["fail_policy"]
                    if "fluid_navigation" in edge:
                        del edge["fluid_navigation"]
                    if "goal" in edge:
                        del edge["goal"]
                    if "recovery_behaviours_config" in edge:
                        del edge["recovery_behaviours_config"]
                    if "restrictions_planning" in edge:
                        del edge["restrictions_planning"]
                    if "restrictions_runtime" in edge:
                        del edge["restrictions_runtime"]

        self.map_pub.publish(std_msgs.msg.String(json.dumps(tmap_to_pub, default=str)))


    def write_topological_map(self, filename, no_alias=False):

        rospy.loginfo("Writing map to {} ...".format(filename))

        self.insert_datum(os.path.split(filename)[0])

        nodes = copy.deepcopy(self.tmap2["nodes"])
        nodes.sort(key=lambda node: node["node"]["name"])
        self.tmap2["nodes"] = nodes

        self.process_floats()

        if no_alias:
            rospy.loginfo("Disabling anchors and aliases in topological map yaml ...")
            yml = yaml.dump(self.tmap2, default_flow_style=False, Dumper=NoAliasDumper)
        else:
            yml = yaml.safe_dump(self.tmap2, default_flow_style=False)

        fh = open(filename, "w")
        fh.write(str(yml))
        fh.close

        rospy.loginfo("Done")


    def process_floats(self):

        def process_float(node_name, field, value):
            if not isinstance(value, float):
                try:
                    value = float(value)
                except Exception as err:
                    rospy.logerr("Could not convert {} value {} to float for node {}. Exception is {}".format(field, value, node_name, err))
            if isinstance(value, float):
                value = float(round(value, 3))
            return value


        for node in self.tmap2["nodes"]:
            name = node["node"]["name"]
            node["node"]["pose"]["position"]["x"] = process_float(name, "position", node["node"]["pose"]["position"]["x"])
            node["node"]["pose"]["position"]["y"] = process_float(name, "position", node["node"]["pose"]["position"]["y"])
            node["node"]["pose"]["position"]["z"] = process_float(name, "position", node["node"]["pose"]["position"]["z"])

            for vert in node["node"]["verts"]:
                vert["x"] = process_float(name, "vert", vert["x"])
                vert["y"] = process_float(name, "vert", vert["y"])


    def update(self, update_time=True):

        if update_time:
            self.tmap2["meta"]["last_updated"] = self.get_time()

        self.publish_topological_map()
        self.names = self.create_list_of_nodes()
        self.map_check()

        if self.tmap2 and self.convert_to_legacy:
            self.tmap2_to_tmap()
            self.points_pub.publish(self.points)


    def broadcast_transform(self):

        trans = message_converter.convert_dictionary_to_ros_message("geometry_msgs/Vector3", self.transformation["translation"])
        rot = message_converter.convert_dictionary_to_ros_message("geometry_msgs/Quaternion", self.transformation["rotation"])

        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.transformation["parent"]
        msg.child_frame_id = self.transformation["child"]
        msg.transform.translation = trans
        msg.transform.rotation = rot

        self.broadcaster.sendTransform(msg)


    def create_list_of_nodes(self):

        names = []
        if "nodes" in self.tmap2:
            names = [node["node"]["name"] for node in self.tmap2["nodes"]]
            names.sort()
        return names


    def get_topological_map_cb(self, req):
        """
        Returns the topological map
        """
        ans = TriggerResponse()
        ans.success = True
        ans.message = json.dumps(self.tmap2)

        return ans


    def switch_topological_map_cb(self, req):
        """
        Changes the topological map
        """
        rospy.set_param('topological_map2_filename', req.filename)
        path = rospy.get_param('topological_map2_path')
        self.filename = path + "/" + req.filename

        self.load_map(self.filename)
        self.update(False)
        self.broadcast_transform()

        return True, json.dumps(self.tmap2)

    def reload_topological_map_cb(self, req):
        """
        Reload the topological map
        """
        self.load_map(self.filename)
        self.update(False)
        self.broadcast_transform()

        self.last_nodes = []

        ans = TriggerResponse()
        ans.success = True
        ans.message = "Topomap reloaded from file"

        return ans


    def get_tagged_cb(self, req):
        """
        Returns a list of nodes that have a given tag
        """
        res = topological_navigation_msgs.srv.GetTaggedNodesResponse()
        res.nodes=[]
        res.poses=[]
        for node in self.tmap2["nodes"]:
            if "tag" in node["meta"]:
                for tag in node["meta"]["tag"]:
                    if str(tag) == str(req.tag):
                        pose = message_converter.convert_dictionary_to_ros_message("geometry_msgs/Pose", node["node"]["pose"])
                        res.nodes.append(node["node"]["name"])
                        res.poses.append(pose)
        return res


    def get_tags_cb(self, req):
        """
        Returns a list of available tags in the map
        """
        tags = [tag for node in self.tmap2["nodes"] if "tag" in node["meta"] for tag in node["meta"]["tag"]]
        return [set(tags)]


    def get_node_tags_cb(self, req):
        """
        Returns a list of a node's tags
        """
        num_available = 0
        for node in self.tmap2["nodes"]:
            if node["node"]["name"] == req.node_name:
                if "tag" in node["meta"]:
                    tags = node["meta"]["tag"]
                else:
                    tags = []

                num_available+=1

        succeded = True
        if num_available != 1:
            succeded = False
            tags = []

        return succeded, tags


    def get_edges_between_cb(self, req):
        """
        Returns a list of the ids of edges from nodea to nodeb and vice-versa
        """
        return self.get_edges_between(req.nodea, req.nodeb)


    def get_edges_between(self, nodea, nodeb):

         ab=[]; ba=[]
         for node in self.tmap2["nodes"]:
             if nodea == node["node"]["name"]:
                 for edge in node["node"]["edges"]:
                     if edge["node"] == nodeb:
                         ab.append(edge["edge_id"])
             if nodeb == node["node"]["name"]:
                 for edge in node["node"]["edges"]:
                     if edge["node"] == nodea:
                         ba.append(edge["edge_id"])

         return ab, ba


    def get_version_cb(self, req):
        """
        get topological map version from meta info
        """
        ans = TriggerResponse()
        if "meta" in self.tmap2 and "version" in self.tmap2["meta"]:
            ans.success = True
            ans.message = str(self.tmap2["meta"]["version"])
        else:
            rospy.logerr("Version not found in topological map meta information")
            ans.success = False

        return ans


    def write_topological_map_cb(self, req):
        """
        Saves the topological map to a yaml file
        """
        filename = req.filename
        if not filename:
            path = rospy.get_param('topological_map2_path')
            fname = rospy.get_param('topological_map2_filename')
            filename = path + "/" + fname

        try:
            message = "Writing map to {}".format(filename)
            self.write_topological_map(filename, req.no_alias)
            success = True
        except Exception as message:
            success = False

        return success, str(message)


    def add_topological_node_cb(self, req):
        """
        Adds a node to the topological map
        """
        return self.add_topological_node(req.name, req.pose, req.add_close_nodes, req.add_previous_node, req.previous_node_name, req.action, req.action_type,
                                         req.one_way)


    def add_topological_node(self, node_name, node_pose, add_close_nodes, add_previous_node, previous_node_name, action, action_type, one_way, dist=8.0,
                             update=True, write_map=True):

        if node_name:
            name = node_name
        else:
            name = self.get_new_name()

        rospy.loginfo("Creating Node {}".format(name))

        if name in self.names:
            rospy.logerr("Node {} already exists, try another name".format(name))
            return False, ""

        pose = message_converter.convert_ros_message_to_dictionary(node_pose)
        close_nodes = []
        if add_close_nodes:
            for node in self.tmap2["nodes"]:
                ndist = pose_dist(pose, node["node"]["pose"])
                if ndist < dist :
                    if node["node"]["name"] != "ChargingPoint":
                        close_nodes.append(node["node"]["name"])

        self.add_node(name, pose)

        if not action:
            action = "move_base"

        if not action_type:
            action_type = "move_base_msgs/MoveBaseGoal"

        for close_node in close_nodes:
            edge_id_1 = name + "_" + close_node
            edge_id_2 = close_node + "_" + name
            self.add_edge(name, close_node, action, action_type, edge_id_1, update=False, write_map=False)
            self.add_edge(close_node, name, action, action_type, edge_id_2, update=False, write_map=False)

        last_node = self.last_nodes[-1] if self.last_nodes else None
        last_node = previous_node_name if previous_node_name else last_node

        if add_previous_node and last_node is not None:
            self.set_node_orientation(name, last_node)
            edge_id_1 = name + "_" + last_node
            edge_id_2 = last_node + "_" + name
            if not one_way:
                self.add_edge(name, last_node, action, action_type, edge_id_1, update=False, write_map=False)
            self.add_edge(last_node, name, action, action_type, edge_id_2, update=False, write_map=False)

        self.last_nodes.append(name)

        if update:
            self.update()
        if self.auto_write and write_map:
            self.write_topological_map(self.filename)

        return True, name


    def get_new_name(self):

        namesnum=[]
        for i in self.names :
            if i.startswith('WayPoint') :
                nam = i.strip('WayPoint')
                namesnum.append(int(nam))
        namesnum.sort()
        if namesnum:
            nodname = 'WayPoint%d'%(int(namesnum[-1])+1)
        else :
            nodname = 'WayPoint1'

        return nodname


    def add_node(self, name, pose, localise_by_topic="", verts="default", properties="default", tags=[],
                 restrictions_planning="True", restrictions_runtime="True"):

        if "orientation" not in pose:
            pose["orientation"] = {}
            pose["orientation"]["w"] = 1.0
            pose["orientation"]["x"] = 0.0
            pose["orientation"]["y"] = 0.0
            pose["orientation"]["z"] = 0.0

        node = {}
        node["meta"] = {}
        node["meta"]["map"] = self.metric_map
        node["meta"]["node"] = name
        node["meta"]["pointset"] = self.pointset
        if tags:
            node["meta"]["tag"] = tags

        node["node"] = {}
        node["node"]["edges"] = []
        node["node"]["localise_by_topic"] = localise_by_topic
        node["node"]["name"] = name
        node["node"]["pose"] = pose

        if verts == "default":
            node["node"]["verts"] = self.generate_circle_vertices()
        else:
            node["node"]["verts"] = verts

        if properties == "default":
            node["node"]["properties"] = {}
            node["node"]["properties"]["xy_goal_tolerance"] = 0.3
            node["node"]["properties"]["yaw_goal_tolerance"] = 0.1
        else:
            node["node"]["properties"] = properties

        node["node"]["restrictions_planning"] = restrictions_planning
        node["node"]["restrictions_runtime"] = restrictions_runtime

        node["node"]["parent_frame"] = self.transformation["parent"]

        self.tmap2["nodes"].append(node)


    def generate_circle_vertices(self, radius=0.75, number=8):

        separation_angle = 2 * math.pi / number
        start_angle = separation_angle / 2
        current_angle = start_angle
        points = []
        for i in range(0, number):
            points.append({"x": math.cos(current_angle) * radius, "y": math.sin(current_angle) * radius})
            current_angle += separation_angle

        return points


    def set_node_orientation(self, new_node_name, last_node_name):

        new_node = get_node_from_tmap2(self.tmap2, new_node_name)
        last_node = get_node_from_tmap2(self.tmap2, last_node_name)

        if new_node is not None and last_node is not None:
            dx = new_node["node"]["pose"]["position"]["x"] - last_node["node"]["pose"]["position"]["x"]
            dy = new_node["node"]["pose"]["position"]["y"] - last_node["node"]["pose"]["position"]["y"]

            angle = tf.transformations.quaternion_from_euler(0.0, 0.0, arctan2(dy, dx))

            for node in self.tmap2["nodes"]:
                name = node["node"]["name"]
                if (name == last_node_name) or (name == new_node_name):
                    node["node"]["pose"]["orientation"]["x"] = float(angle[0])
                    node["node"]["pose"]["orientation"]["y"] = float(angle[1])
                    node["node"]["pose"]["orientation"]["z"] = float(angle[2])
                    node["node"]["pose"]["orientation"]["w"] = float(angle[3])


    def add_edge_cb(self, req):
        """
        Adds an edge to a topological node
        """
        return self.add_edge(req.origin, req.destination, req.action, req.action_type, req.edge_id)


    def add_edge(self, origin, destination, action, action_type, edge_id, update=True, write_map=True):

        rospy.loginfo("Adding Edge from {} to {} using {}".format(origin, destination, action))

        num_available, index = self.get_instances_of_node(origin)

        if num_available == 1 :
            eids = []
            for edge in self.tmap2["nodes"][index]["node"]["edges"]:
                eids.append(edge["edge_id"])

                if edge_id == edge["edge_id"]:
                    rospy.logerr("Error adding edge to node {}. Edge already exists.".format(origin))
                    return False

            if not edge_id or edge_id in eids:
                test=0
                eid = '%s_%s' %(origin, destination)
                while eid in eids:
                    eid = '%s_%s_%03d' %(origin, destination, test)
                    test += 1
            else:
                eid=edge_id

            self.add_edge_to_node(origin, destination, action, eid, action_type=action_type)

            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)
            return True
        else:
            rospy.logerr("Error adding edge to node {}. {} instances of node with name {} found".format(origin, num_available, origin))
            return False


    def add_edge_to_node(self, origin, destination, action="", edge_id="default", config=[],
                         recovery_behaviours_config="", action_type="", goal=None, fail_policy="fail",
                         restrictions_planning="True", restrictions_runtime="True", fluid_navigation=True):

        edge = {}
        edge["action"] = action

        if edge_id == "default":
            edge["edge_id"] = origin + "_" + destination
        else:
            edge["edge_id"] = edge_id

        edge["node"] = destination
        edge["config"] = config
        edge["recovery_behaviours_config"] = recovery_behaviours_config

        if not action_type:
            action_type = "move_base_msgs/MoveBaseGoal"

        the_action_type, the_goal = self.set_goal(action, action_type, goal)

        edge["action_type"] = the_action_type
        edge["goal"] = the_goal

        edge["fail_policy"] = fail_policy
        edge["restrictions_planning"] = restrictions_planning
        edge["restrictions_runtime"] = restrictions_runtime
        edge["fluid_navigation"] = fluid_navigation

        for node in self.tmap2["nodes"]:
            if node["node"]["name"] == origin:
                node["node"]["edges"].append(edge)


    def set_goal(self, action, action_type, _goal=None):

        if action in self.goal_mappings and action_type == self.goal_mappings[action]["action_type"]:
            goal = self.goal_mappings[action]["goal"]
        else:
            if _goal is not None:
                goal = _goal
            else:
                try:
                    package = action_type.split("/")[0]
                    goal_def = action_type.split("/")[1]

                    _file = rospy.get_param("~" + action_type, "")
                    if not _file:
                        _file = "{}/config/{}.yaml".format(rospkg.RosPack().get_path(package), goal_def)

                    with open(_file, "r") as f:
                        goal = yaml.safe_load(f)
                except:
                    action_type = self.move_base_goal["action_type"]
                    goal = self.move_base_goal["goal"]

            self.goal_mappings[action] = {"action_type": action_type, "goal": goal}

        return action_type, goal


    def set_action_type(self, action):

        package = action + "_msgs"
        items = [item[0].upper() + item[1:] for item in action.split("_")]
        goal_type = "".join(items) + "Goal"
        action_type = package + "/" + goal_type

        return action_type


    def remove_node_cb(self, req):
        """
        Removes a node from the topological map
        """
        return self.remove_node(req.name)


    def remove_node(self, node_name, update=True, write_map=True):

        rospy.loginfo("Removing Node {}".format(node_name))

        num_available, rm_id = self.get_instances_of_node(node_name)

        if num_available == 1:
            del self.tmap2["nodes"][rm_id]

            for node in self.tmap2["nodes"]:
                for edge in node["node"]["edges"]:
                    if edge["node"] == node_name:
                        self.remove_edge(edge["edge_id"], False)

            if node_name in self.last_nodes:
                self.last_nodes.remove(node_name)

            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)
            return True
        else:
            rospy.logerr("Error removing node {}. {} instances of node with name {} found".format(node_name, num_available, node_name))
            return False


    def remove_edge_cb(self, req):
        """
        Removes an edge from a topological node
        """
        return self.remove_edge(req.edge_id)


    def remove_edge(self, edge_name, update=True, write_map=True):

        rospy.loginfo("Removing Edge {}".format(edge_name))

        num_available = 0
        for node in self.tmap2["nodes"]:
            for edge in node["node"]["edges"]:
                if edge["edge_id"] == edge_name:
                    num_available+=1

        if num_available >= 1:
            for node in self.tmap2["nodes"]:
                edges = copy.deepcopy(node["node"]["edges"])
                edges_new = list(filter(lambda edge: edge["edge_id"] != edge_name, edges))
                node["node"]["edges"] = edges_new

            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)
            return True
        else:
            rospy.logerr("No edges with id {} found".format(edge_name))
            return False


    def add_content_cb(self, req):
        """
        Adds content to a node's meta information
        """
        data = json.loads(req.content)

        num_available, index = self.get_instances_of_node(req.node)

        if num_available != 1:
             succeded = False
             meta_out = None
             rospy.logerr("There are no nodes or more than one with name {}".format(req.node))
        else:
            succeded = True
            node_meta = self.tmap2["nodes"][index]["meta"]
            if "contains" in node_meta:
                if type(data) is list:
                    for j in data:
                        if "category" in j and "name" in j:
                            node_meta['contains'].append(j)
                elif type(data) is dict:
                    if "category" in data and "name" in data:
                        node_meta["contains"].append(data)
            else:
                a=[]
                if type(data) is list:
                    for j in data:
                        if "category" in j and "name" in j:
                            a.append(j)
                elif type(data) is dict:
                    if "category" in data and "name" in data:
                        a.append(data)
                node_meta["contains"] = a
            meta_out = str(node_meta)

            rospy.loginfo("Updating %s--%s" %(self.tmap2["name"], req.node))
            self.update()
            if self.auto_write:
                self.write_topological_map(self.filename)

        return succeded, meta_out


    def update_node_name_cb(self, req):
        """
        Changes a node's name and updates edges which involve the renamed node
        """
        return self.update_node_name(req.node_name, req.new_name)


    def update_node_name(self, node_name, new_name, update=True, write_map=True):
        if new_name in self.names:
            return False, "node with name {0} already exists".format(new_name)

        num_available, index = self.get_instances_of_node(node_name)

        if num_available == 1:
            # update all the edges which involve the renamed node
            for node in self.tmap2["nodes"]:
                for edge in node["node"]["edges"]:
                    if node["node"]["name"] == node_name:
                        edge["edge_id"] = new_name + "_" + edge["node"]
                    if edge["node"] == node_name:
                        edge["node"] = new_name
                        edge["edge_id"] = node["node"]["name"] + "_" + new_name

            the_node = copy.deepcopy(self.tmap2["nodes"][index])
            the_node["meta"]["node"] = new_name
            the_node["node"]["name"] = new_name
            self.tmap2["nodes"][index] = the_node

            self.last_nodes = list(map(lambda x: new_name if x == node_name else x, self.last_nodes))

            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)
            return True, ""
        else:
            return False, "Multiple nodes with name {} found, or it does not exist".format(node_name)


    def update_node_waypoint_cb(self, req):
        """
        Updates a node's pose
        """
        return self.update_node_waypoint(req.name, req.pose)


    def update_node_waypoint(self, name, pose_msg, update=True, write_map=True):

        num_available, index = self.get_instances_of_node(name)

        if num_available == 1:
            pose = message_converter.convert_ros_message_to_dictionary(pose_msg)
            self.tmap2["nodes"][index]["node"]["pose"] = pose

            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)
            return True, ""
        else:
            rospy.logerr("Error updating the pose of node {}. {} instances of node with name {} found".format(name, num_available, name))
            return False, ""


    def update_node_tolerance_cb(self, req):
        """
        Update node tolerances
        """
        return self.update_node_tolerance(req.node_name, req.xy_tolerance, req.yaw_tolerance)


    def update_node_tolerance(self, name, new_xy, new_yaw, update=True, write_map=True):

        num_available, index = self.get_instances_of_node(name)

        if num_available == 1:
            the_node = copy.deepcopy(self.tmap2["nodes"][index])

            if "properties" in the_node["node"]:
                the_node["node"]["properties"]["xy_goal_tolerance"] = new_xy
                the_node["node"]["properties"]["yaw_goal_tolerance"] = new_yaw
            else:
                properties = {}
                properties["xy_goal_tolerance"] = new_xy
                properties["yaw_goal_tolerance"] = new_yaw
                the_node["node"]["properties"] = properties

            self.tmap2["nodes"][index] = the_node

            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)
            return True, ""
        else:
            rospy.logerr("Error updating the tolerance of node {}. {} instances of node with name {} found".format(name, num_available, name))
            return False, ""


    def modify_tag_cb(self, msg):
        """
        Changes the tag belonging to a node or a list of nodes
        """
        succeded = True
        meta_out = None
        for node_name in msg.node:
            available = [i for i, node in enumerate(self.tmap2["nodes"]) if node["node"]["name"] == node_name]

            for i in available:
                meta = self.tmap2["nodes"][i]["meta"]
                if "tag" in meta:
                    if not msg.tag in meta["tag"]:
                        continue
                    else:
                        tag_ind = meta["tag"].index(msg.tag)
                        meta["tag"][tag_ind] = msg.new_tag

                meta_out = str(meta)

            if len(available) == 0:
                 succeded = False

        if succeded:
            self.update()
            if self.auto_write:
                self.write_topological_map(self.filename)

        return succeded, meta_out


    def add_tag_cb(self, msg):
        """
        Adds a tag to nodes in the map
        """
        succeded = False
        meta_out = None
        for j in msg.node:
            for node in self.tmap2["nodes"]:
                if j == node["node"]["name"]:
                    succeded = True
                    if "tag" in node["meta"]:
                        if msg.tag not in node["meta"]["tag"]:
                            node["meta"]["tag"].append(msg.tag)
                    else:
                        a = []
                        a.append(msg.tag)
                        node["meta"][ "tag"] = a
                    meta_out = str(node["meta"])

        if succeded:
            self.update()
            if self.auto_write:
                self.write_topological_map(self.filename)

        return succeded, meta_out


    def rm_tag_cb(self, msg):
        """
        Remove a tag from nodes in the map
        """
        succeded = True
        for node_name in msg.node:
            available = [i for i, node in enumerate(self.tmap2["nodes"]) if node["node"]["name"] == node_name]

            succeded = False
            meta_out = None
            for i in available:
                meta = self.tmap2["nodes"][i]["meta"]
                if "tag" in meta:
                    if msg.tag in meta["tag"]:
                        print('removing tag')
                        meta["tag"].remove(msg.tag)
                        print('new list of tags')
                        print(meta["tag"])
                        succeded = True
                meta_out = str(meta)

        if succeded:
            self.update()
            if self.auto_write:
                self.write_topological_map(self.filename)

        return succeded, meta_out


    def add_param_to_edge_config_cb(self, req):
        """
        Update edge reconfigure parameters.
        """
        return self.add_param_to_edge_config(req.edge_id, req.namespace, req.name, req.value, req.value_is_string, req.not_reset)


    def add_param_to_edge_config(self, edge_id, namespace, name, value, value_is_string, not_reset, update=True, write_map=True):

        if not value:
            return False, "no value provided"

        if not value_is_string:
            value = eval(value)

        node_name, _ = get_node_names_from_edge_id_2(self.tmap2, edge_id)
        num_available, index = self.get_instances_of_node(node_name)

        param = {"namespace":namespace, "name":name, "value":value, "reset":not not_reset}

        if num_available == 1:
            the_node = copy.deepcopy(self.tmap2["nodes"][index])
            msg = ""
            for edge in the_node["node"]["edges"]:
                if edge["edge_id"] == edge_id:

                    config = copy.deepcopy(edge["config"])
                    config = [i for i in config if not (i["namespace"] == param["namespace"] and i["name"] == param["name"])]

                    rospy.loginfo("Adding param {} to edge {}".format(param, edge["edge_id"]))
                    config.append(param)
                    edge["config"] = config

                    msg = "edge action is {} and edge config is {}".format(edge["action"], edge["config"])

            self.tmap2["nodes"][index] = the_node
            if update:
                self.update()
            if write_map and self.auto_write:
                self.write_topological_map(self.filename)

            return True, msg
        else:
            rospy.logerr("Cannot update edge {}. {} instances of node with name {} found".format(edge_id, num_available, node_name))
            return False, "No edge found or multiple edges found"


    def rm_param_from_edge_config_cb(self, req):
        """
        Remove a param from an edge's config.
        """
        return self.rm_param_from_edge_config(req.edge_id, req.namespace, req.name)


    def rm_param_from_edge_config(self, edge_id, namespace, name, update=True, write_map=True):

        node_name, _ = get_node_names_from_edge_id_2(self.tmap2, edge_id)
        num_available, index = self.get_instances_of_node(node_name)

        if num_available == 1:
            the_node = copy.deepcopy(self.tmap2["nodes"][index])
            msg = ""
            for edge in the_node["node"]["edges"]:
                if edge["edge_id"] == edge_id:

                    config = copy.deepcopy(edge["config"])
                    config = [i for i in config if not (i["namespace"] == namespace and i["name"] == name)]

                    edge["config"] = config
                    msg = "edge action is {} and edge config is {}".format(edge["action"], edge["config"])

            self.tmap2["nodes"][index] = the_node

            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)

            return True, msg
        else:
            rospy.logerr("Cannot update edge {}. {} instances of node with name {} found".format(edge_id, num_available, node_name))
            return False, "No edge found or multiple edges found"


    def rm_param_from_topological_map_cb(self, req):
        """
        Remove all instances of a param from the topological map.
        """
        return self.rm_param_from_topological_map(req.namespace, req.name)


    def rm_param_from_topological_map(self, namespace, name, update=True, write_map=True):

        success = False
        for node in self.tmap2["nodes"]:
            for edge in node["node"]["edges"]:
                config0 = copy.deepcopy(edge["config"])
                config0 = [i for i in config0 if (i["namespace"] == namespace and i["name"] == name)]
                if config0:
                    success = True

                config = copy.deepcopy(edge["config"])
                config = [i for i in config if not (i["namespace"] == namespace and i["name"] == name)]
                edge["config"] = config

        if success:
            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)
            return True, ""
        else:
            return False, "parameter not found in topological map"


    def update_node_restrictions_cb(self, req):
        """
        Update a node's restrictions
        """
        return self.update_node_restrictions(req.name, req.restrictions_planning, req.restrictions_runtime, req.update_edges)


    def update_node_restrictions(self, node_name, restrictions_planning, restrictions_runtime, update_edges, update=True, write_map=True):

        num_available, index = self.get_instances_of_node(node_name)

        if num_available == 1:
            if restrictions_planning:
                self.tmap2["nodes"][index]["node"]["restrictions_planning"] = restrictions_planning
            if restrictions_runtime:
                self.tmap2["nodes"][index]["node"]["restrictions_runtime"] = restrictions_runtime

            edge_ids = []
            for node in self.tmap2["nodes"]:
                for edge in node["node"]["edges"]:
                    if node["node"]["name"] == node_name or edge["node"] == node_name:
                        edge_ids.append(edge["edge_id"])

            if restrictions_planning and update_edges:
                for edge_id in set(edge_ids):
                    self.update_edge_restrictions(edge_id, restrictions_planning, "", False)

            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)
            return True, ""
        else:
            rospy.logerr("Error updating the restrictions of node {}. {} instances of node with name {} found".format(node_name, num_available, node_name))
            return False, ""


    def update_edge_restrictions_cb(self, req):
        """
        Update an edge's restrictions
        """
        return self.update_edge_restrictions(req.name, req.restrictions_planning, req.restrictions_runtime)


    def update_edge_restrictions(self, edge_id, restrictions_planning, restrictions_runtime, update=True, write_map=True):

        node_name, _ = get_node_names_from_edge_id_2(self.tmap2, edge_id)
        num_available, index = self.get_instances_of_node(node_name)

        if num_available == 1:
            the_node = copy.deepcopy(self.tmap2["nodes"][index])
            for edge in the_node["node"]["edges"]:
                if edge["edge_id"] == edge_id:
                    if restrictions_planning:
                        edge["restrictions_planning"] = restrictions_planning
                    if restrictions_runtime:
                        edge["restrictions_runtime"] = restrictions_runtime

            self.tmap2["nodes"][index] = the_node

            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)
            return True, ""
        else:
            rospy.logerr("Error updating the restrictions of edge {}. {} instances of node with name {} found".format(edge_id, num_available, node_name))
            return False, ""


    def update_edge_cb(self, req):
        """
        Updates an edge's args (action, action type, goal etc)
        """
        return self.update_edge(req.edge_id, req.action_name, req.action_type, req.goal, req.fail_policy, req.not_fluid)


    def update_edge(self, edge_id, action_name, action_type, goal, fail_policy, not_fluid, update=True, write_map=True):

        node_name, _ = get_node_names_from_edge_id_2(self.tmap2, edge_id)
        num_available, index = self.get_instances_of_node(node_name)

        if num_available == 1:
            the_node = copy.deepcopy(self.tmap2["nodes"][index])
            for edge in the_node["node"]["edges"]:
                if edge["edge_id"] == edge_id:
                    if action_name:
                        edge["action"] = action_name
                    if action_type:
                        edge["action_type"] = action_type
                    if goal:
                        edge["goal"] = json.loads(goal)
                    elif action_type and not goal:
                        _action_type, _goal = self.set_goal(action_name, action_type)
                        edge["action_type"] = _action_type
                        edge["goal"] = _goal
                    if fail_policy:
                        edge["fail_policy"] = fail_policy
                    if not_fluid:
                        edge["fluid_navigation"] = False
                    else:
                        edge["fluid_navigation"] = True

            self.tmap2["nodes"][index] = the_node

            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)
            return True
        else:
            rospy.logerr("Cannot update edge {}. {} instances of node with name {} found".format(edge_id, num_available, node_name))
            return False


    def update_action_cb(self, req):
        """
        Updates the action type and goal for all action_name edges
        """
        return self.update_action(req.action_name, req.action_type, req.goal)


    def update_action(self, action_name, action_type, goal, update=True, write_map=True):

        success = False
        for node in self.tmap2["nodes"]:
            for edge in node["node"]["edges"]:
                if edge["action"] == action_name:
                    if action_type:
                        edge["action_type"] = action_type
                    if goal:
                        edge["goal"] = json.loads(goal)
                    elif action_type and not goal:
                        _action_type, _goal = self.set_goal(action_name, action_type)
                        edge["action_type"] = _action_type
                        edge["goal"] = _goal
                    success = True

        if success:
            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)

        return success


    def add_datum_cb(self, req):
        """
        Adds GNSS latitude/longitude to the topological map's top-level meta info
        """
        return self.add_datum(req.latitude, req.longitude)


    def add_datum(self, latitude, longitude, update=True, write_map=True):

        try:
            self.tmap2["meta"]["datum_latitude"] = latitude
            self.tmap2["meta"]["datum_longitude"] = longitude
            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)
            return True

        except Exception as e:
            rospy.logerr(e)
            return False


    def update_fail_policy_cb(self, req):
        """
        Update he fail policy of all edges in the map
        """
        return self.update_fail_policy(req.fail_policy)


    def update_fail_policy(self, fail_policy, update=True, write_map=True):

        if not fail_policy:
            return False

        try:
            for node in self.tmap2["nodes"]:
                for edge in node["node"]["edges"]:
                    edge["fail_policy"] = fail_policy

            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)
            return True

        except Exception as e:
            rospy.logerr(e)
            return False


    def set_influence_zone_cb(self, req):
        """
        Set the influence zone (vertices) of a node
        """
        return self.set_influence_zone(req.name, req.vertices_x, req.vertices_y)


    def set_influence_zone(self, node_name, vertices_x, vertices_y, update=True, write_map=True):

        num_available, index = self.get_instances_of_node(node_name)

        if num_available == 1:

            if len(vertices_x) < 3 or len(vertices_y) < 3 or len(vertices_x) != len(vertices_y):
                rospy.logerr("Invalid node vertices")
                return False
            else:
                verts = [{"x":x, "y":y} for x, y in zip(vertices_x, vertices_y)]

            self.tmap2["nodes"][index]["node"]["verts"] = verts
            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)
            return True
        else:
            rospy.logerr("Error updating the influence zone of node {}. {} instances of node with name {} found".format(node_name, num_available, node_name))
            return False


    def clear_nodes_cb(self, req):
        """
        Remove all nodes from the topological map
        """
        self.clear_nodes()

        ans = EmptyResponse()
        return ans


    def clear_nodes(self, update=True, write_map=True):

        self.tmap2["nodes"] = []
        self.last_nodes = []

        if update:
            self.update()
        if self.auto_write and write_map:
            self.write_topological_map(self.filename)


    def add_topological_nodes_cb(self, req):
        """
        Add a list of nodes to the topological map
        """
        return self.add_topological_nodes(req.data)


    def add_topological_nodes(self, data, update=True, write_map=True):

        for item in data:
            success = self.add_topological_node(item.name, item.pose, False, False, "", "", "", False, update=False, write_map=False)
            if not success:
                return False

        if update:
            self.update()
        if self.auto_write and write_map:
            self.write_topological_map(self.filename)
        return True


    def update_node_waypoints_cb(self, req):
        """
        Move a list of nodes in the topological map
        """
        return self.update_node_waypoints(req.data)


    def update_node_waypoints(self, data, update=True, write_map=True):

        for item in data:
            success = self.update_node_waypoint(item.name, item.pose, update=False, write_map=False)
            if not success:
                return False

        if update:
            self.update()
        if self.auto_write and write_map:
            self.write_topological_map(self.filename)
        return True


    def add_edges_cb(self, req):
        """
        Add a list of edges to the topological map
        """
        return self.add_edges(req.data)


    def add_edges(self, data, update=True, write_map=True):

        for item in data:
            success = self.add_edge(item.origin, item.destination, item.action, item.action_type, item.edge_id, update=False, write_map=False)
            if not success:
                return False

        if update:
            self.update()
        if self.auto_write and write_map:
            self.write_topological_map(self.filename)
        return True


    def add_params_to_edges_cb(self, req):
        """
        Add parameters to a list of edges
        """
        return self.add_params_to_edges(req.data)


    def add_params_to_edges(self, data, update=True, write_map=True):

        for item in data:
            success,_ = self.add_param_to_edge_config(item.edge_id, item.namespace, item.name, item.value, item.value_is_string, item.not_reset, update=False, write_map=False)
            if not success:
                return False

        if update:
            self.update()
        if self.auto_write and write_map:
            self.write_topological_map(self.filename)
        return True


    def set_influence_zones_cb(self, req):
        """
        Set the influence zone of a list of edges
        """
        return self.set_influence_zones(req.data)


    def set_influence_zones(self, data, update=True, write_map=True):

        for item in data:
            success = self.set_influence_zone(item.name, item.vertices_x, item.vertices_y, update=False, write_map=False)
            if not success:
                return False

        if update:
            self.update()
        if self.auto_write and write_map:
            self.write_topological_map(self.filename)
        return True


    def set_version_cb(self, req):
        """
        Set topological map version in meta info
        """
        try:
            if "meta" in self.tmap2:
                self.tmap2["meta"]["version"] = req.version
            else:
                self.tmap2["meta"] = {}
                self.tmap2["meta"]["version"] = req.version

        except Exception as err:
            rospy.logerr("Could not update topological map version: {}".format(err))
            return False

        self.update()
        if self.auto_write:
            self.write_topological_map(self.filename)
        return True


    def insert_node_cb(self, req):
        """
        Inserts a node between the origin and destination of req.edge_id
        """
        return self.insert_node(req.edge_id, req.name, req.x, req.y)


    def insert_node(self, edge_id, name, x, y, update=True, write_map=True):

        try:
            origin_name, destination_name = get_node_names_from_edge_id_2(self.tmap2, edge_id)

            edge = get_edge_from_id_tmap2(self.tmap2, origin_name, edge_id)
            action_name = edge["action"]
            action_type = edge["action_type"]

            self.remove_edge(edge_id, False, False)
            self.remove_edge(destination_name + "_" + origin_name, False, False)

            origin = get_node_from_tmap2(self.tmap2, origin_name)
            destination = get_node_from_tmap2(self.tmap2, destination_name)

            pose = copy.deepcopy(origin["node"]["pose"])

            x1 = origin["node"]["pose"]["position"]["x"]
            y1 = origin["node"]["pose"]["position"]["y"]

            x2 = destination["node"]["pose"]["position"]["x"]
            y2 = destination["node"]["pose"]["position"]["y"]

            pose["position"]["x"] = float(mean([x1, x2])) if x == 0.0 else x
            pose["position"]["y"] = float(mean([y1, y2])) if y == 0.0 else y

            angle = tf.transformations.quaternion_from_euler(0.0, 0.0, arctan2((y2-y1), (x2-x1)))

            pose["orientation"]["x"] = float(angle[0])
            pose["orientation"]["y"] = float(angle[1])
            pose["orientation"]["z"] = float(angle[2])
            pose["orientation"]["w"] = float(angle[3])

            pose_msg = message_converter.convert_dictionary_to_ros_message("geometry_msgs/Pose", pose)

            node_name = name if name else self.get_new_name()
            self.add_topological_node(node_name, pose_msg, False, False, "", "", "", False, False, False)

            self.add_edge(node_name, origin_name, action_name, action_type, node_name+"_"+origin_name, False, False)
            self.add_edge(origin_name, node_name, action_name, action_type, origin_name+"_"+node_name, False, False)
            self.add_edge(node_name, destination_name, action_name, action_type, node_name+"_"+destination_name, False, False)
            self.add_edge(destination_name, node_name, action_name, action_type, destination_name+"_"+node_name, False, False)

            success = True; message = node_name
            if update:
                self.update()
            if self.auto_write and write_map:
                self.write_topological_map(self.filename)

        except Exception as err:
            success = False; message = str(err)
        return success, message


    def get_instances_of_node(self, node_name):

        num_available = 0
        index = None
        for i, node in enumerate(self.tmap2["nodes"]):
            if node["node"]["name"] == node_name:
                num_available+=1
                index = i

        return num_available, index


    def map_check(self):

        self.map_ok = True

        # check that all nodes have the same pointset
        pointsets = [node["meta"]["pointset"] for node in self.tmap2["nodes"]]
        if len(set(pointsets)) > 1:
            rospy.logwarn("Multiple poinsets found in meta info: {}".format(set(pointsets)))
            self.map_ok = False

        # check for duplicate node names
        names = self.create_list_of_nodes()
        for name in set(names):
            n = names.count(name)
            if n > 1:
                rospy.logwarn("{} instances of node with name '{}' found".format(n, name))
                self.map_ok = False

        sep = "_" + str(uuid.uuid4()) + "_"
        edge_ids = [node["node"]["name"] + sep + edge["node"] for node in self.tmap2["nodes"] for edge in node["node"]["edges"]]
        edge_ids.sort()

        # check for duplicate edges
        for edge in set(edge_ids):
            edge_nodes = re.match("(.*)" + sep + "(.*)", edge).groups()
            origin = edge_nodes[0]
            destination = edge_nodes[1]

            n = edge_ids.count(edge)
            if n > 1:
                rospy.logwarn("{} instances of edge with origin '{}' and destination '{}' found".format(n, origin, destination))
                self.map_ok = False

        # check that an edge's destination node exists
        for edge in set(edge_ids):
            edge_nodes = re.match("(.*)" + sep + "(.*)", edge).groups()
            origin = edge_nodes[0]
            destination = edge_nodes[1]

            if destination not in names:
                rospy.logwarn("Edge with origin '{}' has a destination '{}' that does not exist".format(origin, destination))
                self.map_ok = False

        # check if a node has an edge to itself
        for node in self.tmap2["nodes"]:
            for edge in node["node"]["edges"]:
                if node["node"]["name"] == edge["node"]:
                    rospy.logwarn("Edge with id '{}' has a destination '{}' equal to its origin".format(edge["edge_id"], edge["node"]))
                    self.map_ok = False


    def tmap2_to_tmap(self):
        self.points = map_manager_2.convert_tmap2_to_tmap(self.tmap2, self.pointset, self.metric_map)


    @classmethod
    def convert_tmap2_to_tmap(cls, tmap2, pointset, metric_map):
        points = topological_navigation_msgs.msg.TopologicalMap()

        try:
            point_set = pointset
            points.name = point_set
            points.pointset = point_set
            points.map = metric_map

            for node in tmap2["nodes"]:
                msg = topological_navigation_msgs.msg.TopologicalNode()
                msg.name = node["node"]["name"]
                msg.map = metric_map
                msg.pointset = point_set

                msg.pose = message_converter.convert_dictionary_to_ros_message(
                    "geometry_msgs/Pose", node["node"]["pose"])

                msg.yaw_goal_tolerance = node["node"]["properties"]["yaw_goal_tolerance"]
                msg.xy_goal_tolerance = node["node"]["properties"]["xy_goal_tolerance"]

                msgs_verts = []
                for v in node["node"]["verts"]:
                    msg_v = topological_navigation_msgs.msg.Vertex()
                    msg_v.x = v["x"]
                    msg_v.y = v["y"]
                    msgs_verts.append(msg_v)
                msg.verts = msgs_verts

                msgs_edges = []
                for e in node["node"]["edges"]:
                    msg_e = topological_navigation_msgs.msg.Edge()
                    msg_e.edge_id = e["edge_id"]
                    msg_e.node = e["node"]
                    msg_e.action = e["action"]
                    msg_e.top_vel = 0.55
                    msg_e.map_2d = metric_map
                    msg_e.inflation_radius = 0.0
                    msg_e.recovery_behaviours_config = e["recovery_behaviours_config"]
                    msgs_edges.append(msg_e)
                msg.edges = msgs_edges

                msg.localise_by_topic = node["node"]["localise_by_topic"]
                points.nodes.append(msg)

        except Exception as e:
            rospy.logerr(
                "Cannot convert map to the legacy format. The conversion requires all fields of the legacy map to be set.")
            points = topological_navigation_msgs.msg.TopologicalMap()

        return points
#########################################################################################################