#!/usr/bin/env python
"""
Created on Tue Apr 13 22:02:24 2021
@author: Adam Binch (abinch@sagarobotics.com)

"""
#########################################################################################################
import rospy, actionlib, json, yaml
import operator, collections, copy

from functools import reduce  # forward compatibility for Python 3
from rospy_message_converter import message_converter
from actionlib_msgs.msg import GoalStatus


def _import(location, name):
    mod = __import__(location, fromlist=[name]) 
    return getattr(mod, name) 


class attributes(object):
    def __init__(self):
        pass


class dict_tools(object):
                
                
    def get_paths_from_nested_dict(self, nested):
        paths = list(self.nested_dict_iter(nested))
        return [{"keys": item[0], "value": item[1]} for item in paths]
    
    
    def nested_dict_iter(self, nested, prefix=""):
        """
        Recursively loops through a nested dictionary. 
        For each inner-most value generates the list of keys needed to access it.
        """
        for key, value in nested.items():
            path = "{},{}".format(prefix, key)
            if isinstance(value, collections.Mapping):
                for inner_key, inner_value in self.nested_dict_iter(value, path):
                    yield inner_key, inner_value
            else:
                yield path[1:].split(","), value
    
    
    def getFromDict(self, dataDict, mapList):
        return reduce(operator.getitem, mapList, dataDict)


    def setInDict(self, dataDict, mapList, value):     
        self.getFromDict(dataDict, mapList[:-1])[mapList[-1]] = value
        return dataDict
#########################################################################################################


#########################################################################################################
class EdgeActionManager(object):
    
    
    def __init__(self, topomap, goals={}):
        
        self.topomap = topomap
        self.goals = goals
        self.client = None        
        self.current_action = "none"
        self.next_action = "none"
        self.dt = dict_tools()

        self.plugins = {}
        for node in self.topomap["nodes"]:
            for edge in node["node"]["edges"]:
                if edge["action"] not in self.plugins:
                    self.import_plugin(edge["action"])


    def import_plugin(self, action):

        if action in self.goals:
            package, name = self.goals[action]["plugin"].split("/")
            func = _import("{}.{}".format(package, name), name)
            self.plugins[action] = func
        else:
            rospy.logwarn("No goal plugin for action {}. The goal from the map will be used".format(action))
            self.plugins[action] = None


    def initialise(self, edge, destination_node, origin_node=None, next_action="none"):
        
        self.edge = yaml.safe_load(json.dumps(edge)) # no unicode in edge
        self.destination_node = destination_node
        self.origin_node = origin_node
        self.next_action = next_action
        
        rospy.loginfo("Edge Action Manager: Processing edge {}".format(self.edge["edge_id"]))
        
        self.action_name = self.edge["action"]
        if self.action_name != self.current_action:
            self.preempt()

        if self.action_name in self.goals:
            action_type = self.goals[self.action_name]["action_type"]
        else:
            action_type = self.edge["action_type"]        

        items = action_type.split("/")
        package = items[0]
        action_spec = items[1][:-4] + "Action"
        
        rospy.loginfo("Edge Action Manager: Importing {} from {}.msg".format(action_spec, package))
        action = _import(package+".msg", action_spec)
        
        rospy.loginfo("Edge Action Manager: Creating a {} client".format(self.action_name.upper()))
        self.client = actionlib.SimpleActionClient(self.action_name, action)        
        self.client.wait_for_server()
        
        if self.action_name in self.plugins and self.plugins[self.action_name] is not None:
            rospy.loginfo("Edge Action Manager: Constructing the goal from the plugin")
            func = self.plugins[self.action_name]
            self.goal = func(action_type, self.get_attributes())
        else:
            rospy.loginfo("Edge Action Manager: Constructing the goal from the map")
            self.construct_goal(action_type, copy.deepcopy(self.edge["goal"]))
        
        
    def preempt(self):
        
        if self.client is not None:
            status = self.client.get_state()
            if status == GoalStatus.PENDING or status == GoalStatus.ACTIVE:
                self.client.cancel_all_goals()


    def get_attributes(self):    
        attrs = attributes()
        for attribute, value in self.__dict__.items():
            setattr(attrs, attribute, value)
        return attrs
        
        
    def construct_goal(self, action_type, goal_args):
        
        paths = self.dt.get_paths_from_nested_dict(goal_args)
        
        for item in paths:
            value = item["value"]
            
            if isinstance(value, str):
                
                if value.startswith("$"):
                    _property = self.dt.getFromDict(self.destination_node, value[1:].split("."))
                    goal_args = self.dt.setInDict(goal_args, item["keys"], _property)
                    
                elif value.startswith("+") and self.origin_node is not None:
                    _property = self.dt.getFromDict(self.origin_node, value[1:].split("."))
                    goal_args = self.dt.setInDict(goal_args, item["keys"], _property)

        if "continue_action" in goal_args:
            goal_args["continue_action"]["data"] = True if self.action_name == self.next_action else False

        self.goal = message_converter.convert_dictionary_to_ros_message(action_type, goal_args)
        
 
    def execute(self):
        
        rospy.loginfo("Edge Action Manager: Executing the action...")
        self.client.send_goal(self.goal)
        self.current_action = self.action_name
#########################################################################################################