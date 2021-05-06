# when using python 2, uncomment this and remove all function annotations
# from __future__ import division, with_statement, print_function
import rospy
from nltk.parse.corenlp import CoreNLPDependencyParser
from typing import List, Tuple, Dict
from spot_audio_command.msg import MessageCMD
# from nltk.stem import WordNetLemmatizer
# lemmatizer = WordNetLemmatizer()
class TestCase():
    def __init__(self):
        self.transcript = [
            "hey Alice can you fetch me a beer and two cakes from the kitchen and send this book or the hammer to the study",
            "hey Frank can you bring me a cup from my bedroom",
            "hey Alice can you go around the campus",
            "hey Charlie can you send this headset to my bedroom"
            ]

class WorldKnowledge():
    # TODO what about the map itself? what kind of format do we have
    # write these into a file 
    known_places = {"start": (0.0, 0.0), "kitchen": (0.0, 6.0), "bedroom": (6.0, 6.0), "study": (5.0, 5.0)}
    default_storage = {"kitchen": ["drink", "coke", "soda", "beer", "water", "cup", "cake"],
                       "study":["laptop", "computer", "headset"]}
    # a pre-defined route 
    route = {"campus":[(2.0, 0.0), (4.0, 0.0), (4.0, 2.0), (4.0, 4.0), (2.0, 2.0), (0.0, 0.0)]}
    item_code = {"drink":0, "coke":1, "soda":2, "beer":3, "water":4,"cup":5, "laptop":6, "computer":7, "headset":8, "cake":9, "book":10}
    
    # where the items are stored by default, dict(item_name: place_name)
    item_index = dict()
    for place, all_items in default_storage.items():
        for one_item in all_items:
            item_index[one_item] = place
            
    del place, all_items, one_item
    
    def __init__(self):
        pass

class Item():

    def __init__(self, name:str=None):
        self.name = name
        self.amount = 1
        self.default_location = WorldKnowledge.item_index.get(name, "start")

# class Place():
    
#     def __init__(self):
#         self.name = None
#         self.x_coord = None
#         self.y_coord = None

class Action():

    # words with an object and a place
    vt_round = ["fetch", "bring", "get", "give"]
    vt_single = ["send", "take"]
    # words with a navigation goal or a fixed routine
    vi = ["go", "walk"]
    __num_dict = {"one":1, "two":2, "three":3, "four":4, "five":5}
    

    def __init__(self, name:str=None):  # -> None
        self.name = name
        self.obj_dict:Dict[str,Item] = dict()
        self.take_all:bool = True # need to take all objects
        self.iobj = None
        self.iobj_type = None
        self.obl = []
        self.obl_case = None # get the items from the location (obl) or send them to the location
        
    @property
    def obj_names(self):
        return [item.name for item in self.obj_dict.values()]
    
    def parse_dep(self, dep_tree):
        # the first loop, find out subject, object, and position
        # they depend only on the verb
        for ((governor, gov_pos), relation, (dependent, dep_pos)) in dep_tree:
            relation:str
            if governor != self.name:
                continue
            # direct object, like the "drink" in "fetch me a beer and two cakes"
            # (('fetch', 'VB'), 'obj', ('beer', 'NN')) (('fetch', 'VB'), 'obj', ('cakes', 'NNS'))
            if relation == "obj":
                # if dep_pos == "NNS":
                #     dependent = lemmatizer.lemmatize(dependent)
                self.obj_dict.update({dependent:Item(name=dependent)})
                    
            # indirect object, like the "me" in "fetch me a drink"
            # (('fetch', 'VB'), 'iobj', ('me', 'PRP'))
            if relation=="iobj":
                self.iobj=Item(name=dependent)
                self.iobj_type=dep_pos
                    
            # (('fetch', 'VB'), 'obl:from', ('kitchen', 'NN')) (('send', 'VB'), 'obl:to', ('library', 'NN'))
            # oblique nominal basically means "to" or "from" from some place
            if relation.startswith("obl"):
                self.obl = dependent
                self.obl_case = relation.split(sep=":")[-1]
        
        # the second loop find out the requirements of the objects, like color, amount etc.
        # they depend on the subject, object and oblique nominal
        for ((governor, gov_pos), relation, (dependent, dep_pos)) in dep_tree:
            # by default get 1 of each unless with a numerical modifier (('cakes', 'NNS'), 'nummod', ('two', 'CD'))
            if relation=="nummod":
                # if gov_pos == "NNS":
                #     governor = lemmatizer.lemmatize(governor)
                if governor in self.obj_names:
                    self.obj_dict[governor].amount = self.__num_dict.get(dependent, 1)
            # (('beer', 'NN'), 'conj:and', ('cakes', 'NNS'))
            if relation =='conj:or' and governor in self.obj_names and dependent in self.obj_names:
                self.take_all = False
                

    def gen_command(self, print_result=True):
        """ generate a series of command according to format:
            Take `item` to goal and then do task(sth, obj) (do something to obj)
        """
        nav_goals, items, tasks = [], [], []
        if not self.take_all and len(self.obj_dict)>=2:
            all_items = [self.obj_dict.get(list(self.obj_dict.keys())[0], None)]
        else:
            all_items = list(self.obj_dict.values())
        # fetch me a drink from the kitchen, 2 stage, go there, do something and go back
        # use the default storage if the user did not specify a place to pick up the items
        if self.name in self.vt_round: # fetch
            if self.obl is None:
                self.obl = WorldKnowledge.item_index.get(self.obj[0], "start")
                self.obl_case = "from"
            item_list = [(item.amount, item.name) for item in all_items]
            items.extend([None, item_list])
            nav_goals.extend([self.obl, "start"])
            tasks.extend([("grab", item_list), ("wait", None)]) # (task, object)
        # one stage actions, go there and do something
        elif self.name in self.vt_single:
            item_list = [(self.__num_dict.get(item.amount, 1), item.name) for item in all_items]
            if len(item_list)==0:
                item_list = [None]
            items.append(item_list)
            nav_goals.append("start" if self.obl==None else self.obl)
            tasks.append(("wait", None)) # (task, object)
        elif self.name in self.vi:
            if self.obl_case=="around":
                nav_goals = WorldKnowledge.route.get(self.obl)
                items = [None for _ in nav_goals]
                tasks = [("nothing", None) for _ in nav_goals]
        else:
            print("Sorry, I didn't understand what you said")
        
        cmd_list = [(goal, item, task) for goal, item, task in zip(nav_goals, items, tasks)]
        
        if print_result:
            for goal, item, (task, obj) in cmd_list:
                task_str = task if obj==None else str(task)+str(obj)
                print("Take {} to point {} and do this: {}".format(item, goal, task_str))
        
        return cmd_list



class RobotSubject(object):

    __num_robot, __max_robot = 0, 5
    __working_bot = dict()
    __available_name = ["Alice", "Charlie", "David", "Frank", "Michael"]
    _publisher = rospy.Publisher("taskChannel", MessageCMD, queue_size=0)
    
    def __init__(self, name):
        super(RobotSubject, self).__init__()
        self.name = name
        self.action_list: List[Action] = []
        self.do_all = True # whether need to do all the tasks (specified by and/or)
        self.available = True
        self.position: Tuple[float] = None # (x, y, orientation)
        self.mission_count = 0
    
    def publish_task_to_ros(self, cmd_list:tuple):
        """ publish the command to ros topic 
        """
        
        task_count = 0
        # take items to place and do task (usually "grab" or "wait") to objs
        for (place, items, (task, objs)) in cmd_list:
            msg = MessageCMD()
            msg.mission_count = self.mission_count
            msg.task_count = task_count
            msg.robot_name = self.name
            msg.cmd_type = task
            msg.goal_x, msg.goal_y = place if isinstance(place, tuple) else WorldKnowledge.known_places.get(place, "start")
            # item should be either None or a list of (amount, name)
            if isinstance(items, list):
                for (amount, item) in items:
                    msg.item_count.append(amount)
                    msg.item_type.append(WorldKnowledge.item_code.get(item, 0))
            # objs has the same structure as item
            if isinstance(objs, list):
                for (amount, obj) in objs:
                    msg.obj_count.append(amount)
                    msg.obj_type.append(WorldKnowledge.item_code.get(obj, 0))
            
            task_count += 1
            self._publisher.publish(msg)
        self.mission_count += 1
            
    def clear_data(self):
        self.action_list: List[Action] = []
        self.do_all = True # whether need to do all the tasks (specified by and/or)
        self.available = True
        self.position: Tuple[float] = None # (x, y, orientation)
        
    @classmethod
    def create_robot(cls, name:str=None):
        if cls.__num_robot >= cls.__max_robot:
            print("All available robots are already working, returning a None value")
        else:
            name = cls.__available_name[cls.__num_robot]
            cls.__num_robot += 1
            new_robot = RobotSubject(name)
            cls.__working_bot.update({name: new_robot})
            print("Robot {} Initialized".format(name))
            cls.fleet_status()

    @classmethod
    def fleet_status(cls):
        print("Right now, {} Robots are working, they are {}".format(cls.__num_robot, cls.working_bot_names()))
        
    @classmethod
    def working_bot_num(cls):
        return cls.__num_robot
    @classmethod
    def get_working_bots(cls):
        return cls.__working_bot
    @classmethod
    def working_bot_names(cls):
        return [name for name, bot in cls.__working_bot.items() if name==bot.name]
    
    @classmethod
    def get_available_name(cls):
        return cls.__available_name
    
    @classmethod
    def assign_task(cls, name, cmd_list):
        if name in cls.working_bot_names():
            cls.__working_bot[name].publish_task_to_ros(cmd_list)

class CoreNLPEnhancedDependencyParser(CoreNLPDependencyParser):
    parser_annotator = "depparse" # possible to add sentiment here "depparse,sentiment"
    def make_tree(self, response, parse_type='enhancedPlusPlusDependencies'):
        
        """
        According to raw parse result, create dependency tuples by parse_type.
        parse_type options: basicDependencies , enhancedDependencies, enhancedPlusPlusDependencies

        :param response:
        :param parse_type:
        :return:
        """

        dep_tuples = []
        for dependency in response[parse_type]:
            governor_token = response["tokens"][dependency["governor"] - 1]
            dependent_token = response["tokens"][dependency["dependent"] - 1]
            if dependency['dep'] == 'ROOT':
                continue
            # governor = (dependency['governorGloss'], response['tokens'][dependency['governor'] - 1]['pos'])
            governor = (governor_token["lemma"], response['tokens'][dependency['governor'] - 1]['pos'])
            relation = dependency['dep']
            dependent = (dependent_token['lemma'], response['tokens'][dependency['dependent'] - 1]['pos'])
            dep_tuples.append((governor, relation, dependent))

        return dep_tuples

  
    
# if __name__ == "__main__":
#     RobotSubject.create_robot()
#     RobotSubject.create_robot()
#     RobotSubject.create_robot()
#     RobotSubject.create_robot()
#     RobotSubject.create_robot()
#     RobotSubject.create_robot()
#     print("")            
