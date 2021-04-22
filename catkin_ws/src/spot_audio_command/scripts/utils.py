
class TestCase():
    def __init__(self):
        pass
        self.transcript = ["hey spot can you fetch me a drink from the kitchen"]

        

class MapKnowledge():
    # TODO what about the map itself? what kind of format do we have
    robot_position = (0, 1)
    known_places = {"start": (0, 0), "kitchen": (0, 6), "bedroom": (6, 6), "study": (5, 5)}
    default_storage = {"kitchen": ["drink", "coke", "soda", "beer", "water", "cup"],
                       "study":["laptop", "computer", "headset"]}
    item_index = dict()
    for place, all_items in default_storage.items():
        for one_item in all_items:
            item_index[one_item] = place
            
    del place, all_items, one_item
    
    def __init__(self):
        pass


class Subject(MapKnowledge):

    def __init__(self, name):
        self.name = name
        self.action = None


class Item(MapKnowledge):

    def __init__(self):
        self.name = None
        self.amount = None

class Action(MapKnowledge):

    # words with an object and a place
    vt_round = ["fetch", "bring", "get", "give"]
    vt_single = ["send", "take"]
    # words with a navigation goal
    vi = ["go"]
    # possible action at the navigation goal
    action_names = ["stop", "grab", "observe", "fix"]

    def __init__(self):  # -> None
        self.name = None
        self.subject = None
        self.obj = None
        self.obj_num = None
        self.iobj = None
        self.iobj_type = None
        self.obl = None
        self.obl_case = None

    def parse_dep(self, dep_tree):
        # the first loop, find out subject, object, and position
        # they depend only on the verb
        for relation in dep_tree:
            
            # ((u'spot', u'NN'), u'dep', (u'fetch', u'VB')) 
            # who will perform this action, "spot"
            if relation[1] == "dep" and relation[2][1] == "VB":
                if self.subject is not None:
                    print("Overwrite existing subject")
                if str(relation[0][1]) not in ["NN"]:
                    print("Unknown type of subject")
                self.subject = str(relation[1][0])
                if self.name is None:
                    self.name = str(relation[2][0])
                    
            # direct object, like the "drink" in "fetch me a drink"
            # maybe need to fetch "a drink and a cake"
            # ((u'fetch', u'VB'), u'obj', (u'drink', u'NN'))
            if relation[1] == "obj" and relation[0][1] == "VB":
                if self.obj is None:
                    self.obj = [str(relation[2][0])]
                else:
                    self.obj.append(str(relation[2][0]))
                    
            # indirect object, like the "me" in "fetch me a drink"
            # ((u'fetch', u'VB'), u'iobj', (u'me', u'PRP'))
            if relation[1] == "iobj" and relation[0][1] == "VB":
                if self.iobj is None:
                    self.iobj = str(relation[2][0])
                    self.iobj_type = str(relation[2][1])
                    
            # ((u'fetch', u'VB'), u'obl', (u'kitchen', u'NN'))
            if relation[1] == "obl" and relation[0][1] == "VB":
                if self.obl is not None:
                    self.obl = str(relation[2][0])
        
        # by default get 1 of each 
        if isinstance(self.obj, list):
            self.obj_num = [1 for _ in self.obj]
        # the second loop find out the requirements of the objects, like color, amount etc.
        # they depend on the subject, object and oblique nominal
        for relation in dep_tree:
            if relation[1] == "case" and relation[0][0] == self.obl:
                self.obl_case = relation[2][0]

    def gen_command(self):
        """ Take `item` to goal and then do task
        """
        nav_goals, items, tasks = [], [], []
        # fetch me a drink from the kitchen
        if self.name in Action.vt_round: # fetch 
            if self.obl is None:
                self.obl = MapKnowledge.item_index[self.obj[0]]
                self.obl_case = "from"

            items.extend([None, self.obj])
            nav_goals.extend([MapKnowledge.known_places[self.obl], MapKnowledge.known_places["start"]])
            tasks.extend([" ".join(["grab", " ".join(self.obj)]), "stop"])

        # # go to the study
        # if self.name in Action.vt_single:
        #     pass

        return [(goal, item, task) for goal, item, task in zip(nav_goals, items, tasks)]

    def print_info(self):
        cmd_list = self.gen_command()
        print("I will do the following things:")
        for goal, item, task in cmd_list:
            print("Take {} to point {} and do this: {}".format(item, goal, task))


class RobotSpeechClient(object):

    num_robot, max_robot = 0, 6
    working_bot = []
    working_bot_names = []
    available_name = ["Alice", "Bob", "Charlie", "David", "Edward", "Frank"]

    def __init__(self):
        super().__init__()
        self.name = RobotSpeechClient.available_name[RobotSpeechClient.num_working_bot]
        RobotSpeechClient.num_robot += 1
        RobotSpeechClient.working_bot.append(self.name)
        RobotSpeechClient.working_bot.append(self)
        print("Robot {} Initialized".format(self.name))
        

    @classmethod
    def print_status(cls):
        print("Right now, {} Robots are working, they are {}".format(cls.num_robot, cls.get_working_bots()))
        
    @classmethod
    def get_working_bots(cls):
        return [bot.name for bot in cls.working_bot]
            
