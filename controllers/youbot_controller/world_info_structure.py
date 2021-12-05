class Object_Id:
    """Enum of the object we are looking at"""
    UNIDENTIFIED = 0
    RED_BERRY = 1
    YELLOW_BERRY = 2
    ORANGE_BERRY = 3
    PINK_BERRY = 4
    UNIDENTIFIED_BERRY = 5
    GREEN_ZOMBIE = 6
    BLUE_ZOMBIE = 7
    AQUA_ZOMBIE = 8
    PURPLE_ZOMBIE = 9
    ZOMBIE_UNIDENTIFIED = 10
    TREE = 11
    
class Object_Id:
    """Enum of the berry behaviors"""
    plus_forty_energy = 1
    plus_twenty_health = 2
    minus_twenty_health = 3
    armor = 4
    
    
class Point:
    """A location and all info to get it to work """

    def __init__(self, x, y):
        self.x = x
        self.y = y
    

class World_Object:
    """An world object struct"""
    
    object_id = Object_Id.UNIDENTIFIED
    on_stump = False
    
    def __init__(self, loc):
        self.location = loc
        
        
class Berry_Array:
    """An object holding all the berry locations"""
    yellow_berry_arr = []
    red_berry_arr = []
    orange_berry_arr = []
    pink_berry_arr = []
    
    def __init__(self):
        
        self.yellow_effects = []
        self.blue_effects = []
        self.orange_effects = []
        self.pink_effects = []
        
        
    def berry_eaten(self, berry_type, ):
        
    