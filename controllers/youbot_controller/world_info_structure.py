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
    STUMP = 11
    TREE = 12
    
    
class Point:
    """A location and all info to get it to work """

    def __init__(self, x, y):
        self.x = x
        self.y = y
    

class World_Object:
    """An world object struct"""
    
    object_id = Object_Id.UNIDENTIFIED
    
    def __init__(self, loc):
        self.location = loc
    