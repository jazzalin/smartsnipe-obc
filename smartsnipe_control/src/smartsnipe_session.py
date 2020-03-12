# Smartsnipe session classes to keep track of drills and shot statistics
import rospy
class SessionStatistics:
    """
    Record of the session statistics
    """
    def __init__(self):
        self.shots = 0
        self.goals = 0
        self.avg_speed = 0.0
        self.avg_rt = 0.0
        self.fastest_shot = 0.0
        self.fastest_rt = 0.0
    
    def __repr__(self):
        return {"shots": self.shots,
                "goals": self.goals,
                "average_shot_speed": self.avg_speed,
                "average_reaction_time": self.avg_rt,
                "fastest_shot": self.fastest_shot,
                "quickest_reaction_time": self.fastest_rt}

class Session:
    """
    Record of a smartsnipe session
    """
    def __init__(self):
        # TODO: add uuid for database storage
        # self.id = ""
        # Status
        self.in_progress = False
        self.override = False
        self.requested = False
        self.reset = False
        self.mode = "default"
        
        # Params
        self.time_between = 0.0
        self.time_open = 0.0
        self.slots = [0, 0, 0, 0, 0] # default: all closed
        self.duration = 0.0
        self.start = 0.0
        self.end = 0.0

        # Data
        self.stats = None

    def set_status(self, is_live):
        if is_live:
            self.start = rospy.Time.now()
        else:
            self.end = rospy.Time.now()
        self.in_progress = is_live
        #(Re)set the session statistics
        self.stats = SessionStatistics()
    
    def set_params(self, data):
        self.time_between = data["time_between_openings"]
        self.time_open = data["time_slot_is_open"]
        self.slots = data["slots"]
        self.requested = True
    
    def set_override(self, data):
        """
        Coach takes over and slots are independently controlled
        """
        self.slots = [0, 0, 0, 0, 0]
        self.override = True
        self.slots[data["slot_to_open_next"]] = 1
    
    def set_statistics(self, data):
        """
        Update stats at the end of a drill
        """
        #TODO: make appropriate changes if accepting intermediate stats
        stats = SessionStatistics()
        stats.shots = data.shots
        stats.goals = data.goals
        stats.avg_speed = data.average_speed
        stats.avg_rt = data.average_reaction_time
        stats.fastest_shot = data.fastest_shot
        stats.fastest_rt = data.fastest_reaction_time
        self.stats = stats

    def __repr__(self):
        return {"session_in_progress": self.in_progress,
                "time_between_openings": self.time_between,
                "time_slot_is_open": self.time_open,
                "slots": self.slots}