import numpy as np

class AlignAndMove():
    def __init__(self, pts, start_ang):
        self.user_pts = list(pts)
        self.user_unreached_pts = list(pts)
        self.final = np.array(pts[-1])
    
    def move_point_to_point(self, start, end, start_ang):
        # Rotate to align with the target
        start_to_end_vec = np.array(end) - np.array(start)
        target_ang = np.arctan2(start_to_end_vec[1], start_to_end_vec[0])
        ang_diff = target_ang - start_ang
        # Rotate in place by publishing appropriate wheel speeds until aligned
        
        # Move forward to the target
        
        # Remove current position from unreached list
        self.user_unreached_pts.pop(0)
        
    def move_along_path(self, pts, start_info):
        # For each point in the path:
        #   move_point_to_point(current_pos, next_pt, current_ang)
        #   update current_pos and current_ang
        #   remove next_pt from unreached list
        # When final point is reached, set reached_goal flag to True
        