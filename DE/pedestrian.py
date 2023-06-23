#!/usr/bin/env python3

class Pedestrian:
    def __init__(self,ID):
        self.ID=ID
        self.trajectory=[]
        self.goal=()
        
        
    def update_goal(self,current_time):
        curr_index=int(current_time*100)-1
        self.goal=()

        if type(self.trajectory[curr_index][0]) == float:
            for i in range(len(self.trajectory[curr_index:])):
                if type(self.trajectory[curr_index+i][0]) == str:
                    self.goal=self.trajectory[curr_index+i-1]
                    return
                else:
                    continue
        else:
            return
        