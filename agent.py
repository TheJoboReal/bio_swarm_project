import cv2
import numpy as np


class Boids:
    #def __init__(self, id, x, y, vx, vy):
    def __init__(self, id, x, y, vx, vy, height, width):
        self.id = id 
        self.xInit = x
        self.yInit = y
        self.x = x
        self.y = y
        self.x_initial = x
        self.y_initial = y
        self.vx = vx
        self.vy = vy
        self.height = height
        self.width = width


    def get_position(self):
        return np.array([self.x,self.y])
    
    def get_initial_position(self):
        return np.array([self.x_initial, self.y_initial])
   
    def get_id(self):
        return self.id

    def get_velocity(self):
        return np.array([self.vx,self.vy])

    def update_position(self):
        self.x = (self.x + self.vx) % self.width  # Wrap around
        self.y = (self.y + self.vy) % self.height
        
        # self.x += self.vx
        # self.y += self.vy

    def update_velocity(self, vx, vy):
        self.vx = vx
        self.vy = vy


