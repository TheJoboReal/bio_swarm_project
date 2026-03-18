import cv2
import numpy as np


class Boids:
    #def __init__(self, id, x, y, vx, vy):
    def __init__(self, id, x, y, vx, vy, height, width):
        self.id = id 
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.height = height
        self.width = width


    def get_position(self):
        return np.array([self.x,self.y])
   
    def get_id(self):
        return self.id

    def get_velocity(self):
        return np.array([self.vx,self.vy])

    def update_position(self):
        self.x = (self.x + self.vx) % self.height  # Wrap around
        self.y = (self.y + self.vy) % self.width
        
        # self.x += self.vx
        # self.y += self.vy

    def update_velocity(self, vx, vy):
        self.vx = vx
        self.vy = vy


