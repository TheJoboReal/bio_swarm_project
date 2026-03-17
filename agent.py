import numpy as np
import cv2

# Hvad skal de have
# x y position
# id
#

class Boids:
    def __init__(self, id, xPos, yPos):
        self.id = id # nok mere for bug fixing
        self.xPos = xPos
        self.yPos = yPos
        self.neighbors = []

    def get_position(self):
        return np.array([self.xPos,self.yPos])
    
    def set_position(desX,desY):
        self.xPos = desX
        self.yPos = desY

    def find_neighbors(self, snippet):
        


def main():
    img = np.ones((500, 500, 3), dtype=np.uint8) * 255 # for at få en hvid baggrund
    # (image, position, radius, color(BGR), thickness -1 = filled)
    #cv2.circle(img, (250, 250), 5, (255, 50, 50), -1)  
    nbrOfAgents = 10
    agentRadius = 5
    blueColor = (255, 50, 50)

    x=np.random.randint(200,300, size=(nbrOfAgents)) # todo: mangler at tjekke om de spawner oven i hinanden
    y=np.random.randint(200,300, size=(nbrOfAgents))
    
    # Velocities
    vx = np.random.uniform(low=-MAX_SPEED, high=MAX_SPEED, size=(NUMBER_OF_ROBOTS,))
    vy = np.random.uniform(low=-MAX_SPEED, high=MAX_SPEED, size=(NUMBER_OF_ROBOTS,))


    flock = []
    for i in range(nbrOfAgents):
        flock.append(Bird(i, x[i], y[i]))

    for i in range(nbrOfAgents):
        xBird,yBird = flock[i].get_position()
        cv2.circle(img, (xBird, yBird), agentRadius, blueColor, -1)  # nu har vi tegnet direkte på vores frame, så kan vi tjekke farve i de omkring liggende pixels

    cv2.imshow("Window", img)
    cv2.waitKey(1)


    cv2.destroyAllWindows()




main()


