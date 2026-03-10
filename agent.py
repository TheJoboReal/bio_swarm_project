import numpy as np
import cv2

# Hvad skal de have
# x y position
# id
#

class Bird:
    def __init__(self, id, xPos, yPos):
        self.id = id # nok mere for bug fixing
        self.InitXPos = xPos
        self.InityPos = yPos
        self.xPos = xPos
        self.yPos = yPos
        self.neighbors = []

    def get_position(self):
        return np.array([self.xPos,self.yPos])
    
    def set_position(desX,desY):
        self.xPos = desX
        self.yPos = desY

    def find_neighbors(xPosList,yPosList, nbrOfAgents):
        #Todo: tager en en kvadrat af imaget omkring vores agent --> skal nok være en cirkel optimalt?
        # så det bliver ud fra hvad den "ser" og ikke hvor de nadre siger de er.
        
        for i in range(nbrOfAgents):
        



def main():
    img = np.ones((500, 500, 3), dtype=np.uint8) * 255 # for at få en hvid baggrund

    # (image, position, radius, color(BGR), thickness -1 = filled)
    #cv2.circle(img, (250, 250), 5, (255, 50, 50), -1)  
    nbrOfAgents = 10
    agentRadius = 5
    blueColor = (255, 50, 50)

    x=np.random.randint(200,300, size=(nbrOfAgents))
    y=np.random.randint(200,300, size=(nbrOfAgents))
    
    flock = []
    for i in range(nbrOfAgents):
        flock.append(Bird(i, x[i], y[i]))
    

    while(1):
        for i in range(nbrOfAgents):
            xBird,yBird = flock[i].get_position()
            cv2.circle(img, (xBird, yBird), agentRadius, blueColor, -1)  # nu har vi tegnet direkte på vores frame, så kan vi tjekke farve i de omkring liggende pixels

        cv2.imshow("Window", img)
        cv2.waitKey(1)
       
        # tager den del af billedet der er omkring vores agents så det ikke bliver euclidian
        for i in range(nbrOfAgents):
            flock[i].find_neighbors(x,y)


        # Vasker boardet rent så vi ikke har dublicates
        img = np.ones((500, 500, 3), dtype=np.uint8) * 255 # for at få en hvid baggrund
        
    


    cv2.imshow("Window", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()




main()


