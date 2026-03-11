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

    #def find_neighbors(xPosList,yPosList, nbrOfAgents):
        #for i in range(nbrOfAgents):
    def find_neighbors(self, snippet):
        #Todo: tager en en kvadrat af imaget omkring vores agent --> skal nok være en cirkel optimalt?
        # så det bliver ud fra hvad den "ser" og ikke hvor de nadre siger de er.
        cv2.imshow(f"bird {self.id}", snippet)
        #               Øverste
        # (0.0)         0 
        #               1
        #               2
        #  0   1    2   3   4   5   7 Højre
        #               4 
        #               5
        #               6           (sn,sn)
        #              Nederste
        #              

        # ikke tjekke de midterste 5x5 pixels
        # skal ikke tjekke lige rundt om sig selv, så det giver +1 --> 6x6 den skipper
        birdRadius = 6
        about = (len(snippet)//2) # // er integer division
        sn = len(snippet) # snippet length

        above = False
        right = False
        down = False
        left = False
        
        # øverste del
        for xaxis in range(0,sn): # x aksen
            for yaxis in range(birdRadius,about-birdRadius):
                pixel = snippet[yaxis, xaxis]
                if not np.all(pixel == 255):
                    #print(xaxis,yaxis)
                # noget true her
                    above = True

        # højre del
        for xaxis in range(birdRadius+about, sn): # x aksen
            for yaxis in range(0, sn):
                pixel = snippet[yaxis, xaxis]
                if not np.all(pixel == 255):
                    #print(xaxis,yaxis)
                    # noget true her
                    right = True

        # nederste del
        for xaxis in range(0,sn): # x aksen
            for yaxis in range(about+birdRadius, sn):
                pixel = snippet[yaxis, xaxis]
                if not np.all(pixel == 255):
                    #print(xaxis,yaxis)
                    # noget true her
                    down = True

        # venstre del
        for xaxis in range(0, about-birdRadius): # x aksen
            for yaxis in range(0,sn):
                pixel = snippet[yaxis, xaxis]
                if not np.all(pixel == 255):
                    #print(xaxis,yaxis)
                    # noget true her
                    left = True

        print(f"bird {self.id} ", above,right,down,left)
        

        # chattens hurtige svar ... deprimerende simpelt --> skal vi bruge det?
        # birdRadius = 3
        # sn = snippet.shape[0]
        # about = sn // 2
        #
        # above_region = snippet[0:about-birdRadius, :]
        # right_region = snippet[:, about+birdRadius:sn]
        # down_region  = snippet[about+birdRadius:sn, :]
        # left_region  = snippet[:, 0:about-birdRadius]
        #
        # above = not np.all(above_region == 255)
        # right = not np.all(right_region == 255)
        # down  = not np.all(down_region == 255)
        # left  = not np.all(left_region == 255)
        # print(f"bird {self.id}", above, right, down, left)


def main():
    img = np.ones((500, 500, 3), dtype=np.uint8) * 255 # for at få en hvid baggrund
    # (image, position, radius, color(BGR), thickness -1 = filled)
    #cv2.circle(img, (250, 250), 5, (255, 50, 50), -1)  
    nbrOfAgents = 10
    agentRadius = 5
    blueColor = (255, 50, 50)

    x=np.random.randint(200,300, size=(nbrOfAgents)) # todo: mangler at tjekke om de spawner oven i hinanden
    y=np.random.randint(200,300, size=(nbrOfAgents))
    
    flock = []
    for i in range(nbrOfAgents):
        flock.append(Bird(i, x[i], y[i]))
    

    #while(1):
    for i in range(nbrOfAgents):
        xBird,yBird = flock[i].get_position()
        cv2.circle(img, (xBird, yBird), agentRadius, blueColor, -1)  # nu har vi tegnet direkte på vores frame, så kan vi tjekke farve i de omkring liggende pixels

    cv2.imshow("Window", img)
    cv2.waitKey(1)

    # tager den del af billedet der er omkring vores agents så det ikke bliver euclidian
    h = 20 #distance rundt om vores bird
    for i in range(nbrOfAgents):
        xBird,yBird = flock[i].get_position()
        crop_img = img[yBird-h:yBird+h, xBird-h:xBird+h]
        flock[i].find_neighbors(crop_img)

    # Vasker boardet rent så vi ikke har dublicates --> til når vi skal køre i loops
    #img = np.ones((500, 500, 3), dtype=np.uint8) * 255 # for at få en hvid baggrund
    
    


    cv2.imshow("Window", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()




main()


