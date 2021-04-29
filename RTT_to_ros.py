#import RTT
import math
from rostron_ia_ms.utils.world import World

# Create an array of obstacles
obstacles = []

#Create pose from node (every 10cm)
def node_to_pose(x, y):
    poseX= -4.9+0.1*x+0.05
    poseY= -3.4+0.1*y+0.05
    return (round(poseX, 1),round(poseY, 1))

# if there is obstacle append in obstacles
def CreateObstacleRobots(node):
    radius=0.085
    distRobot= 6+radius
    for i in range(98) :
        for j in range(68) : 
            pose = node_to_pose(node)
            for r in range(6):
                x = World().allies[r].pose.position.x
                y = World().allies[r].pose.position.y
                xO = World().allies[r].pose.position.x
                yO = World().allies[r].pose.position.y
                if distance(x,y,pose[0],pose[1])<distRobot or distance(xO,yO,pose[0],pose[1])<distRobot : 
                    if i!=0:
                        obstacles.append((i,j))
                    elif distance(x,y,pose[0],pose[1])>distRobot: #if robot not yellow and id=0
                        obstacles.append((i,j))

def CreateObstacleMap():
    # Vertical Map
    for i in range(68) :
        obstacles.append((-1,i))
        obstacles.append((99,i))
    # Horizontal Map
    for j in range(98) :
        obstacles.append((j,-1))
        obstacles.append((j,69))

def distance (x, y, xa, ya):
  return math.sqrt(((xa-x)**2)+((ya-y)**2))

def main(args=None):
    
    startpos = (48, 32)
    endpos = (5, 5)
    n_iter = 200
    radius = 0.5
    stepSize = 0.7

    G = RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize)
    if G.success:
        path = dijkstra(G)

        #Transform path with squares in path with poses
        for i in range(len(path))
            path[i] = node_to_pose(path[i])

        # move to every poses of the path until end pos
        moveTo(path)
