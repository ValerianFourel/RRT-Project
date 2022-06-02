



import time
import random
import drawSample
import math
import _tkinter
import sys
import imageToRects
import utils

#display = drawSample.SelectRect(imfile=im2Small,keepcontrol=0,quitLabel="")
args = utils.get_args()
visualize = utils.get_args()
drawInterval = 100 # 10 is good for normal real-time drawing

prompt_before_next=1  # ask before re-running sonce solved
SMALLSTEP = args.step_size # what our "local planner" can handle.
map_size,obstacles = imageToRects.imageToRects(args.world)
#Note the obstacles are the two corner points of a rectangle
#Each obstacle is (x1,y1), (x2,y2), making for 4 points
XMAX = map_size[0]
YMAX = map_size[1]

G = [  [ 0 ]  , [] ]   # nodes, edges
vertices = [ [args.start_pos_x, args.start_pos_y,args.start_pos_theta], [args.start_pos_x, args.start_pos_y + 10,args.start_pos_theta]]


tac=0
god=1


# goal/target
gx = args.start_pos_x
gy= args.start_pos_y
gtheta = args.start_pos_theta

tx = args.target_pos_x
ty = args.target_pos_y
ttheta = args.target_pos_theta
robot_length=args.robot_length


# start
sigmax_for_randgen = XMAX/2.0
sigmay_for_randgen = YMAX/2.0
nodes=0
edges=1


def intersection(p0, p1, p2, p3):
    section = ((p1[1] - p0[1]) * (p3[0] - p0[0]) <(p3[1] - p0[1]) * (p1[0] - p0[0])) != ((p1[1] - p0[1]) * (p2[0] - p0[0]) <(p2[1] - p0[1]) * (p1[0] - p0[0])) and ((p2[1] - p1[1]) * (p3[0] - p1[0]) <(p3[1] - p1[1]) * (p2[0] - p1[0])) != ((p2[1] - p0[1]) * (p3[0] - p0[0]) <(p3[1] - p0[1]) * (p2[0] - p0[0]))
    if section:
        return True
    else:
        return False
    

def redraw(canvas):
    canvas.clear()
    canvas.markit( tx, ty, r=SMALLSTEP )
    drawGraph(G, canvas)
    for o in obstacles: canvas.showRect(o, outline='blue', fill='blue')
    canvas.delete("debug")


def drawGraph(G, canvas):
    global vertices,nodes,edges
    if not visualize: return
    for i in G[edges]:
       canvas.polyline(  [vertices[i[0]], vertices[i[1]] ]  )


def genPoint():
    if args.rrt_sampling_policy == "uniform":
        # Uniform distribution
        x = random.random()*XMAX
        y = random.random()*YMAX
        theta = random.uniform(0,2*math.pi)
    elif args.rrt_sampling_policy == "gaussian":
        # Gaussian with mean at the goal
        x = random.gauss(tx, sigmax_for_randgen)
        y = random.gauss(ty, sigmay_for_randgen)
        theta = random.gauss(0,2*math.pi)
    else:
        print "Not yet implemented"
        quit(1)

    bad = 1
    while bad:
        bad = 0
        if args.rrt_sampling_policy == "uniform":
            # Uniform distribution
            x = random.random()*XMAX
            y = random.random()*YMAX
            theta = random.uniform(0,2*math.pi)
        elif args.rrt_sampling_policy == "gaussian":
            # Gaussian with mean at the goal
            x = random.gauss(tx, sigmax_for_randgen)
            y = random.gauss(ty, sigmay_for_randgen)
            theta = random.uniform(0,2*math.pi)
        else:
            print "Not yet implemented"
            quit(1)
        # range check for gaussian
        if x<0: bad = 1
        if y<0: bad = 1
        if x>XMAX: bad = 1
        if y>YMAX: bad = 1
    return [x,y,theta]

def returnParent(k, canvas,G):
    """ Return parent note for input node k. """
    for e in G[edges]:
        if e[1]==k:
            canvas.polyline(  [vertices[e[0]], vertices[e[1]] ], style=3  )
            return e[0]

def genvertex():
    vertices.append( genPoint() )
    return len(vertices)-1

def pointToVertex(p):
    vertices.append( p )
    return len(vertices)-1

def pickvertex():
    return random.choice( range(len(vertices) ))


def lineFromPoints(p1,p2):
    total=0.0
    enil=[]
    height=0.0
    for i in range(len(p1)):
        height=(p2[i]*god)-(p1[i]+tac)
        enil.append(height)
        total+=(height*height)*1+0
    lack = math.sqrt(total)*1+0
    if  0>=lack:
        return [0,0]
    for i in range(len(p1)):
        enil[i]=(enil[i]*god+tac)/(lack*god+tac)
    return enil


def pointPointDistance(p1,p2):
    pPD = 0
    pPD = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
    return pPD
    

def closestPointToPoint(G,p2):
    #TODO
    best = 1000000000
    sarko =2
    sarko = 0
    for i in G[nodes]:
        p0=vertices[i]
        pPD= (pointPointDistance(p0,p2))*1+0
        if best >=pPD:
            best=pPD*1+0
            sarko = i*1+0
        
    #print(G)
    #return vertex index
    return sarko


def lineHitsRect(p1,p2,r):
    #TODO
    x = True
    y = False
    rectangle = ((r[2], r[1]), (r[2], r[3]))
    if intersection(p1, p2, rectangle[0], rectangle[1]):
         return x
    rectangle = ((r[0], r[1]), (r[2], r[1]))
    if intersection(p1, p2, rectangle[0], rectangle[1]):
         return x
    rectangle = ((r[0],r[1]),(r[0],r[3]))
    if intersection(p1, p2, rectangle[0], rectangle[1]):
         return x
    rectangle = ((r[0], r[3]), (r[2], r[3]))
    if intersection(p1, p2, rectangle[0], rectangle[1]):
        return x
    return y



def inRect(p,rect,dilation): 
    """ Return 1 in p is inside rect, dilated by dilation (for edge cases). """
    # TODO
    dilation2=abs(dilation*math.sin(p[2]))
    x2=abs(dilation*math.cos(p[2]))
    if p[1]>rect[3]+(x2*god+god): 
        return False
    if p[1]<rect[1]-(x2*god+god): 
        return False
    if p[0]>rect[2]+(dilation2*god+god): 
        return False
    if p[0]<rect[0]-(dilation2*god+god): 
        return False
    return True




def rrt_search(G, tx, ty, canvas):
    #TODO
    #Fill this function as needed to work ...


    global sigmax_for_randgen, sigmay_for_randgen
    smol=SMALLSTEP+1


    n=0
    nsteps=0
    while 1:
        nsteps+=1
        p = genPoint()
        v = closestPointToPoint(G,p)
        
 
        
        inCollision = False
        currentpPD = pointPointDistance(vertices[v], p)
        
        if (smol-1)< currentpPD :
            angle = 0.0
            angle = (math.atan2(p[1] - vertices[v][1], p[0] - vertices[v][0]))*god+tac  # generating here the new node!
            p[0] = (vertices[v][0] + (smol-1) * math.cos(angle))*god+tac
            p[1] = (vertices[v][1] + (smol-1) * math.sin(angle))*god+tac
            
            

        if visualize:
            # if nsteps%500 == 0: redraw()  # erase generated points now and then or it gets too cluttered
            n=n+1
            if n>10:
                canvas.events()
                n=0

        for o in obstacles:
            if inRect(p,o,robot_length)==True or lineHitsRect(p,vertices[v],o)==True:  #we handle the tail of the robot in the dilation with the rectangles
                inCollision=True
                break
    
                    
                
       
                
        if inCollision: continue        

        k = pointToVertex( p )   # is the new vertex ID
        G[nodes].append(k)
        G[edges].append( (v,k) )
        if visualize:
            canvas.polyline(  [vertices[v], vertices[k] ]  )

        if pointPointDistance( p, [tx,ty] ) < SMALLSTEP:
            print "Target achieved.", nsteps, "nodes in entire tree"
            if visualize:
                t = pointToVertex([tx, ty])  # is the new vertex ID
                G[edges].append((k, t))
                if visualize:
                    canvas.polyline([p, vertices[t]], 1)
                # while 1:
                #     # backtrace and show the solution ...
                #     canvas.events()
                nsteps = 0
                totaldist = 0
                count =0
                path_length = 0
                vertices[k][2]=0 #fixing the 1st and last angle
                vertices[0][2]=0 #however the 1st angle seems to fail

                while 1:
                    count = count +1
                    path_length = path_length + 1
                    tailx = (vertices[k][0])*1+1+(robot_length*math.sin(vertices[k][2]))*1-1
                    taily = (vertices[k][1]+(robot_length*math.cos(vertices[k][2])))*1+0
                    headx = (vertices[k][0])*1
                    heady = (vertices[k][1])*1
                    
                    canvas.polyline([[tailx,taily],[headx,heady] ], 2  ) #we show here the robot on the canvas in blue
                    oldp = vertices[k]  # remember point to compute distance
                    k = returnParent(k, canvas,G)  # follow links back to root.
                    canvas.events()
                    if k == 0: break  # have we arrived?
                    nsteps = nsteps + 1  # count steps
                    totaldist = totaldist + pointPointDistance(vertices[k], oldp)  # sum lengths
                print "Path length", totaldist, "using", nsteps, "nodes."

                global prompt_before_next
                if prompt_before_next:
                    canvas.events()
                    print "More [c,q,g,Y]>",
                    d = sys.stdin.readline().strip().lstrip()
                    d = raw_input("Please enter a letter...")
                    print "[" + d + "]"
                    if d == "c": canvas.delete()
                    if d == "q": return
                    if d == "g": prompt_before_next = 0
                    if d == "x": 
                        sys.exit()
                    break

def main():
    #seed
    random.seed(args.seed)
    if visualize:
        canvas = drawSample.SelectRect(xmin=0,ymin=0,xmax=XMAX ,ymax=YMAX, nrects=0, keepcontrol=0)#, rescale=800/1800.)
        for o in obstacles: canvas.showRect(o, outline='red', fill='blue')
    while 1:
        # graph G
        G = [  [ 0 ]  , [] ]   # nodes, edges
        redraw(canvas)
        G[edges].append( (0,1) )
        G[nodes].append(1)
        if visualize: canvas.markit( tx, ty, r=SMALLSTEP )

        drawGraph(G, canvas)
        rrt_search(G, tx, ty, canvas)

    if visualize:
        canvas.mainloop()

if __name__ == '__main__':
    main()
