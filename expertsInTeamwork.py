import random as rnd
from tkinter import *
import time
from math import sin, cos, sqrt, exp
import numpy as np
from PIL import ImageTk, Image


# Environmental Specifciation
num = 0  # initial number of agents
s = 10  # environment size

# Agent parameters
k = 1.5
m = 2.0
t0 = 3
dt = 0.05
rad = 0.25  # Collision radius
sight = 10  # Neighbor search range
maxF = 5  # Maximum force/acceleration
speed_limit = 2 * 10
friend_scale = 10
pixelsize = 600
framedelay = 30

maxIttr = 5000

scale = pixelsize / s

h = 700  # height
height = h / scale  # window height
w = 1800  # width
width = w / scale  # window width

gv_speed = 1.5  # goal velocity speed

drawVels = True
win = Tk()
canvas = Canvas(win, width=w, height=h, background="#444")

# list of walls
walllist = [(0, 0.2 * h, 0.1 * w, 0.2 * h), (0.15 * w, 0.2 * h, 0.25 * w, 0.2 * h),
            (0.25 * w, 0.2 * h, 0.25 * w, 0.1 * h),
            (0.25 * w, 0.1 * h, 0.4 * w, 0.1 * h), (0.4 * w, 0.1 * h, 0.4 * w, 0.2 * h),
            (0.4 * w, 0.2 * h, 0.45 * w, 0.2 * h),
            (0.55 * w, 0.2 * h, 0.60 * w, 0.2 * h), (0.60 * w, 0.2 * h, 0.6 * w, 0.1 * h),
            (0.6 * w, 0.1 * h, 0.75 * w, 0.1 * h),
            (0.75 * w, 0.1 * h, 0.75 * w, 0.2 * h), (0.75 * w, 0.2 * h, 0.85 * w, 0.2 * h),
            (0.9 * w, 0.2 * h, w, 0.2 * h),
            (0, 0.6 * h, 0.1 * w, 0.6 * h), (0.15 * w, 0.6 * h, 0.25 * w, 0.6 * h),
            (0.25 * w, 0.6 * h, 0.25 * w, 0.7 * h),
            (0.25 * w, 0.7 * h, 0.4 * w, 0.7 * h), (0.4 * w, 0.7 * h, 0.4 * w, 0.6 * h),
            (0.4 * w, 0.6 * h, 0.45 * w, 0.6 * h),
            (0.55 * w, 0.6 * h, 0.60 * w, 0.6 * h), (0.60 * w, 0.6 * h, 0.6 * w, 0.7 * h),
            (0.6 * w, 0.7 * h, 0.75 * w, 0.7 * h),
            (0.75 * w, 0.7 * h, 0.75 * w, 0.6 * h), (0.75 * w, 0.6 * h, 0.85 * w, 0.6 * h),
            (0.9 * w, 0.6 * h, w, 0.6 * h)]

for wall in walllist:  # draw walls
    line = canvas.create_line(wall)

walllist = np.array(walllist) / scale  # covert wall list to np.array


class Zone:
    # class definiton of zones
    def __init__(self, pos=[0, 0, 0, 0], field=[1, 0]):
        self.pos = np.array(pos)/scale
        self.field = np.array(field)


# list of zones
zonelist = [Zone([0, 0.2*h, 0.25*w, 0.6*h], [1, 0]), Zone([0.1*w, 0.1*h, 0.15*w, 0.2*h], [0, 1]), Zone([0.1*w, 0.6*h, 0.15*w, 0.7*h], [0, -1]),
            Zone([0.25*w, 0.2*h, 0.4*w, 0.6*h], [1, 0]), Zone([0.25*w, 0.1*h, 0.3*w, 0.2*h], [0.707, 0.707]), Zone([0.3*w, 0.1*h, 0.35*w, 0.2*h], [1, 0]), Zone([0.35*w, 0.1*h, 0.4*w, 0.2*h], [0.707, 0.707]),
            Zone([0.3*w, 0.6*h, 0.35*w, 0.7*h], [1, 0]), Zone([0.25*w, 0.6*h, 0.3*w, 0.7*h], [0.707, -0.707]), Zone([0.35*w, 0.6*h, 0.4*w, 0.7*h], [0.707, -0.707]),
            Zone([0.4 * w, 0.2*h, 0.6 * w, 0.6*h], [1, 0]), Zone([0.4*w, 0.1*h, 0.6*w, 0.2*h], [0, 1]), Zone([0.4*w, 0.6*h, 0.6*w, 0.7*h], [0, -1]),
            Zone([0.6 * w, 0.2*h, 0.75 * w, 0.6*h], [1, 0]), Zone([0.65*w, 0.1*h, 0.7*w, 0.2*h], [1, 0]), Zone([0.6*w, 0.1*h, 0.65*w, 0.2*h], [0.707, 0.707]), Zone([0.7*w, 0.1*h, 0.75*w, 0.2*h], [0.707, 0.707]),
            Zone([0.65*w, 0.6*h, 0.7*w, 0.7*h], [1, 0]), Zone([0.6*w, 0.6*h, 0.65*w, 0.7*h], [0.707, -0.707]), Zone([0.7*w, 0.6*h, 0.75*w, 0.7*h], [0.707, -0.707]),
            Zone([0.75 * w, 0.2*h, w, 0.6*h], [1, 0]), Zone([0.75*w, 0.1*h, 0.9*w, 0.2*h], [0, 1]), Zone([0.75*w, 0.6*h, 0.9*w, 0.7*h], [0, -1])]

# list of goal destinations
goals = [(-0.1 * w, 0.5 * h), (0.125 * w, 0.05 * h), (0.125 * w, 0.75 * h), (0.5 * w, 0.05 * h), (0.5 * w, 0.75 * h),
         (0.875 * w, 0.05 * h), (0.875 * w, 0.75 * h), (1.1 * w, 0.5 * h)]
goals = np.array(goals) / scale


class coffeeStand:
    # class definition of coffee stands
    def __init__(self, pos_x=[0, 0], pos_y=[0, 0], charge=1, coffeeCupsLeft=50, imgpos=[0, 0]):
        self.pos_x = np.array(pos_x)
        self.pos_y = np.array(pos_y)
        self.pos = np.array([np.mean(self.pos_x), np.mean(self.pos_y)])
        self.imgpos = imgpos
        self.id = None
        self.charge = charge
        self.coffeeCupsLeft = coffeeCupsLeft
        self.inline = 0

    def studentCoffee(self):
        if self.coffeeCupsLeft > 0: self.coffeeCupsLeft -= 1
        if self.coffeeCupsLeft == 0: self.charge = 0


coffeeStands = [coffeeStand([0.3*width, 0.35*width], [0.6*height, 0.65*height], imgpos=[0.345*w, 0.62*h]),
                coffeeStand([0.65*width, 0.70*width], [0.6*height, 0.65*height], imgpos=[0.695*w, 0.62*h])]

for stand in coffeeStands:
    rectangle = canvas.create_rectangle(((stand.pos_x[0]*scale,stand.pos_y[1]*scale), (stand.pos_x[1]*scale,stand.pos_y[0]*scale)), fill="#fff")

canvas.pack()

def callback():
    print("click!")


img = Image.open("tekna.jpg")
img = img.resize((60, 25), Image.ANTIALIAS)
pimg = ImageTk.PhotoImage(img)
canvas.create_image(0.305*w, 0.605*h, anchor=NW, image=pimg)

imgn = Image.open("nito.png")
imgn = imgn.resize((60, 25), Image.ANTIALIAS)
pimgn = ImageTk.PhotoImage(imgn)
canvas.create_image(0.655*w, 0.605*h, anchor=NW, image=pimgn)

for stand in coffeeStands:
    stand.id = canvas.create_text(stand.imgpos[0], stand.imgpos[1], fill="black", font="Times 10 bold", text=str(stand.coffeeCupsLeft))

# Initalized variables
ittr = 0
QUIT = False
paused = False
step = False

emergency = False

circles = []
velLines = []
gvLines = []

deleted_indexes = []


class Person:
    # class definition for a single person having position pos, velocity vel, goal velocity gv, goal position goal,
    # radius rad, neighbors neighbors, time-to-collision with neighbors nt
    def __init__(self, pos=[0, 0], vel=[0.01, 0.01], gv=[0.01, 0.01], goal=[0, 0], friendtype=0, coffeeCharge=1):
        self.vel = np.array(vel)
        self.pos = np.array(pos)
        self.gv = np.array(gv)
        self.goal = np.array(goal)
        self.goal_save = self.goal
        self.rad = rad
        self.coffeeCharge = coffeeCharge
        self.inline = False
        self.neighbors = []
        self.nt = []
        self.friendtype = friendtype  # alle personer med samme friendtype er venner


people = [None] * num  # initialise num persons

class Emitter:
    def __init__(self, xmin=0, xmax=0, ymin=0, ymax=0, lam=0.05):
        self.xmin = xmin
        self.xmax = xmax
        self.ymin = ymin
        self.ymax = ymax
        self.lam = lam
        self.spawn = np.random.poisson(lam=lam, size=maxIttr)


emitters = [Emitter(xmin=0,xmax=0.05*width,ymin=0.20*height+rad,ymax=0.60*height-rad, lam=0.05),
            Emitter(xmin=0.95*width,xmax=width,ymin=0.20*height+rad,ymax=0.60*height-rad, lam=0.05),
            Emitter(xmin=0.475*width,xmax=0.525*width,ymin=0.6*height,ymax=0.7*height,lam=0.05)]


def update_goal_velocity(person):
    # update person's goal velocity based on his goal and the zone he's currently in

    zone = which_zone(person)  # find which zone he is in
    goal = person.goal  # retrive his goal destination

    if goal[0] < zone.pos[0]:  # goal is to the left of the zone
        person.gv = gv_speed*np.array([-zone.field[0], zone.field[1]])
    elif goal[0] > zone.pos[2]:  # goal is to the right of the zone
        person.gv = gv_speed*np.array([zone.field[0], zone.field[1]])
    else:  # goal is in the current zone
        p = person.goal - person.pos
        person.gv = p / np.sqrt(p.dot(p)) * gv_speed


def which_zone(person):
    # find which zone person is currently in
    [x, y] = person.pos
    for zone in zonelist:
        if zone.pos[0] < x < zone.pos[2] and zone.pos[1] < y < zone.pos[3]:
            return zone

    return Zone([0, 0, 0, 0], [0, 0])


def initSim():
    global rad, people

    print("")
    print("Simulation of Agents on Stripa.")
    print("Agents avoid collisions using prinicples based on the laws of anticipation seen in human pedestrians.")
    print("Green Arrow is Goal Velocity, Red Arrow is Current Velocity")
    print("SPACE to pause, 'S' to step frame-by-frame, 'V' to turn the velocity display on/off.")
    print("")

    for i in range(num):
        circles.append(canvas.create_oval(0, 0, rad, rad, fill="white"))
        velLines.append(canvas.create_line(0, 0, 10, 10, fill="red"))
        gvLines.append(canvas.create_line(0, 0, 10, 10, fill="green"))

        if i >= np.floor(num / 2):
            people[i] = Person(pos=[rnd.uniform(27, 30), rnd.uniform(4.15, 5.85)],
                               vel=[-1, rnd.uniform(-.01, .01)], gv=[-3, rnd.uniform(-.01, .01)],
                               goal=rnd.choice(goals))
        else:
            people[i] = Person(pos=[rnd.uniform(0, 2), rnd.uniform(4.15, 5.85)],
                               vel=[1, rnd.uniform(-.01, .01)], gv=[3, rnd.uniform(-.01, .01)],
                               goal=rnd.choice(goals))


def drawWorld():
    for i in range(len(people)):
        canvas.coords(circles[i], scale * (people[i].pos[0] - rad), scale * (people[i].pos[1] - rad),
                      scale * (people[i].pos[0] + rad), scale * (people[i].pos[1] + rad))
        canvas.coords(velLines[i], scale * people[i].pos[0], scale * people[i].pos[1],
                      scale * (people[i].pos[0] + 1. * rad * people[i].vel[0]),
                      scale * (people[i].pos[1] + 1. * rad * people[i].vel[1]))
        canvas.coords(gvLines[i], scale * people[i].pos[0], scale * people[i].pos[1],
                      scale * (people[i].pos[0] + 1. * rad * people[i].gv[0]),
                      scale * (people[i].pos[1] + 1. * rad * people[i].gv[1]))
        if drawVels:
            canvas.itemconfigure(velLines[i], state="normal")
            canvas.itemconfigure(gvLines[i], state="normal")
        else:
            canvas.itemconfigure(velLines[i], state="hidden")
            canvas.itemconfigure(gvLines[i], state="hidden")


def findNeighbors():
    global people

    for i in range(len(people)):
        people[i].neighbors = []
        people[i].nt = []
        vel_angle = np.arctan2(people[i].vel[1], people[i].vel[0])
        for j in range(len(people)):
            if i == j: continue;
            d = people[i].pos - people[j].pos
            d_angle = np.arctan2(d[1], d[0])
            l2 = d.dot(d)
            s2 = sight ** 2
            if l2 < s2 and abs(d_angle - vel_angle) > np.pi / 2:
                people[i].neighbors.append(j)
                people[i].nt.append(sqrt(l2))


def dE(persona, personb, r):
    global k, m, t0
    INFTY = 999
    maxt = 999

    w = personb.pos - persona.pos
    v = persona.vel - personb.vel
    radius = r + r
    dist = sqrt(w[0] ** 2 + w[1] ** 2)
    if radius > dist: radius = .99 * dist
    a = v.dot(v)
    b = w.dot(v)
    c = w.dot(w) - radius * radius
    discr = b * b - a * c
    if (discr < 0) or (a < 0.001 and a > - 0.001): return np.array([0, 0])
    discr = sqrt(discr)
    t1 = (b - discr) / a

    t = t1

    if (t < 0): return np.array([0, 0])
    if (t > maxt): return np.array([0, 0])

    d = k * exp(-t / t0) * (v - (v * b - w * a) / (discr)) / (a * t ** m) * (m / t + 1 / t0)

    return d


def closest_point_line_segment(c, wall):
    line_start = wall[0:2]
    line_end = wall[2:4]
    dota = (c - line_start).dot(line_end - line_start)
    if dota <= 0:
        return line_start
    dotb = (c - line_end).dot(line_start - line_end)
    if dotb <= 0:
        return line_end
    slope = dota / (dota + dotb)
    return line_start + (line_end - line_start) * slope


def normal(wall):
    # compute normal vector of wall
    p = wall[2:4] - wall[0:2]
    norm = np.array([-p[1], p[0]])
    return norm / np.sqrt(norm.dot(norm))


def wallforces(person):
    # wall forces acting on particle with center p and velocity
    global walllist, rad
    F = [0, 0]

    for wall in walllist:

        # find closest point to given wall, if too far away, do not care about given wall
        closest = closest_point_line_segment(person.pos, wall) - person.pos
        dw = closest.dot(closest)
        if dw > sight:
            continue

        r = np.sqrt(dw) if dw < rad ** 2 else rad

        t_min = 3

        discCollision = 0
        segmentCollision = 0

        a = person.vel.dot(person.vel)

        # does particle collide with top capsule
        w_temp = wall[0:2] - person.pos
        b_temp = w_temp.dot(person.vel)
        c_temp = w_temp.dot(w_temp) - r ** 2
        discr_temp = b_temp * b_temp - a * c_temp
        if discr_temp > 0 and abs(a) > 0:
            discr_temp = sqrt(discr_temp)
            t = (b_temp - discr_temp) / a
            if 0 < t < t_min:
                t_min = t
                b = b_temp
                discr = discr_temp
                w = w_temp
                discCollision = 1

                # does particle collide with bottom capsule
        w_temp = wall[2:4] - person.pos
        b_temp = w_temp.dot(person.vel)
        c_temp = w_temp.dot(w_temp) - r ** 2
        discr_temp = b_temp * b_temp - a * c_temp
        if discr_temp > 0 and abs(a) > 0:
            discr_temp = sqrt(discr_temp)
            t = (b_temp - discr_temp) / a
            if 0 < t < t_min:
                t_min = t
                b = b_temp
                discr = discr_temp
                w = w_temp
                discCollision = 1

                # does particle collide with line segment from the front
        w1 = wall[0:2] + r * normal(wall)
        w2 = wall[2:4] + r * normal(wall)
        w_temp = w2 - w1
        D = np.cross(person.vel, w_temp)
        if D != 0:
            t = np.cross(w_temp, person.pos - w1) / D
            # s = (p+velocity*t-o1_temp).dot(o_temp)/(o_temp.dot(o_temp))
            s = np.cross(person.vel, person.pos - w1) / D
            if 0 < t < t_min and 0 <= s <= 1:
                t_min = t
                w = w_temp
                discCollision = 0
                segmentCollision = 1

                # does particle collide with line segment from the bottom
        w1 = wall[0:2] - r * normal(wall)
        w2 = wall[2:4] - r * normal(wall)
        w_temp = w2 - w1
        D = np.cross(person.vel, w_temp)
        if D != 0:
            t = np.cross(w_temp, person.pos - w1) / D
            # s = (p + velocity * t - o1_temp).dot(o_temp) / (o_temp.dot(o_temp))
            s = np.cross(person.vel, person.pos - w1) / D
            if 0 < t < t_min and 0 <= s <= 1:
                t_min = t
                w = w_temp
                discCollision = 0
                segmentCollision = 1

                # compute forces acting on the particle
        if discCollision:
            FAvoid = -k * np.exp(-t_min / t0) * (person.vel - (b * person.vel - a * w) / discr) / (a * (t_min ** m)) * (
                    m / t_min + 1 / t0)
            # mag = np.sqrt(FAvoid.dot(FAvoid))
            # if (mag > maxF): FAvoid = maxF * FAvoid / mag
            F += FAvoid
        if segmentCollision:
            FAvoid = k * np.exp(-t_min / t0) / (t_min ** m * np.cross(person.vel, w)) * (m / t_min + 1 / t0) * np.array(
                [-w[1], w[0]])
            # mag = np.sqrt(FAvoid.dot(FAvoid))
            # if (mag > maxF): FAvoid = maxF * FAvoid / mag
            F += FAvoid
    return F


def Lennard_Jones_gradient(persona, personb, r, scaling_factor):
    r_vec = personb.pos - persona.pos
    dist = r_vec.dot(r_vec)
    unit_vec = r_vec / dist
    gradient = unit_vec * ((-6 / (dist - 2 * r) ** 7) + (12 / (dist - 2 * r) ** 13))
    gradient_size = np.sqrt(gradient.dot(gradient))
    if gradient_size > maxF:
        gradient = gradient * maxF / gradient_size  # scaling down the gradient
    return gradient / scaling_factor


def hardwall(i, dt, a):
    global people

    p = people[i].pos + (a * dt) * dt
    r = rad

    for wall in walllist:
        q = closest_point_line_segment(people[i].pos, wall)
        y = (people[i].pos - q).dot(people[i].pos - q)
        if y <= rad ** 2:
            #print("oh hell no!")
            r = np.sqrt(y)/1.005

        q = closest_point_line_segment(p, wall)
        # if q is wall_start or wall_end; do something different
        if (p - q).dot(p - q) <= r ** 2:
            w = wall[2:4] - wall[0:2]
            n = np.array([-w[1], w[0]])
            u = people[i].vel.dot(n) / n.dot(n) * n
            people[i].vel += -2 * u
            people[i].pos += people[i].vel * dt
            #print("Auch! I just walked straight into a wall!")
            return

    people[i].vel += a * dt
    # speed =  np.sqrt(people[i].vel.dot(people[i].vel))
    # if speed > speed_limit:
    #    people[i].vel = people[i].vel*speed_limit/speed
    people[i].pos += people[i].vel * dt


def find_closest_coffeeStand(person):
    epsilon = width*0.1

    if person.coffeeCharge:
        for stand in coffeeStands:
            r = stand.pos - person.pos
            dist = np.sqrt(r.dot(r))
            if dist < epsilon and stand.coffeeCupsLeft and (stand.inline < 5 or person.inline):

                if not person.inline:
                    stand.inline += 1
                    person.inline = True
                    return stand.pos

                if stand.pos_x[0] < person.pos[0] < stand.pos_x[1] and stand.pos_y[0] < person.pos[1] < stand.pos_y[1]:
                    person.coffeeCharge = 0
                    person.inline = False
                    stand.inline -= 1
                    stand.studentCoffee()
                    canvas.delete(stand.id)
                    stand.id = canvas.create_text(stand.imgpos[0], stand.imgpos[1], fill="black", font="Times 10 bold", text=str(stand.coffeeCupsLeft))
                    return person.goal_save

                return stand.pos

    return person.goal_save


def F_hardsphere():
    global s, people, pixelsize
    collisionLastFrame = np.zeros(
        [len(people), len(people)])  # Element i,j er 1 hvis element i og j kolliderte i forrige frame
    collisionWithWall = np.zeros(len(people))  # person kolliderte med person i forrige frame
    # Maa sjekke at de ikke kolliderte i forrige frame
    # print("Avstand:",sqrt(((c[0,0]-c[1,0]) ** 2) + ((c[1,1]-c[0,1]) ** 2)))
    for i in range(len(people)):
        for j in range(i + 1, len(people)):  # egentlig: for j in range(i+1, len(people)):
            d = people[i].pos - people[j].pos
            # print("distance:", sqrt(d[0]**2+d[1]**2))
            if (sqrt((d[0] ** 2) + (d[1] ** 2)) < 2 * rad) and collisionLastFrame[i, j] == False and collisionWithWall[
                i] == False:  # Kollisjon
                collisionLastFrame[i, j] = 1
                print("d:", d)
                print("Kollisjon!")
                if j < len(people):  # kollisjon mellom to personer
                    d_angle = np.arctan2(d[1], d[0])
                    # print(d_angle)
                    unit_vec = np.array([np.cos(d_angle), np.sin(d_angle)])
                    # print("unit vec",unit_vec)
                    v_i_parallell = np.dot(people[i].vel, unit_vec) * unit_vec
                    v_j_parallell = np.dot(people[j].vel, unit_vec) * unit_vec
                    # print("i parallell:", v_i_parallell, "j parallell:", v_j_parallell)
                    people[i].vel += -v_i_parallell + v_j_parallell
                    people[j].vel += v_i_parallell - v_j_parallell
                    # print("v_i:", people[i].vel)
                    # print("v_j:", v[j])

            elif (sqrt((d[0] ** 2) + (d[1] ** 2)) < 2 * rad):
                collisionLastFrame[i, j] = 1
                # else:
                #    collisionLastFrame[i,j] = 0


def outside(person):
    if person.pos[1] < 0.1 * height or person.pos[1] > 0.7 * height or person.pos[0] < 0 or person.pos[0] > width:
        return True
    return False


def update(dt):
    global people, deleted_indexes, coffeeStands, emergency
    findNeighbors()
    F = []  # force
    deleted_indexes = []  # delete selected people

    for i in range(len(people)):
        F.append(np.zeros(2))

    if coffeeStands[0].coffeeCupsLeft is 0:
        canvas.create_text(w/2, h/2, fill="red", font="Times 50 bold", text="EMERGENCY!")
        for i in range(len(people)):
            people[i].goal = goals[0]
            people[i].coffeeCharge = 0
            emergency = True

    for i in range(len(people)):
        update_goal_velocity(people[i])
        F[i] += (people[i].gv - people[i].vel) / .5
        F[i] += 1 * np.array([rnd.uniform(-3, 3), rnd.uniform(-3, 3)])

        for n, j in enumerate(people[i].neighbors):  # j is neighboring agent
            d = people[i].pos - people[j].pos
            r = rad
            dist = sqrt(d.dot(d))
            if dist < 2 * rad: r = dist / 2.001;  # shrink overlapping agents
            dEdx = dE(people[i], people[j], r)
            FAvoid = -dEdx
            F[i] += FAvoid

            # if people[i].friendtype == people[j].friendtype and people[i].friendtype != 0:
            #    F[i] += Lennard_Jones_gradient(people[i],people[j],rad, friend_scale)
            # else:
            #    F[i] += FAvoid

        FAvoid = wallforces(people[i])
        F[i] += FAvoid

        if not emergency:
            people[i].goal = find_closest_coffeeStand(people[i])

        cond = outside(people[i])
        if cond:
            deleted_indexes.append(i)

    #F_hardsphere()
    for i in range(len(people)):
        a = F[i]
        mag = np.sqrt(a.dot(a))
        if (mag > maxF): a = maxF * a / mag
        hardwall(i, dt, a)


def on_key_press(event):
    global paused, step, QUIT, drawVels
    if event.keysym == "space":
        paused = not paused
    if event.keysym == "s":
        step = True
        paused = False
    if event.keysym == "v":
        drawVels = not drawVels
    if event.keysym == "Escape":
        QUIT = True


def drawFrame(dt=0.05):
    global start_time, step, paused, ittr, deleted_indexes, circles, velLines, gvLines

    if ittr > maxIttr or QUIT:  # Simulation Loop ... or len(people)==0
        print("%s itterations ran ... quitting" % ittr)
        win.destroy()
    else:
        elapsed_time = time.time() - start_time
        start_time = time.time()
        if not paused:
            update(dt)
            ittr += 1

            # delete some peepz
            deleted_indexes = np.flip(np.sort(deleted_indexes), 0)
            for i in deleted_indexes:
                canvas.delete(circles[i])
                canvas.delete(velLines[i])
                canvas.delete(gvLines[i])
                people.pop(i)
                circles.pop(i)
                velLines.pop(i)
                gvLines.pop(i)

            for emitter in emitters:
                if emitter.spawn[ittr]:

                    if not emergency:
                        people.append(Person(pos=[rnd.uniform(emitter.xmin, emitter.xmax),
                                                  rnd.uniform(emitter.ymin, emitter.ymax)],
                                             goal=rnd.choice(goals)))
                    else:
                        people.append(Person(pos=[rnd.uniform(emitter.xmin, emitter.xmax),
                                                  rnd.uniform(emitter.ymin, emitter.ymax)],
                                             goal=goals[0]))
                    person = 1

                    # print("person!")
                    for i in range(len(people) - 1):
                        d = people[i].pos - people[-1].pos
                        if np.sqrt(d[0] ** 2 + d[1] ** 2) < 2 * rad and person == 1:
                            people.remove(people[-1])
                            person = 0
                    if person == 1:
                        circles.append(canvas.create_oval(0, 0, rad, rad, fill="white"))
                        velLines.append(canvas.create_line(0, 0, 10, 10, fill="red"))
                        gvLines.append(canvas.create_line(0, 0, 10, 10, fill="green"))

        drawWorld()
        if step == True:
            step = False
            paused = True

            # win.title("K.S.G. 2014 (Under Review) - " + str(round(1/elapsed_time,1)) +  " FPS")
        win.title("K.S.G. 2014 (Under Review)")
        win.after(framedelay, drawFrame)

    # win.on_resize=resize


win.bind("<space>", on_key_press)
win.bind("s", on_key_press)
win.bind("<Escape>", on_key_press)
win.bind("v", on_key_press)

initSim()

start_time = time.time()
win.after(framedelay, drawFrame)
mainloop()