import pygame
import math
import numpy as np
from scipy.integrate import ode
from lqr_starter import lqr, getA, getB, getQ, getR

x0 = [0.0,0,0,np.pi+np.pi/40]          

goal = np.array([ 0, 0, 0, np.pi ])   


def computeControl( x ):

	A = getA(x[3], x[2], x[1], 0)
	B = getB(x[3])
	Q = getQ()
	R = getR()
	k = lqr(A,B,Q,R)
	control = np.dot(-k, (x.T-goal.T))

    	return control

screen_width, screen_height = 800, 400   
                            
Done = False                
Pause = False               
                            
 
#COLORS
white = (255,255,255)
black = (0,0,0)
gray = (150, 150, 150)
Dark_red = (150, 0, 0)
radius = 7
cart_width = 30
cart_height = 15
pole_length = 100
cart_x_to_screen_scaling = 100.0

pygame.init()
background = pygame.display.set_mode((screen_width, screen_height))
clock = pygame.time.Clock()

class CartPole(object):
 
    # State holds x, x_dot, theta_dot, theta 
    def __init__(self, X0):  
        self.g = 9.82
        self.m = 0.5
        self.M = 0.5
        self.l = 0.5
        self.b = 1.0

        self.X0 = self.x = np.array(x0,dtype=np.float64).flatten()
        self.x = self.X0
        self.t = 0

        self.u = 0  

    def set_state(self, x):
        if (self.x is None or np.linalg.norm(x-self.x) > 1e-12):
            self.x = np.array(x,dtype=np.float64).flatten()
        self.solver = self.solver.set_initial_value(self.x)
        self.t = self.solver.t

    def reset(self):
        self.x = self.X0
        self.t = 0
        self.set_state(self.x)

    # Draw the cart and pole
    def draw(self, bg):  
        cart_centre = (int(screen_width/2+self.x[0]*cart_x_to_screen_scaling), int(screen_height/2))
        pole_end = (int(cart_centre[0] + pole_length * math.sin(self.x[3])), int(cart_centre[1]+ pole_length*math.cos(self.x[3])))
        pygame.draw.rect(bg, black, [cart_centre[0]-cart_width/2, cart_centre[1]-cart_height/2, cart_width, cart_height])
        pygame.draw.lines(bg, black, False, [cart_centre, pole_end], 2)
        pygame.draw.circle(bg, Dark_red, cart_centre, radius - 2)
        pygame.draw.circle(bg, Dark_red, pole_end, radius)

    def dynamics(self,t,z):

        f = np.array([self.u])

        sz = np.sin(z[3])
        cz = np.cos(z[3])
        cz2 = cz*cz

        a0 = self.m*self.l*z[2]*z[2]*sz
        a1 = self.g*sz
        a2 = f[0] - self.b*z[1]
        a3 = 4*(self.M+self.m) - 3*self.m*cz2

        dz = np.zeros((4,1))
        dz[0] = z[1]                                                            # x
        dz[1] = (  2*a0 + 3*self.m*a1*cz + 4*a2 )/ ( a3 )                       # dx/dt
        dz[2] = -3*( a0*cz + 2*( (self.M+self.m)*a1 + a2*cz ) )/( self.l*a3 )   # dtheta/dt
        dz[3] = z[2]                                                            # theta

        return dz


    def step(self,u,dt=None):

        self.u = u

        if dt is None:
            dt = 0.005
        t1 = self.solver.t + dt
        while self.solver.successful and self.solver.t < t1:
            self.solver.integrate(self.solver.t+ dt)
        self.x = np.array(self.solver.y)
        self.t = self.solver.t
        return self.x

    def get_state(self):
        return self.x

# Draw a grid behind the cartpole
def grid():  
    for x in range(50, screen_width, 50):
        pygame.draw.lines(background, gray, False, [(x, 0), (x, screen_height)])
        for y in range(50, screen_height, 50):
            pygame.draw.lines(background, gray, False, [(0, y), (screen_width, y)])
 
# Clean up the screen and draw a fresh grid and the cartpole with its latest state coordinates
def redraw():
    background.fill(white)
    grid()
    pendulum.draw(background)
    pygame.display.update()

pendulum = CartPole(x0)
state = pendulum.get_state()

while not Done:
    clock.tick(240)             # GUI refresh rate
                               
    for event in pygame.event.get():                    
        if event.type == pygame.QUIT:                    
            Done = True                                  
        if event.type == pygame.KEYDOWN:    # "r" key resets the simulator
            if event.key == pygame.K_r:
                pendulum.reset()
            if event.key == pygame.K_p:     # holding "p" key freezes time
                Pause = True
        if event.type == pygame.KEYUP:      # releasing "p" makes us live again
            if event.key == pygame.K_p:
                Pause = False

    if not Pause:

        control = computeControl( state )  
        state = pendulum.step(control)

        redraw()
 
pygame.quit()

