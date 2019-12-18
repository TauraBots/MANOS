#--- IMPORT DEPENDENCIES ------------------------------------------------------+

from __future__ import division
import random
import math

#--- MAIN ---------------------------------------------------------------------+

class Particle:
    def __init__(self,x0):
        self.individual_position=[]            # particle position
        self.individual_velocity=[]            # particle velocity
        self.individual_best_position=[]       # best position individual
        self.individual_best_error=-1          # best error individual
        self.individual_error=-1               # error individual
        for i in range(0,num_dimensions):
            self.individual_velocity.append(random.uniform(-1,1))
            self.individual_position.append(x0[i])

    # evaluate current fitness
    def evaluate(self,function):
        self.individual_error=function(self.individual_position)

        # check to see if the current position is an individual best
        if self.individual_error < self.individual_best_error or self.individual_best_error==-1:
            self.individual_best_position=self.individual_position
            self.individual_best_error=self.individual_error

    # update new particle velocity
    def update_velocity(self,global_best_position):
        w=0.5       # constant inertia weight (how much to weigh the previous velocity)
        c1=1        # cognative constant
        c2=2        # social constant

        for i in range(0,num_dimensions):
            r1=random.random()
            r2=random.random()

            vel_cognitive=c1*r1*(self.individual_best_position[i]-self.individual_position[i])
            vel_social=c2*r2*(global_best_position[i]-self.individual_position[i])
            self.individual_velocity[i]=w*self.individual_velocity[i]+vel_cognitive+vel_social

    # update the particle position based off new velocity updates
    def update_position(self,bounds):
        for i in range(0,num_dimensions):
            self.individual_position[i]=self.individual_position[i]+self.individual_velocity[i]

            # adjust maximum position if necessary
            if self.individual_position[i]>bounds[i][1]:
                self.individual_position[i]=bounds[i][1]

            # adjust minimum position if neseccary
            if self.individual_position[i] < bounds[i][0]:
                self.individual_position[i]=bounds[i][0]
                
def pso(function,x0,bounds,num_particles,iterations):
    global num_dimensions

    num_dimensions=len(x0)
    global_best_error=-1                   # best error for group
    global_best_position=[]                # best position for group

    # establish the swarm
    swarm=[]
    for i in range(0,num_particles):
        swarm.append(Particle(x0))

    # begin optimization loop
    i=0
    while i < iterations:
        #print i,global_best_error
        # cycle through particles in swarm and evaluate fitness
        for j in range(0,num_particles):
            swarm[j].evaluate(function)

            # determine if current particle is the best (globally)
            if swarm[j].individual_error < global_best_error or global_best_error == -1:
                global_best_position=list(swarm[j].individual_position)
                global_best_error=float(swarm[j].individual_error)

        # cycle through swarm and update velocities and position
        for j in range(0,num_particles):
            swarm[j].update_velocity(global_best_position)
            swarm[j].update_position(bounds)
        i+=1

    # print final results
    print ('FINAL:')
    print (global_best_position)
    print (global_best_error)
    return global_best_position, global_best_error
