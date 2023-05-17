import pygame
import select
import sys
import os
import numpy as np
import time

sys.path.append("mbot_lcm_msgs")
import lcm
from mbot_lcm_msgs import particle_t, particles_t

pygame.init()
display = pygame.display.set_mode((300,300))
particlePositions = np.array([0,0,0,0,0])
counter = 0.0

def my_handler(channel, data):
    global counter
    global particlePositions
    msg = particles_t.decode(data)
    
    for particle in msg.particles:  
        temp = np.array([counter,msg.utime,particle.pose.x,particle.pose.y,particle.pose.theta])
        particlePositions = np.row_stack((particlePositions,temp))
    counter = counter + 1
        

    

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
lc.subscribe("SLAM_PARTICLES", my_handler)

try:
    timeout = 0.005  # amount of time to wait, in seconds
    while True:
        for event in  pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit

       
        #rfds, wfds, efds = select.select([lc.fileno()], [], [],timeout)
        
        key_input = pygame.key.get_pressed()  
        if key_input[pygame.K_UP]:
            print(counter)
            lc.handle()
            time.sleep(2)
            print("Message Handled\n")
        
        #elif rfds:
            #print("Message received\n")
        #    pass


except KeyboardInterrupt:
    print("Exiting Program and Saving Data\n")
    f = open("./particlesPosInDriveSquare.csv","w")
    np.savetxt(f,particlePositions,"%1.3f",",")
    f.close()
    print("Finished")
