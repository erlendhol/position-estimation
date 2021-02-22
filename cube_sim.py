import pygame
from pygame.locals import *
from angular_convertion import convert_multiple
import math

from OpenGL.GL import *
from OpenGL.GLU import *

verticies = (
    (1, -1, -1),
    (1, 1, -1),
    (-1, 1, -1),
    (-1, -1, -1),
    (1, -1, 1),
    (1, 1, 1),
    (-1, -1, 1),
    (-1, 1, 1)
    )

edges = (
    (0,1),
    (0,3),
    (0,4),
    (2,1),
    (2,3),
    (2,7),
    (6,3),
    (6,4),
    (6,7),
    (5,1),
    (5,4),
    (5,7)
    )


def Cube():
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(verticies[vertex])
    glEnd()


#TODO glRotateF forander bare en orientring, istede for å sette orientering.  

def main():
    pygame.init()
    display = (800,600)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)

    gluPerspective(45, (display[0]/display[1]), 0.1, 50.0)

    glTranslatef(0.0,0.0, -5)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
        old_x = 0
        old_y = 0
        old_z = 0
        old_w = 0
        quat_input = convert_multiple("readings/02-19-2021_rotation/angularVeloc.csv")
        for i in range (len(quat_input)):

            print(quat_input[i])
        
            x = quat_input[i, 0]
            y = quat_input[i, 1]
            z = quat_input[i, 2]
            w = quat_input[i, 3]

            alphah = math.cos(w)
            alpha = 2*alphah
            ex = x - old_x
            ey = y - old_y
            ez = z - old_z
            ew = w - old_w

            old_x = x
            old_y = y
            old_z = z
            old_w = w
            
            glRotatef(w,x,y,z)
            glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
            Cube()
            pygame.display.flip()
            pygame.time.wait(75)
        print('\n')
            
            
            
            
            # for _ in range(len(quat_input)):
                
            #     #glRotatef(-1, 2, 1, -1)
            #     glRotatef(w,x,y,z)
            #     glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
            #     Cube()
            #     pygame.display.flip()
            #     pygame.time.wait(10)



main()