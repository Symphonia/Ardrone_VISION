#!/usr/bin/env python
import roslib; roslib.load_manifest('drone_teleop')
import rospy
import rosbag
import pygame
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import Float64
from turtlesim.msg import Velocity
from turtlesim.msg import Pose
from ardrone_autonomy.msg import Navdata
import sys, select,termios,tty
import os
import time
pygame.init()

xpos = 0
ypos = 0
zpos = 0
zrot = 0
xdis = 0

linearx = 0
lineary = 0
linearz = 0
angularz = 0

linearspeed = 0.5
angularspeed = 0.5


scale = 1
size = width, height = 520*scale, 320*scale #determines how big the size of the controller gui is
black = 0,0,0
white = 255,255,255
blue = 0,0,255
red = 255,0,0
green = 0,255,0
font = pygame.font.SysFont("ubuntumono",12) #sets the font and font size

control = 'Currently no commands'

#callback function that is used for storing information from the Velocity.msg
def callback(RecMsg):
	global zpos
	global zrot
	zpos = RecMsg.linear
	zrot = RecMsg.angular

#callback function that is used for storing information from the Pose.msg
def callback1(laserp):
	global xpos
	xpos = laserp.x

#callback function that is used for storing information from Navdata.msg
def navCallback(navdata):
	global batteryLevel 
	batteryLevel = navdata.batteryPercent
	#currentState data is an int not a string
	global currentState 
	currentState = navdata.state
	global rotationX 
	rotationX = navdata.rotX
	global rotationY 
	rotationY = navdata.rotY
	global rotationZ 
	rotationZ = navdata.rotZ
	global altitude 
	altitude = navdata.altd
	global velocityx  
	velocityx = navdata.vx
	global velocityy 
	velocityy = navdata.vy
	global velocityz 
	velocityz = navdata.vz
	global accelerationx 
	accelerationx = navdata.ax
	global accelerationy 
	accelerationy = navdata.ay
	global accelerationz 
	accelerationz = navdata.az

#function for converting the drone state from int to string
def getDroneStatus(currentState):
	#not sure if global should be declared here, if program fails, take this global and declare it as a global outof the function
	global droneStatus
	if currentState == 0:
		droneStatus = "Emergency"
	if currentState == 2: 
		droneStatus = "Landed"
	if currentState == 4:
		droneStatus = "In Flight"
	if currentState == 6:
		droneStatus = "Taking Off"
	if currentState == 8:
		droneStatus = "Landing"

#function for displaying the  Xpos, Zpos, Zrot messages that comes from the colorshaperosnode.cpp
def dispPID(screen):
	text1 = 'X pos: ' + str(xpos)
	text2 = 'Z pos: ' + str(zpos)
	text3 = 'Z rot: ' + str(zrot)
	label1 = font.render(text1,1,white)
	label2 = font.render(text2,1,white)
	label3 = font.render(text3,1,white)
	screen.blit(label1,(width/2,195))
	screen.blit(label2,(width/2,210))
	screen.blit(label3,(width/2,225))

#function for displaying the twist.msg/ the velocity and direction the copter is traveling in
def dispAutoTwist(screen,twist):
	text1 = "Twist.linear.x: " + str(twist.linear.x)
	text2 = "Twist.linear.y: " + str(twist.linear.y)
	text3 = "Twist.linear.z: " + str(twist.linear.z)
	text4 = "Twist.angular.z: " + str(twist.angular.z)
	label1 = font.render(text1,1,white)
	label2 = font.render(text2,1,white)
	label3 = font.render(text3,1,white)
	label4 = font.render(text4,1,white)
	screen.blit(label1,(width/2,240))
	screen.blit(label2,(width/2,255))
	screen.blit(label3,(width/2,270))
	screen.blit(label4,(width/2,295))

#function for displaying the Navdata onto the control gui
def dispNavdata(screen):
	text1 = 'Drone status: ' + droneStatus
	text2 = 'Battery percent: ' + str(batteryLevel)
	text3 = 'Velocity x: ' + str(velocityx)
	text4 = 'Velocity y: ' + str(velocityy)
	text5 = 'Velocity z: ' + str(velocityz)
	text6 = 'Acceleration x: ' + str(accelerationx)
	text7 = 'Acceleration y: ' + str(accelerationy)
	text8 = 'Acceleration z: ' + str(accelerationz)
	text9 = 'Altitude: ' + str(altitude)
	text10 = 'Rotation x: ' + str(rotationX)
	text11 = 'Rotation y: ' + str(rotationY)
	text12 = 'Rotation z: ' + str(rotationZ)
	label1 = font.render(text1,1,red)
	label2 = font.render(text2,1,red)
	label3 = font.render(text3,1,red)
	label4 = font.render(text4,1,red)
	label5 = font.render(text5,1,red)
	label6 = font.render(text6,1,red)
	label7 = font.render(text7,1,red)
	label8 = font.render(text8,1,red)
	label9 = font.render(text9,1,red)
	label10 = font.render(text10,1,red)
	label11 = font.render(text11,1,red)
	label12 = font.render(text12,1,red)
	screen.blit(label1,(15,135))
	screen.blit(label2,(15,150))
	screen.blit(label3,(15,165))
	screen.blit(label4,(15,180))
	screen.blit(label5,(15,195))
	screen.blit(label6,(15,210))
	screen.blit(label7,(15,225))
	screen.blit(label8,(15,240))
	screen.blit(label9,(15,255))
	screen.blit(label10,(15,270))
	screen.blit(label11,(15,285))
	screen.blit(label12,(15,300))


#function for displaying the speed ratio or the sensitivity of the copter
def dispSpeed(screen,linearspeed,angularspeed):
	text = 'linear speed: ' + str(linearspeed) + ' angular speed: ' + str(angularspeed)
	label = font.render(text,1,white)
	screen.blit(label,(width/2,150))

#function for displaying the control instructions on top the window
def dispControls(screen):
	label1 = font.render("Control Node V1.0",1,white)
	label2 = font.render("| W = Fly Foward  | | S = Fly Backward | | A = Rotate Left | | D = Rotate Right |",1,green)
	label3 = font.render("| Q = Strafe Left | | E = Strafe Right | | F = Fly Higher  | | G = Fly Lower    |",1,green)
	label4 = font.render("| T = Take Off    | | U = Land         | | R = Reset       | | I = Reset Speed  |",1,green)
	label5 = font.render("| N/M = increase/decrease max speed by 10% |",1,green)
	label6 = font.render("| H/J = increase/decrease only linear speed by 10% |",1,green)
	label7 = font.render("| K/L = increase/decrease only angular speed by 10% |",1,green)
	label8 = font.render("| P = turn on autopilot | | O = turn off autopilot",1,green)
    	screen.blit(label1,(15,15))
    	screen.blit(label2,(15,30))
    	screen.blit(label3,(15,45))
    	screen.blit(label4,(15,60))
    	screen.blit(label5,(15,75))
    	screen.blit(label6,(15,90))
    	screen.blit(label7,(15,105))
    	screen.blit(label8,(15,120))

#function for displaying what the mode of the copter is in and what it is currently doing
def dispCurrentControl(screen,control,mode):
	label1 = font.render(mode,1,blue)
	label2 = font.render(control,1,blue)
	screen.blit(label1,(width/2,135))
	screen.blit(label2,(width/2,180))

if __name__ == "__main__":
	settings = termios.tcgetattr(sys.stdin)
	#initializing ros publishers and subscribers
	pub = rospy.Publisher('cmd_vel', Twist) #the topic for copter controls
	land_pub = rospy.Publisher('/ardrone/land', Empty) #the topic for landing
	reset_pub = rospy.Publisher('/ardrone/reset', Empty) #the topic for resetting the copter
    	toggle_pub=rospy.Publisher('/ardrone/togglecam', Empty) #the topic for camera toggle, which currently does not work
	takeoff_pub =rospy.Publisher('/ardrone/takeoff', Empty) #topic for take off
	rospy.Subscriber('/drocanny/vanishing_points',Velocity,callback) #calls to the topic and gets the values of turtlesim/velocity
	rospy.Subscriber('/drone/walldis',Pose,callback1) #calls to the topic and gets the values of turtlesim/pose
	rospy.Subscriber('/ardrone/navdata',Navdata,navCallback) #calls to the topic and gets the navData
    	twist = Twist() #iniatializes twist
	rospy.init_node('drone_teleop') #initializes the drone_teleop node

	screen = pygame.display.set_mode(size) #screen a window of that has the size of the declared "size"

	try:
		while(True):
			auto = True
			mode = 'Manual Flight'
			for event in pygame.event.get():
				if event.type == pygame.QUIT:
					sys.exit()
				if event.type == pygame.KEYDOWN:
					if event.key == pygame.K_w:
						linearx = 1
						control = 'Moving Foward'
					if event.key == pygame.K_s:
						linearx = -1
						control = 'Moving Backward'
					if event.key == pygame.K_a:
						angularz = -1
						control = 'Rotating Left'
					if event.key == pygame.K_d:
						angularz = 1
						control = 'Rotating Right'
					if event.key == pygame.K_q:
						lineary = 1
						control = 'Moving Left'
					if event.key == pygame.K_e:
						lineary = -1
						control = 'Moving Right'
					if event.key == pygame.K_t:
						takeoff_pub.publish(Empty())
						time.sleep(0.25)
						control = 'Taking Off'
					if event.key == pygame.K_u:
						land_pub.publish(Empty())
						time.sleep(0.25)
						control = 'Landing'
					if event.key == pygame.K_r:
						reset_pub.publish(Empty())
						time.sleep(1.5)
						land_pub.publish(Empty())
						time.sleep(0.25)
						os.system('rosservice  call /ardrone/flattrim')
						time.sleep(0.25)
						control = 'Resetted and flat trimed'
					if event.key == pygame.K_f:
						linearz = 1
						control = 'Moving Up'
					if event.key == pygame.K_g:
						linearz = -1
						control = 'Moving down'
					if event.key == pygame.K_n:
						linearspeed = linearspeed*1.1
						angularspeed = angularspeed*1.1
					if event.key == pygame.K_m:
						linearspeed = linearspeed*0.9
						angularspeed = angularspeed*0.9
					if event.key == pygame.K_h:
						linearspeed = linearspeed*1.1
					if event.key == pygame.K_j:
						linearspeed = linearspeed*0.9
					if event.key == pygame.K_k:
						angularspeed = angularspeed*1.1
					if event.key == pygame.K_l:
						angularspeed = angularspeed*0.9
					if event.key == pygame.K_i:
						linearspeed = 0.5
						angularspeed = 0.5
					if event.key == pygame.K_p:
						#autopilot
						while auto:
							mode = 'Autonomous Flight'
							for event in pygame.event.get():
								if event.type == pygame.QUIT:
									sys.exit()
								if event.type == pygame.KEYDOWN:
									if event.key == pygame.K_o:
										auto = False
									if event.key == pygame.K_n:
										linearspeed = linearspeed*1.1
										angularspeed = angularspeed*1.1
									if event.key == pygame.K_m:
										linearspeed = linearspeed*0.9
										angularspeed = angularspeed*0.9
									if event.key == pygame.K_h:
										linearspeed = linearspeed*1.1
									if event.key == pygame.K_j:
										linearspeed = linearspeed*0.9
									if event.key == pygame.K_k:
										angularspeed = angularspeed*1.1
									if event.key == pygame.K_l:
										angularspeed = angularspeed*0.9
									if event.key == pygame.K_i:
										linearspeed = 0.5
										angularspeed = 0.5
							time.sleep(0.015)
							twist.linear.x = -(linearspeed*xpos)
							twist.linear.z = -(linearspeed*zpos)-.05
							twist.angular.z = -angularspeed*zrot
							pub.publish(twist)
							screen.fill(black)
							dispControls(screen)
							dispSpeed(screen,linearspeed,angularspeed)
							dispCurrentControl(screen,control,mode)
							dispPID(screen)
							dispAutoTwist(screen,twist)
							getDroneStatus(currentState)
							dispNavdata(screen)
							time.sleep(0.015)
							twist.linear.x = 0
							pub.publish(twist)
							pygame.display.flip()
				if event.type == pygame.KEYUP:
					linearx = 0
					lineary = 0
					linearz = 0
					angularz = 0
					control = 'Currently no input commands'
			screen.fill(black)
			dispControls(screen)
			dispSpeed(screen,linearspeed,angularspeed)
			dispCurrentControl(screen,control,mode)
			getDroneStatus(currentState)
			dispNavdata(screen)

			twist.linear.x = linearx*linearspeed
			twist.linear.y = lineary*linearspeed
			twist.linear.z = linearz *linearspeed
			twist.angular.x = 0
			twist.angular.y = 0
			twist.angular.z = angularz*angularspeed
			pub.publish(twist)
			pygame.display.flip()
	except Exception as e:
		print e
		print repr(e)





			
