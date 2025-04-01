import pygame
import math

class Environment:
    def __init__(self, dimentions):
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.red = (255, 0, 0)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.yellow = (255, 255, 0)
        
        self.height = dimentions[0]
        self.width = dimentions[1]
        
        pygame.display.set_caption("Mobile Robot Simulation")
        
        self.map = pygame.display.set_mode((self.width, self.height))
        self.font = pygame.font.SysFont('freesansbold.ttf', 20)
        self.text = self.font.render("default", True, self.white, self.black)
        self.textRect = self.text.get_rect()
        self.textRect.center = (dimentions[1]-500, dimentions[0]-100)
        self.trail_set = []
        
    def write_info(self, velocity_left, velocity_right, theta):
        text = f"VeLeft: {velocity_left} m/s, VeRight: {velocity_right} m/s, Theta: {int(math.degrees(theta))} degrees"
        self.text = self.font.render(text, True, self.white, self.black)
        self.map.blit(self.text, self.textRect)
        
    def trail(self, position):
        for idx in range(0, len(self.trail_set)-1):
            pygame.draw.line(self.map, self.blue, (self.trail_set[idx][0], self.trail_set[idx][1]), (self.trail_set[idx+1][0], self.trail_set[idx+1][1]))
        if self.trail_set.__sizeof__() > 30000:
            self.trail_set.pop(0)
        self.trail_set.append(position)
        
    def robot_frame(self, position, rotation):
        n = 80
        center_x, center_y = position
        x_axis = (center_x + n*math.cos(-rotation), center_y + n*math.sin(-rotation))
        y_axis = (center_x + n*math.cos(-rotation+math.pi/2), center_y + n*math.sin(-rotation+math.pi/2))
        pygame.draw.line(self.map, self.red, position, x_axis, 3)
        pygame.draw.line(self.map, self.green, position, y_axis, 3)
        

class Robot:
    def __init__(self, start_position, image, width):
        self.wid = width
        self.x_pos = start_position[0]
        self.y_pos = start_position[1]
        self.theta = 0
        self.velo_x = 0 #pixel/s
        self.velo_y = 0 #pixel/s
        
        # graphics
        self.image = pygame.image.load(image)
        self.rotated = self.image
        self.rect = self.image.get_rect(center=(self.x_pos, self.y_pos))
        
    def draw(self, map):
        map.blit(self.rotated, self.rect)
        
    def move(self, event=None):
        if event is not None:
            if event.type == pygame.KEYDOWN:
                print(dt)
                if event.key == pygame.K_KP4:
                    self.velo_x = -100
                if event.key == pygame.K_KP6:
                    self.velo_x = +100
                if event.key == pygame.K_KP8:
                    self.velo_y = -100
                if event.key == pygame.K_KP2:
                    self.velo_y = +100
                if event.key == pygame.K_KP5:
                    self.theta += 0.1
                if event.key == pygame.K_KP0:
                    self.theta -= 0.1
            self.x_pos = self.x_pos + self.velo_x*dt
            self.y_pos = self.y_pos + self.velo_y*dt
            self.rotated = pygame.transform.rotate(self.image, math.degrees(self.theta))
            self.rect = self.rotated.get_rect(center=(self.x_pos, self.y_pos))
            
# init
pygame.init()

start = (200, 200)
dimentions = (600, 800)

running = True

dt = 0

last_time = pygame.time.get_ticks()
env = Environment(dimentions)
robot = Robot(start, r"D:\Py\MobileRobot\Mobile-Robot\mobile-robot-twowheels\assets\Robot.png" ,1)

# update
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        robot.move(event)
        
    dt = (pygame.time.get_ticks() - last_time)/1000
    last_time = pygame.time.get_ticks()
    pygame.display.update()
    env.map.fill(env.black)
    robot.move()
    robot.draw(env.map)
    
    env.robot_frame((robot.x_pos, robot.y_pos), robot.theta)
    env.trail((robot.x_pos, robot.y_pos))
    env.write_info(robot.velo_x, robot.velo_y, robot.theta)
    