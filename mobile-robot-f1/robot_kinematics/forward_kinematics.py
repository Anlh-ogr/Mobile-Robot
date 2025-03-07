# library
import pygame as pg
import math

class Environment:
    def __init__ (self, dimention):
        # colors
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.red = (255, 0, 0)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        # self.yellow = (255, 255, 0) 
    
        # screen dimention
        self.height = dimention[0]
        self.width = dimention[1]
        
        # screen setup
        pg.display.set_caption("Generation F1")
        self.map = pg.display.set_mode((self.width, self.height))
        self.font = pg.font.SysFont('JetBrains Mono', 20)
        self.text = self.font.render("default", True, self.white, self.black)
        self.textRect = self.text.get_rect()
        self.textRect.center = (dimention[1]-500, dimention[0]-100)
        self.trail_set = []
        
    def info(self, velo_left, velo_right, θ):
        text = f"Left:{velo_left}m/s Right:{velo_right}m/s θ:{int(math.degrees(θ))}°"
        self.text = self.font.render(text, True, self.white, self.black)
        self.map.blit(self.text, self.textRect)
        
    def trail(self, position):
        for idx in range(0, len(self.trail_set)-1):
            pg.draw.line(self.map, self.blue, (self.trail_set[idx][0], self.trail_set[idx][1]), (self.trail_set[idx+1][0], self.trail_set[idx+1][1]))
        if self.trail_set.__sizeof__() > 30000:
            self.trail_set.pop(0)
        self.trail_set.append(position)
        
    def frame(self, position, rotation):
        length_frame = 80
        centerX, centerY = position
        x_axis = (centerX + length_frame*math.cos(-rotation), centerY + length_frame*math.sin(-rotation))
        y_axis = (centerX + length_frame*math.cos(-rotation+math.pi/2), centerY + length_frame*math.sin(-rotation+math.pi/2))
        pg.draw.line(self.map, self.red, position, x_axis, 3)
        pg.draw.line(self.map, self.green, position, y_axis, 3)
        


class Robot:
    def __init__ (self, start_position, image, width):
        self.wid = width
        self.x_pos = start_position[0]
        self.y_pos = start_position[1]
        self.theta = 0
        self.velo_x = 0 #pixel/s
        self.velo_y = 0 #pixel/s
        
        # graphics
        self.image = pg.image.load(image)
        self.rotated = self.image
        self.rect = self.image.get_rect(center=(self.x_pos, self.y_pos))
        
    def draw(self, map):
        map.blit(self.rotated, self.rect)
    
    def move(self, dt):
        print(dt)
        self.x_pos += self.velo_x*dt
        self.y_pos += self.velo_y*dt
        
    def update(self, dt):
        self.rotated = pg.transform.rotate(self.image, math.degrees(self.theta))
        self.rect = self.rotated.get_rect(center=(self.x_pos, self.y_pos))

    def control(self, event):
        if event is not None:
            if event.type == pg.KEYDOWN:
                if event.key == pg.K_KP4:
                    self.velo_x = -100
                if event.key == pg.K_KP6:
                    self.velo_x = +100
                if event.key == pg.K_KP8:
                    self.velo_y = -100
                if event.key == pg.K_KP2:
                    self.velo_y = +100
                if event.key == pg.K_KP5:
                    self.theta += 0.1
                if event.key == pg.K_KP0:
                    self.theta -= 0.1
    
            
class main:
    def __init__ (self):
        pg.init()
        self.env = Environment((600, 800))
        self.robot = Robot((200, 200), r"D:\Py\MobileRobot\Mobile-Robot\mobile-robot-f1\access\Robot.png", 1)
        self.running = True
        
    def run(self):
        dt = 0
        last_time = pg.time.get_ticks()
        while self.running:
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    self.running = False
                self.robot.control(event)
            current_time = pg.time.get_ticks()
            dt = (current_time - last_time)/1000
            self.robot.move(dt)
            self.robot.update(dt)
            self.env.map.fill(self.env.black)
            self.env.frame((self.robot.x_pos, self.robot.y_pos), self.robot.theta)
            self.robot.draw(self.env.map)
            self.env.trail((self.robot.x_pos, self.robot.y_pos))
            self.env.info(self.robot.velo_x, self.robot.velo_y, self.robot.theta)
            pg.display.update()
            last_time = current_time
            
            
    
if __name__ == "__main__":
    main().run()

    