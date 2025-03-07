import pygame as pg
import math

# Environment class : quan ly moi truong, giao dien hien thi
class Environment:
    def __init__ (self, dimention):
        # mau sac
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.red = (255, 0, 0)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.orange = (255, 165, 0)
        
        # kich thuoc man hinh
        self.height = dimention[0]
        self.width = dimention[1]
        
        # thiet lap giao dien
        pg.display.set_caption("model F1")
        self.map = pg.display.set_mode((self.width, self.height))
        
        # font chu + text hien thi
        self.font = pg.font.SysFont('JetBrains Mono', 18)
        self.text = self.font.render("default", True, self.white, self.black)
        self.textRect = self.text.get_rect()
        self.textRect.center = (self.width - 500, self.height - 100)
        
        # bo nho cho duong di chuyen cua robot
        self.trail_set = []
    
    def info (self, velo_left, velo_right, θ):
        # hien thi thong tin toc do va goc quay
        text = f"Left:{velo_left}m/s Right:{velo_right}m/s θ:{int(math.degrees(θ))}°"
        self.text = self.font.render(text, True, self.white, self.black)
        self.map.blit(self.text, self.textRect)
        
    def trail (self, position):
        # luu diem moi neu khoang cach du xa
        if len(self.trail_set) == 0 or math.dist(self.trail_set[-1], position) > 5:
            self.trail_set.append(position)
        
        # ve duong di chuyen cua robot
        for idx in range(len(self.trail_set)-1):
            pg.draw.line(self.map, self.blue,
                         (self.trail_set[idx][0], self.trail_set[idx][1]),
                         (self.trail_set[idx+1][0], self.trail_set[idx+1][1]))
        if self.trail_set.__sizeof__() > 2000:
            self.trail_set.pop(0)
        
    def frame (self, position, rotation):
        # hien thi truc toa do x, y
        length_frame = 70
        centerX, centerY = position
        x_axis = (centerX + length_frame*math.cos(-rotation),
                  centerY + length_frame*math.sin(-rotation))
        y_axis = (centerX + length_frame*math.cos(-rotation+math.pi/2),
                  centerY + length_frame*math.sin(-rotation+math.pi/2))
        pg.draw.line(self.map, self.red, position, x_axis, 3)
        pg.draw.line(self.map, self.green, position, y_axis, 3)
        
        
# Robot class : quan ly robot
class Robot:
    def __init__(self, start_position, image, width):
        self.width = width
        self.x_pos = start_position[0]
        self.y_pos = start_position[1]
        
        self.theta = 0
        self.velo_x = 0
        self.velo_y = 0
        
        # hinh anh robot
        self.image = pg.image.load(image)
        self.image = pg.transform.scale(self.image, (self.width, self.width))
        self.rotated = self.image
        self.rect = self.image.get_rect(center=(self.x_pos, self.y_pos))
        
    def draw(self, map):
        # ve robot tren ban do
        map.blit(self.rotated, self.rect)
        
    def move(self, dt):
        # cap nhat vi tri robot
        self.x_pos += self.velo_x * dt
        self.y_pos += self.velo_y * dt
    
    def update(self, dt):
        # xoay robot + cap nhat hinh anh
        self.rotated = pg.transform.rotate(self.image, math.degrees(self.theta))
        self.rect = self.rotated.get_rect(center=(self.x_pos, self.y_pos))
        
    def control(self, event):
        # dieu khien robot qua phim
        rotation_speed = 0.5    # rad/s
        linear_speed = 100      # pixel/s
        
        if event is not None:
            if event.type == pg.KEYDOWN:
                if event.key == pg.K_a:
                    self.velo_x = -linear_speed
                if event.key == pg.K_d:
                    self.velo_x = linear_speed
                if event.key == pg.K_w:
                    self.velo_y = -linear_speed
                if event.key == pg.K_s:
                    self.velo_y = linear_speed
                if event.key == pg.K_q:
                    self.theta -= rotation_speed
                if event.key == pg.K_e:
                    self.theta += rotation_speed
        
            if event.type == pg.KEYUP:
                if event.key in [pg.K_w, pg.K_s]:
                    self.velo_y = 0
                if event.key in [pg.K_a, pg.K_d]:
                    self.velo_x = 0
        
        return self.velo_x, self.velo_y, self.theta
    
    
# Main class : chay chuong trinh
class Main:
    def __init__(self):
        pg.init()
        
        # thiet lap moi truong
        self.map_size = 900     # 900x900 - 3mx3m
        self.map = pg.display.set_mode((self.map_size, self.map_size))
        self.maps = pg.transform.scale(
            pg.image.load(r"D:\Py\MobileRobot\Mobile-Robot\mobile-robot-f1\map\m1.png"),
            (self.map_size, self.map_size))
        
        # thiet lap robot
        self.env = Environment((self.map_size, self.map_size))
        self.robot_size = 60
        self.robot_image = pg.transform.scale(
            pg.image.load(r"D:\Py\MobileRobot\Mobile-Robot\mobile-robot-f1\access\Robot.png"),
            (self.robot_size, self.robot_size))
        self.robot = Robot((200, 200),
                           r"D:\Py\MobileRobot\Mobile-Robot\mobile-robot-f1\access\Robot.png",
                           self.robot_size)
        self.robot.image = self.robot_image
        self.running = True
        
    def detect_collision(self):
        # phat hien va xu ly va cham
        if self.robot.x_pos < 0 or self.robot.x_pos > self.map_size or \
              self.robot.y_pos < 0 or self.robot.y_pos > self.map_size:
                return True
        return False
        
    def run(self):
        dt = 0
        last_time = pg.time.get_ticks()
        
        while self.running:
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    self.running = False
                self.robot.control(event)
                
            current_time = pg.time.get_ticks()
            dt = (current_time - last_time) / 1000

            # kiem tra va cham bien
            if self.detect_collision():
                print("Cham bien!")
                self.running = False
            
            # cap nhat vi tri + hien thi
            self.robot.move(dt)
            self.robot.update(dt)
            
            # ve ban do + robot
            self.env.map.fill(self.env.white)
            self.env.map.blit(self.maps, (0, 0))
            self.env.frame((self.robot.x_pos, self.robot.y_pos), self.robot.theta)
            self.robot.draw(self.env.map)
            self.env.trail((self.robot.x_pos, self.robot.y_pos))
            self.env.info(self.robot.velo_x, self.robot.velo_y, self.robot.theta)
            
            pg.display.update()
            last_time = current_time
            
        
# chay chuong trinh
if __name__ == "__main__":
    main = Main()
    main.run()
    pg.quit()
    quit()
    
