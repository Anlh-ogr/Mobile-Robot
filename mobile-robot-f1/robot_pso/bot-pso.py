import pygame as pg
import os
import math


# Environment : Quan ly me cung + cac thao tac ve hinh anh
class Environment:
    COLORS = {'black': (0, 0, 0),
              'white': (255, 255, 255),
              'red': (255, 0, 0),
              'green': (0, 255, 0),
              'blue': (0, 0, 255),
              'purple': (128, 0, 128),
              'cyan': (0, 255, 255),
              'yellow': (255, 223, 0),
              'orange': (255, 140, 0),
              'pink': (255, 105, 180)}
    
    def __init__(self, dimensions=(600, 600)):
        # kich thuoc mac dinh cua me cung
        self.height, self.width = dimensions
        self.map = pg.display.set_mode(dimensions)
        
        # thiet lap font chu - topic
        pg.display.set_caption("PSO_Mobile_Robot")
        self.font = pg.font.SysFont('JetBrains Mono', 10)
        self.text = self.font.render("default", True, self.COLORS['white'], self.COLORS['black'])
        self.text_rect = self.text.get_rect(center=(self.width-550, self.height-60))
        
        # vi tri bat dau va ket thuc
        self.start = (300, 1)
        self.end = (300, 599)
                
    def draw_points(self):
        rect_width, rect_height = 103, 27
        offset_x, offset_y = rect_width // 2, rect_height // 2
        
        # ve diem bat dau
        start_rect = pg.Rect(self.start[0] - offset_x, (self.start[1] - offset_y), rect_width, rect_height)
        pg.draw.rect(self.map, self.COLORS['green'], start_rect)
        
        # ve diem ket thuc
        end_rect = pg.Rect(self.end[0] - offset_x, (self.end[1] - offset_y), rect_width, rect_height)
        pg.draw.rect(self.map, self.COLORS['red'], end_rect)
        

# Robot : Quan ly robot + cac thao tac di chuyen + PSO algorithm
class Robot:
    def __init__(self, start_position, image_path):
        # thong so cua robot
        self.width = 60
        self.x_pos = start_position[0]
        self.y_pos = start_position[1]
        self.theta = math.radians(90)
        
        self.velocity_x = 0
        self.velocity_y = 0
        
        # load hinh anh cua robot
        self.image = pg.transform.scale(pg.image.load(image_path), (self.width, self.width))
        self.image = pg.transform.rotate(self.image, -90)
        self.rotated = self.image
        self.rect = self.image.get_rect(center=(self.x_pos, self.y_pos))
        
        # goc cam bien hoat dong (trai -> phai)
        self.sensor_angles = [-100, -45, 0, 45, 100]
        
        self.colors = {'red': (255, 0, 0),
                       'green': (0, 255, 0),
                       'blue': (0, 0, 255),
                       'purple': (128, 0, 128),
                       'cyan': (0, 255, 255),
                       'yellow': (255, 223, 0),
                       'orange': (255, 140, 0),
                       'pink': (255, 105, 180)}
        
    def draw(self, map_surface, walls):
        # cap nhat vi tri cua robot
        self.rect.center = (self.x_pos, self.y_pos)
        
        # cap nhat hinh anh xoay(theta) 
        self.rotated = pg.transform.rotate(self.image, -math.degrees(self.theta))
        self.rect = self.rotated.get_rect(center=(self.x_pos, self.y_pos))
        map_surface.blit(self.rotated, self.rect)
        
        center = (self.x_pos, self.y_pos)
        axis_length = self.width // 2 + 10
        
        # ve cam bien
        for angle_deg, color_key in zip(self.sensor_angles, ['purple', 'cyan', 'yellow', 'orange', 'pink']):
            angle_rad = self.theta + math.radians(angle_deg)
            sensor_length = self._calculate_sensor_length(map_surface, walls, angle_rad)
            sensor_end = (int(center[0] + sensor_length * math.cos(angle_rad)),
                          int(center[1] + sensor_length * math.sin(angle_rad)))
            pg.draw.line(map_surface, self.colors[color_key], center, sensor_end, 2)
            pg.draw.circle(map_surface, self.colors[color_key], sensor_end, 4)
        
        # ve truc x, y cho robot
        x_bot = (center[0] + axis_length * math.cos(self.theta),
                 center[1] + axis_length * math.sin(self.theta))
        y_bot = (center[0] + axis_length * math.cos(self.theta + math.pi/2),
                 center[1] + axis_length * math.sin(self.theta + math.pi/2))
        pg.draw.line(map_surface, self.colors['red'], center, x_bot, 2)
        pg.draw.line(map_surface, self.colors['green'], center, y_bot, 2)
        
    def _calculate_sensor_length(self, map_surface, walls, angle):
        # do dai cam bien (khoang cach -> walls)
        max_length = 100
        center = (self.x_pos, self.y_pos)
        
        for length in range(1, max_length + 1):
            x = int(center[0] + length * math.cos(angle))
            y = int(center[1] + length * math.sin(angle))
            
            # kiem tra ngoai man hinh
            if x < 0 or x >= map_surface.get_width() or y < 0 or y >= map_surface.get_height():
                return length - 1
            # kiem tra va cham voi tuong
            if map_surface.get_at((x, y))[:3] == (0, 0, 0):
                return length - 1
            for wall in walls:
                if wall.collidepoint(x, y):
                    return length - 1
        return max_length


# Main : Cai dat thong so + chay chuong trinh
class Main:
    def __init__(self):
        pg.init()
        self.env = Environment()
        
        # load hinh nen va robot
        base_path = os.path.dirname(__file__)
        bg_path = os.path.join(base_path, "..", "map", "m1.png")
        robot_img_path = os.path.join(base_path, "..", "access", "bot_right.png")
        
        # load hinh nen va robot
        self.background = pg.transform.scale(pg.image.load(bg_path), (self.env.width, self.env.height))
        self.robot = Robot((self.env.start[0], self.env.start[1]+49), robot_img_path)
        self.walls = []
        self.running = True
                
    def run(self):
        delta_time = 0
        last_time = pg.time.get_ticks()
        
        while self.running:
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    self.running = False
            
            # tinh delta time (gioi han toi thieu la 0.01)
            current_time = pg.time.get_ticks()
            delta_time = max(0.01, (current_time - last_time) / 1000)
            
            # cap nhat vi tri (dieu chinh toc do di chuyen va goc quay)
            self.robot.x_pos += self.robot.velocity_x * delta_time
            self.robot.y_pos += self.robot.velocity_y * delta_time
            self.robot.theta += 0.1 * delta_time
            
            # gioi han robot trong man hinh
            self.robot.x_pos = max(self.robot.width / 2,
                                   min(self.env.width - self.robot.width / 2, self.robot.x_pos))
            self.robot.y_pos = max(self.robot.width / 2,
                                   min(self.env.height - self.robot.width / 2, self.robot.y_pos))
            
            # ve lai khung hinh
            self.env.map.blit(self.background, (0, 0))
            self.env.draw_points()
            self.robot.draw(self.env.map, self.walls)
            self.env.map.blit(self.env.text, self.env.text_rect)
            
            pg.display.flip()
            last_time = current_time
        
        pg.quit()
        
if __name__ == "__main__":
    main_program = Main()
    main_program.run()