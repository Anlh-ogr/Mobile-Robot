import pygame as pg
import math

# Lớp Environment: quản lý môi trường, giao diện hiển thị
class Environment:
    def __init__(self, dimension):
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.red = (255, 0, 0)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.purple = (128, 0, 128)  # Tím cho cảm biến trái
        self.yellow = (255, 255, 0)  # Vàng cho cảm biến giữa (trước)
        self.cyan = (0, 255, 255)    # Xanh lam cho cảm biến phải
        
        self.height = dimension[0]
        self.width = dimension[1]
        
        pg.display.set_caption("Mô phỏng Robot trong Mê cung F2")
        self.map = pg.display.set_mode((self.width, self.height))
        
        self.font = pg.font.SysFont('JetBrains Mono', 18)
        self.text = self.font.render("default", True, self.white, self.black)
        self.textRect = self.text.get_rect()
        self.textRect.center = (self.width - 500, self.height - 100)
        
        self.trail_set = []
    
    def info(self, position, sensors, status):
        text = f"Vị trí:({position[0]:.2f}, {position[1]:.2f}) Cảm biến: L={sensors[0]:.2f} F={sensors[1]:.2f} R={sensors[2]:.2f} Trạng thái: {status}"
        self.text = self.font.render(text, True, self.white, self.black)
        self.map.blit(self.text, self.textRect)
        
    def trail(self, position):
        if len(self.trail_set) == 0 or math.dist(self.trail_set[-1], position) > 5:
            self.trail_set.append(position)
        
        for idx in range(len(self.trail_set)-1):
            pg.draw.line(self.map, self.blue,
                         (self.trail_set[idx][0], self.trail_set[idx][1]),
                         (self.trail_set[idx+1][0], self.trail_set[idx+1][1]))
        
    def frame(self, position, rotation):
        length_frame = 70
        centerX, centerY = position
        x_axis = (centerX + length_frame * math.cos(-rotation),
                  centerY + length_frame * math.sin(-rotation))
        y_axis = (centerX + length_frame * math.cos(-rotation - math.pi/2),
                  centerY + length_frame * math.sin(-rotation - math.pi/2))
        pg.draw.line(self.map, self.red, position, x_axis, 3)
        pg.draw.line(self.map, self.green, position, y_axis, 3)

# Lớp Robot: quản lý robot
class Robot:
    def __init__(self, start_position, image, width, map_size):
        self.width = width
        self.x_pos = start_position[0]
        self.y_pos = start_position[1]
        self.map_size = map_size
        
        self.theta = 0  # Hướng ban đầu
        self.velo_x = 100 * math.cos(self.theta)
        self.velo_y = 100 * math.sin(self.theta)
        
        self.image = pg.image.load(image)
        self.image = pg.transform.scale(self.image, (self.width, self.width))
        self.rotated = self.image
        self.rect = self.image.get_rect(center=(self.x_pos, self.y_pos))
        
        self.sensors = [float('inf'), float('inf'), float('inf')]  # Trái, Giữa, Phải
        self.following_wall = False
        self.last_turn = 0
        self.target_angle = None
    
    def draw(self, map):
        map.blit(self.rotated, self.rect)
        
    def move(self, dt, map_surface):
        new_x = self.x_pos + self.velo_x * dt
        new_y = self.y_pos + self.velo_y * dt
        
        new_rect = self.rotated.get_rect(center=(new_x, new_y))
        collision = False
        half_width = self.width // 2
        for x in range(int(new_x - half_width), int(new_x + half_width), 5):
            for y in range(int(new_y - half_width), int(new_y + half_width), 5):
                if 0 <= x < map_surface.get_width() and 0 <= y < map_surface.get_height():
                    if map_surface.get_at((x, y)) == (0, 0, 0, 255):
                        collision = True
                        break
            if collision:
                break
        
        if (not (0 <= new_x <= self.map_size - self.width) or
            not (0 <= new_y <= self.map_size - self.width)):
            self.velo_x = 0
            self.velo_y = 0
            new_x = self.x_pos
            new_y = self.y_pos
        elif not collision:
            self.x_pos = new_x
            self.y_pos = new_y
        
        self.rect = self.rotated.get_rect(center=(self.x_pos, self.y_pos))
        print(f"Vị trí: ({self.x_pos:.2f}, {self.y_pos:.2f}), Vận tốc: ({self.velo_x:.2f}, {self.velo_y:.2f})")
    
    def update(self, dt):
        self.rotated = pg.transform.rotate(self.image, math.degrees(self.theta))
        self.rect = self.rotated.get_rect(center=(self.x_pos, self.y_pos))
        
    def sense(self, map_surface):
        # Cảm biến giữa nằm trên trục X, trái và phải lệch ±45° so với trục X
        sensor_angles = [self.theta - math.pi/4, self.theta, self.theta + math.pi/4]  # Trái, Giữa, Phải
        self.sensors = []
        half_width = self.width // 2
        
        for angle in sensor_angles:
            sensor_distance = 0
            start_x = self.x_pos + half_width * math.cos(angle)
            start_y = self.y_pos + half_width * math.sin(angle)
            
            while True:
                x = int(start_x + sensor_distance * math.cos(angle))
                y = int(start_y + sensor_distance * math.sin(angle))
                
                if (x < 0 or y < 0 or x >= map_surface.get_width() or y >= map_surface.get_height() or
                    map_surface.get_at((x, y)) == (0, 0, 0, 255)):
                    break
                sensor_distance += 1
            self.sensors.append(sensor_distance)
        print(f"Cảm biến: Trái={self.sensors[0]:.2f}, Giữa={self.sensors[1]:.2f}, Phải={self.sensors[2]:.2f}")
    
    def control(self):
        left_dist, front_dist, right_dist = self.sensors[0], self.sensors[1], self.sensors[2]
        
        corner_threshold = 50
        turn_delay = 0.5
        front_threshold = 100
        
        if left_dist < 50:  # Bám tường bên trái
            self.following_wall = True
            if front_dist < front_threshold:  # Tường phía trước, rẽ phải
                if self.target_angle is None:
                    self.target_angle = self.theta - math.pi / 2  # Rẽ phải 90°
                if pg.time.get_ticks() - self.last_turn > turn_delay * 1000:
                    if abs(self.theta - self.target_angle) > 0.1:
                        self.theta -= 0.1
                        self.last_turn = pg.time.get_ticks()
                    else:
                        self.theta = self.target_angle
                        self.target_angle = None
                    self.velo_x = max(100 * math.cos(self.theta), 0)
                    self.velo_y = max(100 * math.sin(self.theta), 0)
            elif left_dist < corner_threshold and front_dist > front_threshold and right_dist > front_threshold:
                if pg.time.get_ticks() - self.last_turn > turn_delay * 1000:
                    self.theta -= 0.05
                    self.last_turn = pg.time.get_ticks()
                    self.velo_x = 100 * math.cos(self.theta)
                    self.velo_y = 100 * math.sin(self.theta)
            else:
                self.velo_x = 100 * math.cos(self.theta)
                self.velo_y = 100 * math.sin(self.theta)
        else:
            if self.following_wall:
                if right_dist < 50:
                    if self.target_angle is None:
                        self.target_angle = self.theta + math.pi / 2  # Rẽ trái 90°
                    if pg.time.get_ticks() - self.last_turn > turn_delay * 1000:
                        if abs(self.theta - self.target_angle) > 0.1:
                            self.theta += 0.1
                            self.last_turn = pg.time.get_ticks()
                        else:
                            self.theta = self.target_angle
                            self.target_angle = None
                        self.velo_x = 100 * math.cos(self.theta)
                        self.velo_y = 100 * math.sin(self.theta)
                else:
                    if pg.time.get_ticks() - self.last_turn > turn_delay * 1000:
                        self.theta += 0.05
                        self.last_turn = pg.time.get_ticks()
                        self.velo_x = 100 * math.cos(self.theta)
                        self.velo_y = 100 * math.sin(self.theta)
            else:
                self.velo_x = 100 * math.cos(self.theta)
                self.velo_y = 100 * math.sin(self.theta)
        
        # Xoay 45° khi cả cảm biến trái và giữa phát hiện tường
        if left_dist < 50 and front_dist < 50:
            if pg.time.get_ticks() - self.last_turn > turn_delay * 1000:
                self.theta += math.pi / 4  # Xoay thêm 45° hướng xuống
                self.last_turn = pg.time.get_ticks()
                self.velo_x = 100 * math.cos(self.theta)
                self.velo_y = 100 * math.sin(self.theta)
                print(f"Xoay 45° xuống để cảm biến phải tìm khoảng trống")
        
        print(f"Hướng: {self.theta:.2f}, Trạng thái bám tường: {self.following_wall}, Góc mục tiêu: {self.target_angle}")
    
    def draw_sensors(self, map, env):
        sensor_angles = [self.theta - math.pi/4, self.theta, self.theta + math.pi/4]  # Trái, Giữa, Phải
        colors = [env.purple, env.yellow, env.cyan]  # Tím, Vàng, Xanh lam
        
        half_width = self.width // 2
        for i, angle in enumerate(sensor_angles):
            start_x = self.x_pos + half_width * math.cos(angle)
            start_y = self.y_pos + half_width * math.sin(angle)
            end_x = start_x + self.sensors[i] * math.cos(angle)
            end_y = start_y + self.sensors[i] * math.sin(angle)
            pg.draw.line(map, colors[i], (start_x, start_y), (end_x, end_y), 2)

# Lớp Main: chạy chương trình
class Main:
    def __init__(self):
        pg.init()
        self.map_size = 600
        self.map = pg.display.set_mode((self.map_size, self.map_size))
        self.maps = pg.transform.scale(
            pg.image.load(r"D:\Py\MobileRobot\Mobile-Robot\mobile-robot-f1\map\m1.png"),
            (self.map_size, self.map_size))
        
        scale_factor = self.map_size / 900
        start_x = 400 * scale_factor
        start_y = 50 * scale_factor
        self.env = Environment((self.map_size, self.map_size))
        self.robot_size = 45 * scale_factor
        self.robot = Robot((start_x, start_y),
                           r"D:\Py\MobileRobot\Mobile-Robot\mobile-robot-f1\access\right.png",
                           self.robot_size, self.map_size)
        self.running = True
        self.status = "Running"
        
    def run(self):
        dt = 0
        last_time = pg.time.get_ticks()
        exit_pos = (self.map_size - 50, self.map_size - 50)
        
        while self.running:
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    self.running = False
                
            current_time = pg.time.get_ticks()
            dt = (current_time - last_time) / 1000
            
            if math.dist((self.robot.x_pos, self.robot.y_pos), exit_pos) < 50:
                self.status = "Finish"
                self.running = False
            
            self.robot.sense(self.maps)
            self.robot.control()
            self.robot.move(dt, self.maps)
            self.robot.update(dt)
            
            self.env.map.fill(self.env.white)
            self.env.map.blit(self.maps, (0, 0))
            self.env.frame((self.robot.x_pos, self.robot.y_pos), self.robot.theta)
            self.robot.draw(self.env.map)
            self.env.trail((self.robot.x_pos, self.robot.y_pos))
            self.env.info((self.robot.x_pos, self.robot.y_pos), self.robot.sensors, self.status)
            self.robot.draw_sensors(self.env.map, self.env)
            
            pg.display.update()
            last_time = current_time
            
if __name__ == "__main__":
    main_program = Main()
    main_program.run()
    pg.quit()