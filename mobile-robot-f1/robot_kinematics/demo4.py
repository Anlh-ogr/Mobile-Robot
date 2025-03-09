import pygame as pg
import math
import numpy as np
import random

# Lớp Environment: quản lý môi trường và giao diện hiển thị
class Environment:
    def __init__(self, dimension):
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.red = (255, 0, 0)      # Màu cho trục x robot
        self.green = (0, 255, 0)    # Màu cho trục y robot
        self.blue = (0, 0, 255)     # Màu cho trail
        self.yellow = (255, 255, 0) # Màu cho điểm bắt đầu
        self.orange = (255, 165, 0) # Màu cho cảm biến bên trái
        self.purple = (128, 0, 128) # Màu cho cảm biến phía trước
        self.pink = (255, 105, 180) # Màu cho cảm biến bên phải
        self.cyan = (0, 255, 255)   # Màu cho cảm biến xa trái (Far Left)
        self.gold = (255, 215, 0)   # Màu cho cảm biến xa phải (Far Right)
        
        self.height = dimension[0]  # 600 pixel
        self.width = dimension[1]   # 600 pixel
        self.scale_factor = 600 / 900  # Tỷ lệ thu nhỏ từ 900x900 sang 600x600
        
        pg.display.set_caption("Mô phỏng Robot trong Mê cung (Cảm biến và Động học Ngược)")
        self.map = pg.display.set_mode((self.width, self.height))
        
        self.font = pg.font.SysFont('JetBrains Mono', 18)
        self.font_small = pg.font.SysFont('JetBrains Mono', 12)
        self.text = self.font.render("default", True, self.white, self.black)
        self.textRect = self.text.get_rect()
        self.textRect.center = (self.width - 400, self.height - 100)
        
        self.cell_size = 60  # 30cm ≈ 60 pixel sau scale
        self.trail_set = []
        self.visited_cells = {}  # Lưu ô đã đi qua với thứ tự
        self.start_pos = (400 * self.scale_factor, 50 * self.scale_factor)  # Điểm vào
        self.exit_pos = (330, self.height - 10)  # Điểm ra

    def info(self, position, status, front_dist, left_dist, right_dist, far_left_dist, far_right_dist):
        text = (f"Vị trí: ({position[0]:.2f}, {position[1]:.2f}) Trạng thái: {status} | "
                f"F: {front_dist:.1f} L: {left_dist:.1f} R: {right_dist:.1f} FL: {far_left_dist:.1f} FR: {far_right_dist:.1f}")
        self.text = self.font.render(text, True, self.white, self.black)
        self.map.blit(self.text, self.textRect)
        
    def trail(self, position):
        grid_x = int(position[0] // self.cell_size)
        grid_y = int(position[1] // self.cell_size)
        cell_key = (grid_x, grid_y)
        
        if cell_key not in self.visited_cells:
            order = len(self.visited_cells) + 1
            self.visited_cells[cell_key] = order
        
        for (gx, gy), order in self.visited_cells.items():
            text = self.font_small.render(str(order), True, self.red)
            text_rect = text.get_rect(center=(gx * self.cell_size + self.cell_size / 2,
                                             gy * self.cell_size + self.cell_size / 2))
            self.map.blit(text, text_rect)
        
        if len(self.trail_set) == 0 or np.linalg.norm(np.array(self.trail_set[-1]) - np.array(position)) > 5:
            self.trail_set.append(position)
        
        for idx in range(len(self.trail_set)-1):
            pg.draw.line(self.map, self.blue, self.trail_set[idx], self.trail_set[idx+1], 1)
        
        pg.draw.circle(self.map, self.yellow, (int(self.start_pos[0]), int(self.start_pos[1])), 10)
        pg.draw.circle(self.map, self.red, (int(self.exit_pos[0]), int(self.exit_pos[1])), 10)

    def frame(self, position, rotation):
        length_frame = 40
        centerX, centerY = position
        x_axis = (centerX + length_frame * math.cos(rotation), centerY + length_frame * math.sin(rotation))
        pg.draw.line(self.map, self.red, position, x_axis, 2)
        y_axis = (centerX + length_frame * math.cos(rotation + math.pi/2), centerY + length_frame * math.sin(rotation + math.pi/2))
        pg.draw.line(self.map, self.green, position, y_axis, 2)

    def draw_sensors(self, position, front_pos, left_pos, right_pos, far_left_pos, far_right_pos):
        # Cảm biến hiện tại
        pg.draw.line(self.map, self.purple, position, front_pos, 2)  # Tím cho cảm biến phía trước
        pg.draw.line(self.map, self.orange, position, left_pos, 2)   # Cam cho cảm biến bên trái
        pg.draw.line(self.map, self.pink, position, right_pos, 2)    # Hồng cho cảm biến bên phải
        
        pg.draw.circle(self.map, self.purple, front_pos, 5)
        pg.draw.circle(self.map, self.orange, left_pos, 5)
        pg.draw.circle(self.map, self.pink, right_pos, 5)
        
        # Cảm biến mới
        pg.draw.line(self.map, self.cyan, position, far_left_pos, 2)  # Xanh lam cho cảm biến xa trái
        pg.draw.line(self.map, self.gold, position, far_right_pos, 2) # Vàng cho cảm biến xa phải
        
        pg.draw.circle(self.map, self.cyan, far_left_pos, 5)
        pg.draw.circle(self.map, self.gold, far_right_pos, 5)

# Lớp Robot: điều khiển robot với cảm biến và động học ngược
class Robot:
    def __init__(self, start_position, image, width, map_size, exit_pos, map_surface):
        self.width = width
        self.x_pos = start_position[0]
        self.y_pos = start_position[1]
        self.map_size = map_size
        self.theta = math.radians(270)  # Bắt đầu hướng xuống dưới
        self.exit_pos = exit_pos
        self.map_surface = map_surface
        
        self.base_speed = 15
        self.v_left = 0
        self.v_right = 0
        self.wheel_distance = 30
        
        self.sensor_range = 100  # Phạm vi cảm biến
        self.front_sensor = 0
        self.left_sensor = 0
        self.right_sensor = 0
        self.far_left_sensor = 0  # Cảm biến xa trái
        self.far_right_sensor = 0 # Cảm biến xa phải
        
        self.front_pos = (self.x_pos, self.y_pos)
        self.left_pos = (self.x_pos, self.y_pos)
        self.right_pos = (self.x_pos, self.y_pos)
        self.far_left_pos = (self.x_pos, self.y_pos)
        self.far_right_pos = (self.x_pos, self.y_pos)
        
        self.image = pg.image.load(image)
        self.image = pg.transform.scale(self.image, (self.width, self.width))
        self.rotated = self.image
        self.rect = self.image.get_rect(center=(self.x_pos, self.y_pos))
        
        self.visited_cells = set()
        self.status = "Running"
        self.turn_history = []
        self.max_history = 5
        
        self.initial_check = True
        self.initial_steps = 0
        self.initial_max_steps = 5

    def normalize_angle(self, angle):
        return ((angle + math.pi) % (2 * math.pi)) - math.pi
    
    def update_kinematics(self, dt):
        v = (self.v_right + self.v_left) / 2
        omega = (self.v_right - self.v_left) / self.wheel_distance
        
        self.x_pos += v * math.cos(self.theta) * dt
        self.y_pos += v * math.sin(self.theta) * dt
        self.theta += omega * dt
        
        self.theta = self.normalize_angle(self.theta)
    
    def draw(self, map):
        map.blit(self.rotated, self.rect)
        
    def move(self, dt):
        self.update_sensors()
        self.control()
        self.update_kinematics(dt)
        
        new_x = self.x_pos
        new_y = self.y_pos
        
        new_rect = self.rotated.get_rect(center=(new_x, new_y))
        collision = False
        half_width = self.width // 2
        for x in range(int(new_x - half_width), int(new_x + half_width), 5):
            for y in range(int(new_y - half_width), int(new_y + half_width), 5):
                if 0 <= x < self.map_surface.get_width() and 0 <= y < self.map_surface.get_height():
                    if self.map_surface.get_at((x, y)) == (0, 0, 0, 255):
                        collision = True
                        break
            if collision:
                break
        
        if (not (0 <= new_x <= self.map_size - self.width) or
            not (0 <= new_y <= self.map_size - self.width) or collision):
            self.v_left = 0
            self.v_right = 0
            self.status = "Collision"
        else:
            grid_x = int(self.x_pos // 60)
            grid_y = int(self.y_pos // 60)
            self.visited_cells.add((grid_x, grid_y))
        
        self.rect = self.rotated.get_rect(center=(self.x_pos, self.y_pos))
        print(f"Vị trí: ({self.x_pos:.2f}, {self.y_pos:.2f}), Góc: {math.degrees(self.theta):.2f}")
    
    def update(self, dt):
        self.rotated = pg.transform.rotate(self.image, -math.degrees(self.theta))
        self.rect = self.rotated.get_rect(center=(self.x_pos, self.y_pos))
    
    def update_sensors(self):
        # Cảm biến hiện tại: ±60°
        self.front_sensor, self.front_pos = self.measure_distance(self.theta)  # 0° (phía trước)
        self.left_sensor, self.left_pos = self.measure_distance(self.theta + math.radians(60))  # 60° bên trái
        self.right_sensor, self.right_pos = self.measure_distance(self.theta - math.radians(60))  # 60° bên phải
        
        # Cảm biến mới: ±90°
        self.far_left_sensor, self.far_left_pos = self.measure_distance(self.theta + math.radians(90))  # 90° bên trái
        self.far_right_sensor, self.far_right_pos = self.measure_distance(self.theta - math.radians(90))  # 90° bên phải
    
    def measure_distance(self, angle):
        distance = 0
        max_range = self.sensor_range
        step = 1
        
        x = self.x_pos
        y = self.y_pos
        
        while distance < max_range:
            x = self.x_pos + distance * math.cos(angle)
            y = self.y_pos + distance * math.sin(angle)
            
            if not (0 <= x < self.map_size and 0 <= y < self.map_size):
                return distance, (x, y)
            
            if self.map_surface.get_at((int(x), int(y))) == (0, 0, 0, 255):
                return distance, (x, y)
            
            distance += step
        
        return max_range, (x, y)

    def initial_direction_check(self):
        threshold = 20
        directions = {
            "front": self.front_sensor > threshold,
            "left": self.left_sensor > threshold,
            "right": self.right_sensor > threshold,
            "far_left": self.far_left_sensor > threshold,
            "far_right": self.far_right_sensor > threshold
        }
        
        possible_directions = sum(directions.values())
        
        dx = self.exit_pos[0] - self.x_pos
        dy = self.exit_pos[1] - self.y_pos
        desired_angle = math.atan2(dy, dx)
        
        if possible_directions == 1:
            if directions["front"]:
                self.theta = desired_angle
            elif directions["left"] or directions["far_left"]:
                self.theta += math.radians(90)
            elif directions["right"] or directions["far_right"]:
                self.theta -= math.radians(90)
            self.v_left = self.base_speed
            self.v_right = self.base_speed
            return True
        
        elif (directions["left"] or directions["far_left"]) and (directions["right"] or directions["far_right"]) and not directions["front"]:
            angle_to_left = self.normalize_angle(desired_angle - (self.theta + math.radians(45)))
            angle_to_right = self.normalize_angle(desired_angle - (self.theta - math.radians(45)))
            
            if abs(angle_to_left) < abs(angle_to_right):
                self.theta += math.radians(90)
            else:
                self.theta -= math.radians(90)
            self.v_left = self.base_speed
            self.v_right = self.base_speed
            return True
        
        elif possible_directions >= 3:
            self.theta = desired_angle
            self.v_left = self.base_speed
            self.v_right = self.base_speed
            return True
        
        elif possible_directions == 2:
            angle_to_left = self.normalize_angle(desired_angle - (self.theta + math.radians(45)))
            angle_to_right = self.normalize_angle(desired_angle - (self.theta - math.radians(45)))
            
            if (directions["left"] or directions["far_left"]) and (directions["right"] or directions["far_right"]):
                if abs(angle_to_left) < abs(angle_to_right):
                    self.theta += math.radians(90)
                else:
                    self.theta -= math.radians(90)
            elif (directions["front"] and directions["left"]) or (directions["front"] and directions["far_left"]):
                if abs(angle_to_left) < math.radians(45):
                    self.theta += math.radians(90)
                else:
                    self.theta = desired_angle
            elif (directions["front"] and directions["right"]) or (directions["front"] and directions["far_right"]):
                if abs(angle_to_right) < math.radians(45):
                    self.theta -= math.radians(90)
                else:
                    self.theta = desired_angle
            self.v_left = self.base_speed
            self.v_right = self.base_speed
            return True
        
        return False

    def control(self):
        # Kiểm tra nếu đã đến điểm ra
        if math.dist((self.x_pos, self.y_pos), self.exit_pos) < 10:
            self.v_left = 0
            self.v_right = 0
            self.status = "Finish"
            print("Đã đến điểm ra!")
            return
        
        # Kiểm tra hướng khởi đầu
        if self.initial_check and self.initial_steps < self.initial_max_steps:
            self.initial_steps += 1
            if self.initial_direction_check():
                self.initial_check = False
            return
        
        # Tính góc đến mục tiêu
        dx = self.exit_pos[0] - self.x_pos
        dy = self.exit_pos[1] - self.y_pos
        desired_angle = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(desired_angle - self.theta)
        
        # Ngưỡng khoảng cách
        wall_threshold = 30
        safe_distance = 15
        stop_distance = 10
        narrow_corridor_threshold = 20
        
        # Điều chỉnh tốc độ
        speed_factor = 0.5
        if self.front_sensor < safe_distance:
            speed_factor = max(0.1, self.front_sensor / safe_distance)
            if self.front_sensor < stop_distance:
                self.v_left = 0
                self.v_right = 0
                return
        
        # Kiểm tra hành lang hẹp
        if (self.left_sensor < narrow_corridor_threshold and self.right_sensor < narrow_corridor_threshold and
            self.front_sensor > wall_threshold):
            angle_to_left = self.normalize_angle(desired_angle - (self.theta + math.radians(45)))
            angle_to_right = self.normalize_angle(desired_angle - (self.theta - math.radians(45)))
            
            if abs(angle_to_left) < abs(angle_to_right):
                self.theta += math.radians(90)
            else:
                self.theta -= math.radians(90)
            self.v_left = -10
            self.v_right = 10
            return
        
        # Kiểm tra đường thẳng
        if self.front_sensor > 100 and min(self.left_sensor, self.right_sensor) < 20:
            if self.left_sensor > self.right_sensor or self.far_left_sensor > self.far_right_sensor:
                self.theta += math.radians(45)
            else:
                self.theta -= math.radians(45)
            self.v_left = self.base_speed * speed_factor
            self.v_right = self.base_speed * speed_factor
            return
        
        # Chiến lược điều hướng
        if self.front_sensor > wall_threshold:
            if abs(angle_diff) > math.radians(5):
                turn_speed = 15 * speed_factor if angle_diff > 0 else -15 * speed_factor
                self.v_left = -turn_speed
                self.v_right = turn_speed
            else:
                self.v_left = self.base_speed * speed_factor
                self.v_right = self.base_speed * speed_factor
                self.theta = desired_angle
        else:
            if len(self.turn_history) >= self.max_history and all(x == 1 for x in self.turn_history[-self.max_history:]):
                self.theta -= math.radians(random.uniform(30, 60))
                self.turn_history.append(-1)
            elif len(self.turn_history) >= self.max_history and all(x == -1 for x in self.turn_history[-self.max_history:]):
                self.theta += math.radians(random.uniform(30, 60))
                self.turn_history.append(1)
            else:
                # Ưu tiên rẽ trái khi phát hiện tường phải
                if self.right_sensor < 20 and (self.left_sensor > 20 or self.far_left_sensor > 20):
                    self.theta += math.radians(90)
                    self.turn_history.append(1)
                # Ưu tiên rẽ phải khi phát hiện tường trái
                elif self.left_sensor < 20 and (self.right_sensor > 20 or self.far_right_sensor > 20):
                    self.theta -= math.radians(90)
                    self.turn_history.append(-1)
                else:
                    # Kiểm tra và điều chỉnh nếu quay ngược
                    angle_diff = self.normalize_angle(desired_angle - self.theta)
                    if abs(angle_diff) > math.pi / 2:
                        self.theta = desired_angle
                        self.v_left = self.base_speed * speed_factor
                        self.v_right = self.base_speed * speed_factor
                    else:
                        # Sử dụng cảm biến xa để quyết định
                        if (self.far_left_sensor > self.far_right_sensor and self.far_left_sensor > 20) or (self.left_sensor > self.right_sensor and self.left_sensor > 20):
                            self.theta += math.radians(90)
                            self.turn_history.append(1)
                        elif (self.far_right_sensor > self.far_left_sensor and self.far_right_sensor > 20) or (self.right_sensor > self.left_sensor and self.right_sensor > 20):
                            self.theta -= math.radians(90)
                            self.turn_history.append(-1)
                        else:
                            turn_speed = 15 * speed_factor  # Mặc định rẽ trái
                            self.v_left = -turn_speed
                            self.v_right = turn_speed
                            self.turn_history.append(1)
            
            if len(self.turn_history) > self.max_history:
                self.turn_history.pop(0)
            
            self.v_left = self.base_speed * speed_factor
            self.v_right = self.base_speed * speed_factor

# Lớp Main: chạy chương trình
class Main:
    def __init__(self):
        pg.init()
        self.map_size = 600
        self.map = pg.display.set_mode((self.map_size, self.map_size))
        self.maps = pg.transform.scale(
            pg.image.load(r"D:\Py\MobileRobot\Mobile-Robot\mobile-robot-f1\map\a.png"),
            (self.map_size, self.map_size))
        
        start_x = 400 * (600 / 900)
        start_y = 50 * (600 / 900)
        self.env = Environment((self.map_size, self.map_size))
        self.robot_size = 45
        self.robot = Robot((start_x, start_y),
                           r"D:\Py\MobileRobot\Mobile-Robot\mobile-robot-f1\access\right.png",
                           self.robot_size, self.map_size, (330, self.map_size - 10), self.maps)
        self.running = True
        self.status = "Running"
        
    def run(self):
        dt = 0
        last_time = pg.time.get_ticks()
        
        while self.running:
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    self.running = False
                
            current_time = pg.time.get_ticks()
            dt = max(0.01, (current_time - last_time) / 1000)
            
            self.robot.move(dt)
            self.robot.update(dt)
            
            self.env.map.fill(self.env.white)
            self.env.map.blit(self.maps, (0, 0))
            self.env.frame((self.robot.x_pos, self.robot.y_pos), self.robot.theta)
            self.robot.draw(self.env.map)
            self.env.trail((self.robot.x_pos, self.robot.y_pos))
            self.env.draw_sensors((self.robot.x_pos, self.robot.y_pos),
                                 self.robot.front_pos, self.robot.left_pos, self.robot.right_pos,
                                 self.robot.far_left_pos, self.robot.far_right_pos)
            self.env.info((self.robot.x_pos, self.robot.y_pos), self.status,
                          self.robot.front_sensor, self.robot.left_sensor, self.robot.right_sensor,
                          self.robot.far_left_sensor, self.robot.far_right_sensor)
            
            pg.display.update()
            last_time = current_time
            
if __name__ == "__main__":
    main_program = Main()
    main_program.run()
    pg.quit()