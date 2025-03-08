import pygame as pg
import math
import numpy as np

# Lớp Environment: quản lý môi trường, giao diện hiển thị
class Environment:
    def __init__(self, dimension):
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.red = (255, 0, 0)
        self.green = (0, 255, 0)
        self.blue = (0, 0, 255)
        self.purple = (128, 0, 128)
        self.yellow = (255, 255, 0)
        self.orange = (255, 165, 0)
        
        self.height = dimension[0]  # 650 pixel
        self.width = dimension[1]   # 650 pixel
        self.scale_factor = 650 / 900  # Tỷ lệ thu nhỏ
        
        pg.display.set_caption("Mô phỏng Robot trong Mê cung (Động học thuận đến góc cua)")
        self.map = pg.display.set_mode((self.width, self.height))
        
        self.font = pg.font.SysFont('JetBrains Mono', 18)
        self.text = self.font.render("default", True, self.white, self.black)
        self.textRect = self.text.get_rect()
        self.textRect.center = (self.width - 500 * self.scale_factor, self.height - 100 * self.scale_factor)
        
        self.trail_set = []
        self.visited_areas = set()
        self.sensor_hit_points = []

    def info(self, position, sensors, status):
        text = f"Vị trí:({position[0]*self.scale_factor:.2f}, {position[1]*self.scale_factor:.2f}) Cảm biến: L={sensors[0]*self.scale_factor:.2f} F={sensors[1]*self.scale_factor:.2f} R={sensors[2]*self.scale_factor:.2f} Trạng thái: {status}"
        self.text = self.font.render(text, True, self.white, self.black)
        self.map.blit(self.text, self.textRect)
        
    def trail(self, position):
        scaled_position = (position[0] * self.scale_factor, position[1] * self.scale_factor)
        grid_x = int(scaled_position[0] // 10)
        grid_y = int(scaled_position[1] // 10)
        self.visited_areas.add((grid_x, grid_y))
        
        if len(self.trail_set) == 0 or np.linalg.norm(np.array(self.trail_set[-1]) - np.array(scaled_position)) > 5:
            self.trail_set.append(scaled_position)
        
        for idx in range(len(self.trail_set)-1):
            pg.draw.line(self.map, self.green if idx == len(self.trail_set)-2 else self.blue, 
                         self.trail_set[idx], self.trail_set[idx+1], 2)
        
    def frame(self, position, rotation):
        length_frame = 70 * self.scale_factor
        centerX, centerY = position[0] * self.scale_factor, position[1] * self.scale_factor
        x_axis = (centerX + length_frame * math.cos(rotation), centerY + length_frame * math.sin(rotation))
        y_axis = (centerX + length_frame * math.cos(rotation - math.pi/2), centerY + math.sin(rotation - math.pi/2))
        pg.draw.line(self.map, self.red, (centerX, centerY), x_axis, 3)
        pg.draw.line(self.map, self.green, (centerX, centerY), y_axis, 3)

    def draw_sensor_hit_points(self):
        for point in self.sensor_hit_points:
            scaled_point = (point[0] * self.scale_factor, point[1] * self.scale_factor)
            pg.draw.circle(self.map, (255, 0, 0), (int(scaled_point[0]), int(scaled_point[1])), 3)

# Lớp Robot: quản lý robot với động học thuận
class Robot:
    def __init__(self, start_position, image, width, map_size, exit_pos):
        self.width = width  # Kích thước robot (45-60 pixel)
        self.x_pos = start_position[0]
        self.y_pos = start_position[1]
        self.map_size = map_size  # 900 pixel
        self.theta = 0
        self.exit_pos = exit_pos
        
        self.base_speed = 60
        self.wheel_distance = 100
        self.v_left = 0
        self.v_right = 0
        
        self.image = pg.image.load(image)
        self.image = pg.transform.scale(self.image, (self.width, self.width))
        self.rotated = self.image
        self.rect = self.image.get_rect(center=(self.x_pos * 650/900, self.y_pos * 650/900))
        
        self.sensors = [float('inf'), float('inf'), float('inf')]
        self.path = []
        self.visited = set()
        self.tried_directions = {}
        self.stuck_counter = 0
        self.last_position = (self.x_pos, self.y_pos)
        self.collision_count = 0
        self.loop_detected = False
        self.last_safe_position = (self.x_pos, self.y_pos)
        self.last_direction = None
        self.locked_direction = False
        self.direction_lock_time = 0
        self.direction_lock_duration = 300
        self.turning = False  # Cờ để kiểm soát việc rẽ tại góc cua

    def normalize_angle(self, angle):
        return ((angle + math.pi) % (2 * math.pi)) - math.pi
    
    def update_kinematics(self, dt):
        v = (self.v_right + self.v_left) / 2
        omega = (self.v_right - self.v_left) / self.wheel_distance
        
        self.x_pos += v * math.cos(self.theta) * dt
        self.y_pos += v * math.sin(self.theta) * dt
        self.theta += omega * dt
        
        self.theta = self.normalize_angle(self.theta)
        self.velo_x = v * math.cos(self.theta)
        self.velo_y = v * math.sin(self.theta)
    
    def draw(self, map):
        scaled_x = self.x_pos * 650/900
        scaled_y = self.y_pos * 650/900
        map.blit(self.rotated, self.rect.move(scaled_x - self.rect.centerx, scaled_y - self.rect.centery))
        
    def move(self, dt, map_surface):
        self.update_kinematics(dt)
        
        new_x = self.x_pos
        new_y = self.y_pos
        
        new_rect = self.rotated.get_rect(center=(new_x * 650/900, new_y * 650/900))
        collision = False
        half_width = self.width // 2
        for x in range(int(new_x - half_width), int(new_x + half_width), 5):
            for y in range(int(new_y - half_width), int(new_y + half_width), 5):
                if 0 <= x < self.map_size and 0 <= y < self.map_size:
                    if map_surface.get_at((x, y)) == (0, 0, 0, 255):
                        collision = True
                        break
            if collision:
                break
        
        if (not (0 <= new_x <= self.map_size - self.width) or
            not (0 <= new_y <= self.map_size - self.width) or collision):
            self.v_left = 0
            self.v_right = 0
            self.x_pos = self.last_safe_position[0]
            self.y_pos = self.last_safe_position[1]
            self.collision_count += 1
            if self.collision_count > 1:
                self.theta = self.normalize_angle(self.theta + math.pi / 2 if np.random.random() > 0.5 else -math.pi / 2)
                self.v_left = self.base_speed
                self.v_right = self.base_speed
                self.collision_count = 0
                self.locked_direction = False
                print("Va chạm, xoay 90° ngẫu nhiên!")
        else:
            self.path.append((self.x_pos, self.y_pos))
            self.last_safe_position = (self.x_pos, self.y_pos)
            self.collision_count = 0
        
        self.rect = self.rotated.get_rect(center=(self.x_pos * 650/900, self.y_pos * 650/900))
        self.check_stuck()
        print(f"Vị trí thực: ({self.x_pos:.2f}, {self.y_pos:.2f}), Góc: {math.degrees(self.theta):.2f}")
    
    def update(self, dt):
        self.rotated = pg.transform.rotate(self.image, -math.degrees(self.theta))
        self.rect = self.rotated.get_rect(center=(self.x_pos * 650/900, self.y_pos * 650/900))
        
    def sense(self, map_surface, env):
        sensor_angles = [self.theta - math.radians(45), self.theta, self.theta + math.radians(45)]
        self.sensors = []
        half_width = self.width // 2
        max_scan_distance = 600
        env.sensor_hit_points.clear()

        for idx, angle in enumerate(sensor_angles):
            max_distance = 0
            hit_point = None
            start_x = self.x_pos + half_width * math.cos(angle)
            start_y = self.y_pos + half_width * math.sin(angle)
            sensor_points = np.array([[start_x + i * math.cos(angle), start_y + i * math.sin(angle)] for i in range(max_scan_distance)])

            for i, point in enumerate(sensor_points):
                x, y = int(point[0]), int(point[1])
                if (x < 0 or y < 0 or x >= self.map_size or y >= self.map_size or
                    map_surface.get_at((x, y)) == (0, 0, 0, 255)):
                    max_distance = i
                    hit_point = (point[0], point[1])
                    break
            else:
                max_distance = float('inf')

            self.sensors.append(max_distance if max_distance < max_scan_distance else float('inf'))
            if hit_point:
                env.sensor_hit_points.append(hit_point)

        print(f"Cảm biến thực: Trái={self.sensors[0]:.2f}, Giữa={self.sensors[1]:.2f}, Phải={self.sensors[2]:.2f}")
    
    def check_stuck(self):
        current_grid = (int(self.x_pos // 10), int(self.y_pos // 10))
        self.visited.add(current_grid)
        
        if math.dist((self.x_pos, self.y_pos), self.last_position) < 1:
            self.stuck_counter += 1
            if self.stuck_counter > 30:
                self.theta = self.normalize_angle(self.theta + math.pi / 2 if np.random.random() > 0.5 else -math.pi / 2)
                self.v_left = self.base_speed
                self.v_right = self.base_speed
                self.stuck_counter = 0
                self.locked_direction = False
                print("Bị kẹt, xoay 90° ngẫu nhiên!")
        else:
            self.stuck_counter = 0
        self.last_position = (self.x_pos, self.y_pos)
        
        if len(self.path) > 20:
            recent_positions = self.path[-20:]
            if any(math.dist(pos, (self.x_pos, self.y_pos)) < 10 for pos in recent_positions[:-1]):
                if not self.loop_detected:
                    self.theta = self.normalize_angle(self.theta + math.pi)
                    self.v_left = self.base_speed
                    self.v_right = self.base_speed
                    self.loop_detected = True
                    self.locked_direction = False
                    print("Phát hiện vòng lặp, quay 180°!")
            else:
                self.loop_detected = False
    
    def control(self, map_surface, env):
        right_dist, front_dist, left_dist = self.sensors[2], self.sensors[1], self.sensors[0]
        base_speed = 60
        front_threshold = 15
        side_threshold = 50
        corner_threshold = 30  # Ngưỡng phát hiện góc cua
        current_time = pg.time.get_ticks()
        
        current_grid = (int(self.x_pos // 10), int(self.y_pos // 10))
        self.tried_directions.setdefault(current_grid, set())
        
        dx = self.exit_pos[0] - self.x_pos
        dy = self.exit_pos[1] - self.y_pos
        target_angle = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(target_angle - self.theta)
        
        if self.locked_direction and (current_time - self.direction_lock_time) < self.direction_lock_duration:
            self.v_left = base_speed
            self.v_right = base_speed
            return
        
        # Phát hiện cảm biến không hiệu quả
        if (right_dist == float('inf') or left_dist == float('inf') or front_dist == float('inf') or
            (abs(right_dist - left_dist) < 5 and abs(front_dist - right_dist) < 5)):
            if math.degrees(self.theta) not in self.tried_directions[current_grid]:
                self.tried_directions[current_grid].add(math.degrees(self.theta))
            else:
                self.theta = self.normalize_angle(self.theta + math.pi)
                self.locked_direction = True
                self.direction_lock_time = current_time
                self.v_left = base_speed
                self.v_right = base_speed
                print("Cảm biến không hiệu quả, quay 180°!")
                return
        
        # Phát hiện góc cua và rẽ
        if front_dist < corner_threshold and (right_dist > side_threshold or left_dist > side_threshold):
            self.turning = True
            if right_dist > left_dist and right_dist > 10:
                self.theta = self.normalize_angle(self.theta + math.radians(90))  # Rẽ phải
                print("Phát hiện góc cua, rẽ phải!")
            elif left_dist > 10:
                self.theta = self.normalize_angle(self.theta - math.radians(90))  # Rẽ trái
                print("Phát hiện góc cua, rẽ trái!")
            else:
                self.theta = self.normalize_angle(self.theta + math.pi)  # Quay 180° nếu không rẽ được
                print("Góc cua không rẽ được, quay 180°!")
            self.locked_direction = True
            self.direction_lock_time = current_time
            self.v_left = base_speed
            self.v_right = base_speed
        elif self.turning and min(right_dist, front_dist, left_dist) > side_threshold:
            self.turning = False  # Kết thúc rẽ khi có không gian
        elif current_grid in self.visited and front_dist < front_threshold and right_dist < side_threshold and left_dist < side_threshold:
            self.theta = self.normalize_angle(self.theta + math.pi)
            self.locked_direction = True
            self.direction_lock_time = current_time
            self.v_left = base_speed
            self.v_right = base_speed
            print("Kẹt ở ô đã thăm, quay 180°!")
            return
        elif right_dist < side_threshold and front_dist > front_threshold:
            self.v_left = base_speed
            self.v_right = base_speed  # Đi thẳng nếu tay phải chạm tường
        elif abs(angle_diff) < math.radians(30) and min(right_dist, front_dist, left_dist) > side_threshold:
            self.v_left = base_speed
            self.v_right = base_speed  # Đi thẳng nếu hướng về điểm thoát
        else:
            self.theta = self.normalize_angle(self.theta + math.radians(45))  # Rẽ phải để tìm góc
            self.locked_direction = True
            self.direction_lock_time = current_time
            self.v_left = base_speed
            self.v_right = base_speed
        
        print(f"Điều khiển: Góc mới {math.degrees(self.theta):.2f}, Hướng thoát {math.degrees(target_angle):.2f}, Phải={right_dist:.2f}")

    def draw_sensors(self, map, env):
        sensor_angles = [self.theta - math.radians(45), self.theta, self.theta + math.radians(45)]
        colors = [env.purple, env.yellow, env.orange]
        
        half_width = self.width // 2
        for i, angle in enumerate(sensor_angles):
            start_x = self.x_pos + half_width * math.cos(angle)
            start_y = self.y_pos + half_width * math.sin(angle)
            end_x = start_x + self.sensors[i] * math.cos(angle)
            end_y = start_y + self.sensors[i] * math.sin(angle)
            pg.draw.line(map, colors[i], 
                         (start_x * 650/900, start_y * 650/900), 
                         (end_x * 650/900, end_y * 650/900), 2)

# Lớp Main: chạy chương trình
class Main:
    def __init__(self):
        pg.init()
        self.map_size = 900  # Kích thước thực tế của mê cung
        self.display_size = 650  # Kích thước hiển thị
        self.map = pg.display.set_mode((self.display_size, self.display_size))
        self.maps = pg.transform.scale(
            pg.image.load(r"D:\Py\MobileRobot\Mobile-Robot\mobile-robot-f1\map\m1.png"),
            (self.map_size, self.map_size))  # Giữ kích thước thực tế 900x900
        
        scale_factor = self.display_size / self.map_size
        start_x = 400
        start_y = 50
        self.env = Environment((self.display_size, self.display_size))
        self.robot_size = 50  # Giữ trong khoảng 45-60 pixel
        self.robot = Robot((start_x, start_y),
                           r"D:\Py\MobileRobot\Mobile-Robot\mobile-robot-f1\access\right.png",
                           self.robot_size, self.map_size, (self.map_size - 50, self.map_size - 50))
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
            dt = max(0.01, (current_time - last_time) / 1000)
            
            if math.dist((self.robot.x_pos, self.robot.y_pos), exit_pos) < 50:
                self.status = "Finish"
                self.running = False
            
            self.robot.sense(self.maps, self.env)
            self.robot.control(self.maps, self.env)
            self.robot.move(dt, self.maps)
            self.robot.update(dt)
            
            self.env.map.fill(self.env.white)
            self.env.map.blit(pg.transform.scale(self.maps, (self.env.width, self.env.height)), (0, 0))
            self.env.frame((self.robot.x_pos, self.robot.y_pos), self.robot.theta)
            self.robot.draw(self.env.map)
            self.env.trail((self.robot.x_pos, self.robot.y_pos))
            self.env.info((self.robot.x_pos, self.robot.y_pos), self.robot.sensors, self.status)
            self.robot.draw_sensors(self.env.map, self.env)
            self.env.draw_sensor_hit_points()
            
            pg.display.update()
            last_time = current_time
            
if __name__ == "__main__":
    main_program = Main()
    main_program.run()
    pg.quit()