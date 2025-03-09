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
        
        self.height = dimension[0]  # 600 pixel
        self.width = dimension[1]   # 600 pixel
        self.scale_factor = 600 / 900  # Tỷ lệ thu nhỏ từ 900x900 sang 600x600
        
        pg.display.set_caption("Mô phỏng Robot trong Mê cung (Đi thẳng & Tìm đường)")
        self.map = pg.display.set_mode((self.width, self.height))
        
        self.font = pg.font.SysFont('JetBrains Mono', 18)
        self.font_small = pg.font.SysFont('JetBrains Mono', 12)
        self.text = self.font.render("default", True, self.white, self.black)
        self.textRect = self.text.get_rect()
        self.textRect.center = (self.width - 400, self.height - 100)
        
        self.cell_size = 60  # 30cm ≈ 60 pixel sau scale
        self.trail_set = []
        self.visited_cells = {}  # Lưu ô đã đi qua với thứ tự
        self.sensor_hit_points = []
        self.start_pos = (400 * self.scale_factor, 50 * self.scale_factor)  # Điểm vào
        self.exit_pos = (330, self.height - 10)  # Điểm ra

    def info(self, position, sensors, status):
        text = f"Vị trí:({position[0]:.2f}, {position[1]:.2f}) Cảm biến: L={sensors[0]:.2f} F={sensors[1]:.2f} R={sensors[2]:.2f} Trạng thái: {status}"
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
        
        pg.draw.circle(self.map, self.green, (int(self.start_pos[0]), int(self.start_pos[1])), 10)
        pg.draw.circle(self.map, self.red, (int(self.exit_pos[0]), int(self.exit_pos[1])), 10)

    def frame(self, position, rotation):
        length_frame = 40
        centerX, centerY = position
        x_axis = (centerX + length_frame * math.cos(rotation), centerY + length_frame * math.sin(rotation))
        pg.draw.line(self.map, self.red, position, x_axis, 2)

    def draw_sensor_hit_points(self):
        for point in self.sensor_hit_points:
            pg.draw.circle(self.map, (255, 0, 0), (int(point[0]), int(point[1])), 3)

# Lớp Robot: quản lý robot với rule đi thẳng và khám phá
class Robot:
    def __init__(self, start_position, image, width, map_size, exit_pos):
        self.width = width  # 45 pixel ≈ 15cm sau scale
        self.x_pos = start_position[0]
        self.y_pos = start_position[1]
        self.map_size = map_size
        self.theta = math.radians(270)  # Bắt đầu hướng xuống dưới
        self.exit_pos = exit_pos
        
        self.base_speed = 40
        self.v_left = 0
        self.v_right = 0
        self.wheel_distance = 30
        
        self.image = pg.image.load(image)
        self.image = pg.transform.scale(self.image, (self.width, self.width))
        self.rotated = self.image
        self.rect = self.image.get_rect(center=(self.x_pos, self.y_pos))
        
        self.sensors = [float('inf'), float('inf'), float('inf')]
        self.path = []
        self.visited_cells = set()
        self.stuck_counter = 0
        self.last_position = (self.x_pos, self.y_pos)
        self.collision_count = 0
        self.last_safe_position = (self.x_pos, self.y_pos)
        self.turn_attempts = 0  # Đếm số lần thử xoay (sẽ được đơn giản hóa)

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
        
    def move(self, dt, map_surface):
        self.update_kinematics(dt)
        
        new_x = self.x_pos
        new_y = self.y_pos
        
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
            not (0 <= new_y <= self.map_size - self.width) or collision):
            self.v_left = 0
            self.v_right = 0
            self.x_pos = self.last_safe_position[0]
            self.y_pos = self.last_safe_position[1]
            self.collision_count += 1
            if self.collision_count > 2:
                self.collision_count = 0
                print("Va chạm, kiểm tra hướng mới!")
        else:
            grid_x = int(self.x_pos // 60)
            grid_y = int(self.y_pos // 60)
            cell_key = (grid_x, grid_y)
            if cell_key not in self.visited_cells:
                self.path.append((self.x_pos, self.y_pos))
                self.visited_cells.add(cell_key)
            self.last_safe_position = (self.x_pos, self.y_pos)
            self.collision_count = 0
        
        self.rect = self.rotated.get_rect(center=(self.x_pos, self.y_pos))
        self.check_stuck()
        print(f"Vị trí: ({self.x_pos:.2f}, {self.y_pos:.2f}), Góc: {math.degrees(self.theta):.2f}")
    
    def update(self, dt):
        self.rotated = pg.transform.rotate(self.image, -math.degrees(self.theta))
        self.rect = self.rotated.get_rect(center=(self.x_pos, self.y_pos))
        
    def sense(self, map_surface, env):
        sensor_angles = [self.theta - math.radians(45), self.theta, self.theta + math.radians(45)]
        self.sensors = []
        half_width = self.width // 2
        max_scan_distance = 200
        env.sensor_hit_points.clear()

        for idx, angle in enumerate(sensor_angles):
            max_distance = 0
            hit_point = None
            start_x = self.x_pos + half_width * math.cos(angle)
            start_y = self.y_pos + half_width * math.sin(angle)
            sensor_points = np.array([[start_x + i * math.cos(angle), start_y + i * math.sin(angle)] 
                                    for i in range(0, max_scan_distance, 5)])

            for point in sensor_points:
                x, y = int(point[0]), int(point[1])
                grid_x = int(x // 60)
                grid_y = int(y // 60)
                cell_key = (grid_x, grid_y)
                if (x < 0 or y < 0 or x >= map_surface.get_width() or y >= map_surface.get_height() or
                    map_surface.get_at((x, y)) == (0, 0, 0, 255) or cell_key in env.visited_cells):
                    max_distance = math.dist((start_x, start_y), point)
                    hit_point = (point[0], point[1])
                    break
            else:
                max_distance = float('inf')

            self.sensors.append(max_distance if max_distance < max_scan_distance else float('inf'))
            if hit_point:
                env.sensor_hit_points.append(hit_point)

        print(f"Cảm biến: Trái={self.sensors[0]:.2f}, Giữa={self.sensors[1]:.2f}, Phải={self.sensors[2]:.2f}")
    
    def check_stuck(self):
        if math.dist((self.x_pos, self.y_pos), self.last_position) < 1:
            self.stuck_counter += 1
            if self.stuck_counter > 20:
                self.stuck_counter = 0
                print("Bị kẹt, kiểm tra hướng mới!")
        else:
            self.stuck_counter = 0
        self.last_position = (self.x_pos, self.y_pos)
    
    def control(self, map_surface, env):
        right_dist, front_dist, left_dist = self.sensors[2], self.sensors[1], self.sensors[0]
        base_speed = 40
        front_threshold = 20
        side_threshold = 15
        
        # Kiểm tra nếu đến gần điểm ra
        if math.dist((self.x_pos, self.y_pos), self.exit_pos) < 50:
            self.v_left = 0
            self.v_right = 0
            self.status = "Finish"
            print("Đã đến điểm ra!")
            return
        
        # Ưu tiên đi thẳng xuống (hướng theta = 270°)
        if front_dist > front_threshold:
            self.v_left = base_speed
            self.v_right = base_speed
            print("Đi thẳng xuống để dò đường!")
            return
        
        # Nếu không đi thẳng xuống được, thử đi thẳng phải (hướng theta = 0°)
        if right_dist > side_threshold:
            self.theta = self.normalize_angle(0)  # Điều chỉnh về hướng phải
            self.v_left = base_speed
            self.v_right = base_speed
            print("Chuyển sang đi thẳng phải!")
            return
        
        # Nếu cả hai hướng không khả thi, xoay 90° sang trái hoặc phải dựa trên khoảng cách lớn nhất
        if left_dist > right_dist and left_dist > side_threshold:
            self.theta = self.normalize_angle(self.theta - math.radians(90))  # Xoay trái 90°
            print("Xoay trái 90° để tìm đường!")
        elif right_dist > side_threshold:
            self.theta = self.normalize_angle(self.theta + math.radians(90))  # Xoay phải 90°
            print("Xoay phải 90° để tìm đường!")
        else:
            print("Kẹt, thử điều chỉnh hướng!")
        
        self.v_left = base_speed
        self.v_right = base_speed

    def draw_sensors(self, map, env):
        sensor_angles = [self.theta - math.radians(45), self.theta, self.theta + math.radians(45)]
        colors = [env.purple, env.yellow, env.orange]
        
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
            pg.image.load(r"D:\Py\MobileRobot\Mobile-Robot\mobile-robot-f1\map\m3.png"),
            (self.map_size, self.map_size))
        
        start_x = 400 * (600 / 900)
        start_y = 50 * (600 / 900)
        self.env = Environment((self.map_size, self.map_size))
        self.robot_size = 45
        self.robot = Robot((start_x, start_y),
                           r"D:\Py\MobileRobot\Mobile-Robot\mobile-robot-f1\access\right.png",
                           self.robot_size, self.map_size, (330, self.map_size - 10))
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
            
            self.robot.sense(self.maps, self.env)
            self.robot.control(self.maps, self.env)
            self.robot.move(dt, self.maps)
            self.robot.update(dt)
            
            self.env.map.fill(self.env.white)
            self.env.map.blit(self.maps, (0, 0))
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