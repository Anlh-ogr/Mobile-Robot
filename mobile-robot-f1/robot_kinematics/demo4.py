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
        
        self.height = dimension[0]
        self.width = dimension[1]
        
        pg.display.set_caption("Mô phỏng Robot trong Mê cung (Ưu tiên rẽ tại góc cua)")
        self.map = pg.display.set_mode((self.width, self.height))
        
        self.font = pg.font.SysFont('JetBrains Mono', 18)
        self.text = self.font.render("default", True, self.white, self.black)
        self.textRect = self.text.get_rect()
        self.textRect.center = (self.width - 500, self.height - 100)
        
        self.trail_set = []
        self.visited_areas = set()

    def info(self, position, sensors, status):
        text = f"Vị trí:({position[0]:.2f}, {position[1]:.2f}) Cảm biến: L={sensors[0]:.2f} F={sensors[1]:.2f} R={sensors[2]:.2f} Trạng thái: {status}"
        self.text = self.font.render(text, True, self.white, self.black)
        self.map.blit(self.text, self.textRect)
        
    def trail(self, position):
        grid_x = int(position[0] // 10)
        grid_y = int(position[1] // 10)
        self.visited_areas.add((grid_x, grid_y))
        
        if len(self.trail_set) == 0 or np.linalg.norm(np.array(self.trail_set[-1]) - np.array(position)) > 5:
            self.trail_set.append(position)
        
        for idx in range(len(self.trail_set)-1):
            pg.draw.line(self.map, self.blue, (self.trail_set[idx][0], self.trail_set[idx][1]), (self.trail_set[idx+1][0], self.trail_set[idx+1][1]), 1)
        
    def frame(self, position, rotation):
        length_frame = 70
        centerX, centerY = position
        x_axis = (centerX + length_frame * math.cos(rotation), centerY + length_frame * math.sin(rotation))
        y_axis = (centerX + length_frame * math.cos(rotation - math.pi/2), centerY + math.sin(rotation - math.pi/2))
        pg.draw.line(self.map, self.red, position, x_axis, 3)
        pg.draw.line(self.map, self.green, position, y_axis, 3)

# Lớp Robot: quản lý robot
class Robot:
    def __init__(self, start_position, image, width, map_size):
        self.width = width
        self.x_pos = start_position[0]
        self.y_pos = start_position[1]
        self.map_size = map_size
        
        self.theta = 0
        self.velo_x = 60 * math.cos(self.theta)
        self.velo_y = 60 * math.sin(self.theta)
        
        self.image = pg.image.load(image)
        self.image = pg.transform.scale(self.image, (self.width, self.width))
        self.rotated = self.image
        self.rect = self.image.get_rect(center=(self.x_pos, self.y_pos))
        
        self.sensors = [float('inf'), float('inf'), float('inf')]
        self.last_turn = 0
        self.target_angle = None
        self.path = []
        self.visited = set()
        self.stuck_counter = 0
        self.last_position = (self.x_pos, self.y_pos)
        self.collision_count = 0
        self.loop_detected = False
        self.last_safe_position = (self.x_pos, self.y_pos)
        self.is_turning = False
        self.turn_steps = 0
        self.turn_direction = 0
        self.last_turn_time = 0  # Thời gian cuối cùng xoay để tránh quay đầu ngay sau đó

    def normalize_angle(self, angle):
        return ((angle + math.pi) % (2 * math.pi)) - math.pi
    
    def draw(self, map):
        map.blit(self.rotated, self.rect)
        
    def move(self, dt, map_surface):
        if self.is_turning:
            # Xoay tại chỗ, giữ nguyên vị trí
            self.velo_x = 0
            self.velo_y = 0
            return
        
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
        
        current_time = pg.time.get_ticks()
        if (not (0 <= new_x <= self.map_size - self.width) or
            not (0 <= new_y <= self.map_size - self.width) or collision):
            self.velo_x = 0
            self.velo_y = 0
            new_x = self.x_pos
            new_y = self.y_pos
            # Tránh quay đầu ngay sau khi xoay trong 1 giây
            if current_time - self.last_turn_time > 1000:
                self.collision_count += 1
                if self.collision_count > 1:
                    self.x_pos, self.y_pos = self.last_safe_position
                    self.theta = self.normalize_angle(self.theta + math.pi)
                    self.collision_count = 0
                    print("Va chạm hoặc ra ranh giới, quay lại và xoay 180°!")
        else:
            self.x_pos = new_x
            self.y_pos = new_y
            self.path.append((self.x_pos, self.y_pos))
            self.last_safe_position = (self.x_pos, self.y_pos)
            self.collision_count = 0
        
        self.rect = self.rotated.get_rect(center=(self.x_pos, self.y_pos))
        self.check_stuck()
        print(f"Vị trí: ({self.x_pos:.2f}, {self.y_pos:.2f}), Vận tốc: ({self.velo_x:.2f}, {self.velo_y:.2f})")
    
    def update(self, dt):
        self.rotated = pg.transform.rotate(self.image, -math.degrees(self.theta))
        self.rect = self.rotated.get_rect(center=(self.x_pos, self.y_pos))
        
    def sense(self, map_surface):
        sensor_angles = [self.theta - math.pi/4, self.theta, self.theta + math.pi/4]
        self.sensors = []
        half_width = self.width // 2
        
        for idx, angle in enumerate(sensor_angles):
            max_distance = 0
            best_angle = angle
            
            if idx == 1:  # Cảm biến giữa
                start_x = self.x_pos + half_width * math.cos(angle)
                start_y = self.y_pos + half_width * math.sin(angle)
                sensor_points = np.array([[start_x + i * math.cos(angle), start_y + i * math.sin(angle)] for i in range(600)])
                for point in sensor_points:
                    x, y = int(point[0]), int(point[1])
                    if (x < 0 or y < 0 or x >= map_surface.get_width() or y >= map_surface.get_height() or
                        map_surface.get_at((x, y)) == (0, 0, 0, 255)):
                        max_distance = sensor_distance
                        break
                    sensor_distance += 1
                if sensor_distance == 600:
                    max_distance = float('inf')
                else:
                    max_distance = max(10, max_distance * 0.9)  # Nhạy hơn với tường gần
            
            else:  # Cảm biến trái và phải quét thêm góc ±90°
                for delta in np.arange(-math.pi/4, math.pi/4 + math.pi/180, math.pi/180):
                    adjusted_angle = angle + delta
                    sensor_distance = 0
                    start_x = self.x_pos + half_width * math.cos(adjusted_angle)
                    start_y = self.y_pos + half_width * math.sin(adjusted_angle)
                    sensor_points = np.array([[start_x + i * math.cos(adjusted_angle), start_y + i * math.sin(adjusted_angle)] for i in range(600)])
                    
                    for point in sensor_points:
                        x, y = int(point[0]), int(point[1])
                        if (x < 0 or y < 0 or x >= map_surface.get_width() or y >= map_surface.get_height() or
                            map_surface.get_at((x, y)) == (0, 0, 0, 255)):
                            break
                        sensor_distance += 1
                    
                    if sensor_distance > max_distance:
                        max_distance = sensor_distance
                        best_angle = adjusted_angle
                
                if idx == 0:
                    sensor_angles[0] = best_angle
                elif idx == 2:
                    sensor_angles[2] = best_angle
            
            self.sensors.append(max_distance if max_distance < 600 else float('inf'))
        
        print(f"Cảm biến: Trái={self.sensors[0]:.2f}, Giữa={self.sensors[1]:.2f}, Phải={self.sensors[2]:.2f}")
    
    def check_stuck(self):
        current_grid = (int(self.x_pos // 10), int(self.y_pos // 10))
        self.visited.add(current_grid)
        
        if math.dist((self.x_pos, self.y_pos), self.last_position) < 1:
            self.stuck_counter += 1
            if self.stuck_counter > 30:
                self.turn_direction = -1  # Rẽ trái để thoát
                self.is_turning = True
                self.turn_steps = 0
                self.stuck_counter = 0
                print("Robot bị kẹt, bắt đầu xoay trái để thoát!")
        else:
            self.stuck_counter = 0
        self.last_position = (self.x_pos, self.y_pos)
        
        if len(self.path) > 20:
            recent_positions = self.path[-20:]
            if any(math.dist(pos, (self.x_pos, self.y_pos)) < 10 for pos in recent_positions[:-1]):
                if not self.loop_detected:
                    self.turn_direction = -1  # Rẽ trái để thoát vòng lặp
                    self.is_turning = True
                    self.turn_steps = 0
                    self.loop_detected = True
                    print("Phát hiện vòng lặp, bắt đầu xoay trái để thoát!")
            else:
                self.loop_detected = False
    
    def control(self):
        right_dist, front_dist, left_dist = self.sensors[2], self.sensors[1], self.sensors[0]
        
        turn_delay = 0.1
        front_threshold = 15  # Giảm ngưỡng để phát hiện gần hơn
        side_threshold = 80
        
        base_speed = 60
        
        if self.is_turning:
            # Xoay tại chỗ 30° từng bước
            self.theta = self.normalize_angle(self.theta + self.turn_direction * math.radians(30))
            self.turn_steps += 1
            print(f"Xoay {self.turn_steps}/3: Hướng {self.theta:.2f}")
            
            if self.turn_steps >= 3:  # Xoay 3 lần 30° = 90°
                self.is_turning = False
                self.turn_steps = 0
                self.turn_direction = 0
                self.last_turn_time = pg.time.get_ticks()  # Đánh dấu thời gian xoay
                # Kiểm tra lại cảm biến sau khi xoay để xác nhận đường mới
                self.sense(pg.display.get_surface())
                new_right_dist, new_front_dist, new_left_dist = self.sensors[2], self.sensors[1], self.sensors[0]
                if new_left_dist > new_right_dist and new_left_dist > 50:
                    self.velo_x = base_speed * math.cos(self.theta)
                    self.velo_y = base_speed * math.sin(self.theta)
                    print("Xác nhận đường mới bên trái, tiếp tục di chuyển!")
                elif new_right_dist > new_left_dist and new_right_dist > 50:
                    self.velo_x = base_speed * math.cos(self.theta)
                    self.velo_y = base_speed * math.sin(self.theta)
                    print("Xác nhận đường mới bên phải, tiếp tục di chuyển!")
                else:
                    self.velo_x = base_speed * math.cos(self.theta)
                    self.velo_y = base_speed * math.sin(self.theta)
                    print("Không xác định đường mới rõ ràng, tiếp tục di chuyển!")
            return
        
        self.velo_x = base_speed * math.cos(self.theta)
        self.velo_y = base_speed * math.sin(self.theta)
        
        # Phát hiện góc cua chữ L và buộc rẽ
        if self.is_corner(front_dist, left_dist, right_dist):
            if pg.time.get_ticks() - self.last_turn > turn_delay * 1000:
                current_grid = (int(self.x_pos // 10), int(self.y_pos // 10))
                
                # Tại góc cua chữ L, ưu tiên rẽ theo hướng mở
                if left_dist > right_dist and left_dist > 50:  # Đường mở bên trái
                    self.turn_direction = -1  # Xoay trái
                    print(f"Bắt đầu xoay trái tại góc cua chữ L: Trái ({left_dist:.2f}) xa hơn Phải ({right_dist:.2f})!")
                elif right_dist > left_dist and right_dist > 50:  # Đường mở bên phải
                    self.turn_direction = 1  # Xoay phải
                    print(f"Bắt đầu xoay phải tại góc cua chữ L: Phải ({right_dist:.2f}) xa hơn Trái ({left_dist:.2f})!")
                else:
                    self.turn_direction = -1  # Mặc định xoay trái nếu không rõ
                    print("Không xác định được hướng mở tại góc cua, xoay trái mặc định!")
                
                self.is_turning = True
                self.turn_steps = 0
                self.last_turn = pg.time.get_ticks()
        
        elif front_dist < front_threshold:
            if pg.time.get_ticks() - self.last_turn > turn_delay * 1000:
                current_grid = (int(self.x_pos // 10), int(self.y_pos // 10))
                
                if left_dist > right_dist:
                    self.turn_direction = -1  # Xoay trái
                    print(f"Bắt đầu xoay trái do cảm biến trái ({left_dist:.2f}) xa hơn phải ({right_dist:.2f})!")
                elif right_dist > left_dist:
                    self.turn_direction = 1  # Xoay phải
                    print(f"Bắt đầu xoay phải do cảm biến phải ({right_dist:.2f}) xa hơn trái ({left_dist:.2f})!")
                else:
                    self.turn_direction = -1  # Mặc định xoay trái nếu bằng nhau
                    print("Khoảng cách bằng nhau, xoay trái mặc định!")
                
                self.is_turning = True
                self.turn_steps = 0
                self.last_turn = pg.time.get_ticks()
        
        elif right_dist > side_threshold and left_dist < side_threshold and not self.loop_detected:
            if pg.time.get_ticks() - self.last_turn > turn_delay * 1000:
                self.turn_direction = 1  # Xoay phải
                self.is_turning = True
                self.turn_steps = 0
                self.last_turn = pg.time.get_ticks()
                print("Bắt đầu xoay phải do mất tường bên phải!")
        
        else:
            print("Ưu tiên đi thẳng!")
        
        self.theta = self.normalize_angle(self.theta)
        print(f"Hướng: {self.theta:.2f}, Góc mục tiêu: {self.target_angle}")
    
    def is_mid_path(self):
        if (self.sensors[0] > 50 and self.sensors[1] > 50 and self.sensors[2] > 50 and
            len(self.path) > 5 and abs(self.path[-1][0] - self.path[-5][0]) < 15 and
            abs(self.path[-1][1] - self.path[-5][1]) < 15):
            return True
        return False
    
    def is_corner(self, front_dist, left_dist, right_dist):
        # Cải thiện phát hiện góc cua gần hơn
        return (front_dist > 15 and front_dist < 30 and  # Gần nhưng không quá sát
                ((left_dist > 50 and right_dist < 50) or  # Góc cua chữ L, đường mở bên trái
                 (right_dist > 50 and left_dist < 50)))   # Góc cua chữ L, đường mở bên phải
    
    def draw_sensors(self, map, env):
        sensor_angles = [self.theta - math.pi/4, self.theta, self.theta + math.pi/4]
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
            dt = max(0.01, (current_time - last_time) / 1000)
            
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