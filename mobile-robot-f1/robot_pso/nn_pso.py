import pygame as pg
import os
import math
import numpy as np

# Fitness function
def fitness(x, y, end_x, end_y):
    return math.sqrt((x - end_x)**2 + (y - end_y)**2)

class Environment:
    Colors = {'black': (0, 0, 0), 'white': (255, 255, 255), 'red': (255, 0, 0),
              'green': (0, 255, 0), 'blue': (0, 0, 255), 'orange': (255, 165, 0)}

    def __init__(self, dimentions = (650,650)):
        self.height, self.width = dimentions
        self.map = pg.display.set_mode(dimentions)
        pg.display.set_caption("NN-PSO")
        
        self.start = (325, 5, 3*np.pi/2)
        self.end = (325, 647, 3*np.pi/2)


    def draw_robot_frame(self, pos, rotation):
        axis_length = 80
        center_x, center_y = pos
        x_axis = (center_x + axis_length * math.cos(rotation), center_y + axis_length * math.sin(rotation))
        y_axis = (center_x + axis_length * math.cos(rotation-math.pi/2), center_y + axis_length * math.sin(rotation-math.pi/2))
        pg.draw.line(self.map, self.Colors['blue'], pos, x_axis, 2)
        pg.draw.line(self.map, self.Colors['green'], pos, y_axis, 2)

    def draw_sensors(self, pos, points):
        for point in points:
            pg.draw.line(self.map, self.Colors['orange'], pos, point)
            pg.draw.circle(self.map, self.Colors['orange'], point, 5)

    def is_wall_pixel(self, x, y, map_surface):
        if 0 <= x < map_surface.get_width() and 0 <= y < map_surface.get_height():
            return map_surface.get_at((int(x), int(y)))[:3] == self.Colors['black']
        return True
    
    def draw_start_end(self):
        rect_size = (73, 8)
        start_x = self.start[0] - rect_size[0] // 2
        start_y = self.start[1] - rect_size[1] // 2
        end_x = self.end[0] - rect_size[0] // 2
        end_y = self.end[1] - rect_size[1] // 2
        pg.draw.rect(self.map, self.Colors['green'], (start_x, start_y, *rect_size))
        pg.draw.rect(self.map, self.Colors['red'], (end_x, end_y, *rect_size))


class Robot:
    def __init__(self, start_pos, image_path, env):
        self.env = env
        self.x, self.y, self.theta = start_pos
        self.theta = math.radians(0)
        
        # van toc banhxe + (tuyentinh + goc) + dulieu cambien
        self.v1, self.v2 = 0, 0
        self.x_d, self.y_d, self.theta_d = 0, 0, 0
        self.sensor_data = [0] * 6
        self.points = []

        self.image = pg.transform.scale(pg.image.load(image_path), (32, 32))
        self.rotated = self.image
        self.rect = self.rotated.get_rect(center = (self.x, self.y))

        # trang thai
        self.crash = False
        self.cost_function = 0
        self.time = 0
        self.N = 0      # Dem so buoc
        
        # thong so neural network
        self.nn_input_size = 9          # 6 sensors + x, y, theta
        self.nn_hidden_size = 10
        self.nn_output_size = 2         # v1, v2
        
    def neural_network(self, inputs, W, V):
        net_h = W.T @ inputs
        y_h = np.tanh(net_h)
        return V.T @ y_h
    
    def move(self, dt, weights):
        nn_input = np.array([
            self.sensor_data[0], self.sensor_data[1], self.sensor_data[5],  # 3 giá trị cảm biến
            self.x, self.y, self.theta,  # Vị trí và hướng hiện tại
            self.env.end[0], self.env.end[1], self.env.end[2]  # Vị trí và hướng đích
        ])
        W = weights[:90].reshape(9, 10)      # trong so lop an
        V = weights[90:].reshape(10, 2)      # trong so lop ra
        output = self.neural_network(nn_input, W, V)
        self.v1, self.v2 = output[0] * 10, output[1] * 10
        
        # ma tran dong hoc
        inv_R = np.array([[np.cos(self.theta), -np.sin(self.theta), 0],
                          [np.sin(self.theta), np.cos(self.theta), 0],
                          [0, 0, 1]])
        l, r = 8, 3     # khoang cach tam banh + ban kinh banh xe
        inv_j1 = np.array([[0.5, -0.5], [0, 0], [-0.0625, -0.0625]])
        j2 = np.array([[r, 0], [0, r]])
        velocity = inv_R @ inv_j1 @ j2 @ np.array([[self.v1], [self.v2]])
        self.x_d, self.y_d, self.theta_d = velocity[0, 0], velocity[1, 0], velocity[2, 0]
        
        # cap nhat vi tri
        self.x += self.x_d * dt
        self.y += self.y_d * dt
        self.theta += self.theta_d * dt
        self.rotated = pg.transform.rotozoom(self.image, -math.degrees(self.theta), 1)
        self.rect = self.rotated.get_rect(center=(self.x, self.y))
        
    def update_sensors(self, map_surface):
        angles = [self.theta + i * np.pi / 3 for i in range(6)]  # 6 hướng cách đều
        edge_points, edge_distances = [], []
        for angle in angles:
            distance = 0
            edge_x, edge_y = int(self.x), int(self.y)
            while not self.env.is_wall_pixel(edge_x, edge_y, map_surface) and distance < 300:
                distance += 1
                edge_x = int(self.x + distance * math.cos(angle))
                edge_y = int(self.y + distance * math.sin(angle))
            edge_points.append((edge_x, edge_y))
            edge_distances.append(distance)
        self.sensor_data = edge_distances
        self.points = edge_points
        
    def check_crash(self, map_surface):
        edge_x, edge_y = int(self.x), int(self.y)
        if not (0 <= edge_x < map_surface.get_width() and 0 <= edge_y < map_surface.get_height()):
            self.crash = True
            self.cost_function += 2000
        elif self.env.is_wall_pixel(edge_x, edge_y, map_surface):
            self.crash = True
            self.cost_function += 3000
        if self.time > 10:
            self.crash = True
            self.cost_function += 4000
            
    def update_cost(self):
        ex = self.env.end[0] - self.x
        ey = self.env.end[1] - self.y
        etheta = self.env.end[2] - self.theta
        distance = math.sqrt(ex**2 + ey**2)
        self.cost_function += (0.5 * ex**2 + 0.5 * ey**2 + 0.5 * etheta**2)
        self.N += 1
        
    def draw(self, map_surface):
        if not self.crash:
            map_surface.blit(self.rotated, self.rect)
            self.env.draw_sensors((self.x, self.y), self.points)
            self.env.draw_robot_frame((self.x, self.y), self.theta)
            
class Main:
    def __init__(self, num_robots=20):
        pg.init()
        self.env = Environment()
        base_path = os.path.dirname(__file__)
        
        self.background = pg.transform.scale(pg.image.load(os.path.join(base_path, "..", "map", "m1.png")), (650, 650))        
        robot_image_path = os.path.join(base_path, "..", "access", "bot_right.png")
        robot_start_pos = (310, 77, 3*np.pi/2)
        self.robots = [Robot(robot_start_pos, robot_image_path, self.env) for _ in range(num_robots)]
        
        # thong so PSO
        self.pop_size = num_robots
        self.npar = 110  # 9*10 + 10*2 = 90 + 20 = 110
        self.min_max = [-5, 5]
        self.w, self.c1, self.c2 = 0.8, 1.5, 1.5
        self.max_iteration = 100
        
        self.P = np.random.uniform(self.min_max[0], self.min_max[1], (self.pop_size, self.npar))
        self.V = self.P * 0
        self.Pbest_fitness = np.full(self.pop_size, np.inf)
        self.Gbest_fitness = np.inf
        self.Pbest_position = np.zeros((self.pop_size, self.npar))  # Fixed np.zero to np.zeros
        self.Gbest_position = np.zeros(self.npar)
        
        self.running = True
        self.iteration = 0
        
    def reset_robot(self):
        for robot in self.robots:
            robot.x, robot.y, robot.theta = (310, 77, 3*np.pi/2)
            robot.v1, robot.v2 = 0, 0
            robot.x_d, robot.y_d, robot.theta_d = 0, 0, 0
            robot.time, robot.N, robot.cost_function = 0, 0, 0
            robot.crash = False
        # tao lai 10% robot voi Gbest_position
        if self.iteration > 1:
            for idx in range(self.pop_size // 10):
                self.P[idx] = self.Gbest_position.copy()
    
    def update_pso(self):
        for idx, robot in enumerate(self.robots):
            # Tránh chia cho 0
            J = robot.cost_function / max(robot.N, 1)
            if J < self.Pbest_fitness[idx]:
                self.Pbest_fitness[idx] = J
                self.Pbest_position[idx] = self.P[idx]
            if J < self.Gbest_fitness:
                self.Gbest_fitness = J
                self.Gbest_position = self.P[idx]
                
        # cap nhat van toc + vi tri
        r1, r2 = np.random.rand(self.pop_size, self.npar), np.random.rand(self.pop_size, self.npar)
        # c1, c2 = 1.5 - khuyen khich hoc tu kinh nghiem ca nhan + toan cuc
        self.V = (self.w * self.V + self.c1 * r1 * (self.Pbest_position - self.P) +
                  self.c2 * r2 * (self.Gbest_position - self.P))
        # gioi han van toc
        self.V = np.clip(self.V, -2, 2)
        self.P = self.P + self.V
        # gioi han vi tri
        self.P = np.clip(self.P, self.min_max[0], self.min_max[1])
        
    def run(self):
        clock = pg.time.Clock()
        while self.running and self.iteration < self.max_iteration:
            self.reset_robot()
            robot_available = self.pop_size
            # dt = 60 FPS
            while robot_available > 0 and self.running:
                dt = clock.tick(60) / 1000.0
                for event in pg.event.get():
                    if event.type == pg.QUIT:
                        self.running = False
                
                
                # cap nhat trang thai
                self.env.map.blit(self.background, (0, 0))
                self.env.draw_start_end()
                for idx, robot in enumerate(self.robots):
                    if not robot.crash:
                        robot.update_sensors(self.background)
                        robot.move(dt, self.P[idx])
                        robot.update_cost()
                        robot.check_crash(self.background)
                        robot.time += dt
                        if robot.crash:
                            robot_available -= 1
                        robot.draw(self.env.map)

                pg.display.flip()
            
            # cap nhat PSO
            self.update_pso()
            self.iteration += 1
            print(f"Iteration {self.iteration}: Gbest fitness = {self.Gbest_fitness:.2f}")
        
        pg.quit()

if __name__ == "__main__":
    main_program = Main(num_robots=20)
    main_program.run()