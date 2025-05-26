import pygame as pg
import os
import math
import numpy as np
import json

class Environment:
    Colors = {'black': (0, 0, 0), 'white': (255, 255, 255), 'red': (255, 0, 0),
              'green': (0, 255, 0), 'blue': (0, 0, 255), 'orange': (255, 165, 0)}
    
    def __init__(self, dimensions=(555, 650)):
        pg.display.set_caption("Last Bot")
        self.width, self.height = dimensions
        self.map = pg.display.set_mode(dimensions)
        self.begin_map, self.end_map = (251, 3), (305, 647)
        self.trail_points = []
    
    def draw_trails(self, position):
        for i in range(len(self.trail_points) - 1):
            pg.draw.line(self.map, self.Colors['red'], self.trail_points[i], self.trail_points[i + 1])
        if len(self.trail_points) > 300:
            self.trail_points.pop(0)
        if position:
            self.trail_points.append(position)
    
    def draw_frames(self, position, rotation):
        axis_length = 40
        center_x, center_y = position
        x_axis_endpoint = (center_x + axis_length * math.cos(rotation), center_y + axis_length * math.sin(rotation))
        y_axis_endpoint = (center_x + axis_length * math.cos(rotation + math.pi/2), center_y + axis_length * math.sin(rotation + math.pi/2))
        pg.draw.line(self.map, self.Colors['blue'], position, x_axis_endpoint, 2)
        pg.draw.line(self.map, self.Colors['green'], position, y_axis_endpoint, 2)
    
    def draw_sensors(self, position, points):
        for point in points:
            pg.draw.line(self.map, self.Colors['orange'], position, point, 2)
            pg.draw.circle(self.map, self.Colors['orange'], point, 2)
    
    def draw_begin_map(self):
        rectangle_size = (70, 6)
        begin_x, begin_y = self.begin_map[0] - rectangle_size[0] // 2, self.begin_map[1] - rectangle_size[1] // 2
        return pg.Rect(begin_x, begin_y, *rectangle_size), self.Colors['green']
    
    def draw_end_map(self):
        rectangle_size = (70, 6)
        end_x, end_y = self.end_map[0] - rectangle_size[0] // 2, self.end_map[1] - rectangle_size[1] // 2
        return pg.Rect(end_x, end_y, *rectangle_size), self.Colors['red']

class Robot:
    def __init__(self, start_position, image_file_path, environment):
        self.environment = environment
        self.x_position, self.y_position, self.theta_position = start_position
        self.x_linear, self.y_linear, self.theta_linear = 0, 0, 0
        self.left_wheel, self.right_wheel = 0, 0
        self.sensor_reading = [0, 0, 0, 0, 0, 0]
        self.sensor_points = []
        self.robot_image = pg.transform.scale(pg.image.load(image_file_path), (21, 21))
        self.rotate_image = self.robot_image
        self.image_rectangle = self.robot_image.get_rect(center=(self.x_position, self.y_position))
        self.time_stuck = 0
        self.time_toGoal = 0
        self.elapsed_time = 0
        self.total_rotation = 0
        self.goal_radius = 20
        self.has_crashed = False
        self.has_reached_goal = False
        self.explored_cells = set()
        self.dead_ends = set()
    
    def draw_bot(self, map_surface):
        map_surface.blit(self.rotate_image, self.image_rectangle)
    
    def move_bot(self, delta_times):
        if self.has_reached_goal:  # Dừng robot nếu đã đạt đích
            self.x_linear, self.y_linear, self.theta_linear = 0, 0, 0
            return
        
        radius, length = 3, 8
        inverse_rotation = np.array([[math.cos(self.theta_position), -math.sin(self.theta_position), 0],
                                     [math.sin(self.theta_position), math.cos(self.theta_position), 0],
                                     [0, 0, 1]])
        inverse_jacobian1 = np.array([[0.5, -0.5],
                                      [0, 0],
                                      [-1/(2*length), -1/(2*length)]])
        jacobian2 = np.array([[radius, 0],
                              [0, radius]])
        velocity_wheels = np.array([[self.left_wheel],
                                    [self.right_wheel]])
        forward_kinematics = inverse_rotation @ inverse_jacobian1 @ jacobian2 @ velocity_wheels
        self.x_linear, self.y_linear, self.theta_linear = forward_kinematics[0, 0], forward_kinematics[1, 0], forward_kinematics[2, 0]
        
        self.x_position += self.x_linear * delta_times
        self.y_position += self.y_linear * delta_times
        self.theta_position += self.theta_linear * delta_times
        self.x_position = max(0, min(self.x_position, self.environment.width - 1))
        self.y_position = max(0, min(self.y_position, self.environment.height - 1))
        self.rotate_image = pg.transform.rotozoom(self.robot_image, -math.degrees(self.theta_position), 1)
        self.image_rectangle = self.rotate_image.get_rect(center=(self.x_position, self.y_position))
        grid_cell = (int(self.x_position//10), int(self.y_position//10))
        self.explored_cells.add(grid_cell)
        if (abs(self.x_position - self.environment.end_map[0]) < self.goal_radius and 
            abs(self.y_position - self.environment.end_map[1]) < self.goal_radius and
            not self.has_reached_goal):
            self.has_reached_goal = True
            self.time_toGoal = self.elapsed_time
            print("Robot has reached the goal!")
        if abs(self.x_linear) < 0.1 and abs(self.y_linear) < 0.1:
            self.time_stuck += delta_times
        else:
            self.time_stuck = 0
        self.total_rotation += abs(self.theta_linear) * delta_times
    
    def scan_environment(self, track_copy):
        # sensor: 6 cảm biến quang học đặt ở các góc 60 độ (0, 60, 120, 180, 240, 300 độ)
        sensor_angles = [self.theta_position, self.theta_position+math.pi/3, self.theta_position+2*math.pi/3,
                         self.theta_position+math.pi, self.theta_position+4*math.pi/3, self.theta_position+5*math.pi/3]
        sensor_endpoints, sensor_distances = [], []
        for sensor_angle in sensor_angles:
            distance = 0
            x_current, y_current = int(self.x_position), int(self.y_position)
            while (0 <= x_current < self.environment.width and 0 <= y_current < self.environment.height):
                if track_copy.get_at((x_current, y_current)) == self.environment.Colors['black']:
                    break
                distance += 1
                x_current = int(self.x_position + distance * math.cos(sensor_angle))
                y_current = int(self.y_position + distance * math.sin(sensor_angle))
                if distance > 300:
                    break
            sensor_endpoints.append((x_current, y_current))
            sensor_distances.append(distance)
        self.sensor_reading = sensor_distances
        self.sensor_points = sensor_endpoints
    
    def check_crash(self, track_copy):
        x_current, y_current = int(self.x_position), int(self.y_position)
        if not (0 <= x_current < track_copy.get_width() and 0 <= y_current < track_copy.get_height()):
            self.has_crashed = True
        if track_copy.get_at((x_current, y_current)) == self.environment.Colors['black']:
            self.has_crashed = True
        if self.elapsed_time > 60:
            self.has_crashed = True

class NeuralNetwork:
    @staticmethod
    def activation_function(X):
        return np.tanh(X)
    
    @staticmethod
    def forward(X, input_to_hidden_weights, hidden_to_output_weights):
        net_h = input_to_hidden_weights.T @ X
        y_h = NeuralNetwork.activation_function(net_h)
        output = hidden_to_output_weights.T @ y_h
        return output

class PSO:
    def __init__(self, population_size, hidden_neurons, parameter_bounds, inertia_weight, cognitive_coeff, social_coeff, max_iterations):
        self.population_size = population_size
        self.hidden_neurons = hidden_neurons
        self.num_parameters = (7 * hidden_neurons[0]) + (hidden_neurons[0] * 2)  # 90
        self.parameter_bounds = parameter_bounds
        self.inertia_weight = inertia_weight
        self.cognitive_coefficient = cognitive_coeff
        self.social_coefficient = social_coeff
        self.max_iterations = max_iterations
        self.current_iteration = 0
        self.particles = np.random.uniform(parameter_bounds[0], parameter_bounds[1], (population_size, self.num_parameters))
        self.velocities = np.zeros((population_size, self.num_parameters))
        self.personal_best_fitness = np.inf * np.ones(population_size)
        self.global_best_fitness = np.inf
        self.personal_best_positions = np.zeros((population_size, self.num_parameters))
        self.global_best_position = np.zeros(self.num_parameters)
        self.no_improvement_count = 0
        self.global_best_path = []
        
    def update(self, robots, fitness_hyperparameters, robot_paths):
        alpha, beta, delta, sensor_weight, epsilon = fitness_hyperparameters
        
        self.current_iteration += 1
        
        for i, robot in enumerate(robots):
            avg_sensor_distance = sum(robot.sensor_reading) / len(robot.sensor_reading) if robot.sensor_reading else 0
            fitness_score = (
                alpha * int(robot.has_crashed) +
                beta * robot.total_rotation +
                delta * max(0, robot.time_stuck - 2.0) +
                sensor_weight * (300 - avg_sensor_distance) -
                epsilon * len(robot.explored_cells)
            )
            
            if fitness_score < self.personal_best_fitness[i]:
                self.personal_best_fitness[i] = fitness_score
                self.personal_best_positions[i] = self.particles[i]
            
            if robot.has_reached_goal:
                self.global_best_fitness = fitness_score
                self.global_best_position = self.particles[i]
                self.global_best_path = robot_paths[i]
                self.no_improvement_count = 0
            elif fitness_score < self.global_best_fitness:
                self.global_best_fitness = fitness_score
                self.global_best_position = self.particles[i]
                self.global_best_path = robot_paths[i]
                self.no_improvement_count = 0
            else:
                self.no_improvement_count += 1
                
        self.velocities = (self.inertia_weight * self.velocities +
                           self.cognitive_coefficient * np.random.rand(self.population_size, self.num_parameters) * (self.personal_best_positions - self.particles) +
                           self.social_coefficient * np.random.rand(self.population_size, self.num_parameters) * (self.global_best_position - self.particles))
        self.particles = self.particles + self.velocities
        return self.global_best_fitness

class Simulation:
    def __init__(self, environment, robots, pso, fitness_hyperparameters, map_surface):
        self.environment = environment
        self.robots = robots
        self.pso = pso
        self.fitness_hyperparameters = fitness_hyperparameters
        self.map_surface = map_surface
        self.clock = pg.time.Clock()
        self.start = (self.environment.begin_map[0]+9, self.environment.begin_map[1]+3)
        
    def reset_robots(self):
        for robot in self.robots:
            robot.x_position, robot.y_position, robot.theta_position = self.start[0], self.start[1], 0
            robot.x_linear, robot.y_linear, robot.theta_linear = 0, 0, 0
            robot.time_stuck = 0
            robot.time_toGoal = 0
            robot.elapsed_time = 0
            robot.fitness_value = 0
            robot.total_rotation = 0
            robot.has_crashed = False
            robot.has_reached_goal = False
            robot.explored_cells.clear()
            robot.dead_ends.clear()
            robot.scan_environment(self.map_surface)
            robot.check_crash(self.map_surface)
            
    def run(self):
        running = True
        iteration = 0
        success = False
        while running and iteration < self.pso.max_iterations and not success:
            self.reset_robots()
            robot_available = len(self.robots)
            robot_paths = [[] for _ in range(len(self.robots))]
            any_reached_goal = False
            while robot_available > 0 and running and not any_reached_goal:
                delta_time = 1.0 / 60.0  # Sử dụng delta_time cố định 60 FPS
                for event in pg.event.get():
                    if event.type == pg.QUIT:
                        running = False
                for i, robot in enumerate(self.robots):
                    if not robot.has_crashed and not robot.has_reached_goal:
                        normalized_sensor_reading = [min(reading/300, 1.0) for reading in robot.sensor_reading]
                        normalized_theta = max(min(robot.theta_position / (2 * math.pi), 1.0), -1.0)
                        neural_network_input = np.array([normalized_sensor_reading[0], normalized_sensor_reading[1], normalized_sensor_reading[2],
                                                         normalized_sensor_reading[3], normalized_sensor_reading[4], normalized_sensor_reading[5],
                                                         normalized_theta/(2*math.pi)])
                        input_to_hidden_weights = self.pso.particles[i, :70].reshape(7, 10)
                        hidden_to_output_weights = self.pso.particles[i, 70:].reshape(10, 2)
                        wheel_velocities = NeuralNetwork.forward(neural_network_input, input_to_hidden_weights, hidden_to_output_weights)
                        robot.left_wheel, robot.right_wheel = wheel_velocities[0]*3, wheel_velocities[1]*3
                        robot.move_bot(delta_time)
                        robot.elapsed_time += delta_time
                        robot.check_crash(self.map_surface)
                        if robot.has_crashed:
                            robot_available -= 1
                        robot.scan_environment(self.map_surface)
                        # Lưu dữ liệu đường đi mỗi bước
                        robot_paths[i].append({
                            "x_position": robot.x_position,
                            "y_position": robot.y_position,
                            "theta_position": robot.theta_position,
                            "left_wheel": robot.left_wheel,
                            "right_wheel": robot.right_wheel,
                            "elapsed_time": robot.elapsed_time
                        })
                any_reached_goal = any(robot.has_reached_goal for robot in self.robots)
                self.environment.map.blit(self.map_surface, (0, 0))
                for robot in self.robots:
                    if not robot.has_crashed:
                        robot.draw_bot(self.environment.map)
                        self.environment.draw_frames((robot.x_position, robot.y_position), robot.theta_position)
                        self.environment.draw_sensors((robot.x_position, robot.y_position), robot.sensor_points)
                        self.environment.draw_trails((robot.x_position, robot.y_position))
                        begin_rect, begin_color = self.environment.draw_begin_map()
                        end_rect, end_color = self.environment.draw_end_map()
                        pg.draw.rect(self.environment.map, begin_color, begin_rect)
                        pg.draw.rect(self.environment.map, end_color, end_rect)
                pg.display.flip()
            best_fitness = self.pso.update(self.robots, self.fitness_hyperparameters, robot_paths)
            print(f"Iteration {iteration}: {best_fitness}")
            if any_reached_goal:
                success = True
                print("A robot has successfully reached the goal! Stopping simulation...")
            iteration += 1
        if self.pso.global_best_path:
            with open("best_path.json", "w") as f:
                json.dump(self.pso.global_best_path, f, indent=4)
        if self.pso.global_best_path:
            self.replay_best_path()
        else:
            print("No best path found for replay!")
        pg.quit()
        
    def replay_best_path(self):
        print("Replaying best path with 1 robot...")
        pg.init()
        self.environment.map = pg.display.set_mode((self.environment.width, self.environment.height))
        
        # Lấy trạng thái ban đầu từ global_best_path
        if self.pso.global_best_path:
            initial_state = self.pso.global_best_path[0]
            start_pos = (initial_state["x_position"], initial_state["y_position"], initial_state["theta_position"])
        else:
            print("Error: global_best_path is empty!")
            pg.quit()
            return
        
        robot = Robot(start_pos, os.path.join(os.path.dirname(__file__), "..", "access", "bot_right.png"), self.environment)
        print(f"Robot starting position in replay: ({robot.x_position}, {robot.y_position}, {robot.theta_position})")
        
        # Sử dụng trọng số tốt nhất từ PSO để điều khiển robot trong replay
        input_to_hidden_weights = self.pso.global_best_position[:70].reshape(7, 10)
        hidden_to_output_weights = self.pso.global_best_position[70:].reshape(10, 2)
        
        replay_data = []
        running = True
        while running:
            delta_time = 1.0 / 60.0  # Sử dụng delta_time cố định 60 FPS
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    running = False
            
            # Sử dụng cảm biến và mạng nơ-ron để điều khiển robot
            robot.scan_environment(self.map_surface)
            normalized_sensor_reading = [min(reading/300, 1.0) for reading in robot.sensor_reading]
            normalized_theta = max(min(robot.theta_position / (2 * math.pi), 1.0), -1.0)
            neural_network_input = np.array([normalized_sensor_reading[0], normalized_sensor_reading[1], normalized_sensor_reading[2],
                                             normalized_sensor_reading[3], normalized_sensor_reading[4], normalized_sensor_reading[5],
                                             normalized_theta/(2*math.pi)])
            wheel_velocities = NeuralNetwork.forward(neural_network_input, input_to_hidden_weights, hidden_to_output_weights)
            robot.left_wheel, robot.right_wheel = wheel_velocities[0]*3, wheel_velocities[1]*3
            robot.move_bot(delta_time)
            robot.elapsed_time += delta_time
            robot.check_crash(self.map_surface)
            
            # Lưu dữ liệu đường đi khi replay
            replay_data.append({
                "x_position": robot.x_position,
                "y_position": robot.y_position,
                "theta_position": robot.theta_position,
                "left_wheel": robot.left_wheel,
                "right_wheel": robot.right_wheel,
                "elapsed_time": robot.elapsed_time
            })
            
            self.environment.map.blit(self.map_surface, (0, 0))
            robot.draw_bot(self.environment.map)
            self.environment.draw_frames((robot.x_position, robot.y_position), robot.theta_position)
            self.environment.draw_sensors((robot.x_position, robot.y_position), robot.sensor_points)
            self.environment.draw_trails((robot.x_position, robot.y_position))
            begin_rect, begin_color = self.environment.draw_begin_map()
            end_rect, end_color = self.environment.draw_end_map()
            pg.draw.rect(self.environment.map, begin_color, begin_rect)
            pg.draw.rect(self.environment.map, end_color, end_rect)
            pg.display.flip()
            
            if robot.has_crashed or robot.has_reached_goal or robot.elapsed_time > 60:  # Tăng thời gian tối đa
                running = False
        
        with open("replay_path.json", "w") as f:
            json.dump(replay_data, f, indent=4)
        print("Replay data saved to replay_path.json")
        pg.quit()

def main():
    pg.init()
    base_dir = os.path.dirname(__file__)
    map_path = os.path.join(base_dir, "..", "map", "m8.png")
    robot_img_path = os.path.join(base_dir, "..", "access", "bot_right.png")
    my_map = pg.transform.scale(pg.image.load(map_path), (555, 650))
    dims = my_map.get_size()
    environment = Environment(dims)
    environment.map.blit(my_map, (0, 0))
    pso_config = {
        "population_size": 100,
        "hidden_neurons": [10],
        "parameter_bounds": [-5, 5],
        "inertia_weight": 0.9,
        "cognitive_coeff": 0.5,
        "social_coeff": 0.5,
        "max_iterations": 10000
    }
    fitness_hyperparameters = [100, 0.1, 40, 0.5, 2.0]  # alpha, beta, delta, sensor_weight, epsilon
    pso = PSO(**pso_config)
    robots = [Robot((environment.begin_map[0]+9, environment.begin_map[1]+3, 0), robot_img_path, environment) for _ in range(pso_config["population_size"])]
    simulation = Simulation(environment, robots, pso, fitness_hyperparameters, my_map)
    simulation.run()

if __name__ == "__main__":
    main()