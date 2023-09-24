import math

class Reward:
    def __init__(self, verbose=False):
        # self.first_racingpoint_index = None
        self.first_racingpoint_index = 0
        self.verbose = verbose

    def reward_function(self, params):

        # Import package (needed for heading)

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            
            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list 
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):

            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time/current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[3.06664, 0.69989, 4.0, 0.03654],
[3.21372, 0.69357, 4.0, 0.0368],
[3.36169, 0.6893, 4.0, 0.03701],
[3.51032, 0.68657, 4.0, 0.03716],
[3.65944, 0.68518, 4.0, 0.03728],
[3.80869, 0.68499, 4.0, 0.03731],
[3.9577, 0.68593, 4.0, 0.03725],
[4.10629, 0.688, 4.0, 0.03715],
[4.25437, 0.69122, 4.0, 0.03703],
[4.40189, 0.69562, 4.0, 0.0369],
[4.54878, 0.70129, 4.0, 0.03675],
[4.69495, 0.7083, 4.0, 0.03659],
[4.84035, 0.71677, 4.0, 0.03641],
[4.9849, 0.7268, 4.0, 0.03622],
[5.12852, 0.73849, 3.79658, 0.03795],
[5.27111, 0.75197, 3.38028, 0.04237],
[5.41256, 0.76741, 3.03221, 0.04693],
[5.55265, 0.78511, 2.7016, 0.05227],
[5.69115, 0.80542, 2.38671, 0.05865],
[5.82783, 0.82863, 2.08713, 0.06642],
[5.96225, 0.85532, 1.84655, 0.07422],
[6.09384, 0.88621, 1.60581, 0.08418],
[6.22194, 0.92207, 1.39696, 0.09523],
[6.34568, 0.96381, 1.39696, 0.09348],
[6.46387, 1.01256, 1.39696, 0.09152],
[6.57482, 1.06969, 1.39696, 0.08933],
[6.67653, 1.13638, 1.3, 0.09356],
[6.76588, 1.21406, 1.3, 0.09108],
[6.83839, 1.3035, 1.3, 0.08856],
[6.8965, 1.40041, 1.3, 0.08693],
[6.94112, 1.50274, 1.3, 0.08587],
[6.96947, 1.60974, 1.3, 0.08515],
[6.97707, 1.71948, 1.40551, 0.07827],
[6.96702, 1.82873, 1.40551, 0.07806],
[6.94149, 1.93565, 1.40551, 0.07821],
[6.90175, 2.03894, 1.40551, 0.07875],
[6.84699, 2.13674, 1.40551, 0.07975],
[6.77532, 2.22592, 1.56141, 0.07327],
[6.69013, 2.30621, 1.74671, 0.06702],
[6.59411, 2.37815, 1.96264, 0.06113],
[6.48935, 2.44258, 2.22571, 0.05526],
[6.37761, 2.50053, 2.59093, 0.04858],
[6.26056, 2.55329, 3.04305, 0.04219],
[6.13955, 2.60203, 3.74908, 0.0348],
[6.01585, 2.648, 4.0, 0.03299],
[5.89082, 2.69257, 4.0, 0.03318],
[5.76067, 2.73919, 4.0, 0.03456],
[5.63058, 2.78629, 4.0, 0.03459],
[5.5006, 2.83412, 4.0, 0.03462],
[5.37081, 2.88295, 3.58884, 0.03864],
[5.2413, 2.93305, 3.58884, 0.03869],
[5.11223, 2.98473, 3.58884, 0.03874],
[4.9838, 3.03838, 3.58884, 0.03878],
[4.85635, 3.09451, 3.58884, 0.03881],
[4.73023, 3.15374, 3.58884, 0.03882],
[4.60596, 3.21695, 4.0, 0.03486],
[4.48296, 3.2828, 4.0, 0.03488],
[4.36104, 3.35081, 4.0, 0.0349],
[4.24006, 3.42061, 4.0, 0.03492],
[4.11988, 3.49191, 4.0, 0.03493],
[4.00046, 3.56448, 4.0, 0.03494],
[3.88179, 3.63809, 3.47383, 0.0402],
[3.76397, 3.71247, 2.99428, 0.04653],
[3.64724, 3.7873, 2.55629, 0.05424],
[3.53105, 3.86073, 2.20952, 0.06221],
[3.41419, 3.93239, 2.20952, 0.06204],
[3.29624, 4.00105, 2.20952, 0.06177],
[3.17677, 4.06545, 2.20952, 0.06142],
[3.0554, 4.12417, 2.20952, 0.06102],
[2.93169, 4.17515, 2.20952, 0.06056],
[2.80549, 4.21581, 2.46695, 0.05375],
[2.67785, 4.24822, 2.56094, 0.05142],
[2.5493, 4.27301, 2.47357, 0.05293],
[2.42021, 4.29067, 2.35598, 0.0553],
[2.29093, 4.30153, 2.13864, 0.06067],
[2.16175, 4.30562, 1.95082, 0.06625],
[2.03303, 4.30283, 1.74656, 0.07372],
[1.90519, 4.29292, 1.53607, 0.08347],
[1.7788, 4.27535, 1.42134, 0.08977],
[1.65459, 4.24957, 1.42134, 0.08926],
[1.53376, 4.21418, 1.42134, 0.08859],
[1.41797, 4.16786, 1.42134, 0.08774],
[1.30974, 4.10893, 1.42134, 0.0867],
[1.21287, 4.03538, 1.42134, 0.08557],
[1.13093, 3.94692, 1.46468, 0.08233],
[1.06435, 3.84609, 1.62451, 0.07438],
[1.01121, 3.73603, 1.78792, 0.06836],
[0.96999, 3.61869, 1.95188, 0.06372],
[0.93956, 3.49541, 2.13059, 0.0596],
[0.91891, 3.36729, 2.32352, 0.05585],
[0.90708, 3.23527, 2.51124, 0.05278],
[0.90334, 3.10018, 2.7333, 0.04944],
[0.90681, 2.9629, 2.89511, 0.04743],
[0.91698, 2.82419, 3.02951, 0.04591],
[0.93341, 2.68483, 3.13796, 0.04472],
[0.95571, 2.54557, 3.1355, 0.04498],
[0.98342, 2.40706, 2.97575, 0.04747],
[1.01626, 2.26986, 2.77801, 0.05078],
[1.05392, 2.13444, 2.53449, 0.05546],
[1.09624, 2.00121, 2.30368, 0.06068],
[1.14311, 1.87057, 2.07354, 0.06694],
[1.19482, 1.7431, 1.84356, 0.07462],
[1.25158, 1.61938, 1.61408, 0.08433],
[1.31382, 1.50015, 1.61408, 0.08333],
[1.38221, 1.38643, 1.61408, 0.08222],
[1.45757, 1.27943, 1.61408, 0.08108],
[1.54096, 1.18072, 1.61408, 0.08006],
[1.63386, 1.09253, 1.61408, 0.07936],
[1.7384, 1.01844, 1.90143, 0.06739],
[1.85098, 0.955, 2.10775, 0.06131],
[1.97002, 0.90067, 2.28389, 0.0573],
[2.09459, 0.85453, 2.4714, 0.05375],
[2.2239, 0.81579, 2.67564, 0.05045],
[2.35729, 0.78373, 2.89947, 0.04732],
[2.49419, 0.75767, 3.15191, 0.04421],
[2.63406, 0.73695, 3.45884, 0.04088],
[2.77639, 0.72086, 3.80768, 0.03762],
[2.92074, 0.70874, 4.0, 0.03621]]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]



        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0 # this is just for testing purposes
        if steps <= 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE


        ##
        DIR_MULTIPLE = 0.5
        # s = min(closest_index, second_closest_index)
        # t = max(closest_index, second_closest_index)
        # if s == 0 and t != 1:
        #     s, t = t, s
        # u = (t + 1) % len(racing_track)

        # ss = racing_track[s][:2]
        # tt = racing_track[t][:2]
        # uu = racing_track[u][:2]
        # angle1 = math.atan2(tt[1]-ss[1], tt[0]-ss[0])
        # angle2 = math.atan2(uu[1]-tt[1], uu[0]-tt[0])
        
        # angle_target = angle1
        # if angle_target < 0:
        #     angle_target += 360
        # angle_curr = heading
        # if angle_curr < 0:
        #     angle_curr += 360
        # angle_reward = - (abs(angle_curr - angle_target) / 30.0) ** 2
        # new_heading = heading + steering_angle
        # if new_heading < -180:
        #     new_heading += 360
        # elif new_heading > 180:
        #     new_heading -= 360
        # angle_reward += 1 - min(abs(angle2 - new_heading), abs(abs(angle2 - new_heading) - 360)) / 60.0

        # reward += angle_reward * DIR_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1 
        STANDARD_TIME = 13
        FASTEST_TIME = 8
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
            
            
        angle_reward = 1 - (direction_diff / 30.0) ** 2
        reward += angle_reward * DIR_MULTIPLE

        if direction_diff > 30:
            reward = 1e-3
            
        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3
            
        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1500 # should be adapted to track length and other rewards
        STANDARD_TIME = 13  # seconds (time that is easily done by model)
        FASTEST_TIME = 8  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                      (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward
        
        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = -1

        ####################### VERBOSE #######################
        
        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)
            
        #################### RETURN REWARD ####################
        
        # Always return a float value
        return float(reward)


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)