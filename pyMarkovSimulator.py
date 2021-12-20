import numpy as np
import matplotlib.pyplot as plt
import pyRobot

class pyMarkovSimulator(object):
    # constructor
    def __init__(self, nr_obs, seed):
        
        #map setup
        np.set_printoptions(precision=3)
        np.random.seed(seed)
        self.xdim = 12
        self.ydim = 12

        #probability setup
        self.sensor_p = 0.8
        self.turn_odom_p = 0.9
        self.move_odom_p = self.turn_odom_p

        #generate environment
        self.real_env = self.__generate_environment(self.xdim, self.ydim, nr_obs)

        #initiate probability space
        self.probability_space = np.zeros((4, self.xdim, self.ydim), dtype=np.float)
        not_walls_idx = np.where(self.real_env>0)
        
        for orient in self.probability_space:
            orient[not_walls_idx] += 1/(self.real_env[not_walls_idx].size*4)
        

        #initiate robot
        self.robot = pyRobot.pyRobot(1,1,0, self.real_env, p_fault_odometry=(1-self.move_odom_p), p_fault_sensor=(1-self.sensor_p))
        #self.robot.print_robot_position()
        #self.robot.sense()      

        #create plot for probability space
        self.pspace_list = list()
        self.rob_dot = list()
        self.__initiate_plot()
        

        pass

    def sense_step(self):
        # get sensor reading
        mask = self.robot.sense()

        # 1. compare reading with every position in map, 
        # 2. update probability space accordingly
        for y in range(self.ydim-2):
            for x in range(self.xdim-2):
                if(self.real_env[y+1,x+1]==1):
                    if (np.array_equal(mask, self.real_env[y:y+3, x:x+3])):
                        self.probability_space[:,y+1,x+1] = self.sensor_p * self.probability_space[:,y+1,x+1]
                    else:
                        self.probability_space[:,y+1,x+1] = (1-self.sensor_p) * self.probability_space[:,y+1,x+1]
        
        # check if sum of space is 1, if not -> normalize
        p_sum = np.sum(self.probability_space)
        if(p_sum != 1):
            self.probability_space = self.probability_space / p_sum

    def __shift_array(self, array, direction):
        if direction == 'up':
            shifted = np.concatenate((array[1],[array[1,0]]), axis=0)
        elif direction == 'down':
            shifted = np.concatenate(([array[3,0]], array[3]), axis=0)
        elif direction == 'left':
            shifted = np.concatenate((array[2].T,[array[2,0]]), axis=0).T
        elif direction == 'right':  
            shifted = np.concatenate(([array[0,0]], array[0].T), axis=0).T
        return shifted

    def __get_wall_block_mask(self, direction):
        if direction == 'up':
            piece = np.array([[0],[1]])
            next2wall_mask = np.zeros((12,12), dtype=int)
            for y in range(self.ydim-piece.shape[0]+1):
                for x in range(self.xdim-piece.shape[1]+1):
                    if (np.array_equal(piece, self.real_env[y:y+piece.shape[0], x:x+piece.shape[1]])):
                        next2wall_mask[y+1,x]=1
        elif direction == 'down':
            piece = np.array([[1],[0]])
            next2wall_mask = np.zeros((12,12), dtype=int)
            for y in range(self.ydim-piece.shape[0]+1):
                for x in range(self.xdim-piece.shape[1]+1):
                    if (np.array_equal(piece, self.real_env[y:y+piece.shape[0], x:x+piece.shape[1]])):
                        next2wall_mask[y,x]=1
        elif direction == 'right':
            piece = np.array([[1,0]])
            next2wall_mask = np.zeros((12,12), dtype=int)
            for y in range(self.ydim-piece.shape[0]+1):
                for x in range(self.xdim-piece.shape[1]+1):
                    if (np.array_equal(piece, self.real_env[y:y+piece.shape[0], x:x+piece.shape[1]])):
                        next2wall_mask[y,x]=1
        elif direction == 'left':  
            piece = np.array([[0,1]])
            next2wall_mask = np.zeros((12,12), dtype=int)
            for y in range(self.ydim-piece.shape[0]+1):
                for x in range(self.xdim-piece.shape[1]+1):
                    if (np.array_equal(piece, self.real_env[y:y+piece.shape[0], x:x+piece.shape[1]])):
                        next2wall_mask[y,x+1]=1
        return next2wall_mask

    def move_step(self, command):
        if command == 'turn left':
            self.robot.turn_left()
            temp = np.concatenate(([self.probability_space[3]],self.probability_space), axis=0)
            self.probability_space = (temp[:4] * self.turn_odom_p) + (temp[1:] * (1 - self.turn_odom_p))
            
        if command == 'turn right':
            self.robot.turn_right()
            temp = np.concatenate((self.probability_space,[self.probability_space[0]]), axis=0)
            self.probability_space = (temp[1:] * self.turn_odom_p) + (temp[:4] * (1 - self.turn_odom_p))
        
        # '→ (0)','↑ (90)', '← (180)', '↓ (270)'
        if command == 'forward':
            self.robot.move_forward()
            
            ps_shifted_up = self.__shift_array(self.probability_space, 'up')
            free_space = ps_shifted_up[1:]*self.real_env * self.move_odom_p             # free sapce movement
            wall_block_mask = self.__get_wall_block_mask('up')                       # in case env block movement
            blocked = self.probability_space[1]*wall_block_mask * self.move_odom_p   # blocked by wall
            odomerror = self.probability_space[1] * (1-self.move_odom_p)                # error from odometry
            self.probability_space[1] = free_space + blocked + odomerror                # sum it all up

            ps_shifted_down = self.__shift_array(self.probability_space, 'down')
            wall_block_mask = self.__get_wall_block_mask('down')                     # in case env block movement
            free_space = ps_shifted_down[:12]*self.real_env * self.move_odom_p          # free sapce movement
            blocked = self.probability_space[3]*wall_block_mask * self.move_odom_p   # blocked by wall
            odomerror = self.probability_space[3] * (1-self.move_odom_p)                # error from odometry
            self.probability_space[3] = free_space + blocked + odomerror                # sum it all up

            ps_shifted_right = self.__shift_array(self.probability_space, 'right')
            wall_block_mask = self.__get_wall_block_mask('right')                    # in case env block movement
            free_space = ps_shifted_right[:,:12]*self.real_env * self.move_odom_p      # free sapce movement
            blocked = self.probability_space[0]*wall_block_mask * self.move_odom_p   # blocked by wall
            odomerror = self.probability_space[0] * (1-self.move_odom_p)                # error from odometry
            self.probability_space[0] = free_space + blocked + odomerror                # sum it all up

            ps_shifted_left = self.__shift_array(self.probability_space, 'left')
            wall_block_mask = self.__get_wall_block_mask('left')                    # in case env block movement
            free_space = ps_shifted_left[:,1:]*self.real_env * self.move_odom_p      # free sapce movement
            blocked = self.probability_space[2]*wall_block_mask * self.move_odom_p   # blocked by wall
            odomerror = self.probability_space[2] * (1-self.move_odom_p)                # error from odometry
            self.probability_space[2] = free_space + blocked + odomerror                # sum it all up

    def update_plot(self):
        (x, y, th) = self.robot.get_robot_postion()
        self.rob_dot.pop(0).remove()
        case = int(th/90)
        if(case==0): robmarker = '>'
        elif(case==1):robmarker = '^'
        elif(case==2):robmarker = '<'
        elif(case==3):robmarker = 'v'
        self.rob_dot.append(self.axs_pplot[1,2].scatter(x, y, color='red', s=200, marker=robmarker))
        
        p_vmin = np.amin(self.probability_space)
        p_vmax = np.amax(self.probability_space)
        n = 0        
        for i in range(2):
            for j in range(2):
                self.pspace_list.pop(0).remove()
                self.pspace_list.append(self.axs_pplot[i,j].imshow(self.probability_space[n], vmin=p_vmin, vmax=p_vmax))
                n+=1

        plt.draw()
        plt.pause(0.01)

    def __initiate_plot(self):
        self.fig_pplot, self.axs_pplot = plt.subplots(nrows=2,ncols=3,figsize=(12,8))
        fontsize = 10
        titles = ('→ (0)','↑ (90)', '← (180)', '↓ (270)')
        plt.ion()
        plt.show()

        p_vmin = np.amin(self.probability_space)
        p_vmax = np.amax(self.probability_space)
        n = 0        
        for i in range(2):
            for j in range(2):
                self.axs_pplot[i,j].set_title(titles[n])
                self.pspace_list.append(self.axs_pplot[i,j].imshow(self.probability_space[n], vmin=p_vmin, vmax=p_vmax))
                n+=1

        (x, y, th) = self.robot.get_robot_postion()
        case = int(th/90)
        if(case==0): robmarker = '>'
        elif(case==1):robmarker = '^'
        elif(case==2):robmarker = '<'
        elif(case==3):robmarker = 'v'
        self.axs_pplot[1,2].imshow(self.real_env, cmap='gray')
        self.rob_dot.append(self.axs_pplot[1,2].scatter(x, y, color='red', s=200, marker=robmarker))

    def __generate_environment(self, xdim, ydim, nr_obs):
        real_environment = np.zeros((xdim, ydim), dtype=np.int16)
        real_environment[1:11,1:11] = 1
        
        all_obs = np.random.randint(1,ydim,(nr_obs, 2))
        
        for obs in all_obs:
            real_environment.itemset((obs[0],obs[1]), 0)
              
        print("Generated envrionment:\n", real_environment)

        return (real_environment)

 
# Private variables:
