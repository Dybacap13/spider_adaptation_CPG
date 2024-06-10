#!/usr/bin/env python3
import numpy as np
import random
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from hexapod_msgs.srv import MoveFeetLearning
from hexapod_msgs.msg import MoveFeet
import pandas as pd
from rospkg import RosPack




class MultiArmedBandit:
    def __init__(self,):
        # _____________________________________________________________________________________
        # начальные параметры

        self.BEGIN_QUANTITY = {0: 3, 1: 0, 2: 0, 3: 2, 4: 0, 5: 2, 6: 1, 7: 1, 8: 1, 9: 0, 10: 0, 11: 3, 12: 4, 13: 0, 14: 1, 15: 2, 16: 0, 17: 3, 18: 3, 19: 2, 20: 1, 21: 53, 22: 1, 23: 0, 24: 0, 25: 5, 26: 2, 27: 0, 28: 2, 29: 3, 30: 3, 31: 0, 32: 1, 33: 4, 34: 2, 35: 5, 36: 4, 37: 4, 38: 2, 39: 2, 40: 0, 41: 1, 42: 47, 43: 1, 44: 2, 45: 0, 46: 3, 47: 3, 48: 2, 49: 1, 50: 2, 51: 0, 52: 1, 53: 3, 54: 2, 55: 2, 56: 1, 57: 2, 58: 1, 59: 1, 60: 1, 61: 1, 62: 1, 63: 0}
        self.BEGIN_Q = {0: 0.0, 1: -0.7533333333333333, 2: 0.0, 3: 0.0, 4: 2.61, 5: 0.0, 6: 0.935, 7: 2.13, 8: 1.91, 9: 2.4, 10: 0.0, 11: 0.0, 12: 1.28, 13: 2.76, 14: 0.0, 15: 0.72, 16: -2.77, 17: 0.0, 18: 1.49, 19: 0.9066666666666668, 20: 2.7, 21: 1.78, 22: 6.351509433962264, 23: 1.93, 24: 0.0, 25: 0.0, 26: 1.624, 27: 2.935, 28: 0.0, 29: 1.85, 30: -2.6866666666666665, 31: 0.0, 32: 1.54, 33: 2.415, 34: 1.035, 35: 3.338, 36: 1.37, 37: 1.505, 38: 1.015, 39: -1.09,
        40: 0.0, 41: 1.95, 42: 6.228936170212765, 43: -2.18, 44: 1.415, 45: 0.0, 46: -2.706666666666667, 47: -3.22, 48: 2.15, 49: 1.22, 50: 1.56, 51: 0.0, 52: 1.13, 53: -2.3866666666666667, 54: -2.235, 55: -2.62, 56: 1.68, 57: -2.245, 58: -3.09, 59: -2.83, 60: -3.46, 61: -2.43, 62: -1.65, 63: 0.0}
        self.BEGIN_REWARD = {0: -1.02, 1: 0.0, 2: 0.0, 3: 2.9699999999999998, 4: 0.0, 5: 0.81, 6: 2.13, 7: 1.91, 8: 2.4, 9: 0.0, 10: 0.0, 11: 0.9299999999999999, 12: 5.88, 13:
            0.0, 14: 0.72, 15: -3.5, 16: 0.0, 17: 1.23, 18: 0.47, 19: 3.17, 20: 1.78, 21: 6.99, 22: 1.93, 23: 0.0, 24: 0.0, 25: 1.0699999999999998, 26: 3.1100000000000003, 27: 0.0, 28: 1.67, 29: -3.21, 30: 3.73, 31: 0.0, 32: 1.54, 33: 1.1600000000000001, 34: 0.8200000000000001, 35: 1.33, 36: 0.8, 37: 0.99,
            38: 0.17999999999999994, 39: -0.5700000000000001, 40: 0.0, 41: 1.95, 42: 6.98, 43: -2.18, 44: 1.0899999999999999, 45: 0.0, 46: -2.89, 47: -3.71, 48: 2.54, 49: 1.22, 50: 1.29, 51: 0.0, 52: 1.13, 53: -2.55, 54: -2.36, 55: -3.26, 56: 1.68, 57: -2.68, 58: -3.09, 59: -2.83, 60: -3.46, 61: -2.43, 62:
            -1.65, 63: 0.0}

        self.options_gait_dict = self.gaitBulkhead([0,0,0,0,0,0])  # все варианты действий
        self.NUMBER = list(self.options_gait_dict.keys())[-1]
        self.quantity_gait_dict = self.BEGIN_QUANTITY.copy()
        self.q_function_dict = self.BEGIN_Q.copy()
        self.reward_dict = self.BEGIN_REWARD.copy()

        # _______________________________________________________________________________________

        self.df_quantity = pd.DataFrame( columns=list(self.quantity_gait_dict.keys()))
        self.df_q_function = pd.DataFrame(columns=list(self.quantity_gait_dict.keys()))
        self.df_reward = pd.DataFrame(columns=list(self.quantity_gait_dict.keys()))
        self.df_cumulative_reward = pd.DataFrame()
        self.pandasInit()

        self.reward_in_time_list = []
        self.cumulative_reward_list = []

        self.reward_odom_list = []
        self.cumulative_reward_odom_list = []

        self.reward_gyro_list = []
        self.cumulative_reward_gyro_list = []

        self.cumulative_reward_float = 0.0
        self.cumulative_reward_odom_float = 0.0
        self.cumulative_reward_gyro_float = 0.0
        #________________________________________________________________________________________


        # пути
        rp = RosPack()
        self.path = rp.get_path("adaptation_multi_armed_bandit")


        # параметры
        self.EPSILON_1_BEGIN = rospy.get_param("EPSILON_1")

        self.TIME_ITERATION = rospy.get_param("TIME_ITERATION")
        self.NAME_FILE_Q = rospy.get_param("NAME_FILE_Q")
        self.NAME_FILE_QUANTITY = rospy.get_param("NAME_FILE_QUANTITY")
        self.NAME_FILE_REWARD = rospy.get_param("NAME_FILE_REWARD_PY")
        self.NAME_FILE_CUMULATIVE_REWARD = rospy.get_param("NAME_FILE_CUMULATIVE_REWARD")


        # рос часть
        # zero_legs_pub = rospy.Publisher('/move_legs/zero_leg', Float32, queue_size=10)
        # zero_legs_reset_pub = rospy.Publisher('/move_legs/zero_leg_reset', Bool, queue_size=10)
        rospy.init_node('adaptation_multi_armed_bandit_node', anonymous=True)

        self.suffix = "0"
        self.iteration = 0
        self.last_option = -1;
        self.EPSILON_1 = self.EPSILON_1_BEGIN


        for i in range(0,10):
            self.suffix = str(i)
            self.algoritm()

            self.quantity_gait_dict = self.BEGIN_QUANTITY.copy()
            self.q_function_dict = self.BEGIN_Q.copy()
            self.reward_dict = self.BEGIN_REWARD.copy()

            self.pandasInit()


            self.cumulative_reward_float = 0.0
            self.cumulative_reward_odom_float = 0.0
            self.cumulative_reward_gyro_float = 0.0

            self.last_option = -1;
            self.EPSILON_1 = self.EPSILON_1_BEGIN

            self.reward_in_time_list = []
            self.cumulative_reward_list = []
            self.reward_odom_list = []
            self.cumulative_reward_odom_list = []

            self.reward_gyro_list = []
            self.cumulative_reward_gyro_list = []
            #zero_legs_reset_pub.publish(True)



    # создаёт словарь {Номер_походки: походка}
    # всего 64 походки ( от 0 - 63 )
    def gaitBulkhead(self, gait: list) -> dict:
        result = {}
        N = 0
        # первая
        for i_1 in range(0,2):
            if ( i_1 == 0 ):
                gait[0] = 0;
            else:
                gait[0] = 1;

            # вторая
            for i_2 in range(1,3):
                if ( i_2 == 1 ):
                    gait[1] = 0;
                else:
                    gait[1] = 1;

                #третья
                for i_3 in range(2,4):
                    if ( i_3 == 2 ):
                        gait[2] = 0;
                    else:
                        gait[2] = 1;

                    #четвертая
                    for i_4 in range(3,5):
                        if ( i_4 == 3 ):
                            gait[3] = 0;
                        else:
                            gait[3] = 1;

                        #пятая
                        for i_5 in range(4,6):
                            if ( i_5 == 4 ):
                                gait[4] = 0;
                            else:
                                gait[4] = 1;

                            #шестая
                            for i_6 in range(5,6):
                                for rew in range(0,2):
                                    gait[i_6] = rew;
                                    aaa = gait.copy()
                                    result[N] = aaa
                                    N = N + 1
                                    # print(gait)
                                    # result.append(gait)

        return result

    def initializationQuantityAndQ (self,size: int) -> dict:
        result = {}
        for i in range(0, size + 1):
            result[i] = 0
        return result

    def algoritm (self):
        print("TIME_ITERATION = ",int(self.TIME_ITERATION))
        anti_epsilon_list = []
        random_bool = False
        for episode in range(0, int(self.TIME_ITERATION)):
            if(episode % 2 == 0):

                if(episode == 0):
                    anti_epsilon_list.append(0)

                else:
                    anti_epsilon_list.append(anti_epsilon_list[-1]+1)
                    self.EPSILON_1 = int(self.EPSILON_1 - 1)
            random_bool = False

            print("episode = ",episode)
            print("anti_epsilon_list = ",anti_epsilon_list)

            epsilon_random = int(random.random() * 100)
            print("epsilon_random = ", epsilon_random)


            print("self.EPSILON_1 = ", self.EPSILON_1)

            for i in anti_epsilon_list:
                if epsilon_random == i:
                    random_bool = True

            if (random_bool == True):
                self.usage(episode)
            else:  # 10 %
                self.research(episode)

            self.pandasInit()
            self.printResult()
        self.pandasRead()

    def printResult(self):
        for indx in range(0,self.NUMBER):
            print(f"{indx}:   {self.quantity_gait_dict.get(indx)}   {self.reward_dict.get(indx)}   {self.q_function_dict.get(indx)}")

    def pandasInit(self):
        self.df_quantity.loc[ len(self.df_quantity.index )] = list(self.quantity_gait_dict.values())
        self.df_q_function.loc[ len(self.df_q_function.index )] = list(self.q_function_dict.values())
        self.df_reward.loc[ len(self.df_reward.index )] = list(self.reward_dict.values())

    def pandasRead(self):
        self.df_quantity.to_csv(self.path + self.NAME_FILE_QUANTITY + "_" + self.suffix + ".csv")
        self.df_q_function.to_csv(self.path + self.NAME_FILE_Q + "_" + self.suffix + ".csv")
        self.df_reward.to_csv(self.path + self.NAME_FILE_REWARD + "_" + self.suffix + ".csv")

        self.df_cumulative_reward["reward_in_time"] = self.reward_in_time_list
        self.df_cumulative_reward["reward_gyro"] = self.reward_gyro_list
        self.df_cumulative_reward["reward_odom"] = self.reward_odom_list

        self.df_cumulative_reward["cumulative_reward_gyro"] = self.cumulative_reward_gyro_list
        self.df_cumulative_reward["cumulative_reward_odom"] = self.cumulative_reward_odom_list
        self.df_cumulative_reward["cumulative_reward"] = self.cumulative_reward_list

        self.df_cumulative_reward.to_csv(self.path + self.NAME_FILE_CUMULATIVE_REWARD + "_" + self.suffix + ".csv")


    def research(self, episode):
        print("____WITH RANDOM____")
        gait_random = int(random.random() * list(self.options_gait_dict.keys())[-1])
        if gait_random == self.last_option:
            gait_random = int(random.random() * list(self.options_gait_dict.keys())[-1])

        print("gait_random = ", gait_random)
        print (self.options_gait_dict[gait_random])
        rospy.wait_for_service('/move_feet_learning')
        try:
            vector_legs = MoveFeet
            vector_legs.legs = self.options_gait_dict[gait_random]
            vector_legs.cmd_vel = 0.5
            reward_clienr = rospy.ServiceProxy('/move_feet_learning', MoveFeetLearning)
            resp = reward_clienr(vector_legs, episode, True)
        except rospy.ServiceException:
            print ("Service call failed")
        print (resp)

        self.reward_in_time_list.append(resp.reward_general)
        self.reward_odom_list.append(resp.reward_odometry)
        self.reward_gyro_list.append(resp.reward_gyroscope)


        self.cumulative_reward_float = self.cumulative_reward_float + resp.reward_general
        self.cumulative_reward_odom_float = self.cumulative_reward_odom_float + resp.reward_odometry
        self.cumulative_reward_gyro_float = self.cumulative_reward_gyro_float + resp.reward_gyroscope


        self.cumulative_reward_list.append(self.cumulative_reward_float)
        self.cumulative_reward_odom_list.append(self.cumulative_reward_odom_float)
        self.cumulative_reward_gyro_list.append(self.cumulative_reward_gyro_float)



        self.reward_dict[gait_random] = resp.reward_general
        self.quantity_gait_dict[gait_random] += 1
        self.q_function_dict[gait_random] = self.q_function_dict[gait_random] + (1/self.quantity_gait_dict[gait_random]) * (self.reward_dict[gait_random] - self.q_function_dict[gait_random])
        self.last_option = gait_random

    def usage(self, episode):
        print("____NOT RANDOM____")
        max = self.q_function_dict[0]
        max_index = 0
        for q in range(0,list(self.options_gait_dict.keys())[-1]):
            if (self.q_function_dict[q] > max and q != self.last_option ):
                max = self.q_function_dict[q]
                max_index = q
        print("max = ",max)
        print("max_index = ",max_index)
        print (self.options_gait_dict[max_index])
        rospy.wait_for_service('/move_feet_learning')
        try:
            vector_legs = MoveFeet
            vector_legs.legs = self.options_gait_dict[max_index]
            vector_legs.cmd_vel = 0.5
            reward_clienr = rospy.ServiceProxy('/move_feet_learning', MoveFeetLearning)
            resp = reward_clienr(vector_legs, episode, True)

        except rospy.ServiceException:
            print ("Service call failed")
        print (resp)

        self.reward_in_time_list.append(resp.reward_general)
        self.reward_odom_list.append(resp.reward_odometry)
        self.reward_gyro_list.append(resp.reward_gyroscope)


        self.cumulative_reward_float = self.cumulative_reward_float + resp.reward_general
        self.cumulative_reward_odom_float = self.cumulative_reward_odom_float + resp.reward_odometry
        self.cumulative_reward_gyro_float = self.cumulative_reward_gyro_float + resp.reward_gyroscope


        self.cumulative_reward_list.append(self.cumulative_reward_float)
        self.cumulative_reward_odom_list.append(self.cumulative_reward_odom_float)
        self.cumulative_reward_gyro_list.append(self.cumulative_reward_gyro_float)

        self.reward_dict[max_index] = resp.reward_general
        self.quantity_gait_dict[max_index] += 1
        self.q_function_dict[max_index] = self.q_function_dict[max_index] + (1/self.quantity_gait_dict[max_index]) * (self.reward_dict[max_index] - self.q_function_dict[max_index])
        self.last_option = max_index









if __name__ == "__main__":
    a = MultiArmedBandit()



