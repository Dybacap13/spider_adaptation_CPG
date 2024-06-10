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
        self.options_gait_dict = self.gaitBulkhead([0,0,0,0,0,0])  # все варианты действий
        self.NUMBER = list(self.options_gait_dict.keys())[-1]
        self.quantity_gait_dict = self.initializationQuantityAndQ (self.NUMBER) # количество использованных походо
        self.q_function_dict = self.initializationQuantityAndQ(list(self.options_gait_dict.keys())[-1]) # КУ функция
        self.reward_dict = self.initializationQuantityAndQ(list(self.options_gait_dict.keys())[-1]) # награда

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
        self.path = rp.get_path("multi_armed_bandit")


        # параметры
        self.EPSILON_1_BEGIN = rospy.get_param("EPSILON_1")

        self.TIME_ITERATION = rospy.get_param("TIME_ITERATION")
        self.NAME_FILE_Q = rospy.get_param("NAME_FILE_Q")
        self.NAME_FILE_QUANTITY = rospy.get_param("NAME_FILE_QUANTITY")
        self.NAME_FILE_REWARD = rospy.get_param("NAME_FILE_REWARD")
        self.NAME_FILE_CUMULATIVE_REWARD = rospy.get_param("NAME_FILE_CUMULATIVE_REWARD")


        # рос часть
        zero_legs_pub = rospy.Publisher('/move_legs/zero_leg', Float32, queue_size=10)
        zero_legs_reset_pub = rospy.Publisher('/move_legs/zero_leg_reset', Bool, queue_size=10)
        rospy.init_node('multi_armed_bandit_node', anonymous=True)

        self.suffix = "0"
        self.iteration = 0
        self.last_option = -1;
        self.EPSILON_1 = self.EPSILON_1_BEGIN


        for i in range(0,22):
            self.suffix = str(i)
            self.algoritm()

            self.quantity_gait_dict = self.initializationQuantityAndQ (self.NUMBER) # количество использованных походо
            self.q_function_dict = self.initializationQuantityAndQ(list(self.options_gait_dict.keys())[-1]) # КУ функция
            self.reward_dict = self.initializationQuantityAndQ(list(self.options_gait_dict.keys())[-1]) # награда


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



        # zero_legs_pub.publish(1.0)
        # zero_legs_reset_pub.publish(True)





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
            resp = reward_clienr(vector_legs, episode, False)
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
            resp = reward_clienr(vector_legs, episode, False)

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



