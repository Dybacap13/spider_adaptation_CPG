#include <hexapod_msgs/CheckGait.h>
#include <hexapod_msgs/MoveFeetLearning.h>
#include <matrix.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <ctime>
#include <fstream>
#include <iostream>

class AdaptationAlgorithm {
 public:
  AdaptationAlgorithm(ros::NodeHandle nh_);
  void algoritm();

 private:
  int NUMBER_NEURON;
  double VOLTAGE_TRESHOLD;
  double DELTA;
  double LEARNING_RATE;
  double LEAK_RATE;
  double TIME_ITERATION;
  int ITERATION_SPIKE_IN;
  std::vector<double> THRESHOLD_GYROSCOPE;
  double THRESHOLD_COORDINATES;
  std::vector<double> ODOM_TEST;

  std::vector<double> VOLTAGE_TRESHOLD_TEST;
  std::vector<double> LEAK_RATE_TEST;
  std::string NAME_FILE_REWARD;
  std::string NAME_FILE_LEGS;
  std::string NAME_FILE_VOLT;
  std::string NAME_FILE_WEIGH;
  std::string NAME_FILE_RANDOM;

  ros::NodeHandle nh;
  ros::Publisher zero_leg_pub;
  ros::Publisher zero_leg_reset_pub;
  ros::Subscriber reward_sub;
  ros::ServiceClient client_;

  ros::ServiceClient client_check;

  double reward_sum_odom = 0.0;
  double reward_sum_gyro = 0.0;
  double reward_sum = 0.0;
  void rewardCallback(std_msgs::Float32 msg);
  void falseInVectorLegs();
  void writerParamReward(double reward_, int exp);
  void writerParamLegs(hexapod_msgs::MoveFeet vector_legs_);
  void writerInitParamTimeData(int i);
  void writerParamVolt(Matrix vector_legs_);
  void writerParamWeigh(Matrix weigh);
  void writerParamElipson(int elipson, int time);

  hexapod_msgs::MoveFeet vector_legs;
  double reward_gyroscope = 0.0;
  double reward_odometry = 0.0;
  double reward = 0.0;

  std::string path_to_pack = ros::package::getPath("adaptation_algorithm");
};

void AdaptationAlgorithm::writerInitParamTimeData(int iner) {
  std::ofstream out;
  time_t now = time(0);
  char *dt = ctime(&now);
  std::string file = path_to_pack + NAME_FILE_REWARD;
  out.open(file, std::ios::app);
  if (out.is_open()) {
    out << "____________________________________" << std::endl;
    out << dt << std::endl;
    out << "" << std::endl;
    out << "PARAMETERS" << std::endl;
    out << "NUMBER_NEURON = " << NUMBER_NEURON << std::endl;
    out << "ODOM_TEST = " << ODOM_TEST[iner] << std::endl;
    out << "VOLTAGE_TRESHOLD = " << VOLTAGE_TRESHOLD_TEST[iner] << std::endl;
    out << "LEARNING_RATE = " << LEARNING_RATE << std::endl;
    out << "LEAK_RATE_TEST = " << LEAK_RATE_TEST[iner] << std::endl;
    out << "TIME_ITERATION = " << TIME_ITERATION << std::endl;
    out << "ITERATION_SPIKE_IN = " << ITERATION_SPIKE_IN << std::endl;
    out << "THRESHOLD_COORDINATES = " << THRESHOLD_COORDINATES << std::endl;
    out << "THRESHOLD_GYROSCOPE = ";
    for (int i = 0; i < 3; i++) {
      out << THRESHOLD_GYROSCOPE[i] << "  ";
    }
    out << " " << std::endl;
  }
  out.close();
  file = path_to_pack + NAME_FILE_LEGS;
  out.open(file, std::ios::app);
  if (out.is_open()) {
    out << "____________________________________" << std::endl;
    out << dt << std::endl;
    out << "" << std::endl;
    out << "PARAMETERS" << std::endl;
    out << "NUMBER_NEURON = " << NUMBER_NEURON << std::endl;
    out << "ODOM_TEST = " << ODOM_TEST[iner] << std::endl;
    out << "VOLTAGE_TRESHOLD = " << VOLTAGE_TRESHOLD_TEST[iner] << std::endl;
    out << "LEARNING_RATE = " << LEARNING_RATE << std::endl;
    out << "LEAK_RATE_TEST = " << LEAK_RATE_TEST[iner] << std::endl;
    out << "TIME_ITERATION = " << TIME_ITERATION << std::endl;
    out << "ITERATION_SPIKE_IN = " << ITERATION_SPIKE_IN << std::endl;
    out << "THRESHOLD_COORDINATES = " << THRESHOLD_COORDINATES << std::endl;
    out << "THRESHOLD_GYROSCOPE = ";
    for (int i = 0; i < 3; i++) {
      out << THRESHOLD_GYROSCOPE[i] << "  ";
    }
    out << " " << std::endl;
  }
  out.close();

  file = path_to_pack + NAME_FILE_VOLT;
  out.open(file, std::ios::app);
  if (out.is_open()) {
    out << "____________________________________" << std::endl;
    out << dt << std::endl;
    out << "" << std::endl;
    out << "PARAMETERS" << std::endl;
    out << "NUMBER_NEURON = " << NUMBER_NEURON << std::endl;
    out << "ODOM_TEST = " << ODOM_TEST[iner] << std::endl;
    out << "VOLTAGE_TRESHOLD = " << VOLTAGE_TRESHOLD_TEST[iner] << std::endl;
    out << "LEARNING_RATE = " << LEARNING_RATE << std::endl;
    out << "LEAK_RATE_TEST = " << LEAK_RATE_TEST[iner] << std::endl;
    out << "TIME_ITERATION = " << TIME_ITERATION << std::endl;
    out << "ITERATION_SPIKE_IN = " << ITERATION_SPIKE_IN << std::endl;
    out << "THRESHOLD_COORDINATES = " << THRESHOLD_COORDINATES << std::endl;
    out << "THRESHOLD_GYROSCOPE = ";
    for (int i = 0; i < 3; i++) {
      out << THRESHOLD_GYROSCOPE[i] << "  ";
    }
    out << " " << std::endl;
  }
  out.close();

  file = path_to_pack + NAME_FILE_WEIGH;
  out.open(file, std::ios::app);
  if (out.is_open()) {
    out << "____________________________________" << std::endl;
    out << dt << std::endl;
    out << "" << std::endl;
    out << "PARAMETERS" << std::endl;
    out << "NUMBER_NEURON = " << NUMBER_NEURON << std::endl;
    out << "ODOM_TEST = " << ODOM_TEST[iner] << std::endl;
    out << "VOLTAGE_TRESHOLD = " << VOLTAGE_TRESHOLD_TEST[iner] << std::endl;
    out << "LEARNING_RATE = " << LEARNING_RATE << std::endl;
    out << "LEAK_RATE_TEST = " << LEAK_RATE_TEST[iner] << std::endl;
    out << "TIME_ITERATION = " << TIME_ITERATION << std::endl;
    out << "ITERATION_SPIKE_IN = " << ITERATION_SPIKE_IN << std::endl;
    out << "THRESHOLD_COORDINATES = " << THRESHOLD_COORDINATES << std::endl;
    out << "THRESHOLD_GYROSCOPE = ";
    for (int i = 0; i < 3; i++) {
      out << THRESHOLD_GYROSCOPE[i] << "  ";
    }
    out << " " << std::endl;
  }
  out.close();

  file = path_to_pack + NAME_FILE_RANDOM;
  out.open(file, std::ios::app);
  if (out.is_open()) {
    out << "____________________________________" << std::endl;
    out << dt << std::endl;
    out << "" << std::endl;
    out << "PARAMETERS" << std::endl;
    out << "NUMBER_NEURON = " << NUMBER_NEURON << std::endl;
    out << "ODOM_TEST = " << ODOM_TEST[iner] << std::endl;
    out << "VOLTAGE_TRESHOLD = " << VOLTAGE_TRESHOLD_TEST[iner] << std::endl;
    out << "LEARNING_RATE = " << LEARNING_RATE << std::endl;
    out << "LEAK_RATE_TEST = " << LEAK_RATE_TEST[iner] << std::endl;
    out << "TIME_ITERATION = " << TIME_ITERATION << std::endl;
    out << "ITERATION_SPIKE_IN = " << ITERATION_SPIKE_IN << std::endl;
    out << "THRESHOLD_COORDINATES = " << THRESHOLD_COORDINATES << std::endl;
    out << "THRESHOLD_GYROSCOPE = ";
    for (int i = 0; i < 3; i++) {
      out << THRESHOLD_GYROSCOPE[i] << "  ";
    }
    out << " " << std::endl;
  }
  out.close();
}

void AdaptationAlgorithm::writerParamElipson(int elipson, int time) {
  std::ofstream out;

  std::string file = path_to_pack + NAME_FILE_RANDOM;
  out.open(file, std::ios::app);
  if (out.is_open()) {
    std::cout << "writer elipson" << std::endl;
    out << time << " ";
    out << elipson << std::endl;
  }
  out.close();
}

void AdaptationAlgorithm::writerParamReward(double reward_, int exp) {
  std::ofstream out;
  std::string file = path_to_pack + NAME_FILE_REWARD;
  out.open(file, std::ios::app);
  if (out.is_open()) {
    std::cout << "writer reward" << std::endl;
    out << exp << "    ";
    out << reward_gyroscope << "    ";
    out << reward_odometry << "    ";
    out << reward_ << "    ";
    out << reward_sum_gyro << "    ";
    out << reward_sum_odom << "    ";
    out << reward_sum << std::endl;
  }
  out.close();
}

void AdaptationAlgorithm::writerParamWeigh(Matrix weigh) {
  std::ofstream out;

  std::string file = path_to_pack + NAME_FILE_WEIGH;
  out.open(file, std::ios::app);
  if (out.is_open()) {
    std::cout << "writer weigh" << std::endl;
    for (int i = 0; i < 36; i++) {
      out << weigh.get_data()[i] << "    ";
    }
    out << " " << std::endl;
  }
  out.close();
}

void AdaptationAlgorithm::writerParamVolt(Matrix vector_legs_) {
  std::ofstream out;

  std::string file = path_to_pack + NAME_FILE_VOLT;
  out.open(file, std::ios::app);
  if (out.is_open()) {
    std::cout << "writer volt" << std::endl;
    for (int i = 0; i < 6; i++) {
      out << vector_legs_.get_data()[i] << "    ";
    }

    out << " " << std::endl;
  }
  out.close();
}

void AdaptationAlgorithm::writerParamLegs(hexapod_msgs::MoveFeet vector_legs_) {
  std::ofstream out;

  std::string file = path_to_pack + NAME_FILE_LEGS;
  out.open(file, std::ios::app);
  if (out.is_open()) {
    std::cout << "writer legs" << std::endl;
    for (int i = 0; i < vector_legs_.legs.size(); i++) {
      if (vector_legs_.legs[i] == true)
        out << 1 << "    ";
      else
        out << 0 << "    ";
    }

    out << " " << std::endl;
  }
  out.close();
}

AdaptationAlgorithm::AdaptationAlgorithm(ros::NodeHandle nh_) : nh(nh_) {
  reward_sub =
      nh_.subscribe("/reward", 5, &AdaptationAlgorithm::rewardCallback, this);

  ros::param::get("NUMBER_NEURON", NUMBER_NEURON);
  ros::param::get("VOLTAGE_TRESHOLD", VOLTAGE_TRESHOLD);
  ros::param::get("DELTA", DELTA);
  ros::param::get("LEARNING_RATE", LEARNING_RATE);
  ros::param::get("LEAK_RATE", LEAK_RATE);
  ros::param::get("TIME_ITERATION", TIME_ITERATION);
  ros::param::get("ITERATION_SPIKE_IN", ITERATION_SPIKE_IN);
  ros::param::get("NAME_FILE_REWARD", NAME_FILE_REWARD);
  ros::param::get("NAME_FILE_LEGS", NAME_FILE_LEGS);
  ros::param::get("THRESHOLD_GYROSCOPE", THRESHOLD_GYROSCOPE);
  ros::param::get("THRESHOLD_COORDINATES", THRESHOLD_COORDINATES);
  ros::param::get("ODOM_TEST", ODOM_TEST);
  ros::param::get("VOLTAGE_TRESHOLD_TEST", VOLTAGE_TRESHOLD_TEST);
  ros::param::get("LEAK_RATE_TEST", LEAK_RATE_TEST);
  ros::param::get("NAME_FILE_VOLT", NAME_FILE_VOLT);
  ros::param::get("NAME_FILE_WEIGH", NAME_FILE_WEIGH);
  ros::param::get("NAME_FILE_RANDOM", NAME_FILE_RANDOM);
  client_ =
      nh_.serviceClient<hexapod_msgs::MoveFeetLearning>("/move_feet_learning");

  client_check = nh_.serviceClient<hexapod_msgs::CheckGait>("/check_gyroscope");

  zero_leg_pub = nh_.advertise<std_msgs::Float32>("/move_legs/zero_leg", 1000);
  zero_leg_reset_pub =
      nh_.advertise<std_msgs::Bool>("/move_legs/zero_leg_reset", 1000);

  falseInVectorLegs();
  // writerInitParamTimeData();
}
void AdaptationAlgorithm::rewardCallback(std_msgs::Float32 msg) {
  reward = msg.data;
}

void AdaptationAlgorithm::algoritm() {
  int total_time = 0;
  bool spike_one_yes = false;

  // количество вариантов тестирования ( по порогу)
  for (int iter = 0; iter < 7; iter++) {
    // writerInitParamTimeData(iter);

    // количество попыток тестирования с одним порогом
    // ЕСЛИ МЕНЯЕШЬ ТО МЕНЯЙ И В ЗАПИСИ НАГРАДЫ!
    for (int i = 0; i < 3; i++) {
      // задаём веса нейронов - которые работают
      Matrix weight_ideal = Matrix(6, 6);
      weight_ideal.get_data()[0] = 0.0;
      weight_ideal.get_data()[1] = 1.0;
      weight_ideal.get_data()[2] = 0.0;
      weight_ideal.get_data()[3] = 1.0;
      weight_ideal.get_data()[4] = 0.0;
      weight_ideal.get_data()[5] = 1.0;
      weight_ideal.get_data()[6] = 1.0;
      weight_ideal.get_data()[7] = 0.0;
      weight_ideal.get_data()[8] = 1.0;
      weight_ideal.get_data()[9] = 0.0;
      weight_ideal.get_data()[10] = 1.0;
      weight_ideal.get_data()[11] = 0.0;
      weight_ideal.get_data()[12] = 0.0;
      weight_ideal.get_data()[13] = 1.0;
      weight_ideal.get_data()[14] = 0.0;
      weight_ideal.get_data()[15] = 1.0;
      weight_ideal.get_data()[16] = 0.0;
      weight_ideal.get_data()[17] = 1.0;
      weight_ideal.get_data()[18] = 1.0;
      weight_ideal.get_data()[19] = 0.0;
      weight_ideal.get_data()[20] = 1.0;
      weight_ideal.get_data()[21] = 0.0;
      weight_ideal.get_data()[22] = 1.0;
      weight_ideal.get_data()[23] = 0.0;
      weight_ideal.get_data()[24] = 0.0;
      weight_ideal.get_data()[25] = 1.0;
      weight_ideal.get_data()[26] = 0.0;
      weight_ideal.get_data()[27] = 1.0;
      weight_ideal.get_data()[28] = 0.0;
      weight_ideal.get_data()[29] = 1.0;
      weight_ideal.get_data()[30] = 1.0;
      weight_ideal.get_data()[31] = 0.0;
      weight_ideal.get_data()[32] = 1.0;
      weight_ideal.get_data()[33] = 0.0;
      weight_ideal.get_data()[34] = 1.0;
      weight_ideal.get_data()[35] = 0.0;
      weight_ideal.get_data()[36] = 1.0;

      Matrix weight_in = Matrix(1, 6);
      Matrix weight_gyro = Matrix(1, 6);

      //**********************

      weight_gyro.get_data()[0] = -0.2;
      weight_gyro.get_data()[1] = -0.2;
      weight_gyro.get_data()[2] = -0.2;
      weight_gyro.get_data()[3] = -0.2;
      weight_gyro.get_data()[4] = -0.2;
      weight_gyro.get_data()[5] = -0.2;

      weight_in.get_data()[0] = 0.8;
      weight_in.get_data()[1] = 0.8;
      weight_in.get_data()[2] = 0.8;
      weight_in.get_data()[3] = 0.8;
      weight_in.get_data()[4] = 0.8;
      weight_in.get_data()[5] = 0.8;

      //**********************

      Matrix spike_sgp_last_memory =
          Matrix(1, 6);  // это спайки прошлой  итерации
      Matrix spike_sgp_last_last_memory =
          Matrix(1, 6);  // это спайки прошлой прошлой итерации
      spike_sgp_last_memory.zero();
      spike_sgp_last_last_memory.zero();

      // инициализация потенциала
      Matrix voltage = Matrix(1, 6);
      Matrix voltage_last = Matrix(1, 6);
      voltage.zero();
      voltage_last.zero();

      // инициализая спайков

      Matrix spike_sgp = Matrix(6, 6);
      Matrix spike_sgp_last = Matrix(6, 6);
      Matrix spike_in = Matrix(1, 6);
      Matrix spike_gyro = Matrix(1, 6);
      spike_sgp.zero();

      spike_in.ones();
      // spike_in.zero();
      spike_sgp_last.zero();

      //****
      // Это нужно, чтобы алгоритм с первого эпизода пошёл
      for (int i = 0; i < NUMBER_NEURON * NUMBER_NEURON; i = i + 2) {
        spike_sgp_last.get_data()[i] = 1.0;
      }
      //****

      spike_gyro.zero();

      // занулить главную диагональ
      weight_ideal.eye_zero();
      spike_sgp.eye_zero();
      spike_sgp_last.eye_zero();

      // ***************************************************
      //                   БЛОК ВЫВОДА
      std::cout << "Random weight CPG" << std::endl;
      weight_ideal.output();
      std::cout << "Random weight in = " << std::endl;
      weight_in.output();
      std::cout << "Random weight gyro = " << std::endl;
      weight_gyro.output();
      std::cout << "Init spike GCP" << std::endl;
      spike_sgp.output();
      spike_sgp_last.output();
      std::cout << "Init spike in = " << std::endl;
      spike_in.output();
      std::cout << "Init spike gyro = " << std::endl;
      spike_gyro.output();
      std::cout << "Init voltage" << std::endl;
      voltage.output();
      //*******************************************************

      // Алгоритм
      // Сумма вклада всех соседних нейронов
      double sum_contribution_neighbours = 0.0;

      // ток
      double current_neuron;
      double dicress = 1;
      int epsilon = 10;
      reward_sum = 0.0;
      reward_sum_gyro = 0.0;
      reward_sum_odom = 0.0;
      std_msgs::Bool msg_reset;
      msg_reset.data = false;
      zero_leg_reset_pub.publish(msg_reset);

      for (unsigned int episode = 0; episode < TIME_ITERATION; episode++) {
        std::cout << " " << std::endl;
        std::cout << "reward_sum / 4 > dicress = " << reward_sum / 4
                  << std::endl;
        std::cout << "dicress = " << dicress << std::endl;
        std::cout << "epsilon =  " << epsilon << std::endl;

        if (reward_sum / 4 >= dicress && episode != 0) {
          epsilon = epsilon + 2;
          dicress = dicress + 2;
          std::cout << "VVERX epsilon" << std::endl;
        } else {
          if (episode > 20) {
            epsilon = epsilon - 2;
            dicress = dicress - 2;
            std::cout << "VNIZ epsilon" << std::endl;
            if (epsilon < 2) {
              epsilon = 2;
            }
          }
        }

        std::cout << "dicress = " << dicress << std::endl;
        std::cout << "epsilon =  " << epsilon << std::endl;

        std::cout << " " << std::endl;
        // для отслежевания
        std::cout << "TOTAL TIME" << total_time << std::endl;
        total_time++;

        //********************************************
        //                ЖАДНАЯ СТРАТЕГИЯ
        srand(time(0));
        int random_koeff = (rand() % (epsilon));
        // Если выпало 2 - Ю попали в случайный эпизод
        std::cout << " random_koeff =  " << random_koeff << std::endl;
        writerParamElipson(epsilon, episode);

        if (random_koeff == -100) {
          std::cout << " +++ RANDOM VECTOR LEGS ++++ " << std::endl;
          writerParamElipson(epsilon, episode);
          for (int neuron = 0; neuron < NUMBER_NEURON; neuron++) {
            // srand(time(0));
            int k = (rand() % 2);
            vector_legs.legs[neuron] = k;
            spike_one_yes = true;
            for (unsigned int neighbour = 0; neighbour < NUMBER_NEURON;
                 neighbour++) {
              spike_sgp.get_data()[neuron + NUMBER_NEURON * neighbour] = k;
            }
          }
        } else {
          // 1) ПРОВЕРЯЕМ НА ВСПЫШКИ НЕЙРОНЫ
          for (unsigned int neuron = 0; neuron < NUMBER_NEURON; neuron++) {
            // считает вклад в ток от соседних нейронов
            for (unsigned int neighbour = 0; neighbour < NUMBER_NEURON;
                 neighbour++) {
              sum_contribution_neighbours =
                  weight_ideal.get_data()[neighbour + NUMBER_NEURON * neuron] *
                      spike_sgp_last
                          .get_data()[neighbour + NUMBER_NEURON * neuron] +
                  sum_contribution_neighbours;
            }

            // считаем ток
            current_neuron =
                weight_in.get_data()[neuron] * spike_in.get_data()[neuron] +
                weight_gyro.get_data()[neuron] * spike_gyro.get_data()[neuron] +
                sum_contribution_neighbours;

            std::cout << " current_neuron =  " << current_neuron << std::endl;

            // считаем потенциал
            voltage.get_data()[neuron] =
                (voltage_last.get_data()[neuron] / LEAK_RATE_TEST[iter]) +
                current_neuron;
            std::cout << " voltage.get_data()[neuron] =  "
                      << voltage.get_data()[neuron] << std::endl;
            std::cout << " voltage_LAST.get_data()[neuron] =  "
                      << voltage_last.get_data()[neuron] << std::endl;

            // если прошли порог
            if (voltage.get_data()[neuron] > VOLTAGE_TRESHOLD_TEST[iter]) {
              std::cout << "------------SPIKE NEURON = " << neuron << std::endl;
              spike_one_yes = true;
              for (unsigned int neighbour = 0; neighbour < NUMBER_NEURON;
                   neighbour++) {
                spike_sgp.get_data()[neuron + NUMBER_NEURON * neighbour] = 1.0;
              }
              voltage.get_data()[neuron] = 0.0;
              vector_legs.legs[neuron] = true;
            }

            sum_contribution_neighbours = 0.0;
          }
        }

        // 2) ОБНОВЛЯЕМ ВЕСА

        // запрос
        hexapod_msgs::MoveFeetLearning srv;
        srv.request.legs = vector_legs;
        srv.request.current_time = episode;
        srv.request.need_adaptation = true;

        if (!client_.call(srv)) {
          std::cout << "Failed to call service /move_feet_learning"
                    << std::endl;
          if (!client_.call(srv)) {
            std::cout << "Failed to call service /move_feet_learning"
                      << std::endl;
            return;
          }
        }

        // получаем награду от сервиса
        reward = srv.response.reward_general;
        reward_odometry = srv.response.reward_odometry;
        reward_gyroscope = srv.response.reward_gyroscope;
        std::string resust_str = srv.response.result;
        std::cout << "srv.response.reward_general = "
                  << srv.response.reward_general << std::endl;
        std::cout << "srv.response.reward_odometry = "
                  << srv.response.reward_odometry << std::endl;
        std::cout << "srv.response.reward_gyroscope = "
                  << srv.response.reward_gyroscope << std::endl;

        // записываем в память текущее состояние потенциала нейрона
        for (unsigned int i = 0; i < NUMBER_NEURON; i++) {
          voltage_last.get_data()[i] = voltage.get_data()[i];
        }

        // сохраняем спайки прошлой ПРОШЛОЙ итерации для ПАМЯТИ
        for (unsigned int i = 0; i < 6; i++) {
          spike_sgp_last_last_memory.get_data()[i] =
              spike_sgp_last_memory.get_data()[i];
        }

        // сохраняем спайки прошлой  итерации для ПАМЯТИ
        for (unsigned int i = 0; i < 6; i++) {
          spike_sgp_last_memory.get_data()[i] = spike_sgp.get_data()[i];
        }

        // здесь мы сохраняем спайки  прошлой итерации ДЛЯ ТОКА ПОТОМ ЗАНУЛИМ
        // ДИАГОНАЛЬ
        for (unsigned int i = 0; i < spike_sgp.get_col() * spike_sgp.get_row();
             i++) {
          spike_sgp_last.get_data()[i] = spike_sgp.get_data()[i];
        }

        // ********************
        // БЛОК ОБНОВЛЕНИЯ ВЕСОВ
        // обновляем веса нейронов
        for (unsigned int neuron = 0; neuron < NUMBER_NEURON; neuron++) {
          if (spike_sgp_last_last_memory.get_data()[neuron] == 1.0) {
            for (unsigned int weigth = 0; weigth < NUMBER_NEURON; weigth++) {
              if (spike_sgp.get_data()[weigth] == 1.0) {
                // srand(time(0));
                int k = (rand() % 2);
                std::cout << "RANDOM" << k << std::endl;
                weight_ideal.get_data()[neuron + weigth * 6] =
                    weight_ideal.get_data()[neuron + weigth * NUMBER_NEURON] +
                    LEARNING_RATE * k * reward;
                if (weight_ideal.get_data()[neuron + weigth * 6] < 0)
                  weight_ideal.get_data()[neuron + weigth * NUMBER_NEURON] =
                      0.0;
                if (weight_ideal.get_data()[neuron + weigth * 6] > 1)
                  weight_ideal.get_data()[neuron + weigth * NUMBER_NEURON] =
                      1.0;
              }
            }
          }
        }

        // Обновляем веса нейрона ИН
        for (unsigned int neuron = 0; neuron < NUMBER_NEURON; neuron++) {
          if (spike_sgp.get_data()[neuron] == 1.0) {
            // srand(time(0));
            int t = (rand() % 2);
            weight_in.get_data()[neuron] =
                weight_in.get_data()[neuron] + LEARNING_RATE * t * reward;
            if (weight_in.get_data()[neuron] < 0)
              weight_in.get_data()[neuron] = 0.0;
            if (weight_in.get_data()[neuron] > 1)
              weight_in.get_data()[neuron] = 1.0;
          }
        }

        std::cout << " weight_in in time =  " << episode << std::endl;
        weight_in.output();

        // обнуляем прошлые спайки
        spike_gyro.zero();
        // Обновляем веса нейрона ГИРО
        if (resust_str == "balance lost max") {
          for (unsigned int neuron = 0; neuron < NUMBER_NEURON; neuron++) {
            if (spike_sgp.get_data()[neuron] == 1.0) {
              spike_gyro.get_data()[neuron] = 1.0;
            }
          }
          ros::Duration(2.0).sleep();
        }
        //**********************************

        if (spike_one_yes) {
          //**************************
          // НОРМАЛИЗАЦИЯ

          // Веса нейронов СНН
          // ищем минимум и максимум
          double min = weight_ideal.get_data()[0];
          double max = weight_ideal.get_data()[0];
          for (int i = 0; i < NUMBER_NEURON * NUMBER_NEURON; i++) {
            if (weight_ideal.get_data()[i] > max)
              max = weight_ideal.get_data()[i];
            if (weight_ideal.get_data()[i] < min)
              min = weight_ideal.get_data()[i];
          }
          // нормализация минмакс
          for (int i = 0; i < NUMBER_NEURON * NUMBER_NEURON; i++) {
            weight_ideal.get_data()[i] =
                (0.0) + (((weight_ideal.get_data()[i] - min) / (max - min)) *
                         (1.0 - (0.0)));
          }

          // Нормализация ИН
          // ищем минимум и максимум
          min = weight_in.get_data()[0];
          max = weight_in.get_data()[0];
          for (int i = 0; i < 6; i++) {
            if (weight_in.get_data()[i] > max) max = weight_in.get_data()[i];
            if (weight_in.get_data()[i] < min) min = weight_in.get_data()[i];
          }
          std::cout << " min in time =  " << min << std::endl;
          std::cout << " max in time =  " << min << std::endl;
          if (min != max) {
            for (int i = 0; i < 6; i++) {
              weight_in.get_data()[i] =
                  (0.0) + (((weight_in.get_data()[i] - min) / (max - min)) *
                           (1.0 - (0.0)));
            }
          }
        }

        // занулить главную диагональ
        weight_ideal.eye_zero();
        spike_sgp.zero();
        spike_sgp_last.eye_zero();
        spike_one_yes = false;
        // last_reward_gyro = reward_gyroscope;

        if (episode > 45) {
          std::cout << "_________________ ZERO LEGS _________________"
                    << std::endl;
          double summ_zero_leg = 0.0;
          double min_summ_zero_leg = 100000.0;
          int zero_legs;
          for (int neuron = 0; neuron < NUMBER_NEURON; neuron++) {
            summ_zero_leg = 0.0;
            // std::cout << "______________" << std::endl;
            for (int neighbour = 0; neighbour < NUMBER_NEURON; neighbour++) {
              summ_zero_leg = summ_zero_leg +
                              weight_ideal.get_data()[neighbour + neuron * 6];
              // std::cout << "weight_ideal.get_data() = "<<
              // weight_ideal.get_data()[neighbour + neuron * 6] << std::endl;
              // std::cout << "summ_zero_leg = "<< summ_zero_leg << std::endl;
            }
            // std::cout << "___" << std::endl;
            // std::cout << "neuron = "<< neuron << std::endl;
            // std::cout << "summ_zero_leg = "<< summ_zero_leg << std::endl;
            // std::cout << "min_summ_zero_leg = "<< min_summ_zero_leg <<
            // std::endl; std::cout << "_____________" << std::endl;
            if (summ_zero_leg < min_summ_zero_leg) {
              min_summ_zero_leg = summ_zero_leg;
              zero_legs = neuron;
            }
          }
          std_msgs::Float32 msg;
          msg.data = zero_legs;
          zero_leg_pub.publish(msg);
          std::cout << "zero_legs = " << zero_legs << std::endl;

          std::cout << "__________________________________" << std::endl;
        }

        //******************************************************************
        //                             БЛОК ВЫВОДА
        std::cout << " Weight in SNN time =  " << episode << std::endl;
        weight_ideal.output();
        std::cout << " weight_gyro in time =  " << episode << std::endl;
        weight_gyro.output();
        std::cout << " weight_in in time =  " << episode << std::endl;
        weight_in.output();
        std::cout << " voltage in time =  " << episode << std::endl;
        voltage.output();
        std::cout << " voltage_last in time =  " << episode << std::endl;
        voltage_last.output();
        std::cout << " spike_sgp in time =  " << episode << std::endl;
        spike_sgp.output();
        std::cout << " spike_sgp_last in time =  " << episode << std::endl;
        spike_sgp_last.output();
        std::cout << " spike_sgp_LAST_LAST in time =  " << episode << std::endl;
        spike_sgp_last_last_memory.output();
        std::cout << " spike_gyro in time =  " << episode << std::endl;
        spike_gyro.output();
        std::cout << " spike_in in time =  " << episode << std::endl;
        spike_in.output();
        std::cout << "reward = " << reward << std::endl;
        std::cout << "resust_str = " << resust_str << std::endl;
        reward_sum = reward_sum + reward;

        reward_sum_odom = reward_sum_odom + reward_odometry;
        reward_sum_gyro = reward_sum_gyro + reward_gyroscope;
        std::cout << "reward_sum = " << reward_sum << std::endl;
        //******************************************************************

        writerParamReward(reward, (iter * 3 + i));
        writerParamLegs(vector_legs);
        writerParamVolt(voltage);
        writerParamWeigh(weight_ideal);
        falseInVectorLegs();
      }
    }
  }
}

void AdaptationAlgorithm::falseInVectorLegs() {
  for (int i = 0; i < NUMBER_NEURON; i++) vector_legs.legs[i] = false;
  vector_legs.cmd_vel = 0.5;
}
