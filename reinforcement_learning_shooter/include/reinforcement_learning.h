#include <hexapod_msgs/MoveFeetLearning.h>
#include <matrix.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <ctime>
#include <fstream>
#include <iostream>

class ReinforcementLearning {
 public:
  ReinforcementLearning(ros::NodeHandle nh_);
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

  std::vector<int> RANDOM_SEED_TEST_SNN;
  std::vector<int> RANDOM_SEED_TEST_IN;
  std::vector<int> RANDOM_SEED_TEST_GYRO;
  std::string NAME_FILE_REWARD;
  std::string NAME_FILE_LEGS;
  std::string NAME_FILE_VOLT;
  std::string NAME_FILE_WEIGH;

  ros::NodeHandle nh;
  ros::Subscriber reward_sub;
  ros::ServiceClient client_;
  void rewardCallback(std_msgs::Float32 msg);
  void falseInVectorLegs();
  void writerParamReward(double reward_);
  void writerParamLegs(hexapod_msgs::MoveFeet vector_legs_);
  void writerInitParamTimeData(int i);
  void writerParamVolt(Matrix vector_legs_);
  void writerParamWeigh(Matrix weigh);

  hexapod_msgs::MoveFeet vector_legs;
  double reward_gyroscope = 0.0;
  double reward_odometry = 0.0;
  double reward = 0.0;

  std::string path_to_pack =
      ros::package::getPath("reinforcement_learning_shooter");
};

void ReinforcementLearning::writerInitParamTimeData(int iner) {
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
    out << "RANDOM_SEED_TEST_SNN = " << RANDOM_SEED_TEST_SNN[iner] << std::endl;
    out << "RANDOM_SEED_TEST_GYRO = " << RANDOM_SEED_TEST_GYRO[iner]
        << std::endl;
    out << "RANDOM_SEED_TEST_IN = " << RANDOM_SEED_TEST_IN[iner] << std::endl;

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
    out << "RANDOM_SEED_TEST_SNN = " << RANDOM_SEED_TEST_SNN[iner] << std::endl;
    out << "RANDOM_SEED_TEST_GYRO = " << RANDOM_SEED_TEST_GYRO[iner]
        << std::endl;
    out << "RANDOM_SEED_TEST_IN = " << RANDOM_SEED_TEST_IN[iner] << std::endl;
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
    out << "RANDOM_SEED_TEST_SNN = " << RANDOM_SEED_TEST_SNN[iner] << std::endl;
    out << "RANDOM_SEED_TEST_GYRO = " << RANDOM_SEED_TEST_GYRO[iner]
        << std::endl;
    out << "RANDOM_SEED_TEST_IN = " << RANDOM_SEED_TEST_IN[iner] << std::endl;
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
    out << "RANDOM_SEED_TEST_SNN = " << RANDOM_SEED_TEST_SNN[iner] << std::endl;
    out << "RANDOM_SEED_TEST_GYRO = " << RANDOM_SEED_TEST_GYRO[iner]
        << std::endl;
    out << "RANDOM_SEED_TEST_IN = " << RANDOM_SEED_TEST_IN[iner] << std::endl;
    out << "THRESHOLD_COORDINATES = " << THRESHOLD_COORDINATES << std::endl;
    out << "THRESHOLD_GYROSCOPE = ";
    for (int i = 0; i < 3; i++) {
      out << THRESHOLD_GYROSCOPE[i] << "  ";
    }
    out << " " << std::endl;
  }
  out.close();
}

void ReinforcementLearning::writerParamReward(double reward_) {
  std::ofstream out;
  std::string file = path_to_pack + NAME_FILE_REWARD;
  out.open(file, std::ios::app);
  if (out.is_open()) {
    std::cout << "writer reward" << std::endl;
    out << reward_gyroscope << "    ";
    out << reward_odometry << "    ";
    out << reward_ << std::endl;
  }
  out.close();
}

void ReinforcementLearning::writerParamWeigh(Matrix weigh) {
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

void ReinforcementLearning::writerParamVolt(Matrix vector_legs_) {
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

void ReinforcementLearning::writerParamLegs(
    hexapod_msgs::MoveFeet vector_legs_) {
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

ReinforcementLearning::ReinforcementLearning(ros::NodeHandle nh_) : nh(nh_) {
  reward_sub =
      nh_.subscribe("/reward", 5, &ReinforcementLearning::rewardCallback, this);

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

  ros::param::get("RANDOM_SEED_TEST_SNN", RANDOM_SEED_TEST_SNN);
  ros::param::get("RANDOM_SEED_TEST_GYRO", RANDOM_SEED_TEST_GYRO);
  ros::param::get("RANDOM_SEED_TEST_IN", RANDOM_SEED_TEST_IN);

  client_ =
      nh_.serviceClient<hexapod_msgs::MoveFeetLearning>("/move_feet_learning");

  falseInVectorLegs();
  // writerInitParamTimeData();
}
void ReinforcementLearning::rewardCallback(std_msgs::Float32 msg) {
  reward = msg.data;
}

void ReinforcementLearning::algoritm() {
  int total_time = 0;
  bool spike_one_yes = false;
  // количество вариантов тестирования ( по порогу)
  for (int iter = 0; iter < 7; iter++) {
    writerInitParamTimeData(iter);

    // количество попыток тестирования с одним порогом
    for (int i = 0; i < 1; i++) {
      writerInitParamTimeData(iter);

      // рандомное распределение весов
      Matrix weight_sgp = Matrix(6, 6);
      Matrix weight_in = Matrix(1, 6);
      Matrix weight_gyro = Matrix(1, 6);
      weight_sgp.fill_random(1, RANDOM_SEED_TEST_SNN[iter]);
      weight_in.fill_random(1, RANDOM_SEED_TEST_IN[iter]);
      weight_gyro.fill_random(1, RANDOM_SEED_TEST_GYRO[iter]);

      //  Возбуждающий нейрон
      Matrix weight_shooter = Matrix(1, 6);
      weight_shooter.get_data()[0] = 2.0;
      weight_shooter.get_data()[1] = 2.0;
      weight_shooter.get_data()[2] = 2.0;
      weight_shooter.get_data()[3] = 2.0;
      weight_shooter.get_data()[4] = 2.0;
      weight_shooter.get_data()[5] = 2.0;
      Matrix spike_shooter = Matrix(1, 6);
      spike_shooter.zero();
      int count_shooter = 0;
      double last_reward_gyro = 0.0;

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

      // занулить главную диагональ
      weight_sgp.eye_zero();

      // инициализация потенциала
      Matrix voltage = Matrix(1, 6);
      Matrix voltage_last = Matrix(1, 6);
      voltage.zero();
      voltage_last.zero();

      // инициализая спайков

      Matrix spike_sgp = Matrix(6, 6);  // текущие спайки нейронов
      Matrix spike_sgp_last =
          Matrix(6, 6);  // прошлые спайки нейронов ИСПОЛЬЗУЕТСЯ ТОЛЬКО В ТОКЕ

      Matrix spike_in = Matrix(1, 6);
      Matrix spike_gyro = Matrix(1, 6);
      spike_sgp.zero();
      spike_sgp_last.zero();
      spike_in.ones();
      spike_gyro.zero();

      // занулить главную диагональ
      weight_sgp.eye_zero();
      spike_sgp.eye_zero();
      spike_sgp_last.eye_zero();

      Matrix spike_sgp_last_memory =
          Matrix(1, 6);  // это спайки прошлой  итерации
      Matrix spike_sgp_last_last_memory =
          Matrix(1, 6);  // это спайки прошлой прошлой итерации
      spike_sgp_last_memory.zero();
      spike_sgp_last_last_memory.zero();

      // ***************************************************
      //                   БЛОК ВЫВОДА
      std::cout << "Random weight CPG" << std::endl;
      weight_sgp.output();
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
      int epsilon = 90;
      bool shooter_bool = false;

      for (unsigned int time = 0; time < TIME_ITERATION; time++) {
        // для отслежевания
        std::cout << "TOTAL TIME" << total_time << std::endl;
        total_time++;
        // Если нужно возбуждать по эпсилон, то раскомитить
        // int u = (rand()%int(TIME_ITERATION));
        // if (u < epsilon) shooter_bool = true;

        // 1) ПРОВЕРЯЕМ НА ВСПЫШКИ НЕЙРОНЫ
        for (unsigned int neuron = 0; neuron < NUMBER_NEURON; neuron++) {
          // считает вклад в ток от соседних нейронов
          for (unsigned int neighbour = 0; neighbour < NUMBER_NEURON;
               neighbour++) {
            sum_contribution_neighbours =
                weight_sgp.get_data()[neighbour + NUMBER_NEURON * neuron] *
                    spike_sgp_last
                        .get_data()[neighbour + NUMBER_NEURON * neuron] +
                sum_contribution_neighbours;
          }

          // считаем ток
          current_neuron =
              weight_in.get_data()[neuron] * spike_in.get_data()[neuron] +
              weight_gyro.get_data()[neuron] * spike_gyro.get_data()[neuron] +
              sum_contribution_neighbours +
              spike_shooter.get_data()[neuron] *
                  weight_shooter.get_data()[neuron];

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

        // 2) ОБНОВЛЯЕМ ВЕСА

        // запрос
        hexapod_msgs::MoveFeetLearning srv;
        srv.request.legs = vector_legs;
        srv.request.current_time = time;

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

        // записываем в память текущее состояние потенциала нейрона
        for (unsigned int i = 0; i < NUMBER_NEURON; i++) {
          voltage_last.get_data()[i] = voltage.get_data()[i];
        }

        // !!!!
        std::cout << "Spike shooter = " << time << std::endl;
        spike_shooter.output();
        spike_shooter.zero();

        // Если нужно возбуждение по эпсилон, то нужно закомитить критерий
        if (reward_gyroscope == 2.0 && last_reward_gyro == reward_gyroscope) {
          // if(shooter_bool){
          count_shooter++;
          std::cout << "" << std::endl;
          std::cout << "count_shooter = " << count_shooter << std::endl;
          std::cout << "" << std::endl;

          if (count_shooter > 2) {
            int random_spike_shooter = (rand() % 6);
            int random_spike_shooter_2 = (rand() % 6);
            int random_spike_shooter_3 = (rand() % 6);
            while (random_spike_shooter_3 == random_spike_shooter_2) {
              random_spike_shooter_3 = (rand() % 6);
            }
            while (random_spike_shooter == random_spike_shooter_2) {
              random_spike_shooter_2 = (rand() % 6);
            }
            while (random_spike_shooter == random_spike_shooter_3) {
              random_spike_shooter = (rand() % 6);
            }

            spike_shooter.get_data()[random_spike_shooter] = 1.0;
            spike_shooter.get_data()[random_spike_shooter_2] = 1.0;
            spike_shooter.get_data()[random_spike_shooter_3] = 1.0;
            std::cout << "PYFFF -> " << random_spike_shooter << std::endl;
            std::cout << "PYFFF -> " << random_spike_shooter_2 << std::endl;
            std::cout << "PYFFF -> " << random_spike_shooter_3 << std::endl;
            count_shooter = 0;
          }
        } else {
          count_shooter--;
          if (count_shooter < 0) count_shooter = 0;
        }
        last_reward_gyro = reward_gyroscope;
        // shooter_bool = false;
        // !!!!

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
                int k = (rand() % 2);
                std::cout << "RANDOM" << k << std::endl;
                weight_sgp.get_data()[neuron + weigth * 6] =
                    weight_sgp.get_data()[neuron + weigth * NUMBER_NEURON] +
                    LEARNING_RATE * k * reward;
                if (weight_sgp.get_data()[neuron + weigth * 6] < 0)
                  weight_sgp.get_data()[neuron + weigth * NUMBER_NEURON] = 0.0;
                if (weight_sgp.get_data()[neuron + weigth * 6] > 1)
                  weight_sgp.get_data()[neuron + weigth * NUMBER_NEURON] = 1.0;
              }
            }
          }
        }

        // // Обновляем веса нейрона ИН
        // for ( unsigned int neuron = 0; neuron < NUMBER_NEURON; neuron++){
        //   if (spike_sgp.get_data()[neuron] == 1.0){
        //     weight_in.get_data()[neuron] =  weight_in.get_data()[neuron] +
        //     LEARNING_RATE *(rand()%2) *  reward; if
        //     (weight_in.get_data()[neuron] < 0 ) weight_in.get_data()[neuron]
        //     = 0; if (weight_in.get_data()[neuron] > 1 )
        //     weight_in.get_data()[neuron] = 1.0;
        //   }
        // }

        // обнуляем прошлые спайки
        spike_gyro.zero();
        // Обновляем веса нейрона ГИРО
        if (resust_str == "balance lost max") {
          std::cout << "*********************GYRO******************"
                    << std::endl;
          for (unsigned int neuron = 0; neuron < NUMBER_NEURON; neuron++) {
            if (spike_sgp.get_data()[neuron] == 1.0) {
              spike_gyro.get_data()[neuron] = 1.0;
              // weight_gyro.get_data()[neuron] = weight_gyro.get_data()[neuron]
              // + LEARNING_RATE * (rand()%2) *  reward; if
              // (weight_gyro.get_data()[neuron] < 0 )
              // weight_gyro.get_data()[neuron] = 0; if
              // (weight_gyro.get_data()[neuron] > 1 )
              // weight_gyro.get_data()[neuron] = 1.0;
            }
          }
          ros::Duration(2.0).sleep();
        }
        //**********************************

        if (spike_one_yes) {
          std::cout << "__________________NORMALIZATION_______________"
                    << std::endl;
          //**************************
          // НОРМАЛИЗАЦИЯ

          // Веса нейронов СНН
          // ищем минимум и максимум
          double min = weight_sgp.get_data()[0];
          double max = weight_sgp.get_data()[0];
          for (int i = 0; i < NUMBER_NEURON * NUMBER_NEURON; i++) {
            if (weight_sgp.get_data()[i] > max) max = weight_sgp.get_data()[i];
            if (weight_sgp.get_data()[i] < min) min = weight_sgp.get_data()[i];
          }
          // нормализация минмакс
          for (int i = 0; i < NUMBER_NEURON * NUMBER_NEURON; i++) {
            weight_sgp.get_data()[i] =
                (0.0) + (((weight_sgp.get_data()[i] - min) / (max - min)) *
                         (1.0 - (0.0)));
          }
        }

        //     // Нормализация ИН
        //     //ищем минимум и максимум
        //     min = weight_in.get_data()[0];
        //     max = weight_in.get_data()[0];
        //     for ( int i = 0; i < 6 ; i ++){
        //         if (weight_in.get_data()[i] > max) max =
        //         weight_in.get_data()[i]; if (weight_in.get_data()[i] < min)
        //         min = weight_in.get_data()[i];
        //     }
        //     //
        //     for ( int i = 0; i < 6 ; i ++){
        //         weight_in.get_data()[i] = (-1.0) + (((weight_in.get_data()[i]
        //         - min)/(max - min))  * (1.0 - (-1.0)));
        //     }

        //     // Нормализация GYRo
        //     //ищем минимум и максимум
        //     min = weight_gyro.get_data()[0];
        //     max = weight_gyro.get_data()[0];
        //     for ( int i = 0; i < 6 ; i ++){
        //         if (weight_gyro.get_data()[i] > max) max =
        //         weight_gyro.get_data()[i]; if (weight_gyro.get_data()[i] <
        //         min) min = weight_gyro.get_data()[i];
        //     }
        //     //
        //     for ( int i = 0; i < 6 ; i ++){
        //         weight_gyro.get_data()[i] = (-1.0) +
        //         (((weight_gyro.get_data()[i] - min)/(max - min))  * (1.0 -
        //         (-1.0)));
        //     }
        // //****************************

        // }

        // занулить главную диагональ
        weight_sgp.eye_zero();

        spike_sgp.zero();
        spike_sgp_last.eye_zero();
        spike_one_yes = false;

        //******************************************************************
        //                             БЛОК ВЫВОДА
        std::cout << " Weight in SNN time =  " << time << std::endl;
        weight_sgp.output();
        std::cout << " weight_gyro in time =  " << time << std::endl;
        weight_gyro.output();
        std::cout << " weight_in in time =  " << time << std::endl;
        weight_in.output();
        std::cout << " voltage in time =  " << time << std::endl;
        voltage.output();
        std::cout << " voltage_last in time =  " << time << std::endl;
        voltage_last.output();
        std::cout << " spike_sgp in time =  " << time << std::endl;
        spike_sgp.output();
        std::cout << " spike_sgp_last in time =  " << time << std::endl;
        spike_sgp_last.output();
        std::cout << " spike_sgp_LAST_LAST in time =  " << time << std::endl;
        spike_sgp_last_last_memory.output();
        std::cout << " spike_gyro in time =  " << time << std::endl;
        spike_gyro.output();
        std::cout << " spike_in in time =  " << time << std::endl;
        spike_in.output();
        std::cout << "reward = " << reward << std::endl;
        std::cout << "resust_str = " << resust_str << std::endl;
        //******************************************************************
        epsilon = epsilon - 2;
        writerParamReward(reward);
        writerParamLegs(vector_legs);
        writerParamVolt(voltage);
        writerParamWeigh(weight_sgp);
        falseInVectorLegs();
      }
    }
  }
}

void ReinforcementLearning::falseInVectorLegs() {
  for (int i = 0; i < NUMBER_NEURON; i++) vector_legs.legs[i] = false;
  vector_legs.cmd_vel = 0.5;
}
