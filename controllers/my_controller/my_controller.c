#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/led.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include <string.h>
#include <webots/receiver.h>
#define TIME_STEP 32
#define NUM_SENSORS 8
#define PROX_THRESHOLD 80.0
#define CAIXA_LEVE_THRESHOLD 500.0
#define SENSOR_QUASE_ZERO 100.0
#define AFASTAMENTO_STEPS 15
#define GIRO_STEPS 20
#define RECOVERY_STEPS 25

int main() {
  wb_robot_init();
  WbDeviceTag receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, TIME_STEP);

  // Sensores
  WbDeviceTag prox_sensors[NUM_SENSORS];
  const char *sensor_names[NUM_SENSORS] = {
    "ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"
  };
  for (int i = 0; i < NUM_SENSORS; i++) {
    prox_sensors[i] = wb_robot_get_device(sensor_names[i]);
    wb_distance_sensor_enable(prox_sensors[i], TIME_STEP);
  }

  // Motores
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  // LED
  WbDeviceTag led = wb_robot_get_device("led0");
  wb_led_set(led, 0);

  srand(time(NULL));

  bool caixa_foi_empurrada = false;
  bool caixa_estava_na_frente = false;
  bool colisao_caixa11 = false;

  while (wb_robot_step(TIME_STEP) != -1) {
    if (colisao_caixa11) {
    wb_motor_set_velocity(left_motor, -3.0);
    wb_motor_set_velocity(right_motor, 3.0);
    wb_led_set(led, 1);
    continue;
  }
    double sensor_values[NUM_SENSORS];
    for (int i = 0; i < NUM_SENSORS; i++) {
      sensor_values[i] = wb_distance_sensor_get_value(prox_sensors[i]);
    }
    if (wb_receiver_get_queue_length(receiver) > 0) {
      const void *msg = wb_receiver_get_data(receiver);
      if (strstr((const char*)msg, "COLISAO_CAIXA11") != NULL) {
        colisao_caixa11 = true;
      }
      wb_receiver_next_packet(receiver);
    }
  
    double frente = (sensor_values[3] + sensor_values[4]) / 2.0;

    // Se já empurrou a caixa leve, gira sobre o próprio eixo infinitamente
    if (caixa_foi_empurrada) {
      wb_motor_set_velocity(left_motor, -3.0);
      wb_motor_set_velocity(right_motor, 3.0);
      wb_led_set(led, 1);
      continue;
    }

    // Detecta presença de caixa leve inicialmente
    if (!caixa_foi_empurrada) {
      if (frente > CAIXA_LEVE_THRESHOLD) {
        caixa_estava_na_frente = true;
      } else {
        caixa_estava_na_frente = false;
      }
    }

    // Se antes tinha caixa, e agora não tem mais, é porque empurrou
    if (!caixa_foi_empurrada && caixa_estava_na_frente && frente < SENSOR_QUASE_ZERO) {
      printf("Caixa leve empurrada!\n");
      caixa_foi_empurrada = true;
      continue;
    }

    // Detecção de obstáculos
    bool obstaculo_frente = frente > PROX_THRESHOLD;
    bool obstaculo_lateral_esq = sensor_values[0] > PROX_THRESHOLD || sensor_values[1] > PROX_THRESHOLD;
    bool obstaculo_lateral_dir = sensor_values[6] > PROX_THRESHOLD || sensor_values[7] > PROX_THRESHOLD;

    if (obstaculo_frente || (obstaculo_lateral_esq && obstaculo_lateral_dir)) {
      // Recuar
      wb_motor_set_velocity(left_motor, -3.0);
      wb_motor_set_velocity(right_motor, -3.0);
      for (int i = 0; i < RECOVERY_STEPS; i++) wb_robot_step(TIME_STEP);

      // Girar aleatoriamente
      int direcao = rand() % 2;
      if (direcao == 0) {
        wb_motor_set_velocity(left_motor, -3.0);
        wb_motor_set_velocity(right_motor, 3.0);
      } else {
        wb_motor_set_velocity(left_motor, 3.0);
        wb_motor_set_velocity(right_motor, -3.0);
      }
      for (int i = 0; i < GIRO_STEPS; i++) wb_robot_step(TIME_STEP);

      // Avançar um pouco
      wb_motor_set_velocity(left_motor, 4.0);
      wb_motor_set_velocity(right_motor, 4.0);
      for (int i = 0; i < AFASTAMENTO_STEPS; i++) wb_robot_step(TIME_STEP);

      continue;
    }

    // Movimento normal
    wb_motor_set_velocity(left_motor, 4.0);
    wb_motor_set_velocity(right_motor, 4.0);
    wb_led_set(led, 0);
  }

  wb_robot_cleanup();
  return 0;
}
