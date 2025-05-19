#include <webots/robot.h>
#include <webots/supervisor.h>
#include <stdio.h>
#include <webots/emitter.h>
#include <string.h>
#include <math.h>
#define TIME_STEP 512
#define TOTAL_CAIXAS 20

int main() {
  wb_robot_init();
  WbDeviceTag emitter = wb_robot_get_device("emitter");
  WbNodeRef caixas[TOTAL_CAIXAS];
  char defName[12];

  for (int i = 0; i < TOTAL_CAIXAS; i++) {
    sprintf(defName, "CAIXA%02d", i + 1);
    caixas[i] = wb_supervisor_node_get_from_def(defName);
    if (caixas[i]) {
      const double *pos = wb_supervisor_node_get_position(caixas[i]);
      printf("CAIXA%02d posição inicial: X=%.2f Y=%.2f Z=%.2f\n", i + 1, pos[0], pos[1], pos[2]);
    }
  }

  // Armazena posição inicial da caixa 11
  double pos_inicial_caixa11[3] = {0};
  if (caixas[10]) {
    const double *pos = wb_supervisor_node_get_position(caixas[10]);
    pos_inicial_caixa11[0] = pos[0];
    pos_inicial_caixa11[1] = pos[1];
    pos_inicial_caixa11[2] = pos[2];
  }
  int colisao_enviada = 0;

  // Mantém a simulação rodando
  while (wb_robot_step(TIME_STEP) != -1) {
    if (caixas[10] && !colisao_enviada) {
      const double *pos = wb_supervisor_node_get_position(caixas[10]);
      double dx = pos[0] - pos_inicial_caixa11[0];
      double dz = pos[2] - pos_inicial_caixa11[2];
      if (fabs(dx) > 0.05 || fabs(dz) > 0.05) {
        char mensagem[] = "COLISAO_CAIXA11";
        wb_emitter_send(emitter, mensagem, strlen(mensagem) + 1);
        printf("Mensagem enviada: %s\n", mensagem);
        colisao_enviada = 1;
      }
    }
  }

  wb_robot_cleanup();
  return 0;
}