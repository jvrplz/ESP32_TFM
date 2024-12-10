#include <MAVLink.h>

void setup() {
  Serial.begin(57600, SERIAL_8N1, 16, 17);  // Inicializar puerto serie a 57600 baudios
}

void loop() {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t msg;

  // Array de PWM para los 4 motores (puedes ajustar según cómo estén conectados los motores)
  int motor_pwm[4] = { 1000, 1000, 1000, 1000 };

  // Enviar progresivamente de 10% en 10% hasta llegar al 90%
  for (int i = 1; i <= 9; i++) {
    int pwm_value = 1000 + (i * 100);

    // Establecer el valor de PWM para los 4 motores
    for (int motor = 0; motor < 4; motor++) {
      motor_pwm[motor] = pwm_value;
      // Enviar el comando para establecer el PWM en el motor correspondiente (motores 1 a 4)
      mavlink_msg_command_long_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg,
                                    1,                     // system ID de la Pixhawk
                                    1,                     // component ID de la Pixhawk
                                    MAV_CMD_DO_SET_SERVO,  // Comando para establecer servo (motor)
                                    0,                     // Confirmación
                                    motor + 1,
                                    motor_pwm[motor],
                                    0, 0, 0, 0, 0);
      mav_

        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      Serial.write(buf, len);  // Enviar el mensaje al puerto serie

      delay(1000);
    }
  }


  delay(10000);
}
