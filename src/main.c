#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "log.h"
#include "param.h"
#include "pm.h"

#include "app_channel.h"
#include "led.h"
#include "unistd.h"

const static float bat671723HS25C[10] = {
    3.00, // 00%
    3.78, // 10%
    3.83, // 20%
    3.87, // 30%
    3.89, // 40%
    3.92, // 50%
    3.96, // 60%
    4.00, // 70%
    4.04, // 80%
    4.10  // 90%
};

static int32_t pmBatteryChargeFromVoltage(float voltage)
{
  int charge = 0;

  if (voltage < bat671723HS25C[0])
  {
    return 0;
  }
  if (voltage > bat671723HS25C[9])
  {
    return 9;
  }
  while (voltage > bat671723HS25C[charge])
  {
    charge++;
  }

  return charge;
}

struct testPacketRX
{
  bool setLeds;
} __attribute__((packed));

struct testPacketTX
{
  float speed;
  float battery;
  bool flying;
  bool ledOn;

  uint16_t front;
  uint16_t left;
  uint16_t back;
  uint16_t right;
  uint16_t up;

} __attribute__((packed));

void appMain()
{

  ledClearAll();

  DEBUG_PRINT("Waiting for activation ...\n");

  struct testPacketRX rxPacket;
  struct testPacketTX txPacket;

  logVarId_t idUp = logGetVarId("range", "up");
  logVarId_t idLeft = logGetVarId("range", "left");
  logVarId_t idRight = logGetVarId("range", "right");
  logVarId_t idFront = logGetVarId("range", "front");
  logVarId_t idBack = logGetVarId("range", "back");

  while (1)
  {

    if (appchannelReceivePacket(&rxPacket, sizeof(rxPacket), 0))
    {
      DEBUG_PRINT("App channel received setLeds: %d\n", (int)rxPacket.setLeds);
      if (rxPacket.setLeds)
      {
        ledSetAll();
        txPacket.ledOn = true;
      }
      else
      {
        ledClearAll();
        txPacket.ledOn = false;
      }
    }

    vTaskDelay(M2T(1000));

    txPacket.speed = 0.0;
    txPacket.battery = pmBatteryChargeFromVoltage(pmGetBatteryVoltage()) * 10;
    txPacket.flying = false;

	txPacket.back = logGetUint(idBack);
    txPacket.front = logGetUint(idFront);
    txPacket.left = logGetUint(idLeft);
    txPacket.right = logGetUint(idRight);
    txPacket.up = logGetUint(idUp);


    DEBUG_PRINT("Sending packets");

    appchannelSendPacket(&txPacket, sizeof(txPacket));
  }
}
