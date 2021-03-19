#define DEBUG_MODULE "HELLOWORLD"

extern "C" {
#include <FreeRTOS.h>
#include <app.h> /* appMain */
#include <app_channel.h>
#include <debug.h> /* DEBUG_PRINT */
#include <task.h>  /* vTaskDelay */

#include <led.h>
#include <log.h>
#include <pm.h>
}

#include <cstdint>
#include <ctime>

constexpr static float bat671723HS25C[] = {
    3.00f, // 00%
    3.78f, // 10%
    3.83f, // 20%
    3.87f, // 30%
    3.89f, // 40%
    3.92f, // 50%
    3.96f, // 60%
    4.00f, // 70%
    4.04f, // 80%
    4.10f  // 90%
};

static int pmBatteryChargeFromVoltage(float voltage) {
	if (voltage < bat671723HS25C[0]) {
		return 0;
	}

	if (voltage > bat671723HS25C[9]) {
		return 9;
	}

	int charge = 0;
	while (voltage > bat671723HS25C[charge]) {
		++charge;
	}
	return charge;
}

float getBatteryPercentage() {
	float voltage = pmGetBatteryVoltage();
	return (float) pmBatteryChargeFromVoltage(voltage) * 10.0f;
}

struct testPacketRX
{
  bool setLeds;
} __attribute__((packed));

struct testPacketTX
{
  uint16_t front;
  uint16_t left;
  uint16_t back;
  uint16_t right;
  uint16_t up;

  float speed;
  float battery;
  bool flying;
  bool ledOn;

} __attribute__((packed));

void appMain()
{

  ledClearAll();

  DEBUG_PRINT("Waiting for activation ...\n");

  struct testPacketRX rxPacket;
  struct testPacketTX txPacket;

  logVarId_t idUp 		= logGetVarId("range", "up");
  logVarId_t idLeft 	= logGetVarId("range", "left");
  logVarId_t idRight 	= logGetVarId("range", "right");
  logVarId_t idFront 	= logGetVarId("range", "front");
  logVarId_t idBack 	= logGetVarId("range", "back");

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
    txPacket.battery = getBatteryPercentage();
    txPacket.flying = false;

	txPacket.back 	= (uint16_t) logGetUint(idBack);
    txPacket.front 	= (uint16_t) logGetUint(idFront);
    txPacket.left 	= (uint16_t) logGetUint(idLeft);
    txPacket.right 	= (uint16_t) logGetUint(idRight);
    txPacket.up 	= (uint16_t)logGetUint(idUp);


    DEBUG_PRINT("Sending packets");

    appchannelSendPacket(&txPacket, sizeof(txPacket));
  }
}
