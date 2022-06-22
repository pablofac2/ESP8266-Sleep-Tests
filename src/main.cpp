/*
CONCLUSIONS:
- GPIOs configured as gpio_wakeup AND interruption:
  - ESP wakes right away the GPIO changes state, but keeps calling the "interruption function"
    until the state returns to the normal (HIGH)
  - CHANGE, FALLING, RISING... all do the same... the ESP keeps calling "interruption function"
    until the GPIO goes to the normal state (i.e the opposite to the defined GPIO_PIN_INTR_xxLEVEL)
  - After that, the Wake Up call back function is executed... so... not usefull at all...
    if the GPIO doesn't return quick to the normal state, the ESP reboots because of whatchdog.
- GPIOs configured only as interruption:
  - ESP doesn't wake up by the interruption... but ones it wakes because of the light sleep timeout,
    the Interrupt Function executes (if corresponds)
  - i.e. if the Iterrupt was configured as FALLING, and the GPIO was HIGH before sleeping,
    and is LOW when waking up, then right after waking up it executes first the interrupt function,
    later the wake up call back function, and finally continues executing the sleep calling function.
- GPIOs configured only as gpio_wakeup :
  - ESP wakes up right away the GPIO takes the defined state (GPIO_PIN_INTR_LOLEVEL or HULEVEL)
  - it is possible to read which GPIO caused the wake up with digitalread().
*/


#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "user_interface.h"

#define DEBUG_PRINTLN(x)  Serial.println(x)
#define DEBUG_PRINT(x)  Serial.print(x)
#define DEBUG_FLUSH Serial.flush()
uint32 SLEEP_TIME_MS = 20000;
#define DEBUGbaudrate 115200
#define SIZEOF_ZONE 5
#define SIZEOF_SIREN 3
#define SIM800_RING_RESET_PIN D3    //input and output pin, used to reset the sim800
const uint8_t ZONE_PIN[SIZEOF_ZONE] = {D1, D2, D5, D6, D7}; //D6 and D7 will not wake ESP but will trigger interrupts ones it wakes by time.
const uint8_t SIREN_PIN[SIZEOF_SIREN] = {D0, D4, D8};
//unsigned long startT;

void Sleep_Timed();
void WakeUpCallBackFunction();
uint32_t RTCmillis();
IRAM_ATTR void DIInterruptFunction0();
IRAM_ATTR void DIInterruptFunction1();
IRAM_ATTR void DIInterruptFunction2();
IRAM_ATTR void DIInterruptFunction3();
IRAM_ATTR void DIInterruptFunction4();

void setup() {
  //PARA ASIGNAR LA FUNCIÓN ADECUADA A CADA PIN (ESTÁN MULTIPLEXADOS, VER EXCEL)
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0);
  //PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_GPIO1);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);
  //PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_GPIO3);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U, FUNC_GPIO4);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5);
  //PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_CLK_U, FUNC_GPIO6);
  //PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_DATA0_U, FUNC_GPIO7);
  //PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_DATA1_U, FUNC_GPIO8);
  //PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_DATA2_U, FUNC_GPIO9);
  //PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_DATA3_U, FUNC_GPIO10);
  //PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_SD_CMD_U, FUNC_GPIO11);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12); //Use	MTDI	pin	as	GPIO12.
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13); //Use	MTCK	pin	as	GPIO13.
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_GPIO14); //Use	MTMS	pin	as	GPIO14.
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_GPIO15); //Use	MTDO	pin	as	GPIO15.

  for (int i = 0; i < SIZEOF_ZONE; i++)
    GPIO_DIS_OUTPUT(GPIO_ID_PIN(ZONE_PIN[i]));    //GPIOs as INPUTS only
  //GPIO_DIS_OUTPUT(GPIO_ID_PIN(SIM800_RING_RESET_PIN));    //Configura la pata como entrada, traido de Sleep_Forced

  for (int i = 0; i < SIZEOF_ZONE; i++)
    pinMode(GPIO_ID_PIN(ZONE_PIN[i]), INPUT_PULLUP);  //Input Mode with pull up -> will be used to wake up the ESP
  pinMode(SIM800_RING_RESET_PIN, INPUT_PULLUP);

  for (int i = 0; i < SIZEOF_SIREN; i++)
    pinMode(GPIO_ID_PIN(SIREN_PIN[i]), OUTPUT);       //Outputs

  Serial.begin(DEBUGbaudrate);
  while(!Serial)
    yield();
  DEBUG_PRINTLN(F("\nReset reason = ") + ESP.getResetReason());
  DEBUG_FLUSH;
  
  //Interrupts and light sleep, setting up interrupts:
  attachInterrupt(digitalPinToInterrupt(GPIO_ID_PIN(ZONE_PIN[0])), DIInterruptFunction0, CHANGE); //CHANGE RISING FALLING
  attachInterrupt(digitalPinToInterrupt(GPIO_ID_PIN(ZONE_PIN[1])), DIInterruptFunction1, FALLING); //CHANGE RISING FALLING
  attachInterrupt(digitalPinToInterrupt(GPIO_ID_PIN(ZONE_PIN[2])), DIInterruptFunction2, CHANGE); //CHANGE RISING FALLING
  attachInterrupt(digitalPinToInterrupt(GPIO_ID_PIN(ZONE_PIN[3])), DIInterruptFunction3, FALLING); //CHANGE RISING FALLING
  attachInterrupt(digitalPinToInterrupt(GPIO_ID_PIN(ZONE_PIN[4])), DIInterruptFunction4, CHANGE); //CHANGE RISING FALLING

  //Configuring the ESP to be able to LIGHT SLEEP:
  delay(1);                                   //Needs a small delay at the begining!
  //gpio_pin_wakeup_disable();                //If only timed sleep, not pin interrupt
  wifi_station_disconnect();                  //disconnect wifi
  wifi_set_opmode(NULL_MODE);							 	  //set WiFi	mode	to	null	mode.
  wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);		  //This API can only be called before wifi_fpm_open 	light	sleep
  wifi_fpm_open();													  //Enable force sleep function
  for (int i = 0; i < SIZEOF_ZONE-2; i++) //LAST 2 ZONES WILL NOT WAKE, TEST IF INTERRUPT WAKES UP THE ESP
    wifi_enable_gpio_wakeup(GPIO_ID_PIN(ZONE_PIN[i]), GPIO_PIN_INTR_LOLEVEL); //Sending this GPIOs to ground will wake the ESP.
  wifi_enable_gpio_wakeup(GPIO_ID_PIN(SIM800_RING_RESET_PIN), GPIO_PIN_INTR_LOLEVEL); //Sending this GPIOs to ground will wake the ESP.
  wifi_fpm_set_wakeup_cb(WakeUpCallBackFunction);	//This API can only be called when force sleep function is enabled, after calling wifi_fpm_open. Will be called after system wakes up only if the force sleep time out (wifi_fpm_do_sleep and the parameter is not 0xFFFFFFF)
}

void loop() {
  while(Serial.available()){
    Serial.readString();
  }
  Sleep_Timed();
  for (int i = 0; i < SIZEOF_ZONE; i++)
    DEBUG_PRINTLN(F("GPIO") + String(GPIO_ID_PIN(ZONE_PIN[i])) + ": " + String(digitalRead(GPIO_ID_PIN(ZONE_PIN[i]))));
  DEBUG_PRINTLN(F("GPIO") + String(GPIO_ID_PIN(SIM800_RING_RESET_PIN)) + ": " + String(digitalRead(GPIO_ID_PIN(SIM800_RING_RESET_PIN))));
  DEBUG_PRINTLN(F(" "));
  delay(10000);
}

void Sleep_Timed() {
  DEBUG_PRINTLN(F("Going to Sleep at ms: ") + String(RTCmillis()));
  DEBUG_FLUSH;
  
  extern os_timer_t *timer_list;  //for timer-based light sleep to work, the os timers need to be disconnected
  timer_list = nullptr;

  //If forced light sleep without timed wake up, only external gpio wake:
  //sint8 res = wifi_fpm_do_sleep(0xFFFFFFF);
  //delay(10);

  //If timed light sleep and external gpio wake:
  sint8 res = wifi_fpm_do_sleep(SLEEP_TIME_MS * 1000);  //microseconds
  delay(SLEEP_TIME_MS + 1);  // it goes to sleep //The system will not enter sleep mode instantly when force-sleep APIs are called, but only after executing an idle task.
  
  DEBUG_PRINTLN(F("Sleep result (0 is ok): ") + String(res));  // the interrupt callback hits before this is executed
  //0, setting successful;
  //-1, failed to sleep, sleep status error;
  //-2, failed to sleep, force sleep function is not enabled
}

void WakeUpCallBackFunction()
{
  DEBUG_PRINTLN(F("Woke Up - CallBack Function executed at ms: ") + String(RTCmillis()));
  DEBUG_FLUSH;
  //wifi_fpm_close();					 	        //Only if not sleeping gain, disable force sleep function
  //wifi_set_opmode(STATION_MODE);			//If need to set station mode
  //wifi_station_connect();							//If need to connect to AP
}

IRAM_ATTR void DIInterruptFunction0(){
  //ISRs need to have ICACHE_RAM_ATTR before the function definition to run the interrupt code in RAM.
  DEBUG_PRINTLN(F("GPIO Changed: ") + String(GPIO_ID_PIN(ZONE_PIN[0])) + " to " + digitalRead(GPIO_ID_PIN(ZONE_PIN[0])));
}
IRAM_ATTR void DIInterruptFunction1(){
  //ISRs need to have ICACHE_RAM_ATTR before the function definition to run the interrupt code in RAM.
  DEBUG_PRINTLN(F("GPIO Changed: ") + String(GPIO_ID_PIN(ZONE_PIN[1])) + " to " + digitalRead(GPIO_ID_PIN(ZONE_PIN[1])));
}
IRAM_ATTR void DIInterruptFunction2(){
  //ISRs need to have ICACHE_RAM_ATTR before the function definition to run the interrupt code in RAM.
  DEBUG_PRINTLN(F("GPIO Changed: ") + String(GPIO_ID_PIN(ZONE_PIN[2])) + " to " + digitalRead(GPIO_ID_PIN(ZONE_PIN[2])));
}
IRAM_ATTR void DIInterruptFunction3(){
  //ISRs need to have ICACHE_RAM_ATTR before the function definition to run the interrupt code in RAM.
  DEBUG_PRINTLN(F("GPIO Changed: ") + String(GPIO_ID_PIN(ZONE_PIN[3])) + " to " + digitalRead(GPIO_ID_PIN(ZONE_PIN[3])));
}
IRAM_ATTR void DIInterruptFunction4(){
  //ISRs need to have ICACHE_RAM_ATTR before the function definition to run the interrupt code in RAM.
  DEBUG_PRINTLN(F("GPIO Changed: ") + String(GPIO_ID_PIN(ZONE_PIN[4])) + " to " + digitalRead(GPIO_ID_PIN(ZONE_PIN[4])));
}

uint32_t RTCmillis()
{
  return (system_get_rtc_time() * (system_rtc_clock_cali_proc() >> 12)) / 1000;  // system_get_rtc_time() is in us (but very inaccurate anyway)
}
