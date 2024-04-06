#include <Arduino.h>
#include "Transducer.h"

/*
Code By Nick Katz and
Writen to control a Electronic Speed Contoler and a Motor
*/
/************************************************
 * HERE STARTS THE REAL CODE           
 *  /\/\/\                            /  \
 * | \  / |                         /      \
 * |  \/  |                       /          \
 * |  /\  |----------------------|     /\     |
 * | /  \ |                      |    /  \    |
 * |/    \|                      |   /    \   |
 * |\    /|                      |  | (  ) |  |
 * | \  / |                      |  | (  ) |  |
 * |  \/  |                 /\   |  |      |  |   /\
 * |  /\  |                /  \  |  |      |  |  /  \
 * | /  \ |               |----| |  |      |  | |----|
 * |/    \|---------------|    | | /|   .  |\ | |    |
 * |\    /|               |    | /  |   .  |  \ |    |
 * | \  / |               |    /    |   .  |    \    |
 * |  \/  |               |  /      |   .  |      \  |
 * |  /\  |---------------|/        |   .  |        \|
 * | /  \ |              /   SPT    |   .  |  SPT     \
 * |/    \|              (          |      |           )
 * |/\/\/\|               |    | |--|      |--| |    |
 * ------------------------/  \-----/  \/  \-----/  \--------
 *                         \\//     \\//\\//     \\//
 *                          \/       \/  \/       \/
 ************************************************
*/

//TODO RPM Control 
//TODO Pressure Data
//TODO Tempurature Data
//TODO DAC Display
//TODO Solenoid control

Transducer ducer_1 (17);

void setup() {
  Serial.begin(115200);
  
}

void loop() {
u_int16_t psi = ducer_1.get_PSI();

Serial.print("PSIG: ");
Serial.println(psi);
delay(1000);
}