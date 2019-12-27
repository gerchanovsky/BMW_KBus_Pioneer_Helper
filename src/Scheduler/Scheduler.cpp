#include "Scheduler.h"

Scheduler* Scheduler::actions = NULL;
func_sec_t Scheduler::func_sec = NULL;
signed long Scheduler::next_sec = 0;
#if 0
void action(byte v) { Serial.println(v);}

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  Scheduler::dump();
  Scheduler::add(1,2000,action);
  Scheduler::dump();
  Scheduler::add(2,500,action);
  Scheduler::dump();
  Scheduler::add(3,1500,action);
  Scheduler::dump();
  Scheduler::add(4,4000,action);
  Scheduler::dump();
  Scheduler::add(6,6000,action);
  Scheduler::dump();
  Scheduler::add(7,400,action);
  Scheduler::dump();
  Scheduler::add(5,700,action);
  Scheduler::dump();
  Scheduler::add(2,701,action);
  Scheduler::dump();
  return;
  Scheduler::del(5);
  Scheduler::dump();
  Scheduler::del(6);
  Scheduler::dump();
  Scheduler::del(7);
  Scheduler::dump();
}

void loop() {
  // put your main code here, to run repeatedly:
  Scheduler::idle();
  delay(10);
}
#endif
