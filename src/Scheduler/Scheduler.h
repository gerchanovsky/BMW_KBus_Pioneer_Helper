#pragma once
#include <Arduino.h>

typedef void (*func_delay_t)(byte);
typedef void (*func_sec_t)();

class Scheduler {
  func_delay_t func;
  static func_sec_t func_sec;
  static signed long next_sec;
  byte tag;
  signed long ms;
  Scheduler *next;
  int value;
public:
  static Scheduler *actions;
  Scheduler(const byte _tag, const signed long _ms, func_delay_t _func, Scheduler *_next) :
    func(_func), tag(_tag), ms(_ms), next(_next) {}
  static void dump() {
    signed long ms = millis();
    for (Scheduler *p = actions; p != NULL; p = p->next) {
      Serial.printf("%ld(%d) ", p->ms-ms, p->tag);
    }
    Serial.println("\r\n----");
  }
  static void setup(func_sec_t func) {
    func_sec = func;
    next_sec = millis()+1000;
  }
  static void idle() {
    signed long ms = millis();
    while (actions && (actions->ms-ms)<=0) {
      Scheduler *t = actions;
      actions = actions->next;
      t->func(t->tag);
      delete t;
    }
    if (func_sec && (next_sec-ms)<=0) {
      next_sec = ms+1000;
      func_sec();
    }
  }
  static bool find(const byte tag) {
    Scheduler *p = actions;
    while (p!=NULL) {
      if (p->tag==tag)
        return true;
      p = p->next;
    }
    return false;
  }
  static bool del(const byte tag) {
    Scheduler **p = &actions;
    while (*p!=NULL) {
      if ((*p)->tag==tag) {
        Scheduler *t = *p;
        *p = t->next;
        delete t;
        return true;
      }
      p = &((*p)->next);
    }
    return false;
  }
  static void add(const byte tag, signed long delay_ms, func_delay_t func) {
    signed long ms = millis();
    Scheduler **p = &actions;
    del(tag);
    while (*p!=NULL && ((*p)->ms-ms)<delay_ms) {
      p = &((*p)->next);
    }
    *p = new Scheduler(tag, ms+delay_ms, func, (*p)?*p:NULL);
  }
};
