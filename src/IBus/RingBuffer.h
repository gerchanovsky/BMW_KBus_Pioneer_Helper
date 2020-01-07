#pragma once

class RingBuffer {
  int size, tail, length;
  byte *buffer;
public:
  friend class IBus;
  RingBuffer(const int _size) : size(_size), tail(0), length(0) {
    buffer = (byte*)malloc(_size);
    memset(buffer, 0, _size);
  }
  ~RingBuffer() { if (buffer) free(buffer); }
  inline int available()   { return length; }
  byte peek(const int n=0) { return (length>n)?buffer[(tail + n) % size]:-1; }
  void remove(int n = 1)   { if (length<n) n = length;tail = (tail + n) % size;length -= n; }
  int pop() {
    if (length<=0) // Buffer empty
      return -1;
    --length;
    byte b = buffer[tail];
    tail = (tail + 1) % size;
    return b;
  }
  bool push(const byte b) {
    if (length>=size) {
      //return false;
      tail = (tail + 1) % size;--length;
    }
    buffer[(tail + length++) % size] = b;
    return true;
  }
  void dump(int n = -1) {
    int pos = tail;
    if (n<0) n = length;
    while (n-->0) {
      Serial.printf("%02X ", buffer[pos]);
      pos = (pos+1) % size;
    }
    Serial.println();
  }
};
