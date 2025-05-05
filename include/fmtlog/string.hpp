#pragma once

#include <cstdint>
#include <ios>

template<size_t SIZE>
class Str
{
public:
  static const int Size = SIZE;
  char s[SIZE];

  Str() {}
  Str(const char* p) { *this = *(const Str<SIZE>*)p; }

  char& operator[](int i) { return s[i]; }
  char operator[](int i) const { return s[i]; }

  template<typename T>
  void fromi(T num) {
    if constexpr (Size & 1) {
      s[Size - 1] = '0' + (num % 10);
      num /= 10;
    }
    switch (Size & -2) {
      case 18: *(uint16_t*)(s + 16) = *(uint16_t*)(digit_pairs + ((num % 100) << 1)); num /= 100;
      case 16: *(uint16_t*)(s + 14) = *(uint16_t*)(digit_pairs + ((num % 100) << 1)); num /= 100;
      case 14: *(uint16_t*)(s + 12) = *(uint16_t*)(digit_pairs + ((num % 100) << 1)); num /= 100;
      case 12: *(uint16_t*)(s + 10) = *(uint16_t*)(digit_pairs + ((num % 100) << 1)); num /= 100;
      case 10: *(uint16_t*)(s + 8) = *(uint16_t*)(digit_pairs + ((num % 100) << 1)); num /= 100;
      case 8: *(uint16_t*)(s + 6) = *(uint16_t*)(digit_pairs + ((num % 100) << 1)); num /= 100;
      case 6: *(uint16_t*)(s + 4) = *(uint16_t*)(digit_pairs + ((num % 100) << 1)); num /= 100;
      case 4: *(uint16_t*)(s + 2) = *(uint16_t*)(digit_pairs + ((num % 100) << 1)); num /= 100;
      case 2: *(uint16_t*)(s + 0) = *(uint16_t*)(digit_pairs + ((num % 100) << 1)); num /= 100;
    }
  }

  static constexpr const char* digit_pairs = "00010203040506070809"
                                             "10111213141516171819"
                                             "20212223242526272829"
                                             "30313233343536373839"
                                             "40414243444546474849"
                                             "50515253545556575859"
                                             "60616263646566676869"
                                             "70717273747576777879"
                                             "80818283848586878889"
                                             "90919293949596979899";
};