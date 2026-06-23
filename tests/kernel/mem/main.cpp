#include <stdio.h>
#include <cstdint>

volatile uint64_t val1 = 0xA;
volatile uint64_t val2 = 0xB;
volatile uint64_t val3 = 0xC;

int main() {
   val1 += 0xAAAAAAAAAAAAAAA0;
   val2 += 0xBBBBBBBBBBBBBBB0;
   val3 += 0xCCCCCCCCCCCCCCC0;
   return 0;
}