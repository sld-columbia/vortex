#include <stdio.h>

volatile double val1 = 0.1;

int main() {
   val1 += 5.4;
   val1 *= 0.01;
   return 0;
}