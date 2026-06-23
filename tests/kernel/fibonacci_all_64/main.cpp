#include <stdio.h>
#include <cstdint>

volatile uint64_t num = 9;
volatile uint64_t fib = 0;

uint64_t fibonacci(uint64_t n) {
   if (n <= 1) {
      return n;
   }
   return fibonacci(n-1) + fibonacci(n-2);
}

int main() {
	fib = fibonacci(num);
   return 0;
}