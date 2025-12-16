#include <stdio.h>
#include <stdint.h>

    void printBinary(uint8_t value)
{
    for (int i = 7; i >= 0; i--)
    {
        printf("%d", (value >> i) & 1);
    }


}
int main()
{
    uint8_t reg= 0x2A;
    reg = reg|1<<4;
         printf("0x%02X  Binary:",reg);
         printBinary(reg);
         
     reg = reg&~(1<<1);
          printf("\nafter ckearing a bit");
                   printf("0x%02X  Binary:",reg);
                            printBinary(reg);

     reg = reg^(1<<5);
      printf("\nafter ckearing a bit");
                   printf("0x%02X  Binary:",reg);
                            printBinary(reg);

}