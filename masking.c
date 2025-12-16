#include <stdio.h>
#include <stdint.h>

    void printBinary(uint16_t value)
{
    for (int i = 15; i >= 0; i--)
    {
        printf("%d", (value >> i) & 1);
    }


}
int main()
{
    uint16_t temp;
    
     uint16_t reg= 0xABCD;
printf("hex:0x%04x binary:",reg);
printBinary(reg);
temp=reg;
temp = reg&0x000F;
printf("\nextracting  lower  four bits");
printf("\nhex:0x%04x binary:",temp);
printBinary(temp);

temp = reg&0xF000;
printf("\nextracting upper four bits");
printf("\nhex:0x%04x binary:",temp);
printBinary(temp);
 

}