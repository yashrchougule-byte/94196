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
    uint8_t reg = 5;
    uint8_t left, right;

    printf("Original value\n");
    printf("0x%02X binary: ", reg);
    printBinary(reg);

    left = reg << 2;
    printf("\nLeft shift by 2\n");
    printf("0x%02X binary: ", left);
    printBinary(left);

    right = reg >> 1;
    printf("\nRight shift by 1\n");
    printf("0x%02X binary: ", right);
    printBinary(right);

    return 0;
}
