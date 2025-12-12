#include <stdio.h>

unsigned char evenParity(unsigned char x)
{
    int i, count = 0;
    unsigned char temp = x;

    for(i = 0; i < 8; i++)     // check all 8 bits
    {
        if(temp & 1)
        {
            count++;          // count how many 1s
        }
        temp = temp >> 1;
    }

    if(count % 2 != 0)        // if not even parity
    {
        x = x | 0x80;         // set MSB to make parity even
    }

    return x;
}

void alphabetCheck(char c)
{
    if((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'))
    {
        printf("It is an alphabet.\n");

        char r = c ^ 32;      // XOR with 32
        printf("After XOR with 32: %c\n", r);
    }
    else
    {
        printf("It is not an alphabet.\n");
    }
}

void swapXOR(int *a, int *b)
{
    *a = *a ^ *b;
    *b = *a ^ *b;
    *a = *a ^ *b;
}

int main()
{
    unsigned char num;
    char ch;
    int x, y;

    printf("Enter a number (0-255): ");
    scanf("%hhu", &num);

    unsigned char result = evenParity(num);
    printf("Number after fixing parity (if needed) = %d\n", result);

    printf("\nEnter a character: ");
    scanf(" %c", &ch);

    alphabetCheck(ch);

    printf("\nEnter two numbers to swap:\n");
    printf("x = ");
    scanf("%d", &x);
    printf("y = ");
    scanf("%d", &y);

    swapXOR(&x, &y);

    printf("After swapping using XOR:\n");
    printf("x = %d\n", x);
    printf("y = %d\n", y);

    return 0;
}
