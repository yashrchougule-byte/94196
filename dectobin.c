#include <stdio.h>

int main() {
    int num, binary[32], i = 0;

    printf("Enter decimal number: ");
    scanf("%d", &num);

    int temp = num;

    while (temp > 0) {
        binary[i] = temp % 2;
        temp = temp / 2;
        i++;
    }

    printf("Binary of %d = ", num);

    for (int j = i - 1; j >= 0; j--) {
        printf("%d", binary[j]);
    }

    return 0;
}