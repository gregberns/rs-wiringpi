#include <stdio.h>
#include <stdint.h>

#define BIT(bit) (1 << (bit))

static uint8_t latch_state; // The state of the shift register on the physical board.

static int motors[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// Latch=08     001000   8  FR
// Latch=10     010000  16  FL
// Latch=20     100000  32  BR
// Latch=01     000001   1  BL
// Latch=39     111001  57  ALL

// 4_A 0
// 2_B 1
// 1_B 2
// 1_A 3
// 2_A 4
// 3_A 5
// 4_B 6
// 3_B 7

/*
Write to shift register.
*/
// void latch_tx(void)
void latch_tx()
{
    unsigned char i;

    // 	#define MOTORLATCH      29
    // #define MOTORCLK        28
    // //#define MOTORENABLE   22
    // #define MOTORDATA       27

    // digitalWrite(MOTORLATCH, LOW);

    // digitalWrite(MOTORDATA, LOW);

    for (i = 0; i < 8; i++)
    {
        // delayMicroseconds(1); // 10 micros  delayMicroseconds

        // digitalWrite(MOTORCLK, LOW);

        if (motors[i])
        // if (latch_state & BIT(7 - i))
        {
            printf("HI   %08X\n", latch_state);
            // digitalWrite(MOTORDATA, HIGH);
        }
        else
        {
            printf("low\n", latch_state);
            // digitalWrite(MOTORDATA, LOW);
        }

        // delayMicroseconds(1); // 10 micros  delayMicroseconds
        // digitalWrite(MOTORCLK, HIGH);
    }

    // digitalWrite(MOTORLATCH, HIGH);
    return;
}

void test_forward()
{
    motors[2] = 1;
    motors[3] = 1;
    motors[4] = 1;
    motors[7] = 1;
}

void test_stop()
{
    motors[2] = 0;
    motors[3] = 0;
    motors[4] = 0;
    motors[7] = 0;
}

int ControllerInit(void)
{
    wiringPiSetup();

    pinMode(MOTORLATCH, OUTPUT);
    pinMode(MOTORDATA, OUTPUT);
    pinMode(MOTORCLK, OUTPUT);
    motors[0] = 0;
    motors[1] = 0;
    motors[2] = 0;
    motors[3] = 0;
    motors[4] = 0;
    motors[5] = 0;
    motors[6] = 0;
    motors[7] = 0;
    latch_tx();
    return 0;
}

void test()
{
    forward();
    latch_tx();
    sleep(2);

    printf("=====\n");

    stop();
    latch_tx();
    sleep(2);

    printf("=====\n");

    forward();
    latch_tx();
    sleep(2);

    printf("=====\n");

    stop();
    latch_tx();
}

int main(int argc, char *argv[])
{
    test();

    return 0;
}
