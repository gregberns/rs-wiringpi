// http://osoyoo.com/driver/pi3_start_learning_kit_lesson_19/IRtest.c
/********************************************************************************************
*Filename      : IRtest.c
*Description   : IR remote control LEDs.
*Website       : www.osoyoo.com
*Update        : 2017/07/07
*              KEY_1----------------LED1
*              KEY_2                LED2
*              KEY_3----------------LED3
********************************************************************************************/
#include <wiringPi.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <lirc/lirc_client.h>
#include <time.h>

void flipLED(int led);

//The WiringPi pin numbers used by our LEDs
#define LED1 5
#define LED2 6
#define LED3 7

#define ON 1
#define OFF 0

int main(int argc, char *argv[])
{
    struct lirc_config *config;

    //Timer for our buttons
    int buttonTimer = millis();

    char *code;
    char *c;

    //Initiate WiringPi and set WiringPi pins 4, 5 & 6 (GPIO 23, 24 & 25) to output. These are the pins the LEDs are connected to.
    if (wiringPiSetup() == -1)
        exit(1);

    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(LED3, OUTPUT);

    //Initiate LIRC. Exit on failure
    if (lirc_init("lirc", 1) == -1)
        exit(EXIT_FAILURE);

    //Read the default LIRC config at /etc/lirc/lircd.conf  This is the config for your remote.
    if (lirc_readconfig(NULL, &config, NULL) == 0)
    {
        //Do stuff while LIRC socket is open  0=open  -1=closed.
        while (lirc_nextcode(&code) == 0)
        {
            //If code = NULL, meaning nothing was returned from LIRC socket,
            //then skip lines below and start while loop again.
            if (code == NULL)
                continue;
            {
                //Make sure there is a 400ms gap before detecting button presses.
                if (millis() - buttonTimer > 400)
                {
                    //Check to see if the string "KEY_1" appears anywhere within the string 'code'.
                    if (strstr(code, "KEY_1"))
                    {
                        printf("MATCH on KEY_1\n");
                        flipLED(LED1);
                        buttonTimer = millis();
                    }
                    else if (strstr(code, "KEY_2"))
                    {
                        printf("MATCH on KEY_2\n");
                        flipLED(LED2);
                        buttonTimer = millis();
                    }
                    else if (strstr(code, "KEY_3"))
                    {
                        printf("MATCH on KEY_3\n");
                        flipLED(LED3);
                        buttonTimer = millis();
                    }
                }
            }
            //Need to free up code before the next loop
            free(code);
        }
        //Frees the data structures associated with config.
        lirc_freeconfig(config);
    }
    //lirc_deinit() closes the connection to lircd and does some internal clean-up stuff.
    lirc_deinit();
    exit(EXIT_SUCCESS);
}

void flipLED(int led)
{
    //If LED is on, turn it off. Otherwise it is off, so thefore we need to turn it on.
    if (digitalRead(led) == ON)
        digitalWrite(led, OFF);
    else
        digitalWrite(led, ON);
}