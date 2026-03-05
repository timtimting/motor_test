#include <unistd.h>
#include <stdio.h>
#include "common.h"
#include "can_thread.h"

int main() {
    extern int motor_init();
    if (motor_init() != 0) {
        printf("motor init failed!\n");
        return -1;
    }else{
        printf("motor init success!\n");
    }

    sleep(1);
    motor_enable();
    printf("motor enabled!\n");

    sleep(3);

    printf("motor disabled!\n");
    motor_disable();

    sleep(1);
    return 0;
}
