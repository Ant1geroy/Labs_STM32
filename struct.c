#include <stdio.h>

struct microcontroller
{
    float clock_speed;
    int cores;
    int flash_memory;
    int ram;
};

int main()
{
    struct microcontroller stm32f407vg =
    {
        0.168,
        1,
        1024,
        192
    };
    printf("Микроконтроллер STM32F407VG (до изменения):\n");
    printf("Тактовая частота: %.3f GHz\n", stm32f407vg.clock_speed);
    printf("Количество ядер: %d\n", stm32f407vg.cores);
    printf("Флеш-память: %d KB\n", stm32f407vg.flash_memory);
    printf("ОЗУ: %d KB\n\n", stm32f407vg.ram);


    stm32f407vg.clock_speed = 0.180;
    stm32f407vg.cores = 2;  
    stm32f407vg.flash_memory = 2048;
    stm32f407vg.ram = 512;


    printf("Микроконтроллер STM32F407VG (после изменения):\n");
    printf("Тактовая частота: %.3f GHz\n", stm32f407vg.clock_speed);
    printf("Количество ядер: %d\n", stm32f407vg.cores);
    printf("Флеш-память: %d KB\n", stm32f407vg.flash_memory);
    printf("ОЗУ: %d KB\n", stm32f407vg.ram);

    return 0;
}
