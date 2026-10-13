#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led.h"

/* 任务函数：必须死循环，且循环里必须有阻塞(vTaskDelay等)，否则会饿死别的任务 */
static void led_task(void *arg)
{
    static bool led_on = false;
    while (1)
    {
        if (led_on)
        {
            LED_SetRGB(0, 0, 0);
        }
        else
        {
            LED_SetRGB(20, 0, 0);
        }
        led_on = !led_on;
        vTaskDelay(pdMS_TO_TICKS(2000));   /* 关键：让出 CPU */
    }
}

/* 对外只暴露"启动"，不暴露任务函数本身 */
void LedTask_Start(void)
{
    xTaskCreate(led_task, "led_task", 3072, NULL, 5, NULL);
}
