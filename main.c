#include "AllHeader.h"

#define UPLOAD_DATA 1  
#define MOTOR_TYPE 5   

uint8_t times = 0;

int main(void)
{   
    bsp_init();
    TIM3_Init();
    IIC_Motor_Init();
    printf("System Init OK...\r\n");

    #if MOTOR_TYPE == 5
    Set_motor_type(1); delay_ms(100);
    Set_Pluse_Phase(40); delay_ms(100);
    Set_Pluse_line(11); delay_ms(100);
    Set_Wheel_dis(67.00); delay_ms(100);
    Set_motor_deadzone(1900); delay_ms(100);
    #endif

    control_pwm(0, 0, 0, 0);
    delay_ms(100);

    uint8_t last_cmd = 'S'; 
    uint16_t watchdog_timeout = 0;
    
    // ?? 新增：转向微步计时器（单位：毫秒）
    uint16_t turn_timer = 0; 
    uint8_t is_turning_step = 0;

    while(1)
    {
        // 清除 ORE 错误，永不死机
        if(USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET) {
            USART_ReceiveData(USART1); 
        }

        if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET)
        {
            uint8_t cmd = USART_ReceiveData(USART1); 
            watchdog_timeout = 0; // ?? 只要收到任意串口数据，立刻喂狗！全面拒绝锁死
            
            if(cmd != last_cmd)
            {
                switch(cmd)
                {
                    case 'F': 
                        is_turning_step = 0; // 直行不限制时间
                        control_pwm(0, 1915, 0, 1965); 
                        break;
                    case 'B': 
                        is_turning_step = 0;
                        control_pwm(0, -1950, 0, -1960); // 温柔倒车
                        break;
                        
                    case 'L': 
                        // 微步左：给足动力(1940)，但启动硬件限时
                        control_pwm(0, -1930, 0, 1940); 
                        turn_timer = 0;         // 计时器清零
                        is_turning_step = 1;    // 开启限时标记
                        break;
                    case 'R': 
                        // 微步右转
                        control_pwm(0, 1930, 0, -1930);
                        turn_timer = 0;
                        is_turning_step = 1;
                        break;

                    case 'Q': 
                        // 微步左后倒车】：双轮同时后退，右轮稍快，开启硬件35ms限时
                        control_pwm(0, -1915, 0, -1935); 
                        turn_timer = 0;
                        is_turning_step = 1; 
                        break;
                    case 'E': 
                        // ??【原生微步右后倒车】：双轮同时后退，左轮稍快，开启硬件35ms限时
                        control_pwm(0, -1935, 0, -1915); 
                        turn_timer = 0;
                        is_turning_step = 1; 
                        break;

                    case 'S': 
                    default:
                        is_turning_step = 0;
                        control_pwm(0, 0, 0, 0); 
                        break;
                }
                last_cmd = cmd; 
            }
        }
        else 
        {
            watchdog_timeout++;
            if(watchdog_timeout > 500) 
            {
                control_pwm(0, 0, 0, 0);       
                last_cmd = 'S';          
                is_turning_step = 0;
                watchdog_timeout = 500;  
            }
        }

        // ?? 核心硬件控速：如果正在进行手势转向，强制限制通电时间
        if(is_turning_step)
        {
            turn_timer++;
            // 35毫秒内：提供充沛动力破冰。超过35毫秒：立刻硬件刹车锁定！
            // 这样无论 Python 发得多猛，小车每次转向都只走极小的一微步
            if(turn_timer > 35) 
            {
                control_pwm(0, 0, 0, 0); // 刹车
                // 注意：这里不要改 last_cmd，保持为 'L' 或 'R'
                // 这样当 Python 下次发不同的指令或者重新触发时，才能正常响应
            }
        }

        // 汇报编码器数据
        times++;
        if(times >= 50) 
        {
            #if UPLOAD_DATA == 1
            Read_ALL_Enconder();
            printf("M2:%d\t M4:%d\t \r\n", Encoder_Now[1], Encoder_Now[3]);
            #endif
            times = 0;
        }
        delay_ms(1); 
    }
}