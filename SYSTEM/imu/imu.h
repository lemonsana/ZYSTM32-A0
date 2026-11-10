#ifndef __IMU_H_
#define __IMU_H_

#include "stm32f10x.h"

/* ============================================================================
 *  MPU6050（GY-521）陀螺仪  V7
 *  ----------------------------------------------------------------------------
 *  只做一件事：给出**航向角 yaw**（累计角度，degree），给十字直行和找线用。
 *  不读加速度、不做姿态解算、不用 DMP —— 那些对"十字里走直线"没有帮助，
 *  反而要多花 1ms 的 I2C 时间。
 *
 *  【重要：它走 I2C，不占串口】
 *      MPU6050/GY-521 是 I2C 器件（SCL/SDA 两根线），USART2（PA2/PA3）留给
 *      K210 视觉，USART1（PA9/PA10）是调试口，都不动。
 *      如果你买的是 JY901/JY61 那种"TX/RX 串口模块"，那是另一套驱动，
 *      在 issue 里说一声，我再补。
 *
 *  【为什么单独一条 I2C，不和灰度模块共用 PB10/PB11】
 *      灰度模块的 PULL 跳线是 **5V 上拉**，总线空闲电平 5V；
 *      而 MPU6050 的 SCL/SDA 是 3.3V 器件、**不耐 5V**，直接并上去有烧片风险。
 *      所以这里用另一条软件 I2C（两条空闲脚），互不干扰。
 *
 *  【接线（默认 PC4/PC5）】
 *      GY-521 VCC  → 5V   （模块自带 3.3V LDO；接 3.3V 也行）
 *      GY-521 GND  → GND
 *      GY-521 SCL  → PC4  （丝印看板子，找不到就换引脚，见下）
 *      GY-521 SDA  → PC5
 *      GY-521 AD0  → GND  （地址 0x68；接 3.3V 则变 0x69）
 *      GY-521 INT  → 不接
 *
 *  【PC4/PC5 没引出来怎么办】
 *      只改下面 4 个宏就行，任意两个"3.3V、空闲、没被 5V 上拉"的脚都可以：
 *          备选 1：PB12 / PB13
 *          备选 2：PA5  / PA7
 *      不要用：PB10/PB11（灰度 5V 上拉）、PA2/PA3（留给 K210）、
 *              PB3/PB4（JTAG）、PA13/PA14（SWD 下载）。
 *
 *  【安装方向】
 *      平贴在车上，X/Y 轴水平即可；yaw 只对 Z 轴敏感，倾斜 <10° 影响可忽略。
 *      装好后**必须标定一次方向**：手拿车**左转 90°**，看串口 yaw 是变 +
 *      还是变 -；如果是 -，把 IMU_YAW_SIGN 改成 -1。
 * ========================================================================== */

/* ---------------- 接线（改这里就能换引脚） ---------------- */
#define IMU_SCL_GPIO        GPIOC
#define IMU_SCL_PIN         GPIO_Pin_4
#define IMU_SDA_GPIO        GPIOC
#define IMU_SDA_PIN         GPIO_Pin_5
#define IMU_GPIO_RCC        RCC_APB2Periph_GPIOC   /* 两个脚必须在同一个端口 */

#define IMU_I2C_ADDR        0x68    /* AD0 接 GND；接 3.3V 改成 0x69 */
#define IMU_I2C_DELAY_US    4       /* 半周期延时，4us ≈ 125kHz */

/* ---------------- 采样与积分 ---------------- */
#define IMU_UPDATE_MS       10      /* 读取+积分周期，必须与 main.c 注册的周期一致 */
#define IMU_CALIB_MS        1500    /* 上电静止标定时长（这段时间车不能动） */
#define IMU_GYRO_LSB        65.5f   /* ±500 dps 量程：65.5 LSB/(°/s) */
#define IMU_YAW_SIGN        1       /* 左转 yaw 增大填 1；左转减小填 -1 */

/* ---------------- 接口 ---------------- */
u8    Imu_Init(void);           /* 1 = 找到芯片并标定完成（期间阻塞 ~1.5s） */
u8    Imu_IsReady(void);        /* 1 = 可用；0 = 没接/通信失败/连续读错 */
u8    Imu_WhoAmI(void);         /* 读到的 WHO_AM_I，正常 0x68 */
float Imu_GetYaw(void);         /* 累计航向角（度），左转为正 */
float Imu_GetGz(void);          /* 当前 Z 轴角速度（度/秒），已减零偏 */
float Imu_GetBias(void);        /* 标定得到的零偏（度/秒），调试用 */
void  Imu_ResetYaw(void);       /* 把当前角度归零 */
void  Imu_Update(void);         /* 由 10ms 后台任务调用：读一次、积分一次 */

#endif
