# 数传电台
> PLC-18SPR-32080001
> 一个Sub-GHz工业数传电台
## 功能概述
1. 16通道数字隔离输出（NPN）和16通道数字隔离输入（NPN和PNP）
2. 4通道模拟输入和4通道模拟输出（10Bit分辨率，输入和输出通过接线方式改变实现0~10V电压和4~20mA电流切换）
3. 内部具有一个带独立读写保护开关的GD25Q64ESIGR存储器
4. 具有一个RA-01射频模块
5. 具有一个带隔离的RS-485接口
## 芯片列表
1. 主控制器芯片：STM8S208MBT6
2. DAC芯片：AD5314ARMZ
3. 存储芯片：GD25Q64ESIGR
## 主要连接
1. DI7~DI0：PB7~PB0
2. DI15~DI8：PE7~PE0
3. DO7~DO0：PG7~PG0
4. DO15~DO8：PI7~PI0
5. UART1_TX=PA5, UART1_RX=PA4, UART1_CTRL=PA3：接 ISO3082DWR 的 D/R，CTRL 同时连 DE 和 RE#
   （D→UART1_TX, R→UART1_RX, DE+RE#→UART1_CTRL；半双工，CTRL 高=发送 低=接收）
6. UART3_RX/TX：CH340N，用于上位机改参数
## 注记
1. 原计划是用模拟开关实现CAN切换+光耦，后考虑成本，放弃CAN支持
2. DI输入引发双向光耦输出端导通时，对应的MCU上的IO变为低电平
3. DO在MCU端侧输出低电平时，单向光耦输出测截止，此时输出NMOS对COM-导通
4. AI将0~20mA或0~10V映射到0~3.3V，ADC的VREF=3.3V，AO将0~20mA或0~10V映射到0~3.3V，DAC的VREF=3.3V
5. UART1是专用485透传，UART3是数传电台/上位机接口
6. MCU上的RF_IRQ、CE、CSN均连接到SPI射频通信模块RA-01上
7. MCU的SPI_CS_N连接到GD25Q64ESIGR的CS上，其中GD25Q64ESIGR的WP由另一个电路机械开关处理
8. MCU的SPI_nSYNC连接到AD5314ARMZ上
9. 三个指示灯（RUN_LED、DEBUG_LED、SYSTEM_LED）均是推挽点亮
10. 两个拨码开关（RUN_CTRL和DEBUG_CTRL）使能时候应是低电平
11. NRST和TLI均是上拉的，当对应按钮按下则变为低电平
12. 主时钟采用16MHz内部RC（HSI，外部晶振暂未启用）
13. 拨码模式（拨码使能=低电平；LED 跟随拨码，使能=亮）：
    DEBUG=PH3, RUN=PH2, 模式号=1+RUN*2+DEBUG
    | 模式 | DEBUG | RUN | 说明 | UART3 | DEBUG_LED | RUN_LED |
    |------|:-:|:-:|------|-------|:-:|:-:|
    | 1 | 0 | 0 | 只读配置：仅允许读取，禁止写入 | 允许(只读) | 灭 | 灭 |
    | 2 | 1 | 0 | 读写配置：上位机读写参数 | 允许(读写) | 亮 | 灭 |
    | 3 | 0 | 1 | 远程发射：按序列远程发送任务(待实现) | 禁止 | 灭 | 亮 |
    | 4 | 1 | 1 | 本机直通：所有DI->DO，所有AI->AO | 禁止 | 亮 | 亮 |