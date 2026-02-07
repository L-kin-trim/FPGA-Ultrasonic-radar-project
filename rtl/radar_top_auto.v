// 自动启动版本的雷达顶层模块
// 将start_en连接到高电平，系统复位后自动启动
module radar_top_auto (
    input wire clk,         // 50MHz
    input wire rst_n,
    
    // HC_SR04接口
    output wire trig,
    input wire echo,
    
    // SG90接口
    output wire servo_pwm,
    
    // 蜂鸣器接口
    output wire buzzer_out,
    
    // UART接口
    output wire uart_tx_out,
    input wire uart_rx_in
);

// 将start_en连接到高电平，实现自动启动
radar_top u_radar_top (
    .clk(clk),
    .rst_n(rst_n),
    .start_en(1'b1),  // 自动启动
    .trig(trig),
    .echo(echo),
    .servo_pwm(servo_pwm),
    .buzzer_out(buzzer_out),
    .uart_tx_out(uart_tx_out),
    .uart_rx_in(uart_rx_in)
);

endmodule
