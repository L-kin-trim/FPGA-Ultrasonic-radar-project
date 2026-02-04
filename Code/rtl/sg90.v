module sg90 (
    input wire clk,         // 50MHz
    input wire rst_n,
    
    input wire [7:0] angle,     // 角度值 0-180
    input wire angle_update,    // 角度更新使能
    output reg pwm_out,         // PWM输出
    output reg angle_ready      // 角度设置完成信号
);

parameter CLK_FREQ = 50_000_000;
parameter PWM_PERIOD = 1_000_000;  // 20ms = 1,000,000个时钟周期 (20ms * 50MHz)
parameter MIN_PULSE = 25_000;      // 0.5ms = 25,000个时钟周期 (0度)
parameter MAX_PULSE = 125_000;     // 2.5ms = 125,000个时钟周期 (180度)

reg [19:0] period_counter;
reg [19:0] pulse_width;
reg [7:0] current_angle;
reg angle_update_reg1, angle_update_reg2;
wire angle_update_pos_edge;

// 边沿检测
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        angle_update_reg1 <= 1'b0;
        angle_update_reg2 <= 1'b0;
    end else begin
        angle_update_reg1 <= angle_update;
        angle_update_reg2 <= angle_update_reg1;
    end
end

assign angle_update_pos_edge = angle_update_reg1 & ~angle_update_reg2;

// 计算PWM脉宽：pulse = MIN_PULSE + (angle / 180) * (MAX_PULSE - MIN_PULSE)
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        current_angle <= 8'd0;
        pulse_width <= MIN_PULSE;
        angle_ready <= 1'b0;
    end else begin
        if (angle_update_pos_edge) begin
            // 检测到angle_update上升沿，更新角度
            current_angle <= angle;
            // 计算脉宽：25,000 + angle * 555.56
            // 使用整数运算：pulse = 25000 + (angle * 100000) / 180
            pulse_width <= MIN_PULSE + ((angle * (MAX_PULSE - MIN_PULSE)) / 180);
            angle_ready <= 1'b1;
        end else if (angle_update_reg2) begin
            // angle_update为高时，保持angle_ready为1
            angle_ready <= 1'b1;
        end else begin
            // angle_update为低时，清除angle_ready
            angle_ready <= 1'b0;
        end
    end
end

// PWM生成
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        period_counter <= 20'd0;
        pwm_out <= 1'b0;
    end else begin
        if (period_counter < PWM_PERIOD - 1) begin
            period_counter <= period_counter + 1'b1;
        end else begin
            period_counter <= 20'd0;
        end
        
        if (period_counter < pulse_width) begin
            pwm_out <= 1'b1;
        end else begin
            pwm_out <= 1'b0;
        end
    end
end

endmodule
