module HC_SR04 (
    (*keep*)input wire clk,         // 50MHz
    input wire rst_n,

    output reg trig,
    (*keep*)input wire echo,

    output reg [15:0] distance_mm,  // 距离值，单位mm
    output reg measure_done,        // 测量完成信号
    input wire measure_enable       // 测量使能信号（由外部控制）
);

parameter CLK_FREQ = 50_000_000;    // 50MHz
parameter TRIG_PULSE_TIME = 500;    // 10us = 500个时钟周期 (10us * 50MHz = 500)
parameter MAX_DISTANCE_MM = 3000;   // 最大量程3m = 3000mm
// 声速340m/s，1mm对应时间 = 0.001m / 340m/s * 2 = 5.882us
// 5.882us * 50MHz = 294.1个时钟周期，取294
parameter CLK_PER_MM = 294;         // 每毫米对应的时钟周期数

// 状态机定义
localparam IDLE = 3'd0;
localparam TRIG_PULSE = 3'd1;
localparam WAIT_ECHO = 3'd2;
localparam MEASURE_ECHO = 3'd3;
localparam CALCULATE = 3'd4;
localparam DONE = 3'd5;

reg [2:0] state;
reg [9:0] trig_counter;
reg [23:0] echo_counter;            // 最大支持约57m，足够3m量程
reg echo_reg1, echo_reg2;
wire echo_pos_edge;
wire echo_neg_edge;

// Echo信号边沿检测
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        echo_reg1 <= 1'b0;
        echo_reg2 <= 1'b0;
    end else begin
        echo_reg1 <= echo;
        echo_reg2 <= echo_reg1;
    end
end

assign echo_pos_edge = ~echo_reg2 & echo_reg1;  // 上升沿
assign echo_neg_edge = echo_reg2 & ~echo_reg1;  // 下降沿

// HC_SR04测距状态机
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
        trig <= 1'b0;
        trig_counter <= 10'd0;
        echo_counter <= 24'd0;
        distance_mm <= 16'd0;
        measure_done <= 1'b0;
    end else begin
        case (state)
            IDLE: begin
                measure_done <= 1'b0;
                trig <= 1'b0;
                echo_counter <= 24'd0;
                if (measure_enable) begin
                    state <= TRIG_PULSE;
                    trig_counter <= 10'd0;
                end
            end
            
            TRIG_PULSE: begin
                trig <= 1'b1;
                if (trig_counter < TRIG_PULSE_TIME - 1) begin
                    trig_counter <= trig_counter + 1'b1;
                end else begin
                    trig <= 1'b0;
                    state <= WAIT_ECHO;
                    echo_counter <= 24'd0;
                end
            end
            
            WAIT_ECHO: begin
                if (echo_pos_edge) begin
                    state <= MEASURE_ECHO;
                    echo_counter <= 24'd0;
                end else if (echo_counter >= (MAX_DISTANCE_MM * CLK_PER_MM)) begin
                    // 超时，超出量程
                    state <= DONE;
                    distance_mm <= 16'd0;  // 超出量程返回0
                end else begin
                    echo_counter <= echo_counter + 1'b1;
                end
            end
            
            MEASURE_ECHO: begin
                if (echo_neg_edge) begin
                    // Echo下降沿，测量完成
                    state <= CALCULATE;
                end else if (echo_counter >= (MAX_DISTANCE_MM * CLK_PER_MM)) begin
                    // 超出量程
                    state <= DONE;
                    distance_mm <= 16'd0;
                end else begin
                    echo_counter <= echo_counter + 1'b1;
                end
            end
            
            CALCULATE: begin
                // 计算距离：distance = echo_counter / CLK_PER_MM
                if (echo_counter / CLK_PER_MM > MAX_DISTANCE_MM) begin
                    distance_mm <= 16'd0;  // 超出量程
                end else begin
                    distance_mm <= echo_counter / CLK_PER_MM;
                end
                state <= DONE;
            end
            
            DONE: begin
                measure_done <= 1'b1;
                if (!measure_enable) begin
                    state <= IDLE;
                end
            end
            
            default: state <= IDLE;
        endcase
    end
end

endmodule