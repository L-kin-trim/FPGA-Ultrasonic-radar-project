module uart_rx(
    input wire clk,
    input wire rst_n,
    input wire rx_pin_in,
    input wire rx_en,
    output reg rx_busy,
    output reg rx_done,
    output reg [7:0] rx_out
); 

parameter BAUD_RATE = 115200;
parameter CLK_FREQ = 50_000_000;  // 50MHz时钟
parameter BAUD_TICK = CLK_FREQ / BAUD_RATE;

// 状态机定义
localparam IDLE = 4'd0;
localparam START_BIT = 4'd1;
localparam DATA_BITS = 4'd2;
localparam STOP_BIT = 4'd3;
localparam DONE = 4'd4;

reg [3:0] state;
reg [3:0] bit_count;
reg [25:0] baud_counter;
reg [7:0] rx_data;
reg rx_reg1;
reg rx_reg2;
wire baud_tick;
wire half_tick;
wire neg_edge_detect;
wire baud_counter_reset;

// 边沿检测 - 检测起始位下降沿
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_reg1 <= 1'b1;
        rx_reg2 <= 1'b1;
    end else begin
        rx_reg1 <= rx_pin_in;
        rx_reg2 <= rx_reg1;
    end
end

assign neg_edge_detect = rx_reg2 & ~rx_reg1;  // 下降沿检测

// 波特率时钟生成（仅在接收期间计数，起始位对齐）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        baud_counter <= 26'd0;
    end else begin
        if (baud_counter_reset) begin
            baud_counter <= 26'd0;
        end else if (state != IDLE) begin
            if (baud_counter == BAUD_TICK - 1) 
                baud_counter <= 26'd0;
            else 
                baud_counter <= baud_counter + 1'b1;
        end else begin
            baud_counter <= 26'd0;
        end
    end
end

assign baud_tick = (baud_counter == BAUD_TICK - 1);
assign half_tick = (baud_counter == (BAUD_TICK / 2));
assign baud_counter_reset = (state == IDLE) || (state == START_BIT && half_tick);

// UART接收状态机
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
        rx_busy <= 1'b0;
        rx_done <= 1'b0;
        bit_count <= 4'd0;
        rx_out <= 8'd0;
    end else begin
        case (state)
            IDLE: begin
                rx_done <= 1'b0;
                if (rx_en && neg_edge_detect) begin  // 检测到下降沿且使能接收
                    rx_busy <= 1'b1;
                    state <= START_BIT;
                end else begin
                    rx_busy <= 1'b0;
                end
            end
            
            START_BIT: begin
                if (half_tick) begin
                    // 在起始位中间采样，确认有效起始位
                    if (rx_pin_in == 1'b0) begin
                        state <= DATA_BITS;
                        bit_count <= 4'd0;
                    end else begin
                        rx_busy <= 1'b0;
                        state <= IDLE;
                    end
                end
            end
            
            DATA_BITS: begin
                if (baud_tick) begin
                    // 在每个数据位的中间采样
                    rx_data[bit_count] <= rx_pin_in;
                    if (bit_count == 4'd7) begin
                        state <= STOP_BIT;
                    end else begin
                        bit_count <= bit_count + 1'b1;
                    end
                end
            end
            
            STOP_BIT: begin
                if (baud_tick) begin
                    // 检查停止位是否为高电平
                    if (rx_pin_in == 1'b1) begin
                        rx_out <= rx_data;
                        state <= DONE;
                    end else begin
                        // 停止位错误，返回空闲状态
                        rx_busy <= 1'b0;
                        state <= IDLE;
                    end
                end
            end
            
            DONE: begin
                rx_busy <= 1'b0;
                rx_done <= 1'b1;
                state <= IDLE;
            end
            
            default: state <= IDLE;
        endcase
    end
end

endmodule