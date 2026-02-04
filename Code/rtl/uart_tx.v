module uart_tx(
    input wire clk,
    input wire rst_n,
    input wire [7:0]data_tx,
    input wire tx_en,
    output reg tx_busy,
    output reg tx_done,
    output reg tx_out
);

parameter BAUD_RATE = 115_200;
parameter CLK_FREQ = 50_000_000; 
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
reg [7:0] tx_data;
wire baud_tick;

// 波特率时钟生成
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        baud_counter <= 26'd0;
    end else begin
        if (baud_counter == BAUD_TICK - 1) 
            baud_counter <= 26'd0;
        else 
            baud_counter <= baud_counter + 1'b1;
    end
end

assign baud_tick = (baud_counter == BAUD_TICK - 1);

// UART发送状态机
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
        tx_out <= 1'b1;
        tx_busy <= 1'b0;
        tx_done <= 1'b0;
        bit_count <= 4'd0;
    end else begin
        case (state)
            IDLE: begin
                tx_out <= 1'b1;
                tx_done <= 1'b0;
                if (tx_en) begin
                    tx_busy <= 1'b1;
                    tx_data <= data_tx;
                    state <= START_BIT;
                end else begin
                    tx_busy <= 1'b0;
                end
            end
            
            START_BIT: begin
                if (baud_tick) begin
                    tx_out <= 1'b0;  // 发送起始位(低电平)
                    state <= DATA_BITS;
                    bit_count <= 4'd0;
                end
            end
            
            DATA_BITS: begin
                if (baud_tick) begin
                    tx_out <= tx_data[bit_count];  // 从低位开始发送数据位
                    if (bit_count == 4'd7) begin
                        state <= STOP_BIT;
                    end else begin
                        bit_count <= bit_count + 1'b1;
                    end
                end
            end
            
            STOP_BIT: begin
                if (baud_tick) begin
                    tx_out <= 1'b1;  // 发送停止位(高电平)
                    state <= DONE;
                end
            end
            
            DONE: begin
                tx_busy <= 1'b0;
                tx_done <= 1'b1;
                state <= IDLE;
            end
            
            default: state <= IDLE;
        endcase
    end
end

endmodule