module radar_top (
    input wire clk,         // 50MHz
    input wire rst_n,
    input wire start_en,    // 启动使能信号，高电平启动系统
    
    // HC_SR04接口
    output wire trig,
    input wire echo,
    
    // SG90接口
    output wire servo_pwm,
    
    // UART接口
    output wire uart_tx_out,
    input wire uart_rx_in
);

// 参数定义
parameter DEFAULT_ANGLE_STEP = 8'd10;      // 默认每次旋转10度
parameter DEFAULT_MEASURE_TIME = 32'd25_000_000;  // 默认每次测距0.5s (0.5s * 50MHz)
localparam MEASURE_CLK_PER_MS = 32'd50_000;
localparam MAX_MEASURE_MS = 32'd60000;
localparam MAX_MULTI_MEAS = 8'd20;
localparam SCAN_BIDIR = 2'd0;
localparam SCAN_SINGLE = 2'd1;
localparam SCAN_FIXED = 2'd2;

// 内部信号
wire [15:0] distance_mm;
wire measure_done;
reg measure_enable;
wire [7:0] servo_angle;
reg angle_update;
wire angle_ready;

// UART相关信号
reg [7:0] uart_data;
reg uart_tx_en;
wire uart_tx_busy;
wire uart_tx_done;
wire [7:0] uart_rx_data;
wire uart_rx_done;
wire uart_rx_busy;
wire uart_rx_en;

// 控制状态机
localparam IDLE = 4'd0;
localparam SET_ANGLE = 4'd1;
localparam WAIT_SERVO = 4'd2;
localparam START_MEASURE = 4'd3;
localparam WAIT_MEASURE = 4'd4;
localparam SEND_DISTANCE = 4'd5;
localparam SEND_ANGLE = 4'd6;
localparam WAIT_INTERVAL = 4'd7;

reg [3:0] state;
reg [7:0] current_angle;
reg [1:0] angle_direction;  // 0: 0->180, 1: 180->0
reg [31:0] measure_timer;
reg [31:0] interval_timer;
reg system_started;  // 系统启动标志

// 配置参数（由上位机下发）
reg [7:0] angle_step_cfg;
reg [31:0] measure_time_cfg;
reg [7:0] min_angle_cfg;
reg [7:0] max_angle_cfg;
reg [1:0] scan_mode_cfg;
reg auto_range_cfg;
reg multi_measure_en_cfg;
reg [7:0] multi_measure_count_cfg;
reg calib_mode_cfg;
reg [7:0] calib_angle_cfg;

// UART配置解析状态
localparam CFG_IDLE = 3'd0;
localparam CFG_READ = 3'd1;
reg [2:0] cfg_state;
reg [7:0] cfg_cmd;
reg [5:0] cfg_digits;
reg [31:0] cfg_value;

// 数据发送相关
reg [3:0] send_state;
reg [3:0] digit_index;
reg [15:0] temp_distance;
reg [7:0] temp_angle;
reg [7:0] digit_value;

// 多次测量相关
reg [23:0] multi_meas_sum;
reg [7:0] multi_meas_count;

function [7:0] get_distance_digit;
    input [15:0] dist;
    input [3:0] idx;
    begin
        case (idx)
            4'd4: get_distance_digit = dist / 1000;
            4'd3: get_distance_digit = (dist / 100) % 10;
            4'd2: get_distance_digit = (dist / 10) % 10;
            4'd1: get_distance_digit = dist % 10;
            default: get_distance_digit = 8'd0;
        endcase
    end
endfunction

function [7:0] get_angle_digit;
    input [7:0] angle;
    input [3:0] idx;
    begin
        case (idx)
            4'd3: get_angle_digit = angle / 100;
            4'd2: get_angle_digit = (angle / 10) % 10;
            4'd1: get_angle_digit = angle % 10;
            default: get_angle_digit = 8'd0;
        endcase
    end
endfunction

// 实例化HC_SR04模块
HC_SR04 u_hc_sr04 (
    .clk(clk),
    .rst_n(rst_n),
    .trig(trig),
    .echo(echo),
    .distance_mm(distance_mm),
    .measure_done(measure_done),
    .measure_enable(measure_enable)
);

// 实例化SG90模块
sg90 u_sg90 (
    .clk(clk),
    .rst_n(rst_n),
    .angle(servo_angle),
    .angle_update(angle_update),
    .pwm_out(servo_pwm),
    .angle_ready(angle_ready)
);

// 实例化UART发送模块
uart_tx u_uart_tx (
    .clk(clk),
    .rst_n(rst_n),
    .data_tx(uart_data),
    .tx_en(uart_tx_en),
    .tx_busy(uart_tx_busy),
    .tx_done(uart_tx_done),
    .tx_out(uart_tx_out)
);

assign uart_rx_en = 1'b1;
uart_rx u_uart_rx (
    .clk(clk),
    .rst_n(rst_n),
    .rx_pin_in(uart_rx_in),
    .rx_en(uart_rx_en),
    .rx_busy(uart_rx_busy),
    .rx_done(uart_rx_done),
    .rx_out(uart_rx_data)
);

assign servo_angle = current_angle;

// UART参数下发解析
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cfg_state <= CFG_IDLE;
        cfg_cmd <= 8'd0;
        cfg_digits <= 6'd0;
        cfg_value <= 32'd0;
        angle_step_cfg <= DEFAULT_ANGLE_STEP;
        measure_time_cfg <= DEFAULT_MEASURE_TIME;
        min_angle_cfg <= 8'd0;
        max_angle_cfg <= 8'd180;
        scan_mode_cfg <= SCAN_BIDIR;
        auto_range_cfg <= 1'b0;
        multi_measure_en_cfg <= 1'b0;
        multi_measure_count_cfg <= 8'd1;
        calib_mode_cfg <= 1'b0;
        calib_angle_cfg <= 8'd0;
    end else begin
        if (uart_rx_done) begin
            if (cfg_state == CFG_IDLE) begin
                if (uart_rx_data == 8'h53 || uart_rx_data == 8'h54 ||
                    uart_rx_data == 8'h4C || uart_rx_data == 8'h48 ||
                    uart_rx_data == 8'h4D || uart_rx_data == 8'h52 ||
                    uart_rx_data == 8'h45 || uart_rx_data == 8'h4E ||
                    uart_rx_data == 8'h43) begin // 'S'/'T'/'L'/'H'/'M'/'R'/'E'/'N'/'C'
                    cfg_cmd <= uart_rx_data;
                    cfg_state <= CFG_READ;
                    cfg_digits <= 6'd0;
                    cfg_value <= 32'd0;
                end
            end else begin
                if (uart_rx_data >= 8'h30 && uart_rx_data <= 8'h39) begin
                    if (cfg_digits < 6'd6) begin
                        cfg_value <= (cfg_value * 10) + (uart_rx_data - 8'h30);
                        cfg_digits <= cfg_digits + 1'b1;
                    end
                end else if (uart_rx_data == 8'h0D) begin // '\r'
                    // ignore CR
                end else if (uart_rx_data == 8'h0A) begin // '\n'
                    if (cfg_cmd == 8'h53) begin
                        if (cfg_value >= 32'd1 && cfg_value <= 32'd180)
                            angle_step_cfg <= cfg_value[7:0];
                    end else if (cfg_cmd == 8'h54) begin
                        if (cfg_value >= 32'd1 && cfg_value <= MAX_MEASURE_MS)
                            measure_time_cfg <= cfg_value * MEASURE_CLK_PER_MS;
                    end else if (cfg_cmd == 8'h4C) begin
                        if (cfg_value <= 32'd180 && cfg_value <= max_angle_cfg)
                            min_angle_cfg <= cfg_value[7:0];
                    end else if (cfg_cmd == 8'h48) begin
                        if (cfg_value <= 32'd180 && cfg_value >= min_angle_cfg)
                            max_angle_cfg <= cfg_value[7:0];
                    end else if (cfg_cmd == 8'h4D) begin
                        if (cfg_value <= 32'd2)
                            scan_mode_cfg <= cfg_value[1:0];
                        calib_mode_cfg <= 1'b0;
                    end else if (cfg_cmd == 8'h52) begin
                        if (cfg_value <= 32'd1)
                            auto_range_cfg <= cfg_value[0];
                    end else if (cfg_cmd == 8'h45) begin
                        if (cfg_value <= 32'd1)
                            multi_measure_en_cfg <= cfg_value[0];
                    end else if (cfg_cmd == 8'h4E) begin
                        if (cfg_value >= 32'd1 && cfg_value <= MAX_MULTI_MEAS)
                            multi_measure_count_cfg <= cfg_value[7:0];
                    end else if (cfg_cmd == 8'h43) begin
                        if (cfg_value <= 32'd180) begin
                            calib_angle_cfg <= cfg_value[7:0];
                            calib_mode_cfg <= 1'b1;
                        end
                    end
                    cfg_state <= CFG_IDLE;
                end else begin
                    cfg_state <= CFG_IDLE;
                end
            end
        end
    end
end

// 主控制状态机
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
        current_angle <= 8'd0;
        angle_direction <= 2'd0;
        measure_enable <= 1'b0;
        angle_update <= 1'b0;
        measure_timer <= 32'd0;
        interval_timer <= 32'd0;
        send_state <= 4'd0;
        system_started <= 1'b0;
        uart_tx_en <= 1'b0;
        multi_meas_sum <= 24'd0;
        multi_meas_count <= 8'd0;
    end else begin
        case (state)
            IDLE: begin
                angle_update <= 1'b0;
                measure_enable <= 1'b0;
                // 如果start_en为高，启动系统
                if (start_en && !system_started) begin
                    system_started <= 1'b1;
                    state <= SET_ANGLE;
                    if (calib_mode_cfg)
                        current_angle <= calib_angle_cfg;
                    else
                        current_angle <= min_angle_cfg;
                    angle_direction <= 2'd0;
                end else if (!start_en) begin
                    // start_en为低时，停止系统
                    system_started <= 1'b0;
                end else if (system_started) begin
                    // 系统已启动，继续运行
                    state <= SET_ANGLE;
                end
            end
            
            SET_ANGLE: begin
                angle_update <= 1'b1;
                measure_timer <= 32'd0;
                multi_meas_sum <= 24'd0;
                multi_meas_count <= 8'd0;
                // 等待一个时钟周期让angle_ready响应
                state <= WAIT_SERVO;
            end
            
            WAIT_SERVO: begin
                angle_update <= 1'b0;
                // 等待舵机稳定（约100ms = 5,000,000个时钟周期）
                if (measure_timer < 32'd5_000_000) begin
                    measure_timer <= measure_timer + 1'b1;
                end else begin
                    measure_timer <= 32'd0;
                    state <= START_MEASURE;
                end
            end
            
            START_MEASURE: begin
                measure_enable <= 1'b1;
                measure_timer <= 32'd0;
                state <= WAIT_MEASURE;
            end
            
            WAIT_MEASURE: begin
                if (measure_done) begin
                    measure_enable <= 1'b0;
                    measure_timer <= 32'd0;
                    if (multi_measure_en_cfg && multi_measure_count_cfg > 8'd1) begin
                        if (multi_meas_count + 1'b1 < multi_measure_count_cfg) begin
                            multi_meas_sum <= multi_meas_sum + distance_mm;
                            multi_meas_count <= multi_meas_count + 1'b1;
                            state <= START_MEASURE;
                        end else begin
                            multi_meas_sum <= multi_meas_sum + distance_mm;
                            multi_meas_count <= multi_meas_count + 1'b1;
                            temp_distance <= (multi_meas_sum + distance_mm) / multi_measure_count_cfg;
                            temp_angle <= current_angle;
                            send_state <= 4'd0;
                            digit_index <= 4'd0;
                            state <= SEND_DISTANCE;
                        end
                    end else begin
                        state <= SEND_DISTANCE;
                        temp_distance <= distance_mm;
                        temp_angle <= current_angle;
                        send_state <= 4'd0;
                        digit_index <= 4'd0;
                    end
                end else if (measure_timer >= measure_time_cfg) begin
                    // 超时，放弃本次测量
                    measure_enable <= 1'b0;
                    measure_timer <= 32'd0;
                    if (multi_measure_en_cfg && multi_measure_count_cfg > 8'd1) begin
                        if (multi_meas_count + 1'b1 < multi_measure_count_cfg) begin
                            multi_meas_count <= multi_meas_count + 1'b1;
                            state <= START_MEASURE;
                        end else begin
                            temp_distance <= (multi_meas_sum) / multi_measure_count_cfg;
                            temp_angle <= current_angle;
                            send_state <= 4'd0;
                            digit_index <= 4'd0;
                            state <= SEND_DISTANCE;
                        end
                    end else begin
                        state <= WAIT_INTERVAL;
                    end
                end else begin
                    measure_timer <= measure_timer + 1'b1;
                end
            end
            SEND_DISTANCE: begin
                case (send_state)
                    4'd0: begin  // 发送'D'
                        if (!uart_tx_busy && !uart_tx_en) begin
                            uart_data <= 8'h44;  // 'D'的ASCII码
                            uart_tx_en <= 1'b1;
                            send_state <= 4'd1;
                        end
                    end
                    4'd1: begin
                        uart_tx_en <= 1'b0;
                        if (uart_tx_done) begin
                            send_state <= 4'd2;
                            // 计算距离的位数
                            if (temp_distance >= 1000) digit_index <= 4'd4;
                            else if (temp_distance >= 100) digit_index <= 4'd3;
                            else if (temp_distance >= 10) digit_index <= 4'd2;
                            else digit_index <= 4'd1;
                        end
                    end
                    4'd2: begin  // 发送距离数字
                        if (!uart_tx_busy && !uart_tx_en) begin
                            digit_value <= get_distance_digit(temp_distance, digit_index);
                            uart_data <= 8'h30 + get_distance_digit(temp_distance, digit_index);  // 转换为ASCII
                            uart_tx_en <= 1'b1;
                            send_state <= 4'd3;
                        end
                    end
                    4'd3: begin
                        uart_tx_en <= 1'b0;
                        if (uart_tx_done) begin
                            if (digit_index > 4'd1) begin
                                digit_index <= digit_index - 1'b1;
                                send_state <= 4'd2;
                            end else begin
                                send_state <= 4'd4;
                            end
                        end
                    end
                    4'd4: begin  // 发送'A'
                        if (!uart_tx_busy && !uart_tx_en) begin
                            uart_data <= 8'h41;  // 'A'的ASCII码
                            uart_tx_en <= 1'b1;
                            send_state <= 4'd5;
                        end
                    end
                    4'd5: begin
                        uart_tx_en <= 1'b0;
                        if (uart_tx_done) begin
                            send_state <= 4'd6;
                            // 计算角度的位数
                            if (temp_angle >= 100) digit_index <= 4'd3;
                            else if (temp_angle >= 10) digit_index <= 4'd2;
                            else digit_index <= 4'd1;
                        end
                    end
                    4'd6: begin  // 发送角度数字
                        if (!uart_tx_busy && !uart_tx_en) begin
                            digit_value <= get_angle_digit(temp_angle, digit_index);
                            uart_data <= 8'h30 + get_angle_digit(temp_angle, digit_index);  // 转换为ASCII
                            uart_tx_en <= 1'b1;
                            send_state <= 4'd7;
                        end
                    end
                    4'd7: begin
                        uart_tx_en <= 1'b0;
                        if (uart_tx_done) begin
                            if (digit_index > 4'd1) begin
                                digit_index <= digit_index - 1'b1;
                                send_state <= 4'd6;
                            end else begin
                                // 发送换行符
                                send_state <= 4'd8;
                            end
                        end
                    end
                    4'd8: begin  // 发送换行符
                        if (!uart_tx_busy && !uart_tx_en) begin
                            uart_data <= 8'h0A;  // '\n'
                            uart_tx_en <= 1'b1;
                            send_state <= 4'd9;
                        end
                    end
                    4'd9: begin
                        uart_tx_en <= 1'b0;
                        if (uart_tx_done) begin
                            state <= WAIT_INTERVAL;
                            send_state <= 4'd0;
                        end
                    end
                    default: send_state <= 4'd0;
                endcase
            end
            
            WAIT_INTERVAL: begin
                // 等待间隔时间（默认0.5s）
                if (interval_timer < measure_time_cfg) begin
                    interval_timer <= interval_timer + 1'b1;
                end else begin
                    interval_timer <= 32'd0;
                    // 根据扫描模式更新角度
                    if (calib_mode_cfg) begin
                        current_angle <= calib_angle_cfg;
                        angle_direction <= 2'd0;
                    end else if (scan_mode_cfg == SCAN_FIXED) begin
                        current_angle <= min_angle_cfg;
                        angle_direction <= 2'd0;
                    end else if (scan_mode_cfg == SCAN_SINGLE) begin
                        if (current_angle < min_angle_cfg) begin
                            current_angle <= min_angle_cfg;
                        end else if (current_angle > max_angle_cfg) begin
                            current_angle <= min_angle_cfg;
                        end else if (current_angle + angle_step_cfg <= max_angle_cfg) begin
                            current_angle <= current_angle + angle_step_cfg;
                        end else begin
                            current_angle <= min_angle_cfg;
                        end
                        angle_direction <= 2'd0;
                    end else begin
                        // 往返扫描（限制在最小/最大角度范围内）
                        if (current_angle < min_angle_cfg) begin
                            current_angle <= min_angle_cfg;
                            angle_direction <= 2'd0;
                        end else if (current_angle > max_angle_cfg) begin
                            current_angle <= max_angle_cfg;
                            angle_direction <= 2'd1;
                        end else if (angle_direction == 2'd0) begin
                            if (current_angle + angle_step_cfg <= max_angle_cfg) begin
                                current_angle <= current_angle + angle_step_cfg;
                            end else begin
                                current_angle <= max_angle_cfg;
                                angle_direction <= 2'd1;
                            end
                        end else begin
                            if (current_angle >= (min_angle_cfg + angle_step_cfg)) begin
                                current_angle <= current_angle - angle_step_cfg;
                            end else begin
                                current_angle <= min_angle_cfg;
                                angle_direction <= 2'd0;
                            end
                        end
                    end
                    state <= SET_ANGLE;
                end
            end
            
            default: state <= IDLE;
        endcase
    end
end

endmodule
