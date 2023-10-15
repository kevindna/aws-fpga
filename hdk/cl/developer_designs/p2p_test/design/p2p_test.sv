module p2p_test (
  input clk,
  input rst_n,
  // Configuration
  input cfg_wr,
  input cfg_rd,
  input logic [31:0] cfg_addr,
  input logic [31:0] cfg_wdata,
  output logic tst_cfg_ack,
  output logic [31:0] cfg_rd_data,

  // Testing
  input logic pcim_arrdy,             // Read address channel ready
  input logic pcim_rvalid,            // Read channel valid
  // General Configuration
  // input start_test,             // Single cycle; starts testing
  // input test_type,              // 0 - Round Trip Time, 1 - Throughput
  // input [7:0]  num_trials,      // Number of trails; shared between tests
  // input [31:0] pause_cyc,       // Pause in cycles between consecutive trials
  // // Round Trip Configuration
  // // input [31:0] rt_pause_cyc,    // Pause in cycles
  // // Throughput Configuration
  // input [5:0]  tp_num_trans,    // Number transmissions
  // input [31:0] tp_pause_cyc,    // Pause in cycles
  //
  input tp_tx_done,             // Asserted when cl_tst has finished transmitting packets
  output logic [1:0] pcim_cntrl,      // Asserted to trigger functionality in PCIM; MSb is for write, LSb for reads
  output logic write_p2p_en);         // Hold this signal high for throughput test; add to logic for cfg_wr_go

//-----------------------------------------//
// Constants                               //
//-----------------------------------------//
localparam ADDR_LOW = 32'hE00;
localparam ADDR_HIGH = 32'hEFF;
localparam CNTR_WIDTH = 64;
localparam CNTR_MAX = {CNTR_WIDTH{1'b1}};

//-----------------------------------------//
// Types                                   //
//-----------------------------------------//
typedef enum logic[2:0] {     // Type for FSM
  IDLE      = 0,
  RTT_RD    = 1,  // Transmit packet
  RTT_WAIT  = 2,  // Wait until rvalid goes low
  PAUSE     = 3,  // Wait state in between round trip sends
  TP_TX     = 4,  // Transmit data
  TP_PAUSE  = 5   // Wait between throughput test (i.e. one throughput test, then wait in this state then go for rest of trials)
  } state_t;

//-----------------------------------------//
// Function                                //
//-----------------------------------------//
function [CNTR_WIDTH-1:0] add_sat (input [CNTR_WIDTH-1:0] cnt); 
  begin 
    add_sat = (cnt < CNTR_MAX) ? cnt + 1 : CNTR_MAX;
  end
endfunction

//-----------------------------------------//
// Signals                                 //
//-----------------------------------------//
state_t fsm_st;                 // FSM state variable
logic busy;                     // Asserted when test occurring 
logic test_type_reg;            // Register for test type
logic [CNTR_WIDTH-1:0] cnt;     // 64-bit counter
logic [7:0]  trials;            // Current numeber of trails
logic [31:0] trial_cnts [125:0]; // 100 entry array to store repeated trails for test
logic pcim_rvalid_reg;          // Register used for edge detection
logic pcim_rvalid_fe;           // Asserted on falling edge of pcim_rvalid
logic [1:0] ack_shft;           // Shift register for ack signal

// Config
logic start_test;             // Single cycle; starts testing
logic test_type;              // 0 - Round Trip Time, 1 - Throughput
logic [31:0] pause_cyc;       // General pause
logic [7:0]  num_trials;      // Number of trails; shared between tests
logic [7:0]  tp_num_trans;    // Number transmissions
logic [7:0]  tp_trans;        // Current transmission

//-----------------------------------------//
// Configuration bus FSM
//-----------------------------------------//
//    32'hE00: 0 - start, 1 - test type
//    32'hE04: 7:0 - num_trials, 13:8 - tp_num_trans
//    32'hE08: pause_cyc
//    32'hE0C: N/A
//    32'hE10: sample 0 low
//    32'hE14: sample 0 high
//    ...
//    32'hEF8: sample 62 low
//    32'hEFC: sample 62 high
always_ff @(posedge clk or negedge rst_n) begin
  if(rst_n == 1'b0) begin
    start_test    <= 1'b0;
    test_type     <= 1'b0;
    pause_cyc     <= 'b1;
    num_trials    <= 'b0;
    tp_num_trans  <= 'b0;
    ack_shft      <= 'b0;
	//tst_cfg_ack   <= 1'b0;
  end else begin

    //tst_cfg_ack <= (cfg_rd || cfg_wr) ? 1'b1 : 1'b0;  // Assert one cycle after
    ack_shft <= {ack_shft[0], (cfg_rd || cfg_wr)};  // Assert one cycle after

    // Place data as output
    if (cfg_rd == 1'b1) begin
      if (cfg_addr == 32'hE00) begin 
        cfg_rd_data <= {31'b0, busy};
      end else if (cfg_addr >= 32'hE10) begin
        cfg_rd_data <= ((cfg_addr <= ADDR_HIGH) && (cfg_addr >= ADDR_LOW)) ? trial_cnts[(cfg_addr - 32'hE10) >> 2] : 'h0BAD_F00D; // May fail timing
      end
    end

    // Extract configuration data
    if (cfg_wr == 1'b1) begin
      case (cfg_addr)
        32'hE00 : begin
          start_test <= cfg_wdata[0];
          test_type  <= cfg_wdata[1];
        end

        32'hE04 : begin
          num_trials   <= cfg_wdata[7:0];
          tp_num_trans <= cfg_wdata[15:8];
        end

        32'hE08 : begin
          pause_cyc <= cfg_wdata ;
        end

        default : test_type <= test_type; // Do Nothing
      endcase
    end
  end
end


// Edge detection for valid
always_ff @(posedge clk or negedge rst_n) begin
  if(rst_n == 1'b0) begin
     pcim_rvalid_reg <= 0;
  end else begin
     pcim_rvalid_reg <= pcim_rvalid;
  end
end


always_ff @(negedge rst_n or posedge clk) begin
  if(rst_n == 1'b0) begin
    fsm_st        <= IDLE;
    cnt           <= 'b0;
    trials        <= 'b0;
    tp_trans      <= 'b0;
    write_p2p_en  <= 1'b0;
    test_type_reg <= 1'b0;
    busy          <= 1'b0;
    pcim_cntrl    <= 2'b00;
    trial_cnts    <= '{default:'b0};
  end else begin
    case(fsm_st)
       IDLE: begin
        if (start_test == 1'b1) begin
          if (test_type == 1'b0) begin
            fsm_st     <= RTT_RD;
            pcim_cntrl <= 2'b01;
          end else begin
            fsm_st     <= TP_TX;
            pcim_cntrl <= 2'b10;
          end

          test_type_reg <= test_type;   // Register the test type to prevent change during test
          busy          <= 1'b1;
          trial_cnts    <= '{default:'b0};
        end else begin
          pcim_cntrl    <= 2'b00;
        end

        cnt           <= 'b0;
        trials        <= 'b0;
        write_p2p_en  <= 1'b0;


       end

       RTT_RD: begin
        if (pcim_rvalid_fe == 1'b1) begin
          if (trials == num_trials-1) begin
            busy   <= 1'b0;
            fsm_st <= IDLE;
          end else begin
            fsm_st <=  PAUSE;
          end

          cnt <= 'b0;
          trial_cnts[2*trials]   <= cnt[CNTR_WIDTH/2-1:0];
          trial_cnts[2*trials+1] <= cnt[CNTR_WIDTH-1:CNTR_WIDTH/2];
          // fsm_st <= (trials == num_trials-1) ? IDLE : PAUSE;
        end else begin
          cnt <= add_sat(cnt);
          // cnt <= cnt + 1;
        end

        pcim_cntrl    <= 2'b00;
       end


       TP_TX: begin
        if (tp_tx_done == 1'b1) begin
          if (tp_trans == tp_num_trans-1) begin
            if (trials == num_trials-1) begin
              busy   <= 1'b0;
              fsm_st <= IDLE;
            end else begin
              fsm_st <= PAUSE;
            end

            cnt         <= 'b0;
            tp_trans    <= 'b0;
            pcim_cntrl  <= 2'b00;
            trial_cnts[2*trials]   <= cnt[CNTR_WIDTH/2-1:0];
            trial_cnts[2*trials+1] <= cnt[CNTR_WIDTH-1:CNTR_WIDTH/2];
          end else begin
            cnt <= add_sat(cnt);
            tp_trans <= tp_trans + 1;
          end
        end
       end

      PAUSE: begin
        if (cnt[$left(pause_cyc):0] == pause_cyc-1) begin
          // if (test_type_reg == 1'b0) begin
          //   fsm_st <= RTT_RD;
          // end else begin
          //   fsm_st <= TP_TX;
          // end

          cnt    <= 'b0;
          trials <= trials + 1;
          fsm_st <= (test_type_reg == 1'b0) ? RTT_RD : TP_TX;
          pcim_cntrl <= (test_type_reg == 1'b0) ? 2'b01 : 2'b10;
        end else begin
          cnt <= cnt + 1;   // No need to protect if counter width > pause_cyc width
        end
       end


      default: write_p2p_en <= 1'b0;
		endcase
  end
end



/////////////////////////////////////////////
// Signals Assignments                     //
/////////////////////////////////////////////
// assign pause_cyc      <= (test_type_reg == 1'b0) ? rt_pause_cyc : tp_pause_cyc;    // Select puase
assign tst_cfg_ack    = ack_shft[1];
assign pcim_rvalid_fe = (pcim_rvalid_reg == 1'b1) && (pcim_rvalid == 1'b0);

endmodule
