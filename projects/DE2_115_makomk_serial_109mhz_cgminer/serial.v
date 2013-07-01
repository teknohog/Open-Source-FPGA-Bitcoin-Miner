// by teknohog, replaces virtual_wire by rs232

// Now using fpgaminer's uart code instead of fpga4fun's. Besides
// GPL3, the practical difference is that instead of TxD_busy we have
// its inverse tx_ready.

module serial_receive(clk, RxD, midstate, data2, rx_done);
   input      clk;
   input      RxD;
   
   wire       RxD_data_ready;
   wire [7:0] RxD_data;

   parameter comm_clk_frequency = 109_000_000;

   `ifdef CONFIG_SERIAL_TIMEOUT
	parameter SERIAL_TIMEOUT = `CONFIG_SERIAL_TIMEOUT;
   `else
        // Timeout after 8 million clock at 100Mhz is 80ms, which should be
        // OK for all sensible clock speeds eg 20MHz is 400ms, 200MHz is 40ms
	parameter SERIAL_TIMEOUT = 24'h800000;
   `endif
   
   uart_receiver #(.comm_clk_frequency(comm_clk_frequency)) urx (.clk(clk), .uart_rx(RxD), .tx_new_byte(RxD_data_ready), .tx_byte(RxD_data));
   
   output [255:0] midstate;
   output [255:0] data2;

   output 	  rx_done;
   
   // 256 bits midstate + 256 bits data at the same time = 64 bytes
   
   reg [511:0] input_buffer;
   reg [511:0] input_copy;
   reg [6:0]   demux_state = 7'b0000000;
   reg [23:0]  timer = 0;

   assign midstate = input_copy[511:256];
   assign data2 = input_copy[255:0];

   // Needed to reset the nonce for new work. There is a 1-cycle delay
   // in the main miner loop, because reset only zeroes nonce_next, so
   // we can do this already at this stage.
   assign rx_done = demux_state[6];
   
   always @(posedge clk)
     case (demux_state)
       7'b1000000:
	 begin
	    input_copy <= input_buffer;
	    demux_state <= 0;
	 end
       
       default:
	 if(RxD_data_ready)
	   begin
	      input_buffer <= input_buffer << 8;
	      input_buffer[7:0] <= RxD_data;
	      demux_state <= demux_state + 1;
	      timer <= 0;
	   end
	 else
	   begin
	      timer <= timer + 1;
	      if (timer == SERIAL_TIMEOUT)
	        demux_state <= 0;
	   end
     endcase // case (demux_state)
   
endmodule // serial_receive

module serial_transmit (clk, TxD, busy, send, word);
   parameter comm_clk_frequency = 109_000_000;
   
   // split 4-byte output into bytes

   wire TxD_start;
   wire TxD_ready;
   
   reg [7:0]  out_byte;
   reg        serial_start;
   reg [3:0]  mux_state = 4'b0000;

   assign TxD_start = serial_start;

   input      clk;
   output     TxD;
   
   input [31:0] word;
   input 	send;
   output 	busy;

   reg [31:0] 	word_copy;
   
   assign busy = (|mux_state);

   always @(posedge clk)
     begin
	// Testing for busy is problematic if we are keeping the
	// module busy all the time :-/ So we need some wait stages
	// between the bytes.

	if (!busy && send)
	  begin
	     mux_state <= 4'b1000;
	     word_copy <= word;
	  end  

	else if (mux_state[3] && ~mux_state[0] && TxD_ready)
	  begin
	     serial_start <= 1;
	     mux_state <= mux_state + 1;

	     out_byte <= word_copy[31:24];
	     word_copy <= (word_copy << 8);
	  end
	
	// wait stages
	else if (mux_state[3] && mux_state[0])
	  begin
	     serial_start <= 0;
	     if (TxD_ready) mux_state <= mux_state + 1;
	  end
     end

   uart_transmitter #(.comm_clk_frequency(comm_clk_frequency)) utx (.clk(clk), .uart_tx(TxD), .rx_new_byte(TxD_start), .rx_byte(out_byte), .tx_ready(TxD_ready));

endmodule // serial_send
