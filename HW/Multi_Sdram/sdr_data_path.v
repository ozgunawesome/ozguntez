module sdr_data_path(
        CLK,
        RESET_N,
        DATAIN,
        DM,
        DQOUT,
        DQM
        );

`include        "Sdram_Params.h"

input                           CLK;                    // System Clock
input                           RESET_N;                // System Reset
input   [`DSIZE-1:0]            DATAIN;                 // Data input from the host
input   [`DSIZE/8-1:0]          DM;                     // byte data masks
output  [`DSIZE-1:0]            DQOUT;
output  [`DSIZE/8-1:0]          DQM;                    // SDRAM data mask ouputs
reg     [`DSIZE/8-1:0]          DQM;
            
// internal 
reg     [`DSIZE-1:0]            DIN1;
reg     [`DSIZE-1:0]            DIN2;

reg     [`DSIZE/8-1:0]          DM1;



// Allign the input and output data to the SDRAM control path
always @(posedge CLK or negedge RESET_N)
begin
        if (RESET_N == 0) 
        begin
                DIN1    <= 0;
                DIN2    <= 0;
                DM1     <= 0;
        end        
        else
        begin
                DIN1	<=	DATAIN;
                DIN2	<=	DIN1;
 				DQM		<=	DM;                 
        end
end

assign DQOUT = DIN2;

endmodule

