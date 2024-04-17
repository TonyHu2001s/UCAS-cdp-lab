`include "mycpu.h"

module mem_stage(
    input                          clk           ,
    input                          reset         ,
    //allowin
    input                          ws_allowin    ,
    output                         ms_allowin    ,
    //from es
    input                          es_to_ms_valid,
    input  [`ES_TO_MS_BUS_WD -1:0] es_to_ms_bus  ,
    //to ws
    output                         ms_to_ws_valid,
    output [`MS_TO_WS_BUS_WD -1:0] ms_to_ws_bus  ,
    //from data-sram
    input  [31                 :0] data_sram_rdata,
	//to ds
	output [`MS_TO_DS_BUS_WD -1:0] ms_to_ds_bus
);

reg         ms_valid;
wire        ms_ready_go;

reg [`ES_TO_MS_BUS_WD -1:0] es_to_ms_bus_r;
wire        ms_res_from_mem;
wire [ 3:0] ms_gr_we;
wire [ 4:0] ms_dest;
wire [31:0] ms_alu_result;
wire [31:0] ms_pc;
wire		ms_load_byte;
wire		ms_load_half;
wire		ms_load_unsigned;
wire [ 1:0] ms_mem_sel;

assign {ms_mem_sel     ,  //78:77
		ms_load_unsigned, //76:76
		ms_load_half   ,  //75:75
		ms_load_byte   ,  //74:74
		ms_res_from_mem,  //73:73
        ms_gr_we       ,  //72:69
        ms_dest        ,  //68:64
        ms_alu_result  ,  //63:32
        ms_pc             //31:0
       } = es_to_ms_bus_r;

wire [31:0] mem_result;
wire [31:0] ms_final_result;
wire [ 7:0] ms_byte_result;
wire [15:0] ms_half_result;

assign ms_to_ws_bus = {ms_gr_we       ,  //72:69
                       ms_dest        ,  //68:64
                       ms_final_result,  //63:32
                       ms_pc             //31:0
                      };

assign ms_ready_go    = 1'b1;
assign ms_allowin     = !ms_valid || ms_ready_go && ws_allowin;
assign ms_to_ws_valid = ms_valid && ms_ready_go;
always @(posedge clk) begin
    if (reset) begin
        ms_valid <= 1'b0;
    end
    else if (ms_allowin) begin
        ms_valid <= es_to_ms_valid;
    end

    if (es_to_ms_valid && ms_allowin) begin
        es_to_ms_bus_r  = es_to_ms_bus;
    end
end

assign ms_byte_result = {8{ms_mem_sel==2'b00}} & data_sram_rdata[ 7: 0] |
						{8{ms_mem_sel==2'b01}} & data_sram_rdata[15: 8] |
						{8{ms_mem_sel==2'b10}} & data_sram_rdata[23:16] |
						{8{ms_mem_sel==2'b11}} & data_sram_rdata[31:24] ;
assign ms_half_result = {16{ms_mem_sel == 2'b00}} & data_sram_rdata[15: 0] |
						{16{ms_mem_sel == 2'b10}} & data_sram_rdata[31:16] ;
assign mem_result = ms_load_byte ? {{24{ms_byte_result[7]&(~ms_load_unsigned)}},ms_byte_result} : 
					ms_load_half ? {{16{ms_half_result[15]&(~ms_load_unsigned)}},ms_half_result} : 
					data_sram_rdata;

assign ms_final_result = ms_res_from_mem ? mem_result
                                         : ms_alu_result;
										 
assign ms_to_ds_bus = {{4{ms_valid}}&ms_gr_we,  //40:37
					   ms_dest				  ,  //36:32
					   ms_final_result 			 //31: 0
					  };

endmodule
