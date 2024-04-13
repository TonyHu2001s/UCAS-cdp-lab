`include "mycpu.h"

module exe_stage(
    input                          clk           ,
    input                          reset         ,
    //allowin
    input                          ms_allowin    ,
    output                         es_allowin    ,
    //from ds
    input                          ds_to_es_valid,
    input  [`DS_TO_ES_BUS_WD -1:0] ds_to_es_bus  ,
    //to ms
    output                         es_to_ms_valid,
    output [`ES_TO_MS_BUS_WD -1:0] es_to_ms_bus  ,
    // data sram interface
    output        data_sram_en   ,
    output [ 3:0] data_sram_wen  ,
    output [31:0] data_sram_addr ,
    output [31:0] data_sram_wdata,
	//to ds
	output [`ES_TO_DS_BUS_WD -1:0] es_to_ds_bus
);

reg         es_valid      ;
wire        es_ready_go   ;

reg  [`DS_TO_ES_BUS_WD -1:0] ds_to_es_bus_r;
wire [19:0] es_alu_op     ;
wire        es_load_op    ;
wire        es_src1_is_sa ;
wire        es_src1_is_pc ;
wire        es_src2_is_imm;
wire		es_src2_is_immu;
wire        es_src2_is_8  ;
wire        es_gr_we      ;
wire        es_mem_we     ;
wire [ 4:0] es_dest       ;
wire [15:0] es_imm        ;
wire [31:0] es_rs_value   ;
wire [31:0] es_rt_value   ;
wire [31:0] es_pc         ;
wire		es_mult;
wire		es_multu;
wire [63:0] es_mult_result;
wire [63:0] es_multu_result;
wire		es_div;
wire		es_s_axis_divisor_tvalid;
wire		es_s_axis_divisor_tready;
wire [31:0] es_s_axis_divisor_tdata;
wire		es_s_axis_dividend_tvalid;
wire		es_s_axis_dividend_tready;
wire [31:0] es_s_axis_dividend_tdata;
wire		es_m_axis_dout_tvalid;
wire [63:0] es_m_axis_dout_tdata;
wire		es_divu;
wire		es_s_axis_udivisor_tvalid;
wire		es_s_axis_udivisor_tready;
wire [31:0] es_s_axis_udivisor_tdata;
wire		es_s_axis_udividend_tvalid;
wire		es_s_axis_udividend_tready;
wire [31:0] es_s_axis_udividend_tdata;
wire		es_m_axis_udout_tvalid;
wire [63:0] es_m_axis_udout_tdata;
wire [31:0] es_hi;
reg  [31:0] es_hi_reg;
wire [31:0] es_lo;
reg  [31:0] es_lo_reg;
wire		es_mtlo;
wire		es_mthi;
wire		es_mflo;
wire		es_mfhi;
wire		es_lo_valid;
wire		es_hi_valid;
wire		es_multicycle;
wire		es_multicycle_valid;

assign {es_alu_op      ,  //140:125
        es_load_op     ,  //124:124
        es_src1_is_sa  ,  //123:123
        es_src1_is_pc  ,  //122:122
        es_src2_is_imm ,  //121:121
		es_src2_is_immu,  //120:120
        es_src2_is_8   ,  //119:119
        es_gr_we       ,  //118:118
        es_mem_we      ,  //117:117
        es_dest        ,  //116:112
        es_imm         ,  //111:96
        es_rs_value    ,  //95 :64
        es_rt_value    ,  //63 :32
        es_pc             //31 :0
       } = ds_to_es_bus_r;

wire [31:0] es_alu_src1   ;
wire [31:0] es_alu_src2   ;
wire [31:0] es_alu_result ;
wire [31:0] es_result     ;

wire        es_res_from_mem;

assign es_res_from_mem = es_load_op;
assign es_to_ms_bus = {es_res_from_mem,  //70:70
                       es_gr_we       ,  //69:69
                       es_dest        ,  //68:64
                       es_result  ,  //63:32
                       es_pc             //31:0
                      };

assign es_ready_go    = 1'b1;
assign es_allowin     = !es_valid || es_ready_go && ms_allowin;
assign es_to_ms_valid =  es_valid && es_ready_go;
assign es_mult = es_alu_op[12];
assign es_multu= es_alu_op[13];
assign es_div  = es_alu_op[14];
assign es_s_axis_divisor_tvalid = es_div && ds_to_es_valid;
assign es_s_axis_divisor_tdata  = es_rt_value;
assign es_s_axis_dividend_tvalid= es_div && ds_to_es_valid;
assign es_s_axis_dividend_tdata = es_rs_value;
assign es_divu = es_alu_op[15];
assign es_s_axis_udivisor_tvalid = es_divu && ds_to_es_valid;
assign es_s_axis_udivisor_tdata  = es_rt_value;
assign es_s_axis_udividend_tvalid= es_divu && ds_to_es_valid;
assign es_s_axis_udividend_tdata = es_rs_value;
assign es_mtlo = es_alu_op[16];
assign es_mthi = es_alu_op[17];
assign es_mflo = es_alu_op[18];
assign es_mfhi = es_alu_op[19];
assign es_multicycle = es_div | es_divu;
assign es_multicycle_valid = es_m_axis_dout_tvalid | es_m_axis_udout_tvalid;

always @(posedge clk) begin
    if (reset) begin
        es_valid <= 1'b0;
    end
    else if (es_allowin) begin
        es_valid <= ds_to_es_valid;
    end

    if (ds_to_es_valid && es_allowin) begin
        ds_to_es_bus_r <= ds_to_es_bus;
    end
end

assign es_alu_src1 = es_src1_is_sa  ? {27'b0, es_imm[10:6]} : 
                     es_src1_is_pc  ? es_pc[31:0] :
                                      es_rs_value;
assign es_alu_src2 = es_src2_is_imm ? {{16{es_imm[15]}}, es_imm[15:0]} : 
					 es_src2_is_immu ? {16'b0,es_imm[15:0]} :
                     es_src2_is_8   ? 32'd8 :
                                      es_rt_value;

alu u_alu(
	.clk		(clk		    ),
    .alu_op     (es_alu_op[11:0]),
    .alu_src1   (es_alu_src1    ),
    .alu_src2   (es_alu_src2    ),
    .alu_result (es_alu_result  )
    );
	
assign es_result = es_mflo ? es_lo :
					es_mfhi ? es_hi : es_alu_result;

// MULT result
assign es_mult_result  = $signed(es_alu_src1) * $signed(es_alu_src2);

//MULTU result
assign es_multu_result = es_alu_src1 * es_alu_src2;

//DIV result
div_gen u_div_gen (
  .aclk(clk),                                       // input wire aclk
  .s_axis_divisor_tvalid(es_s_axis_divisor_tvalid),    // input wire s_axis_divisor_tvalid
  .s_axis_divisor_tready(es_s_axis_divisor_tready),    // output wire s_axis_divisor_tready
  .s_axis_divisor_tdata(es_s_axis_divisor_tdata),      // input wire [31 : 0] s_axis_divisor_tdata
  .s_axis_dividend_tvalid(es_s_axis_dividend_tvalid),  // input wire s_axis_dividend_tvalid
  .s_axis_dividend_tready(es_s_axis_dividend_tready),  // output wire s_axis_dividend_tready
  .s_axis_dividend_tdata(es_s_axis_dividend_tdata),    // input wire [31 : 0] s_axis_dividend_tdata
  .m_axis_dout_tvalid(es_m_axis_dout_tvalid),          // output wire m_axis_dout_tvalid
  .m_axis_dout_tdata(es_m_axis_dout_tdata)            // output wire [63 : 0] m_axis_dout_tdata
);

//DIVU result
divu_gen u_divu_gen (
  .aclk(clk),                                      // input wire aclk
  .s_axis_divisor_tvalid(es_s_axis_udivisor_tvalid),    // input wire s_axis_divisor_tvalid
  .s_axis_divisor_tready(es_s_axis_udivisor_tready),    // output wire s_axis_divisor_tready
  .s_axis_divisor_tdata(es_s_axis_udivisor_tdata),      // input wire [31 : 0] s_axis_divisor_tdata
  .s_axis_dividend_tvalid(es_s_axis_udividend_tvalid),  // input wire s_axis_dividend_tvalid
  .s_axis_dividend_tready(es_s_axis_udividend_tready),  // output wire s_axis_dividend_tready
  .s_axis_dividend_tdata(es_s_axis_udividend_tdata),    // input wire [31 : 0] s_axis_dividend_tdata
  .m_axis_dout_tvalid(es_m_axis_udout_tvalid),          // output wire m_axis_dout_tvalid
  .m_axis_dout_tdata(es_m_axis_udout_tdata)            // output wire [63 : 0] m_axis_dout_tdata
);

assign es_lo = {32{es_mult}} && es_mult_result[31:0] |
				{32{es_multu}} && es_multu_result[31:0] |
				{32{es_m_axis_dout_tvalid}} && es_m_axis_dout_tdata[63:32] |
				{32{es_m_axis_udout_tvalid}} && es_m_axis_udout_tdata[63:32] |
				{32{es_mtlo}} && es_rs_value;
assign es_hi = {32{es_mult}} && es_mult_result[63:32] |
				{32{es_multu}} && es_multu_result[63:32] |
				{32{es_m_axis_dout_tvalid}} && es_m_axis_dout_tdata[31:0] |
				{32{es_m_axis_udout_tvalid}} && es_m_axis_udout_tdata[31:0] |
				{32{es_mthi}} && es_rs_value;
assign es_lo_valid = es_mult | es_multu | es_m_axis_dout_tvalid | es_m_axis_udout_tvalid | es_mtlo;
assign es_hi_valid = es_mult | es_multu | es_m_axis_dout_tvalid | es_m_axis_udout_tvalid | es_mthi;
				
always @(posedge clk) begin
    if (reset) begin
        es_lo_reg <= 32'b0;
		es_hi_reg <= 32'b0;
    end
    else begin
		if(es_lo_valid)begin
			es_lo_reg <= es_lo;
		end
		if(es_hi_valid)begin
			es_hi_reg <= es_hi;
		end
    end
end

assign data_sram_en    = 1'b1;
assign data_sram_wen   = es_mem_we&&es_valid ? 4'hf : 4'h0;
assign data_sram_addr  = es_alu_result;
assign data_sram_wdata = es_rt_value;

assign es_to_ds_bus = {`ES_TO_DS_BUS_WD{es_valid}} &
					  {es_res_from_mem,  //37:37
					   es_dest		,  //36:32
					   es_result   //31:0
					  };

endmodule
