//
// PCILeech FPGA.
//
// PCIe configuration module - CFG handling for Artix-7.
//
// (c) Ulf Frisk, 2018-2024
// Author: Ulf Frisk, pcileech@frizk.net
//

`timescale 1ns / 1ps
`include "pcileech_header.svh"

module pcileech_pcie_cfg_a7(
    input                   rst,
    input                   clk_sys,
    input                   clk_pcie,
    IfPCIeFifoCfg.mp_pcie   dfifo,
    IfPCIeSignals.mpm       ctx,
    IfAXIS128.source        tlps_static,
    output [15:0]           pcie_id
    );

    // ----------------------------------------------------
    // TickCount64
    // ----------------------------------------------------
    
    time tickcount64 = 0;
    always @ ( posedge clk_pcie )
        tickcount64 <= tickcount64 + 1;
    
    // ----------------------------------------------------------------------------
    // Convert received CFG data from FT601 to PCIe clock domain
    // ----------------------------------------------------------------------------
    reg             in_rden;
    wire [63:0]     in_dout;
    wire            in_empty;
    wire            in_valid;
    
    reg [63:0]      in_data64;
    wire [31:0]     in_data32   = in_data64[63:32];
    wire [15:0]     in_data16   = in_data64[31:16];
    wire [3:0]      in_type     = in_data64[15:12];
	
    fifo_64_64 i_fifo_pcie_cfg_tx(
        .rst            ( rst                    ),
        .wr_clk         ( clk_sys                ),
        .rd_clk         ( clk_pcie               ),
        .din            ( dfifo.tx_data          ),
        .wr_en          ( dfifo.tx_valid         ),
        .rd_en          ( in_rden                ),
        .dout           ( in_dout                ),
        .full           (                        ),
        .empty          ( in_empty               ),
        .valid          ( in_valid               )
    );
    
    // ------------------------------------------------------------------------
    // Convert received CFG from PCIe core and transmit onwards to FT601
    // ------------------------------------------------------------------------
    reg             out_wren;
    reg [31:0]      out_data;
    wire            pcie_cfg_rx_almost_full;
    
    fifo_32_32_clk2 i_fifo_pcie_cfg_rx(
        .rst            ( rst                    ),
        .wr_clk         ( clk_pcie               ),
        .rd_clk         ( clk_sys                ),
        .din            ( out_data               ),
        .wr_en          ( out_wren               ),
        .rd_en          ( dfifo.rx_rd_en         ),
        .dout           ( dfifo.rx_data          ),
        .full           (                        ),
        .almost_full    ( pcie_cfg_rx_almost_full ),
        .empty          (                        ),
        .valid          ( dfifo.rx_valid         )
    );
    
    // ------------------------------------------------------------------------
    // REGISTER FILE: COMMON
    // ------------------------------------------------------------------------
    
    wire    [383:0]     ro;
    reg     [703:0]     rw;
    
    reg                 rwi_cfg_mgmt_rd_en;
    reg                 rwi_cfg_mgmt_wr_en;
    reg                 rwi_cfgrd_valid;
    reg     [9:0]       rwi_cfgrd_addr;
    reg     [3:0]       rwi_cfgrd_byte_en;
    reg     [31:0]      rwi_cfgrd_data;
    reg                 rwi_tlp_static_valid;
    reg                 rwi_tlp_static_2nd;
    reg                 rwi_tlp_static_has_data;
    reg     [31:0]      rwi_count_cfgspace_status_cl;
   
    // MAGIC
    assign ro[15:0]     = 16'h2301;                     // +000: MAGIC
    // SPECIAL
    assign ro[16]       = ctx.cfg_mgmt_rd_en;           // +002: SPECIAL
    assign ro[17]       = ctx.cfg_mgmt_wr_en;           //
    assign ro[31:18]    = 0;                            //
    // SIZEOF / BYTECOUNT
    assign ro[63:32]    = $bits(ro) >> 3;               // +004: BYTECOUNT
    // PCIe CFG STATUS
    assign ro[71:64]    = ctx.cfg_bus_number;           // +008:
    assign ro[76:72]    = ctx.cfg_device_number;        //
    assign ro[79:77]    = ctx.cfg_function_number;      //
    // PCIe PL PHY
    assign ro[85:80]    = ctx.pl_ltssm_state;           // +00A
    assign ro[87:86]    = ctx.pl_rx_pm_state;           //
    assign ro[90:88]    = ctx.pl_tx_pm_state;           // +00B
    assign ro[93:91]    = ctx.pl_initial_link_width;    //
    assign ro[95:94]    = ctx.pl_lane_reversal_mode;    //
    assign ro[97:96]    = ctx.pl_sel_lnk_width;         // +00C
    assign ro[98]       = ctx.pl_phy_lnk_up;            //
    assign ro[99]       = ctx.pl_link_gen2_cap;         //
    assign ro[100]      = ctx.pl_link_partner_gen2_supported; //
    assign ro[101]      = ctx.pl_link_upcfg_cap;        //
    assign ro[102]      = ctx.pl_sel_lnk_rate;          //
    assign ro[103]      = ctx.pl_directed_change_done;  // +00D:
    assign ro[104]      = ctx.pl_received_hot_rst;      //
    assign ro[126:105]  = 0;                            //
    assign ro[127]      = ctx.cfg_mgmt_rd_wr_done;      //
    // PCIe CFG MGMT
    assign ro[159:128]  = ctx.cfg_mgmt_do;              // +010:
    // PCIe CFG STATUS
    assign ro[175:160]  = ctx.cfg_command;              // +014:
    assign ro[186:176]  = 0;
    assign ro[187]      = ctx.cfg_pmcsr_pme_en;         //
    assign ro[189:188]  = ctx.cfg_pmcsr_powerstate;      //
    assign ro[190]      = ctx.cfg_pmcsr_pme_status;     //
    assign ro[191]      = 0;                            //
    assign ro[207:192]  = ctx.cfg_dcommand;             // +018:
    assign ro[223:208]  = ctx.cfg_dcommand2;            // +01A:
    assign ro[239:224]  = ctx.cfg_dstatus;              // +01C:
    assign ro[255:240]  = ctx.cfg_lcommand;             // +01E:
    assign ro[271:256]  = ctx.cfg_lstatus;              // +020:
    assign ro[287:272]  = ctx.cfg_status;               // +022:
    assign ro[293:288]  = ctx.tx_buf_av;                // +024:
    assign ro[294]      = ctx.tx_cfg_req;                //
    assign ro[295]      = ctx.tx_err_drop;              //
    assign ro[302:296]  = ctx.cfg_vc_tcvc_map;          // +025:
    assign ro[335:303]  = 0;
    // CFG SPACE READ RESULT
    assign ro[345:336]  = rwi_cfgrd_addr;               // +02A:
    assign ro[347:346]  = {rwi_cfgrd_valid, 1'b0};
    assign ro[351:348]  = rwi_cfgrd_byte_en;            //
    assign ro[383:352]  = rwi_cfgrd_data;               // +02C:
	
    // ------------------------------------------------------------------------
    // READ-WRITE INITIALIZATION (CUSTOM IDs)
    // ------------------------------------------------------------------------
	
    localparam integer  RWPOS_CFG_RD_EN                 = 16;
    localparam integer  RWPOS_CFG_WR_EN                 = 17;
    localparam integer  RWPOS_CFG_WAIT_COMPLETE         = 18;
    localparam integer  RWPOS_CFG_STATIC_TLP_TX_EN      = 19;
    localparam integer  RWPOS_CFG_CFGSPACE_STATUS_CL_EN = 20;
    localparam integer  RWPOS_CFG_CFGSPACE_COMMAND_EN   = 21;
    
    task pcileech_pcie_cfg_a7_initialvalues;
        begin
            out_wren <= 1'b0;
            rwi_cfg_mgmt_rd_en <= 1'b0;
            rwi_cfg_mgmt_wr_en <= 1'b0;
    
            // --- CUSTOM DEVICE ID (Qualcomm) ---
            rw[15:0]    <= 16'h1107;                // Device ID: 1107
            rw[16]      <= 0;                       // CFG RD EN
            rw[17]      <= 0;                       // CFG WR EN
            rw[18]      <= 0;                       // WAIT
            rw[19]      <= 0;                       // TLP STATIC
            rw[20]      <= 0;                       // STATUS CL
            rw[21]      <= 0;                       // COMMAND EN
            rw[31:22]   <= 0;
            rw[63:32]   <= $bits(rw) >> 3;          // bytecount
            // --- CUSTOM SERIAL NUMBER ---
            rw[127:64]  <= 64'h000123456789BCBC;    // DSN
            rw[159:128] <= 0;
            rw[169:160] <= 0;
            rw[170]     <= 0;
            rw[171]     <= 0;
            rw[175:172] <= 4'hf;
            rw[176]     <= 0;
            rw[178:177] <= 0;
            rw[179]     <= 1;
            rw[181:180] <= 0;
            rw[182]     <= 1;
            rw[183]     <= 0;
            rw[184]     <= 0;
            rw[223:185] <= 0;
            rw[703:224] <= 0;
        end
    endtask
    
    assign ctx.cfg_mgmt_rd_en               = rwi_cfg_mgmt_rd_en & ~ctx.cfg_mgmt_rd_wr_done;
    assign ctx.cfg_mgmt_wr_en               = rwi_cfg_mgmt_wr_en & ~ctx.cfg_mgmt_rd_wr_done;
    assign ctx.cfg_dsn                      = rw[127:64];
    assign ctx.cfg_mgmt_di                  = rw[159:128];
    assign ctx.cfg_mgmt_dwaddr              = rw[169:160];
    assign ctx.cfg_mgmt_wr_readonly         = rw[170];
    assign ctx.cfg_mgmt_wr_rw1c_as_rw       = rw[171];
    assign ctx.cfg_mgmt_byte_en             = rw[175:172];
    
    assign ctx.pl_directed_link_auton       = rw[176];
    assign ctx.pl_directed_link_change      = rw[178:177];
    assign ctx.pl_directed_link_speed       = rw[179];
    assign ctx.pl_directed_link_width       = rw[181:180];
    assign ctx.pl_upstream_prefer_deemph    = rw[182];
    assign ctx.pl_transmit_hot_rst          = rw[183];
    assign ctx.pl_downstream_deemph_source  = rw[184];
    
    assign ctx.cfg_interrupt_di             = 0;
    assign ctx.cfg_pciecap_interrupt_msgnum = 0;
    assign ctx.cfg_interrupt_assert         = 0;
    assign ctx.cfg_interrupt                = 0;
    assign ctx.cfg_interrupt_stat           = 0;

    assign ctx.cfg_pm_force_state           = 0;
    assign ctx.cfg_pm_force_state_en        = 0;
    assign ctx.cfg_pm_halt_aspm_l0s         = 0;
    assign ctx.cfg_pm_halt_aspm_l1          = 0;
    assign ctx.cfg_pm_send_pme_to           = 0;
    assign ctx.cfg_pm_wake                  = 0;
    assign ctx.cfg_trn_pending              = 0;
    assign ctx.cfg_turnoff_ok               = 0;
    assign ctx.rx_np_ok                    = 1;
    assign ctx.rx_np_req                   = 1;
    assign ctx.tx_cfg_gnt                  = 1;
    
    assign tlps_static.tdata                = 0;
    assign tlps_static.tkeepdw              = 0;
    assign tlps_static.tlast                = 0;
    assign tlps_static.tuser[0]             = 0;
    assign tlps_static.tvalid               = 0;
    assign tlps_static.has_data             = 0;
    assign pcie_id                          = ro[79:64];
    
    integer i_write;
    wire [15:0] in_cmd_address_byte = in_dout[31:16];
    wire [17:0] in_cmd_address_bit  = {in_cmd_address_byte[14:0], 3'b000};
    wire [15:0] in_cmd_value        = {in_dout[48+:8], in_dout[56+:8]};
    wire [15:0] in_cmd_mask         = {in_dout[32+:8], in_dout[40+:8]};
    wire        f_rw                = in_cmd_address_byte[15]; 
    wire [15:0] in_cmd_data_in      = (in_cmd_address_bit < (f_rw ? $bits(rw) : $bits(ro))) ? (f_rw ? rw[in_cmd_address_bit+:16] : ro[in_cmd_address_bit+:16]) : 16'h0000;
    wire        in_cmd_read         = in_dout[12] & in_valid;
    wire        in_cmd_write        = in_dout[13] & in_cmd_address_byte[15] & in_valid;
    wire        pcie_cfg_rw_en      = rwi_cfg_mgmt_rd_en | rwi_cfg_mgmt_wr_en | rw[RWPOS_CFG_RD_EN] | rw[RWPOS_CFG_WR_EN];
    assign in_rden = tickcount64[1] & ~pcie_cfg_rx_almost_full & ( ~rw[RWPOS_CFG_WAIT_COMPLETE] | ~pcie_cfg_rw_en);
    
    initial pcileech_pcie_cfg_a7_initialvalues();
    
    always @ ( posedge clk_pcie )
        if ( rst )
            pcileech_pcie_cfg_a7_initialvalues();
        else
            begin
                out_wren <= in_cmd_read;
                if ( in_cmd_read )
                    begin
                        out_data[31:16] <= in_cmd_address_byte;
                        out_data[15:0]  <= {in_cmd_data_in[7:0], in_cmd_data_in[15:8]};
                    end

                if ( in_cmd_write )
                    for ( i_write = 0; i_write < 16; i_write = i_write + 1 )
                        begin
                            if ( in_cmd_mask[i_write] )
                                rw[in_cmd_address_bit+i_write] <= in_cmd_value[i_write];
                        end

                if ( ctx.cfg_mgmt_rd_wr_done )
                    begin
                        rwi_cfg_mgmt_rd_en  <= 1'b0;
                        rwi_cfg_mgmt_wr_en  <= 1'b0;
                        rwi_cfgrd_valid     <= 1'b1;
                        rwi_cfgrd_addr      <= ctx.cfg_mgmt_dwaddr;
                        rwi_cfgrd_data      <= ctx.cfg_mgmt_do;
                        rwi_cfgrd_byte_en   <= ctx.cfg_mgmt_byte_en;
                    end
                else if ( rw[RWPOS_CFG_RD_EN] )
                    begin
                        rw[RWPOS_CFG_RD_EN] <= 1'b0;
                        rwi_cfg_mgmt_rd_en  <= 1'b1;
                        rwi_cfgrd_valid     <= 1'b0;
                    end
                else if ( rw[RWPOS_CFG_WR_EN] )
                    begin
                        rw[RWPOS_CFG_WR_EN] <= 1'b0;
                        rwi_cfg_mgmt_wr_en  <= 1'b1;
                        rwi_cfgrd_valid     <= 1'b0;
                    end
            end
endmodule
