set_clock_groups -asynchronous -quiet \
    -group [get_clocks -quiet sys_clk_pin] \
    -group [get_clocks -quiet phy_byte_clk] \
    -group [get_clocks -quiet clk_fpga_0] \
    -group [get_clocks -quiet pix_clk_unbuf] \
    -group [get_clocks -quiet tmds_clk_unbuf]
