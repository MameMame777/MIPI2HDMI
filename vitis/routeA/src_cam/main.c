/* Route A M2-M4 — bare-metal OV5640 -> FPGA -> live HDMI, no PYNQ/Linux.
 * Ports scripts/v65_capture.py + bitslip_lock.py + camera_hdmi_demo.py control flow to C.
 *   M2: SCCB engine + chip init (227-step OV5640 replay) + chip-ID
 *   M3: software 8x8 bitslip D-PHY lock (HW-lock FSM bogus-locks at 96MHz byte_clk) + settle_blank K=14
 *   M4: VDMA S2MM(camera->DDR) + MM2S(DDR->HDMI) -> live image on HDMI
 * All status is printed on PS7 UART1 (COM4, 115200 8N1) for headless debug. */
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "xil_printf.h"
#include "sleep.h"
#include "ov5640_init.h"

typedef uint32_t u32; typedef uint16_t u16; typedef uint8_t u8;

/* ---- IP base addresses (confirmed from the BD assign_bd_address) ---- */
#define DBG_BASE     0x41200000u
#define SCCB_BASE    0x41210000u
#define IDELAY_BASE  0x41220000u
#define BITSLIP_BASE 0x41230000u
#define FL_BASE      0x41240000u
#define VDMA_BASE    0x43000000u

/* AXI GPIO: ch1 DATA +0x00 / TRI +0x04 ; ch2 DATA +0x08 / TRI +0x0C */
#define G_DATA 0x00u
#define G_TRI  0x04u
#define G_DATA2 0x08u
#define G_TRI2  0x0Cu

static inline void   wr(u32 base, u32 off, u32 v){ *(volatile u32*)(base+off) = v; }
static inline u32    rd(u32 base, u32 off){ return *(volatile u32*)(base+off); }

/* ============================ SCCB engine ============================ */
#define WRITE_APPLY (1u<<24)
#define READ_APPLY  (1u<<26)
#define SCCB_STAT   G_DATA2          /* status read at +0x08 (ch2) */

static int sccb_wait_idle(void){
    for(int i=0;i<10000;i++){
        u32 st = rd(SCCB_BASE, SCCB_STAT);
        if((st & 0x02) && ((st & 0x11)==0)) return 1;   /* ready & !pending & !busy */
        usleep(500);
    }
    return 0;
}
/* returns 1=ACK ok, 0=NACK/timeout */
static int sccb_write(u16 addr, u8 val){
    sccb_wait_idle();
    wr(SCCB_BASE, G_TRI, 0);
    u32 base = ((u32)val<<16) | (addr & 0xFFFF);
    wr(SCCB_BASE, G_DATA, base);
    wr(SCCB_BASE, G_DATA, base | WRITE_APPLY);
    wr(SCCB_BASE, G_DATA, base);
    for(int i=0;i<3000;i++){
        usleep(1000);
        u32 st = rd(SCCB_BASE, SCCB_STAT);
        if(((st>>24)&0xFF) == (addr & 0xFF) && (st & ((1u<<2)|(1u<<3))))
            return (st & (1u<<2)) && !(st & (1u<<3));
    }
    return 0;
}
/* returns byte 0..255, or -1 on error/timeout */
static int sccb_read(u16 addr){
    sccb_wait_idle();
    u32 a = addr & 0xFFFF;
    wr(SCCB_BASE, G_DATA, a);
    wr(SCCB_BASE, G_DATA, a | READ_APPLY);
    wr(SCCB_BASE, G_DATA, a);
    for(int i=0;i<1000;i++){
        u32 st = rd(SCCB_BASE, SCCB_STAT);
        if(st & (1u<<5)) return (st & (1u<<6)) ? -1 : (int)((st>>8)&0xFF);
        usleep(1000);
    }
    return -1;
}

/* ============================ debug pages ============================ */
static u32 read_dbg(u8 page){
    wr(DBG_BASE, G_DATA2, page);     /* ch2 = page select; pages>=0x20 use 0x80|(p&0x1F) */
    usleep(150);
    return rd(DBG_BASE, G_DATA);     /* ch1 = page value */
}

/* ============================ IDELAY ============================ */
static u8 id_t0=8, id_t1=8, id_tclk=0, id_proc=0, id_blank=0;
static u32 idelay_word(void){
    return ((u32)(id_blank&0xF)<<27) | ((u32)(id_proc&0xF)<<21)
         | ((u32)(id_tclk&0x1F)<<16) | ((u32)(id_t1&0x1F)<<8) | (id_t0&0x1F);
}
static void idelay_set(u8 t0, u8 t1){           /* pulses APPLY (bit24) to latch taps */
    id_t0=t0&0x1F; id_t1=t1&0x1F;
    u32 w = idelay_word();
    wr(IDELAY_BASE, G_TRI, 0);
    wr(IDELAY_BASE, G_DATA, w);
    wr(IDELAY_BASE, G_DATA, w | (1u<<24));
    wr(IDELAY_BASE, G_DATA, w);
    usleep(30000);
}
static void set_settle_blank(u8 k){             /* level-read [30:27], NO apply edge */
    id_blank=k&0xF;
    wr(IDELAY_BASE, G_TRI, 0);
    wr(IDELAY_BASE, G_DATA, idelay_word());
    usleep(1000);
}
static void set_proc_op(u8 op){                 /* MID proc select [24:21], level-read, NO apply */
    id_proc = op & 0xF;
    wr(IDELAY_BASE, G_TRI, 0);
    wr(IDELAY_BASE, G_DATA, idelay_word());
    usleep(1000);
}

/* ============================ BITSLIP ============================ */
static u8 bs_p0=0, bs_p1=6, bs_hwlock=0, bs_inhibit=0, bs_settle=0;
static u32 bitslip_word(int apply){
    return (apply?(1u<<24):0) | (bs_hwlock?(1u<<25):0) | (bs_inhibit?(1u<<26):0)
         | ((u32)(bs_settle&0x7F)<<17) | ((u32)(bs_p1&0x7)<<8) | (bs_p0&0x7);
}
static void bitslip_set(u8 p0, u8 p1){
    bs_p0=p0&7; bs_p1=p1&7;
    wr(BITSLIP_BASE, G_DATA, bitslip_word(1)); usleep(20000);
    wr(BITSLIP_BASE, G_DATA, bitslip_word(0)); usleep(30000);
}
static void set_hw_lock(int en){                /* en=0 -> inhibit FSM, software lock path */
    bs_hwlock = en?1:0; bs_inhibit = en?0:1;
    wr(BITSLIP_BASE, G_DATA, bitslip_word(0)); usleep(10000);
}

/* ============================ frame_lines ============================ */
#define CAM_GPIO (1u<<25)
static u32 fl_base = CAM_GPIO;
static void fl_write_raw(u32 word){
    wr(FL_BASE, G_DATA, word);              usleep(5000);
    wr(FL_BASE, G_DATA, word | (1u<<24));   usleep(5000);
    wr(FL_BASE, G_DATA, word);              usleep(5000);
}
static void frame_lines_set(u16 value, int use_lsle, u8 expected_dt,
                            int sof_synth, int force_expected){
    u32 base = CAM_GPIO | (value & 0xFFFF)
             | (use_lsle ? (1u<<16) : 0)
             | ((u32)(expected_dt & 0x7F) << 17)
             | (sof_synth ? (1u<<30) : 0)
             | (force_expected ? (1u<<31) : 0);
    fl_base = base;
    fl_write_raw(base);
}
static void cam_resetb_pulse(int low_ms, int post_ms){
    fl_write_raw(CAM_GPIO); usleep(5000);
    fl_write_raw(0);        usleep(low_ms*1000);
    fl_write_raw(CAM_GPIO); usleep(post_ms*1000);
}
static void bufr_clr_pulse(void){
    wr(FL_BASE, G_DATA, fl_base | (1u<<27)); usleep(2000);
    wr(FL_BASE, G_DATA, fl_base);            usleep(5000);
}

/* ============================ counters (snap) ============================ */
typedef struct { u32 crc_ok,crc_err,short_pkt,long_pkt,fs,fe,ls,le; } snap_t;
static snap_t snap(void){
    snap_t s; u32 p02=read_dbg(0x02),p03=read_dbg(0x03),p18=read_dbg(0x18),p19=read_dbg(0x19);
    s.crc_ok=(p02>>16)&0xFFFF; s.crc_err=p02&0xFFFF;
    s.short_pkt=(p03>>16)&0xFFFF; s.long_pkt=p03&0xFFFF;
    s.fs=(p18>>16)&0xFFFF; s.fe=p18&0xFFFF;
    s.ls=(p19>>16)&0xFFFF; s.le=p19&0xFFFF;
    return s;
}
static u32 d16(u32 a, u32 b){ return (a-b) & 0xFFFF; }

/* ============================ software lock ============================ */
#define LONG_LOCK 3000u
static void measure(snap_t *d, int dur_ms){
    snap_t b=snap(); usleep(dur_ms*1000); snap_t a=snap();
    d->long_pkt=d16(a.long_pkt,b.long_pkt); d->short_pkt=d16(a.short_pkt,b.short_pkt);
    d->crc_err=d16(a.crc_err,b.crc_err); d->fs=d16(a.fs,b.fs); d->fe=d16(a.fe,b.fe);
}
static int find_best_bitslip(int *bp0, int *bp1){
    int bl=-1, bsh=0, p0b=0, p1b=0;
    for(int p0=0;p0<8;p0++) for(int p1=0;p1<8;p1++){
        bitslip_set(p0,p1); usleep(50000);
        snap_t b=snap(); usleep(300000); snap_t a=snap();
        int dl=(int)d16(a.long_pkt,b.long_pkt), ds=(int)d16(a.short_pkt,b.short_pkt);
        if(dl>0 || ds>0)
            xil_printf("    p0=%d p1=%d long=%d short=%d\r\n", p0,p1,dl,ds);
        if(dl>bl || (dl==bl && ds>bsh)){ bl=dl; bsh=ds; p0b=p0; p1b=p1; }
    }
    *bp0=p0b; *bp1=p1b;
    bitslip_set(p0b,p1b); usleep(300000);
    return bl;
}
static int stable_lock(u32 *outlong, u32 *outcrc){
    snap_t d1,d2; measure(&d1,800); measure(&d2,800);
    int ok = (d1.long_pkt>LONG_LOCK && d2.long_pkt>LONG_LOCK
              && d1.crc_err==0 && d2.crc_err==0);
    *outlong = (d1.long_pkt<d2.long_pkt)?d1.long_pkt:d2.long_pkt;
    *outcrc  = d1.crc_err + d2.crc_err;
    return ok;
}
static int lock_mode(int rerolls, int settle_blank){
    for(int ph=0; ph<=rerolls; ph++){
        if(ph>0){ bufr_clr_pulse(); usleep(300000); }
        int p0,p1; find_best_bitslip(&p0,&p1);
        u32 lng,crc; int ok=stable_lock(&lng,&crc);
        xil_printf("  %s: bitslip=(%d,%d) long~%u crc=%u stable=%d\r\n",
                   ph?"re-roll":"boot", p0,p1,lng,crc,ok);
        if(ok){
            /* Keep the locked idelay (8,8); apply settle_blank (band fix). K=14 is the
             * deployed 96MHz value and user-confirmed most stable / full 480-line frame
             * (the long counter read is unreliable here, so K was tuned by the HDMI image,
             * not by the counter). settle_blank is level-read (no apply) -> taps preserved. */
            set_settle_blank((u8)settle_blank);
            xil_printf("  ==> LOCKED ph=%d bitslip=(%d,%d) idelay=(%d,%d) blank=%d\r\n",
                       ph,p0,p1,id_t0,id_t1,settle_blank);
            return 0;
        }
    }
    return 1;
}

/* ============================ chip init ============================ */
#define VAL_4800 0x14u    /* live cam path: continuous + line_sync (use_lsle) */
static int run_full_init(void){
    int nack=0;
    for(unsigned i=0;i<OV5640_INIT_N;i++){
        u16 r=OV5640_INIT[i].reg;
        u8  v=(r==0x4800)? VAL_4800 : OV5640_INIT[i].val;
        if(!sccb_write(r,v)) nack++;
    }
    sccb_write(0x3008, 0x02);   /* STREAM_ON */
    usleep(300000);
    return nack;
}
static void arm_rgb565(void){
    sccb_write(0x300E,0x40); usleep(30000);   /* stream off */
    sccb_write(0x4202,0x0F); usleep(30000);   /* pause MIPI */
    sccb_write(0x4300,0x6F);                  /* RGB565 */
    sccb_write(0x501F,0x01);                  /* ISP RGB */
    sccb_write(0x300E,0x45); usleep(30000);   /* stream on (0x40->0x45 latches format) */
    sccb_write(0x4202,0x00); usleep(300000);  /* resume */
    set_settle_blank(0);
    usleep(1000000);
}

/* ============================ VDMA ============================ */
#define MM2S_DMACR 0x00u
#define MM2S_VSIZE 0x50u
#define MM2S_HSIZE 0x54u
#define MM2S_FRMDLY_STRIDE 0x58u
#define MM2S_ADDR0 0x5Cu
#define S2MM_DMACR 0x30u
#define S2MM_VSIZE 0xA0u
#define S2MM_HSIZE 0xA4u
#define S2MM_FRMDLY_STRIDE 0xA8u
#define S2MM_ADDR0 0xACu
#define VDMA_RS (1u<<0)
#define VDMA_CIRC (1u<<1)
#define VDMA_RESET (1u<<2)

#define VWIDTH 640u
#define VHEIGHT 480u
#define VBPP 4u                         /* RGBA32 */
#define VSTRIDE (VWIDTH*VBPP)           /* 2560 */
#define VBUF_SZ (VHEIGHT*VSTRIDE)       /* 1,228,800 */
#define NFS 3u
static const u32 fbuf[NFS] = { 0x10000000u, 0x10140000u, 0x10280000u };

static void vdma_prefill(void){
    for(unsigned b=0;b<NFS;b++){
        volatile u32 *p=(volatile u32*)(uintptr_t)fbuf[b];
        for(unsigned i=0;i<VBUF_SZ/4;i++) p[i]=0xAAAAAAAAu;
    }
}
static void vdma_start(void){
    wr(VDMA_BASE,S2MM_DMACR,VDMA_RESET); while(rd(VDMA_BASE,S2MM_DMACR)&VDMA_RESET){}
    wr(VDMA_BASE,MM2S_DMACR,VDMA_RESET); while(rd(VDMA_BASE,MM2S_DMACR)&VDMA_RESET){}
    for(unsigned i=0;i<NFS;i++){
        wr(VDMA_BASE,S2MM_ADDR0+i*4,fbuf[i]);
        wr(VDMA_BASE,MM2S_ADDR0+i*4,fbuf[i]);
    }
    wr(VDMA_BASE,S2MM_HSIZE,VSTRIDE);
    wr(VDMA_BASE,S2MM_FRMDLY_STRIDE,(VSTRIDE&0xFFFF));
    wr(VDMA_BASE,MM2S_HSIZE,VSTRIDE);
    wr(VDMA_BASE,MM2S_FRMDLY_STRIDE,(1u<<24)|(VSTRIDE&0xFFFF));  /* frmdly=1 frame */
    /* S2MM first (camera->DDR); VSIZE last = arm */
    wr(VDMA_BASE,S2MM_DMACR,VDMA_RS|VDMA_CIRC);
    wr(VDMA_BASE,S2MM_VSIZE,VHEIGHT);
    usleep(100000);                                              /* prime >=1 frame */
    /* MM2S (DDR->HDMI) */
    wr(VDMA_BASE,MM2S_DMACR,VDMA_RS|VDMA_CIRC);
    wr(VDMA_BASE,MM2S_VSIZE,VHEIGHT);
    usleep(100000);
}

/* ============================ UART filter console ============================ */
/* REPL-equivalent over COM4 (PS7 UART1). Single-key commands, applied live via the
 * 0xFE-page SCCB intercept (image-proc config, not sent to the chip) + set_proc_op. */
#define UART1_BASE 0xE0001000u    /* PS7 UART1 = on-board USB-UART (COM4) */
#define UART_SR    0x2Cu          /* channel status reg; RXEMPTY = bit1 */
#define UART_FIFO  0x30u          /* RX/TX FIFO */
static int  uart_rx_ready(void){ return !(rd(UART1_BASE, UART_SR) & 0x02u); }
static char uart_getc(void){ return (char)(rd(UART1_BASE, UART_FIFO) & 0xFF); }

/* The BD PS7 defaulted the MIO to LVCMOS33 (3.3V), but the Zybo MIO bank is 1.8V.
 * TX works (drive level = bank 1.8V), but the 3.3V RX input threshold (Vih~2.0V)
 * can't read the 1.8V serial RX -> UART RX dead (RXEMPTY forever). Re-set MIO 48/49
 * to LVCMOS18 ([11:9]=001) + enable/reset UART1 RX. (Proper fix = set the PS7 MIO
 * bank voltage to 1.8V in the BD and rebuild; this is the rebuild-free equivalent.) */
static void fix_uart_rx_mio18(void){
    wr(0xF8000008u, 0, 0x0000DF0Du);    /* SLCR_UNLOCK */
    wr(0xF80007C0u, 0, 0x000012E0u);    /* MIO48 = UART1_TX, LVCMOS18 */
    wr(0xF80007C4u, 0, 0x000012E1u);    /* MIO49 = UART1_RX, LVCMOS18 */
    wr(0xF8000004u, 0, 0x0000767Bu);    /* SLCR_LOCK */
    wr(UART1_BASE, 0x00, 0x00000017u);  /* UART1 CR: RXRST|TXRST|RX_EN|TX_EN */
}

/* 0xFE-page config write: the RTL INTERCEPTS 0xFExx and latches it on the apply edge
 * (no real SCCB chip transaction), so do NOT poll the chip-write status (that would wait
 * out a timeout every call = the console lag). Just pulse apply + a 2ms settle, exactly
 * like scripts/v65_capture.py fe_write. */
static void fe(u8 lo, u8 v){
    u32 base = ((u32)v<<16) | (0xFE00u | lo);
    wr(SCCB_BASE, G_TRI, 0);
    wr(SCCB_BASE, G_DATA, base);
    wr(SCCB_BASE, G_DATA, base | WRITE_APPLY);
    wr(SCCB_BASE, G_DATA, base);
    usleep(2000);
}
static void load_kernel(const signed char k[9], u8 shift){    /* MID conv3x3 0xFE00-09 */
    for(int i=0;i<9;i++) fe((u8)i, (u8)k[i]);
    fe(0x09, shift);
}

static void print_menu(void){
    xil_printf("\r\n==== filter console (single key, live) ====\r\n");
    xil_printf(" MID point: 0 pass 1 invert 2 gray 3 BGRswap 4 thresh 5 R 6 G 7 B\r\n");
    xil_printf(" MID conv : g gauss  s sobelX  h sharpen  l laplacian  o outline  e emboss\r\n");
    xil_printf(" PRE      : v median  b blur   V off\r\n");
    xil_printf(" POST     : t threshold  T off\r\n");
    xil_printf(" dither   : d off  n halftone(1b)  p poster(2b)  r random(2b)\r\n");
    xil_printf(" params (type + Enter):\r\n");
    xil_printf("   k c0 c1..c8 [sh]  custom 3x3 kernel (signed, +conv)   op N  proc_op\r\n");
    xil_printf("   pt N  PRE threshold(0..255)   qt N  POST threshold    pre N / post N\r\n");
    xil_printf("   fe LO VAL  raw 0xFE-page write (hex/dec) = any config reg\r\n");
    xil_printf(" x = reset all to passthrough     ? = menu\r\n> ");
}

static void apply_cmd(char c){
    static const signed char K_GAUSS[9]={1,2,1,2,4,2,1,2,1};
    static const signed char K_SOBEL[9]={-1,0,1,-2,0,2,-1,0,1};
    static const signed char K_SHARP[9]={0,-1,0,-1,5,-1,0,-1,0};
    static const signed char K_LAPL [9]={0,-1,0,-1,4,-1,0,-1,0};
    static const signed char K_OUTL [9]={-1,-1,-1,-1,8,-1,-1,-1,-1};
    static const signed char K_EMBO [9]={-2,-1,0,-1,1,1,0,1,2};
    switch(c){
        case '0': set_proc_op(0); xil_printf("MID passthrough\r\n"); break;
        case '1': set_proc_op(1); xil_printf("MID invert\r\n"); break;
        case '2': set_proc_op(2); xil_printf("MID grayscale\r\n"); break;
        case '3': set_proc_op(3); xil_printf("MID BGR-swap\r\n"); break;
        case '4': set_proc_op(4); xil_printf("MID threshold\r\n"); break;
        case '5': set_proc_op(5); xil_printf("MID R-only\r\n"); break;
        case '6': set_proc_op(6); xil_printf("MID G-only\r\n"); break;
        case '7': set_proc_op(7); xil_printf("MID B-only\r\n"); break;
        case 'g': load_kernel(K_GAUSS,4); set_proc_op(8); xil_printf("conv gaussian (blur)\r\n"); break;
        case 's': load_kernel(K_SOBEL,0); set_proc_op(8); xil_printf("conv sobelX (edge)\r\n"); break;
        case 'h': load_kernel(K_SHARP,0); set_proc_op(8); xil_printf("conv sharpen\r\n"); break;
        case 'l': load_kernel(K_LAPL ,0); set_proc_op(8); xil_printf("conv laplacian (edge)\r\n"); break;
        case 'o': load_kernel(K_OUTL ,0); set_proc_op(8); xil_printf("conv outline\r\n"); break;
        case 'e': load_kernel(K_EMBO ,0); set_proc_op(8); xil_printf("conv emboss\r\n"); break;
        case 'v': fe(0x46, 9); xil_printf("PRE median (denoise)\r\n"); break;
        case 'b': fe(0x46, 8); xil_printf("PRE gaussian blur\r\n"); break;
        case 'V': fe(0x46, 0); xil_printf("PRE off\r\n"); break;
        case 't': fe(0x49, 0x80); fe(0x48, 4); xil_printf("POST threshold (mid)\r\n"); break;
        case 'T': fe(0x48, 0); xil_printf("POST off\r\n"); break;
        case 'd': fe(0x4A, 0x00); xil_printf("dither off\r\n"); break;
        case 'n': fe(0x4A, 0x05); xil_printf("dither halftone 1b\r\n"); break;   /* en|ordered|bits1 */
        case 'p': fe(0x4A, 0x09); xil_printf("dither poster 2b\r\n"); break;     /* en|ordered|bits2 */
        case 'r': fe(0x4A, 0x0B); xil_printf("dither random 2b\r\n"); break;     /* en|random|bits2 */
        case 'x': set_proc_op(0); fe(0x46,0); fe(0x48,0); fe(0x4A,0);
                  xil_printf("reset all -> passthrough\r\n"); break;
        case '?': case 'H': print_menu(); return;
        case '\r': case '\n': case ' ': return;
        default: xil_printf("? '%c' (? for menu)\r\n", c); break;
    }
    xil_printf("> ");
}

/* line-command parser: numeric params (kernels, thresholds, raw 0xFE writes).
 * strtol base 0 -> accepts decimal, 0x hex, and negative (signed kernel coeffs). */
static void exec_line(char *ln){
    char *tk[16]; int n=0;
    for(char *p=ln; *p && n<16; ){
        while(*p==' '||*p=='\t') p++;
        if(!*p) break;
        tk[n++]=p;
        while(*p && *p!=' ' && *p!='\t') p++;
        if(*p) *p++=0;
    }
    if(n==0){ xil_printf("> "); return; }
    char *c=tk[0];
    if(n==1 && c[1]==0){ apply_cmd(c[0]); return; }          /* single-letter preset */
    if(!strcmp(c,"op")  && n>=2){ int v=strtol(tk[1],0,0); set_proc_op((u8)v); xil_printf("proc_op=%d\r\n> ",v); return; }
    if(!strcmp(c,"pre") && n>=2){ int v=strtol(tk[1],0,0); fe(0x46,(u8)v); xil_printf("pre_op=%d\r\n> ",v); return; }
    if(!strcmp(c,"post")&& n>=2){ int v=strtol(tk[1],0,0); fe(0x48,(u8)v); xil_printf("post_op=%d\r\n> ",v); return; }
    if(!strcmp(c,"pt")  && n>=2){ int v=strtol(tk[1],0,0); fe(0x47,(u8)v); fe(0x46,4); xil_printf("PRE threshold=%d\r\n> ",v); return; }
    if(!strcmp(c,"qt")  && n>=2){ int v=strtol(tk[1],0,0); fe(0x49,(u8)v); fe(0x48,4); xil_printf("POST threshold=%d\r\n> ",v); return; }
    if(!strcmp(c,"fe")  && n>=3){ int lo=strtol(tk[1],0,0),v=strtol(tk[2],0,0); fe((u8)lo,(u8)v); xil_printf("fe[0x%02x]=0x%02x\r\n> ",lo&0xFF,v&0xFF); return; }
    if(!strcmp(c,"k")   && n>=10){
        for(int i=0;i<9;i++) fe((u8)i,(u8)strtol(tk[1+i],0,0));
        int sh=(n>=11)?strtol(tk[10],0,0):0;
        fe(0x09,(u8)sh); set_proc_op(8);
        xil_printf("custom 3x3 kernel loaded (shift=%d), conv on\r\n> ",sh); return;
    }
    xil_printf("? unknown '%s' (? for menu)\r\n> ", c);
}

/* ============================ main ============================ */
int main(void){
    fix_uart_rx_mio18();   /* MIO 48/49 -> 1.8V so the UART console RX works */
    xil_printf("\r\n============================================================\r\n");
    xil_printf(" Route A M2-M4: bare-metal OV5640 -> FPGA -> HDMI (no PYNQ)\r\n");
    xil_printf("============================================================\r\n");

    /* boot race: re-assert cam_gpio (RESETB high) right after boot */
    fl_write_raw(CAM_GPIO); usleep(1000);

    /* wait for the in-bitstream init FSM to drain (page0 bit26 = setup_ready) */
    int ready=0;
    for(int i=0;i<150;i++){ if((read_dbg(0x00)>>26)&1){ ready=1; break; } usleep(100000); }
    xil_printf("[1] setup_ready=%d (page0)\r\n", ready);

    /* chip_init: cam_gpio high, RESETB pulse, SW reset + power-down */
    fl_write_raw(CAM_GPIO); usleep(500000);
    cam_resetb_pulse(5,30);
    sccb_write(0x3008,0x82); usleep(20000);   /* SW reset */
    sccb_write(0x3008,0x42); usleep(20000);   /* power down */

    int idh=sccb_read(0x300A), idl=sccb_read(0x300B);
    xil_printf("[2] chip ID = %02X%02X (expect 5640)\r\n", idh&0xFF, idl&0xFF);

    xil_printf("[3] running %u-step OV5640 init...\r\n", (unsigned)OV5640_INIT_N);
    int nack=run_full_init();
    xil_printf("    init done, NACKs=%d (STREAM_ON issued)\r\n", nack);
    sleep(5);   /* chip PLL/AEC settle before lock (Python uses 10s; 5s is enough) */

    idelay_set(8,8);
    frame_lines_set(480, /*use_lsle*/1, /*dt*/0x22, /*sof_synth*/1, /*force_expected*/1);
    xil_printf("[4] frame_lines=480 lsle=1 dt=0x22 synth=1 force=1\r\n");

    xil_printf("[5] arm RGB565 (stream cycle)\r\n");
    arm_rgb565();

    xil_printf("[6] D-PHY software lock (8x8 bitslip sweep, up to 9 phases)...\r\n");
    set_hw_lock(0); usleep(300000);           /* inhibit HW FSM -> software lock */
    int lr = lock_mode(8, 14);
    xil_printf("    lock result = %s\r\n", lr?"FAILED":"LOCKED");

    /* status */
    snap_t d; measure(&d,1000);
    u32 lastfl = read_dbg(0x05) & 0xFFFF;
    idh=sccb_read(0x300A); idl=sccb_read(0x300B);
    xil_printf("[7] link: long=%u/s crc_err=%u fs=%u fe=%u ls=%u le=%u lastlines=%u chip=%02X%02X\r\n",
               d.long_pkt, d.crc_err, d.fs, d.fe, d.ls, d.le, lastfl, idh&0xFF, idl&0xFF);

    xil_printf("[8] starting VDMA (S2MM cam->DDR + MM2S DDR->HDMI)\r\n");
    vdma_prefill();
    vdma_start();
    xil_printf("    HDMI should now be LIVE.\r\n");

    /* live: camera -> HDMI standalone. Interactive filter console over UART1 (COM4). */
    xil_printf("[9] LIVE: OV5640 -> FPGA -> HDMI standalone (blank=14).\r\n");
    print_menu();
    char line[96]; int li=0;
    for(;;){
        if(!uart_rx_ready()){ usleep(2000); continue; }
        char ch = uart_getc();
        if(ch=='\r' || ch=='\n'){
            line[li]=0; xil_printf("\r\n");
            exec_line(line); li=0;
        } else if(ch==0x08 || ch==0x7F){           /* backspace/del */
            if(li>0){ li--; xil_printf("\b \b"); }
        } else if(li < (int)sizeof(line)-1){
            line[li++]=ch; xil_printf("%c", ch);   /* echo (most serial terminals don't local-echo) */
        }
    }
    return 0;
}
