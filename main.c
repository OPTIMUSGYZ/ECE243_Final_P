/* This files provides address values that exist in the system */

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
#define BOARD                 "DE1-SoC"

/* Memory */
#define DDR_BASE              0x00000000
#define DDR_END               0x3FFFFFFF
#define A9_ONCHIP_BASE        0xFFFF0000
#define A9_ONCHIP_END         0xFFFFFFFF
#define SDRAM_BASE            0xC0000000
#define SDRAM_END             0xC3FFFFFF
#define FPGA_ONCHIP_BASE      0xC8000000
#define FPGA_ONCHIP_END       0xC803FFFF
#define FPGA_CHAR_BASE        0xC9000000
#define FPGA_CHAR_END         0xC9001FFF

/* Cyclone V FPGA devices */
#define LEDR_BASE             0xFF200000
#define HEX3_HEX0_BASE        0xFF200020
#define HEX5_HEX4_BASE        0xFF200030
#define SW_BASE               0xFF200040
#define KEY_BASE              0xFF200050
#define JP1_BASE              0xFF200060
#define JP2_BASE              0xFF200070
#define PS2_BASE              0xFF200100
#define PS2_DUAL_BASE         0xFF200108
#define JTAG_UART_BASE        0xFF201000
#define JTAG_UART_2_BASE      0xFF201008
#define IrDA_BASE             0xFF201020
#define TIMER_BASE            0xFF202000
#define AV_CONFIG_BASE        0xFF203000
#define PIXEL_BUF_CTRL_BASE   0xFF203020
#define CHAR_BUF_CTRL_BASE    0xFF203030
#define AUDIO_BASE            0xFF203040
#define VIDEO_IN_BASE         0xFF203060
#define ADC_BASE              0xFF204000

/* Cyclone V HPS devices */
#define HPS_GPIO1_BASE        0xFF709000
#define HPS_TIMER0_BASE       0xFFC08000
#define HPS_TIMER1_BASE       0xFFC09000
#define HPS_TIMER2_BASE       0xFFD00000
#define HPS_TIMER3_BASE       0xFFD01000
#define FPGA_BRIDGE           0xFFD0501C

/* ARM A9 MPCORE devices */
#define   PERIPH_BASE         0xFFFEC000    // base address of peripheral devices
#define   MPCORE_PRIV_TIMER   0xFFFEC600    // PERIPH_BASE + 0x0600

/* Interrupt controller (GIC) CPU interface(s) */
#define MPCORE_GIC_CPUIF      0xFFFEC100    // PERIPH_BASE + 0x100
#define ICCICR                0x00          // offset to CPU interface control reg
#define ICCPMR                0x04          // offset to interrupt priority mask reg
#define ICCIAR                0x0C          // offset to interrupt acknowledge reg
#define ICCEOIR               0x10          // offset to end of interrupt reg
/* Interrupt controller (GIC) distributor interface(s) */
#define MPCORE_GIC_DIST       0xFFFED000    // PERIPH_BASE + 0x1000
#define ICDDCR                0x00          // offset to distributor control reg
#define ICDISER               0x100         // offset to interrupt set-enable regs
#define ICDICER               0x180         // offset to interrupt clear-enable regs
#define ICDIPTR               0x800         // offset to interrupt processor targets regs
#define ICDICFR               0xC00         // offset to interrupt configuration regs


/* VGA colors */
#define WHITE 0xFFFF
#define YELLOW 0xFFE0
#define cPlayer 0xF800
#define cPlayerBorder 0x07E0
#define BLUE 0x001F
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define cCannonBase 0xC618
#define cBullet 0xFC18
#define cBulletDeco1 0xFFFE
#define cBulletDeco2 0x0002
#define ORANGE 0xFC00
#define cCannonBarrel 0x0000
#define cPlayerDeco 0x0001

#define ABS(x) (((x) > 0) ? (x) : -(x))

/* Screen size. */
#define RESOLUTION_X 320
#define RESOLUTION_Y 240

/* Constants for animation */
#define BOX_LEN 2
#define NUM_CANNONS 10
#define cannonSize 9
#define playerSize 15

#define FALSE 0
#define TRUE 1

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

// Begin part3.c code for Lab 7


volatile int pixel_buffer_start; // global variable
volatile short int display[RESOLUTION_Y][RESOLUTION_X];
// 2 sets of drewPixel for front and back buffer
volatile int drewPixelBack[RESOLUTION_X * RESOLUTION_Y][2];
volatile int drewPixelFront[RESOLUTION_X * RESOLUTION_Y][2];
volatile int back = 1;

void draw_line(int x0, int y0, int x1, int y1, short int colour);

// number of drawn pixel in each buffer
volatile int ndrewPixelBack = 0;
volatile int ndrewPixelFront = 0;

volatile int dead = 0;

void swap(int *a, int *b);

void plot_pixel(int x, int y, short int line_color);

void clear_screen();

void wait4VSync();

void drawCannonLeftEdge(int x, int y, double direction, int appear);

void drawCannonRightEdge(int x, int y, double direction, int appear);

void drawCannonTopEdge(int x, int y, double direction, int appear);

void drawCannonBottomEdge(int x, int y, double direction, int appear);

void drawPlayer(int x, int y);

void drawBullet(int x, int y, int appear);

int drawing = 1;

int main(void) {
    volatile int *pixel_ctrl_ptr;
    if (drawing)
        pixel_ctrl_ptr = (int *) PIXEL_BUF_CTRL_BASE;
    int i;
    /*
         *     c4   c5   c6
         *
         * c0                c2
         *
         * c1                c3
         *
         *     c7   c8   c9
         */
    int cannonX[NUM_CANNONS] = {0, 0, RESOLUTION_X - 1, RESOLUTION_X - 1, RESOLUTION_X / 4 - cannonSize,
                                RESOLUTION_X / 4 * 2 - cannonSize, RESOLUTION_X / 4 * 3 - cannonSize,
                                RESOLUTION_X / 4 - cannonSize, RESOLUTION_X / 4 * 2 - cannonSize,
                                RESOLUTION_X / 4 * 3 - cannonSize};
    int cannonY[NUM_CANNONS] = {RESOLUTION_Y / 3 - cannonSize, RESOLUTION_Y / 3 * 2 - cannonSize,
                                RESOLUTION_Y / 3 - cannonSize, RESOLUTION_Y / 3 * 2 - cannonSize, 0, 0, 0,
                                RESOLUTION_Y - 1, RESOLUTION_Y - 1, RESOLUTION_Y - 1};
    //int cannonYes[10] = {TRUE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE};
    int cannonYes[NUM_CANNONS] = {TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, TRUE};
    int cannonDirections[NUM_CANNONS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int d[NUM_CANNONS] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

    /* set front pixel buffer to start of FPGA On-chip memory */
    if (drawing)
        *(pixel_ctrl_ptr + 1) = FPGA_ONCHIP_BASE; // first store the address in the back buffer
    /* now, swap the front/back buffers, to set the front buffer location */
    if (drawing)
        wait4VSync();
    /* initialize a pointer to the pixel buffer, used by drawing functions */
    if (drawing)
        pixel_buffer_start = *pixel_ctrl_ptr;
    clear_screen(); // pixel_buffer_start points to the pixel buffer
    /* set back pixel buffer to start of SDRAM memory */
    if (drawing)
        *(pixel_ctrl_ptr + 1) = SDRAM_BASE;
    if (drawing)
        pixel_buffer_start = *(pixel_ctrl_ptr + 1); // we draw on the back buffer
    clear_screen(); // pixel_buffer_start points to the pixel buffer

    int playerX = RESOLUTION_X / 2 - 1, playerY = RESOLUTION_Y / 2 - 1;
    int dxPlayer = 1, dyPlayer = 1;
    while (1) {
        /* Erase any boxes and lines that were drawn in the last iteration */
        clear_screen();
        drawPlayer(playerX, playerY);
        for (i = 0; i < NUM_CANNONS; i++) {
            if (i < 4) {
                cannonDirections[i] =
                        atan(((double) (playerY - cannonY[i]) / (double) (playerX - cannonX[i]))) / M_PI * 180;
            } else {
                cannonDirections[i] =
                        atan(((double) (playerX - cannonX[i]) / (double) (playerY - cannonY[i]))) / M_PI * 180;
            }
            if (cannonDirections[i] > 63) {
                cannonDirections[i] = 63;
            } else if (cannonDirections[i] < -63) {
                cannonDirections[i] = -63;
            }
        }
        drawCannonLeftEdge(cannonX[0], cannonY[0], cannonDirections[0], cannonYes[0]);
        drawCannonLeftEdge(cannonX[1], cannonY[1], cannonDirections[1], cannonYes[1]);
        drawCannonRightEdge(cannonX[2], cannonY[2], cannonDirections[2], cannonYes[2]);
        drawCannonRightEdge(cannonX[3], cannonY[3], cannonDirections[3], cannonYes[3]);
        drawCannonTopEdge(cannonX[4], cannonY[4], cannonDirections[4], cannonYes[4]);
        drawCannonTopEdge(cannonX[5], cannonY[5], cannonDirections[5], cannonYes[5]);
        drawCannonTopEdge(cannonX[6], cannonY[6], cannonDirections[6], cannonYes[6]);
        drawCannonBottomEdge(cannonX[7], cannonY[7], cannonDirections[7], cannonYes[7]);
        drawCannonBottomEdge(cannonX[8], cannonY[8], cannonDirections[8], cannonYes[8]);
        drawCannonBottomEdge(cannonX[9], cannonY[9], cannonDirections[9], cannonYes[9]);


        drawBullet(150, 150, 1);


        int j;
//        int k;
        for (i = 0; i < playerSize && !dead; i++) {
            for (j = 0; j < playerSize && !dead; j++) {
//                printf("px:%d, py:%d, i:%d, j:%d, d:%d\n", playerX, playerY, i, j, display[playerY + i][playerX + j]);
                if (display[playerY + i][playerX + j]) {
                    dead = 1;
//                    for (k = 0; k < playerSize; k++) {
//                        draw_line(playerX, playerY + k, playerX + playerSize - 1, playerY + k, MAGENTA);
//                    }
                    printf("Dead at: (%d, %d)", playerX + j, playerY + i);
                }
            }
        }
        if (playerX <= 0 && dxPlayer < 0) {
            dxPlayer = 1;
        } else if (playerX >= RESOLUTION_X - 1 - playerSize && dxPlayer > 0) {
            dxPlayer = -1;
        }
        if (playerY <= 0 && dyPlayer < 0) {
            dyPlayer = 1;
        } else if (playerY >= RESOLUTION_Y - 1 - playerSize && dyPlayer > 0) {
            dyPlayer = -1;
        }
        playerX += dxPlayer;
        playerY += dyPlayer;

        if (dead) {
            if (!drawing) {
                FILE *f = fopen("test.txt", "w");
                for (i = 0; i < RESOLUTION_X; i++) {
                    for (j = 0; j < RESOLUTION_Y; j++) {
                        fprintf(f, "%d ", display[i][j]);
                    }
                    fprintf(f, "\n");
                }
                exit(0);
            }

            while (1);
        }
        if (drawing)
            wait4VSync(); // swap front and back buffers on VGA vertical sync
        if (drawing)
            pixel_buffer_start = *(pixel_ctrl_ptr + 1); // new back buffer
        back = !back;
    }
}

void wait4VSync() {
    volatile int *pixelCtrlPtr = (int *) PIXEL_BUF_CTRL_BASE; // pixel controller
    register int status;
    *pixelCtrlPtr = 1; // write 1 to buffer reg. to start sync process
    status = *(pixelCtrlPtr + 3);
    while ((status & 0b1) != 0) {
        status = *(pixelCtrlPtr + 3);
    }
}

void swap(int *a, int *b) {
    int temp = *a;
    *a = *b;
    *b = temp;
}

void draw_line(int x0, int y0, int x1, int y1, short int colour) {
    int is_steep = ABS(y1 - y0) > ABS(x1 - x0);
    if (is_steep) {
        swap(&x0, &y0);
        swap(&x1, &y1);
    }
    if (x0 > x1) {
        swap(&x0, &x1);
        swap(&y0, &y1);
    }

    int dX = x1 - x0;
    int dY = ABS(y1 - y0);
    int error = -(dX / 2);
    int y = y0;
    int y_step;

    if (y0 < y1) {
        y_step = 1;
    } else {
        y_step = -1;
    }

    int x;
    for (x = x0; x <= x1; x++) {
        if (is_steep) {
            plot_pixel(y, x, colour);
//            if (back) {
//                drewPixelBack[ndrewPixelBack][0] = y;
//                drewPixelBack[ndrewPixelBack][1] = x;
//            } else {
//                drewPixelFront[ndrewPixelFront][0] = y;
//                drewPixelFront[ndrewPixelFront][1] = x;
//            }
        } else {
            plot_pixel(x, y, colour);
            // add pixel to corresponding drewPixel
//            if (back) {
//                drewPixelBack[ndrewPixelBack][0] = x;
//                drewPixelBack[ndrewPixelBack][1] = y;
//            } else {
//                drewPixelFront[ndrewPixelFront][0] = x;
//                drewPixelFront[ndrewPixelFront][1] = y;
//            }
        }
        error += dY;
        if (error > 0) {
            y += y_step;
            error -= dX;
        }
//        if (back) {
//            ndrewPixelBack += 1;
//        } else {
//            ndrewPixelFront += 1;
//        }
    }
}

void clear_screen() {
    int x, y, i;
    // if no pixel are drawn in both buffer, clear everything
    if ((back && ndrewPixelBack == 0) || (!back && ndrewPixelFront == 0)) {
        for (x = 0; x < RESOLUTION_X; x++) {
            for (y = 0; y < RESOLUTION_Y; y++) {
                plot_pixel(x, y, WHITE);
            }
        }
    }

    // for either back or front buffer, loop through drawn pixels, white each, and clear
    if (back) {
        for (i = 0; i < ndrewPixelBack; i++) {
            x = drewPixelBack[i][0];
            y = drewPixelBack[i][1];
            plot_pixel(x, y, WHITE);
            drewPixelBack[i][0] = -1;
            drewPixelBack[i][1] = -1;
        }
        ndrewPixelBack = 0; // reset counter
    } else {
        for (i = 0; i < ndrewPixelFront; i++) {
            x = drewPixelFront[i][0];
            y = drewPixelFront[i][1];
            plot_pixel(x, y, WHITE);
            drewPixelFront[i][0] = -1;
            drewPixelFront[i][1] = -1;
        }
        ndrewPixelFront = 0; // reset counter
    }
}

void plot_pixel(int x, int y, short int line_color) {
    if (line_color != (short) WHITE) {
        if (back) {
            drewPixelBack[ndrewPixelBack][0] = x;
            drewPixelBack[ndrewPixelBack][1] = y;
        } else {
            drewPixelFront[ndrewPixelFront][0] = x;
            drewPixelFront[ndrewPixelFront][1] = y;
        }
        if (back) {
            ndrewPixelBack += 1;
        } else {
            ndrewPixelFront += 1;
        }
    }
    //display[y][x] = line_color;
    if (line_color == (short) WHITE || line_color == (short) cPlayer || line_color == (short) cPlayerDeco ||
        line_color == (short) cPlayerBorder) {
        display[y][x] = 0;
    } else if (line_color == (short) MAGENTA) {
        display[y][x] = 2;
    } else {
        display[y][x] = 1;
    }

    if (drawing)
        *(short int *) (pixel_buffer_start + (y << 10) + (x << 1)) = line_color;
}

void drawCannonLeftEdge(int x, int y, double direction, int appear) {
    if (appear) {
        direction = -direction / 360 * 2 * M_PI;
        int i;
        int vX0 = x;
        int vY0 = y + cannonSize;
        double vX1 = vX0 + sqrt(5) * cannonSize * cos(direction + atan(0.5));
        double vY1 = vY0 - sqrt(5) * cannonSize * sin(direction + atan(0.5));
        double hX = vX0 + 2 * cannonSize * cos(direction);
        double hY = vY0 - 2 * cannonSize * sin(direction);
        double dx = (hX - vX1) / cannonSize;
        double dy = (hY - vY1) / cannonSize;
        for (i = 0; i <= 2 * cannonSize; i++) {
            draw_line(vX0, vY0, round(vX1 + dx * i), round(vY1 + dy * i), cCannonBarrel);
        }
        for (i = 0; i <= cannonSize; i++) {
            draw_line(x + i, y - round(sqrt(pow(cannonSize, 2) - pow(i, 2))) + cannonSize, x + i,
                      y + round(sqrt(pow(cannonSize, 2) - pow(i, 2))) + cannonSize,
                      cCannonBase);
        }
    }
}

void drawCannonRightEdge(int x, int y, double direction, int appear) {
    if (appear) {
        int i;
        direction = direction / 360 * 2 * M_PI;
        int vX0 = x;
        int vY0 = y + cannonSize;
        double vX1 = vX0 - sqrt(5) * cannonSize * cos(direction + atan(0.5));
        double vY1 = vY0 - sqrt(5) * cannonSize * sin(direction + atan(0.5));
        double hX = vX0 - 2 * cannonSize * cos(direction);
        double hY = vY0 - 2 * cannonSize * sin(direction);
        double dx = (hX - vX1) / cannonSize;
        double dy = (hY - vY1) / cannonSize;
        for (i = 0; i <= 2 * cannonSize; i++) {
            draw_line(vX0, vY0, round(vX1 + dx * i), round(vY1 + dy * i), cCannonBarrel);
        }
        for (i = 0; i <= cannonSize; i++) {
            draw_line(x - i, y - round(sqrt(pow(cannonSize, 2) - pow(i, 2))) + cannonSize, x - i,
                      y + round(sqrt(pow(cannonSize, 2) - pow(i, 2))) + cannonSize,
                      cCannonBase);
        }
    }
}

void drawCannonTopEdge(int x, int y, double direction, int appear) {
    if (appear) {
        int i;
        direction = (90 + direction) / 360 * 2 * M_PI;
        int vX0 = x + cannonSize;
        int vY0 = y;
        double vX1 = vX0 - sqrt(5) * cannonSize * cos(direction - atan(0.5));
        double vY1 = vY0 + sqrt(5) * cannonSize * sin(direction - atan(0.5));
        double hX = vX0 - 2 * cannonSize * cos(direction);
        double hY = vY0 + 2 * cannonSize * sin(direction);
        double dx = (hX - vX1) / cannonSize;
        double dy = (hY - vY1) / cannonSize;
        for (i = 0; i <= 2 * cannonSize; i++) {
            draw_line(vX0, vY0, round(vX1 + dx * i), round(vY1 + dy * i), cCannonBarrel);
        }
        for (i = 0; i <= cannonSize; i++) {
            draw_line(x - round(sqrt(pow(cannonSize, 2) - pow(i, 2))) + cannonSize, y + i,
                      x + round(sqrt(pow(cannonSize, 2) - pow(i, 2))) + cannonSize,
                      y + i,
                      cCannonBase);
        }
    }
}

void drawCannonBottomEdge(int x, int y, double direction, int appear) {
    if (appear) {
        int i;
        direction = (90 - direction) / 360 * 2 * M_PI;
        int vX0 = x + cannonSize;
        int vY0 = y;
        double vX1 = vX0 - sqrt(5) * cannonSize * cos(direction - atan(0.5));
        double vY1 = vY0 - sqrt(5) * cannonSize * sin(direction - atan(0.5));
        double hX = vX0 - 2 * cannonSize * cos(direction);
        double hY = vY0 - 2 * cannonSize * sin(direction);
        double dx = (hX - vX1) / cannonSize;
        double dy = (hY - vY1) / cannonSize;
        for (i = 0; i <= 2 * cannonSize; i++) {
            draw_line(vX0, vY0, round(vX1 + dx * i), round(vY1 + dy * i), cCannonBarrel);
        }
        for (i = 0; i <= cannonSize; i++) {
            draw_line(x - round(sqrt(pow(cannonSize, 2) - pow(i, 2))) + cannonSize, y - i,
                      x + round(sqrt(pow(cannonSize, 2) - pow(i, 2))) + cannonSize,
                      y - i,
                      cCannonBase);
        }
    }
}

void drawPlayer(int x, int y) {
    int borderWidth = round(playerSize / 7);
    int i, j, k, l;
    for (i = 0; i < borderWidth; i++) {
        draw_line(x + i, y + i, x + playerSize - 1 - i, y + i, cPlayerBorder);
        draw_line(x + i, y + playerSize - 1 - i, x + playerSize - 1 - i, y + playerSize - 1 - i, cPlayerBorder);
        draw_line(x + i, y + i, x + i, y + playerSize - 1 - i, cPlayerBorder);
        draw_line(x + playerSize - 1 - i, y + i, x + playerSize - 1 - i, y + playerSize - 1 - i, cPlayerBorder);
    }
    for (i = 0; i < playerSize - borderWidth * 2; i++) {
        draw_line(x + borderWidth, y + borderWidth + i, x + playerSize - 1 - borderWidth, y + borderWidth + i, cPlayer);
    }
    int seedRow = 2;
    int seedWidth = borderWidth;
    int dSeed = round((playerSize - borderWidth * 2 - seedRow * seedWidth) / (seedRow + 1));
    for (i = 0; i < seedRow; i++) {
        for (j = 0; j < seedRow; j++) {
            for (k = 0; k < seedWidth; k++) {
                for (l = 0; l < seedWidth; l++) {
                    plot_pixel(x + borderWidth + (i + 1) * dSeed + i * seedWidth + k,
                               y + borderWidth + (j + 1) * dSeed + j * seedWidth + l,
                               cPlayerDeco);
                }
            }
        }
    }
}

void drawBullet(int x, int y, int appear) {
    if (appear) {
        int bulletSize = cannonSize - 1;
        int i;
        for (i = 0; i < bulletSize; i++) {
            draw_line(x, y + i, x + bulletSize - 1, y + i, cBullet);
        }
        int nEye = 2;
        int eyeWidth = bulletSize / 4;
        int dEye = round((bulletSize - nEye * eyeWidth) / (nEye + 1));
        for (i = 0; i < eyeWidth; i++) {
            if (i == eyeWidth - 1) {
                draw_line(x + dEye + i, y + dEye, x + dEye + i, y + dEye + eyeWidth - 1, cBulletDeco1);
                draw_line(x + dEye + i, y + dEye + eyeWidth + eyeWidth / 2, x + dEye + i,
                          y + dEye + eyeWidth * 2 + eyeWidth / 2 - 1, cBulletDeco2);
            } else {
                draw_line(x + dEye + i, y + dEye, x + dEye + i, y + dEye + eyeWidth - 1, cBulletDeco2);
            }
        }
        for (i = 0; i < eyeWidth; i++) {
            if (i == 0) {
                draw_line(x + dEye * 2 + eyeWidth + i, y + dEye, x + dEye * 2 + eyeWidth + i, y + dEye + eyeWidth - 1,
                          cBulletDeco1);
                draw_line(x + dEye * 2 + eyeWidth + i, y + dEye + eyeWidth + eyeWidth / 2, x + dEye * 2 + eyeWidth + i,
                          y + dEye + eyeWidth * 2 + eyeWidth / 2 - 1,
                          cBulletDeco2);
            } else {
                draw_line(x + dEye * 2 + eyeWidth + i, y + dEye, x + dEye * 2 + eyeWidth + i, y + dEye + eyeWidth - 1,
                          cBulletDeco2);
            }
        }
    }
}

#pragma clang diagnostic pop