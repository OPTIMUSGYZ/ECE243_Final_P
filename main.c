/* This files provides address values that exist in the system */

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
#define SDRAM_BASE            0xC0000000
#define FPGA_ONCHIP_BASE      0xC8000000
#define FPGA_CHAR_BASE        0xC9000000

/* Cyclone V FPGA devices */
#define LEDR_BASE             0xFF200000
#define HEX3_HEX0_BASE        0xFF200020
#define HEX5_HEX4_BASE        0xFF200030
#define SW_BASE               0xFF200040
#define KEY_BASE              0xFF200050
#define TIMER_BASE            0xFF202000
#define PIXEL_BUF_CTRL_BASE   0xFF203020
#define CHAR_BUF_CTRL_BASE    0xFF203030

/* VGA colors */
#define WHITE 0xFFFF
#define YELLOW 0xFFE0
#define RED 0xF800
#define GREEN 0x07E0
#define BLUE 0x001F
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define GREY 0xC618
#define PINK 0xFC18
#define ORANGE 0xFC00
#define BLACK 0x0000

#define ABS(x) (((x) > 0) ? (x) : -(x))

/* Screen size. */
#define RESOLUTION_X 320
#define RESOLUTION_Y 240

/* Constants for animation */
#define BOX_LEN 2
#define NUM_CANNONS 10
#define cannonSize 9

#define FALSE 0
#define TRUE 1

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

// Begin part3.c code for Lab 7


volatile int pixel_buffer_start; // global variable
// 2 sets of drewPixel for front and back buffer
volatile int drewPixelBack[RESOLUTION_X * RESOLUTION_Y][2];
volatile int drewPixelFront[RESOLUTION_X * RESOLUTION_Y][2];
volatile int back = 1;

void draw_line(int x0, int y0, int x1, int y1, short int colour);

// number of drawn pixel in each buffer
volatile int ndrewPixelBack = 0;
volatile int ndrewPixelFront = 0;

void swap(int *a, int *b);

void plot_pixel(int x, int y, short int line_color);

void clear_screen();

void wait4VSync();

void drawBorder();

void drawCannonLeftEdge(int x, int y, double direction, int appear);

void drawCannonRightEdge(int x, int y, double direction, int appear);

void drawCannonTopEdge(int x, int y, double direction, int appear);

void drawCannonBottomEdge(int x, int y, double direction, int appear);

int main(void) {
    volatile int *pixel_ctrl_ptr = (int *) PIXEL_BUF_CTRL_BASE;

    // clear drewPixelBack&2
    int i;
    for (i = 0; i < RESOLUTION_X * RESOLUTION_Y; i++) {
        drewPixelBack[i][0] = -1;
        drewPixelBack[i][1] = -1;
        drewPixelFront[i][0] = -1;
        drewPixelFront[i][1] = -1;
    }

    int lineColour[NUM_CANNONS], boxColour[NUM_CANNONS], dxBullets[NUM_CANNONS], dyBullets[NUM_CANNONS], xCannon[NUM_CANNONS], yCannon[NUM_CANNONS];
    short int colourArr[10] = {BLACK, YELLOW, RED, GREEN, BLUE, CYAN, MAGENTA, GREY, PINK, ORANGE};
    //int cannonYes[10] = {TRUE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE, FALSE};
    int cannonYes[NUM_CANNONS] = {TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, TRUE};
    int cannonDirections[NUM_CANNONS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int d[NUM_CANNONS] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

    // initialize location and direction of rectangles
    for (i = 0; i < NUM_CANNONS; i++) {
        boxColour[i] = colourArr[rand() % 10];
        lineColour[i] = colourArr[rand() % 10];
        xCannon[i] = rand() % RESOLUTION_X;
        yCannon[i] = rand() % RESOLUTION_Y;
        dxBullets[i] = rand() % 2 * 2 - 1;
        dyBullets[i] = rand() % 2 * 2 - 1;
    }

    /* set front pixel buffer to start of FPGA On-chip memory */
    *(pixel_ctrl_ptr + 1) = FPGA_ONCHIP_BASE; // first store the address in the back buffer
    /* now, swap the front/back buffers, to set the front buffer location */
    wait4VSync();
    /* initialize a pointer to the pixel buffer, used by drawing functions */
    pixel_buffer_start = *pixel_ctrl_ptr;
    clear_screen(); // pixel_buffer_start points to the pixel buffer
    /* set back pixel buffer to start of SDRAM memory */
    *(pixel_ctrl_ptr + 1) = SDRAM_BASE;
    pixel_buffer_start = *(pixel_ctrl_ptr + 1); // we draw on the back buffer
    clear_screen(); // pixel_buffer_start points to the pixel buffer

    int x0, x1, y0, y1;
    // remember to change to 1
    while (1) {
        /* Erase any boxes and lines that were drawn in the last iteration */
        clear_screen();
        /*
         *     c4   c5   c6
         *
         * c0                c2
         *
         * c1                c3
         *
         *     c7   c8   c9
         */
        drawCannonLeftEdge(0, RESOLUTION_Y / 3 - cannonSize, cannonSize, cannonDirections[0], cannonYes[0]);
        drawCannonLeftEdge(0, RESOLUTION_Y / 3 * 2 - cannonSize, cannonSize, cannonDirections[1], cannonYes[1]);
        drawCannonRightEdge(RESOLUTION_X - 1, RESOLUTION_Y / 3 - cannonSize, cannonSize, cannonDirections[2],
                            cannonYes[2]);
        drawCannonRightEdge(RESOLUTION_X - 1, RESOLUTION_Y / 3 * 2 - cannonSize, cannonSize, cannonDirections[3],
                            cannonYes[3]);
        drawCannonTopEdge(RESOLUTION_X / 4 - cannonSize, 0, cannonSize, cannonDirections[4], cannonYes[4]);
        drawCannonTopEdge(RESOLUTION_X / 4 * 2 - cannonSize, 0, cannonSize, cannonDirections[5], cannonYes[5]);
        drawCannonTopEdge(RESOLUTION_X / 4 * 3 - cannonSize, 0, cannonSize, cannonDirections[6], cannonYes[6]);
        drawCannonBottomEdge(RESOLUTION_X / 4 - cannonSize, RESOLUTION_Y - 1, cannonSize, cannonDirections[7],
                             cannonYes[7]);
        drawCannonBottomEdge(RESOLUTION_X / 4 * 2 - cannonSize, RESOLUTION_Y - 1, cannonSize, cannonDirections[8],
                             cannonYes[8]);
        drawCannonBottomEdge(RESOLUTION_X / 4 * 3 - cannonSize, RESOLUTION_Y - 1, cannonSize, cannonDirections[9],
                             cannonYes[9]);


        for (i = 0; i < NUM_CANNONS; i++) {
            if (cannonDirections[i] == 63 && d[i] == 1) { // if at left edge and wants to go left
                d[i] = -1;
            } else if (cannonDirections[i] == -63 && d[i] == -1) {
                d[i] = 1;
            }
            // update the box's location
            cannonDirections[i] += d[i];
        }

        // code for drawing the boxes and lines
        int j;
        for (i = 0; i < NUM_CANNONS; i++) {
            x0 = xCannon[i];
            y0 = yCannon[i];
            x1 = xCannon[(i + 1) % NUM_CANNONS];
            y1 = yCannon[(i + 1) % NUM_CANNONS];
            draw_line(x0, y0, x1, y1, lineColour[i]);
            // boxes are drawn at the bottom right of the line's end point
            for (j = 0; j <= BOX_LEN; j++) {
                x1 = x0 + BOX_LEN;
                y1 = y0;
                draw_line(x0, y0, x1, y1, boxColour[i]);
                y0 += 1;
            }
        }
        // code for updating the locations of boxes
        for (i = 0; i < NUM_CANNONS; i++) {
            if (xCannon[i] <= 0 && dxBullets[i] < 0) { // if at left edge and wants to go left
                dxBullets[i] = 1;
            } else if (xCannon[i] >= RESOLUTION_X - 1 - BOX_LEN &&
                       dxBullets[i] > 0) { // if at right edge and wants to go right
                dxBullets[i] = -1;
            }
            if (yCannon[i] <= 0 && dyBullets[i] < 0) { // if at top edge and wants to go up
                dyBullets[i] = 1;
            } else if (yCannon[i] >= RESOLUTION_Y - 1 - BOX_LEN &&
                       dyBullets[i] > 0) { // if at bottom edge and wants to go down
                dyBullets[i] = -1;
            }
            // update the box's location
            xCannon[i] += dxBullets[i];
            yCannon[i] += dyBullets[i];
        }

        wait4VSync(); // swap front and back buffers on VGA vertical sync
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
            if (back) {
                drewPixelBack[ndrewPixelBack][0] = y;
                drewPixelBack[ndrewPixelBack][1] = x;
            } else {
                drewPixelFront[ndrewPixelFront][0] = y;
                drewPixelFront[ndrewPixelFront][1] = x;
            }
        } else {
            plot_pixel(x, y, colour);
            // add pixel to corresponding drewPixel
            if (back) {
                drewPixelBack[ndrewPixelBack][0] = x;
                drewPixelBack[ndrewPixelBack][1] = y;
            } else {
                drewPixelFront[ndrewPixelFront][0] = x;
                drewPixelFront[ndrewPixelFront][1] = y;
            }
        }
        error += dY;
        if (error > 0) {
            y += y_step;
            error -= dX;
        }
        if (back) {
            ndrewPixelBack += 1;
        } else {
            ndrewPixelFront += 1;
        }
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

    // for either back or front buffer, loop through drawn pixels, black each, and clear
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
    *(short int *) (pixel_buffer_start + (y << 10) + (x << 1)) = line_color;
}

void drawCannonLeftEdge(int x, int y, double direction, int appear) {
    if (appear) {
        direction = direction / 360 * 2 * M_PI;
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
            draw_line(vX0, vY0, round(vX1 + dx * i), round(vY1 + dy * i), BLACK);
        }
        for (i = 0; i <= cannonSize; i++) {
            draw_line(x + i, y - round(sqrt(pow(cannonSize, 2) - pow(i, 2))) + cannonSize, x + i,
                      y + round(sqrt(pow(cannonSize, 2) - pow(i, 2))) + cannonSize,
                      GREY);
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
            draw_line(vX0, vY0, round(vX1 + dx * i), round(vY1 + dy * i), BLACK);
        }
        for (i = 0; i <= cannonSize; i++) {
            draw_line(x - i, y - round(sqrt(pow(cannonSize, 2) - pow(i, 2))) + cannonSize, x - i,
                      y + round(sqrt(pow(cannonSize, 2) - pow(i, 2))) + cannonSize,
                      GREY);
        }
    }
}

void drawCannonTopEdge(int x, int y, double direction, int appear) {
    if (appear) {
        int i;
        direction = (90 - direction) / 360 * 2 * M_PI;
        int vX0 = x + cannonSize;
        int vY0 = y;
        double vX1 = vX0 - sqrt(5) * cannonSize * cos(direction - atan(0.5));
        double vY1 = vY0 + sqrt(5) * cannonSize * sin(direction - atan(0.5));
        double hX = vX0 - 2 * cannonSize * cos(direction);
        double hY = vY0 + 2 * cannonSize * sin(direction);
        double dx = (hX - vX1) / cannonSize;
        double dy = (hY - vY1) / cannonSize;
        for (i = 0; i <= 2 * cannonSize; i++) {
            draw_line(vX0, vY0, round(vX1 + dx * i), round(vY1 + dy * i), BLACK);
        }
        for (i = 0; i <= cannonSize; i++) {
            draw_line(x - round(sqrt(pow(cannonSize, 2) - pow(i, 2))) + cannonSize, y + i,
                      x + round(sqrt(pow(cannonSize, 2) - pow(i, 2))) + cannonSize,
                      y + i,
                      GREY);
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
            draw_line(vX0, vY0, round(vX1 + dx * i), round(vY1 + dy * i), BLACK);
        }
        for (i = 0; i <= cannonSize; i++) {
            draw_line(x - round(sqrt(pow(cannonSize, 2) - pow(i, 2))) + cannonSize, y - i,
                      x + round(sqrt(pow(cannonSize, 2) - pow(i, 2))) + cannonSize,
                      y - i,
                      GREY);
        }
    }
}

#pragma clang diagnostic pop