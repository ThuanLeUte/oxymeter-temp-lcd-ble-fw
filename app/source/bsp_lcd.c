/**
 * @file       bsp_lcd.c
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-23
 * @author     Thuan Le
 * @brief      Board Support LCD for Spo2 and Heartrate board
 * 
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include <math.h>
#include "bsp_lcd.h"

// Font and image bit map
#include "font.h"
#include "big_number.h"
#include "small_number.h"
#include "miscellaneous.h"
#include "background.h"

/* Private defines ---------------------------------------------------- */
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const uint16_t *)(addr))

#define PI                   (3.14159265)
#define LCD_WIDTH            (240)
#define LCD_HEIGHT           (240)

#define LCD_WHITE            (0xFFFF)
#define LCD_BLACK            (0x0000)
#define LCD_BLUE             (0x001F)
#define LCD_BRED             (0XF81F)
#define LCD_GRED             (0XFFE0)
#define LCD_GBLUE            (0X07FF)
#define LCD_RED              (0xF800)
#define LCD_MAGENTA          (0xF81F)
#define LCD_GREEN            (0x07E0)
#define LCD_CYAN             (0x7FFF)
#define LCD_YELLOW           (0xFFE0)
#define LCD_BROWN            (0XBC40)
#define LCD_BRRED            (0XFC07)
#define LCD_GRAY             (0X8430)
#define LCD_DARKBLUE         (0X01CF)
#define LCD_LIGHTBLUE        (0X7D7C)
#define LCD_GRAYBLUE         (0X5458)
#define LCD_LIGHTGREEN       (0X841F)
#define LCD_LGRAY            (0XC618)
#define LCD_LGRAYBLUE        (0XA651)
#define LCD_LBBLUE           (0X2B12)

/* Private enumerate/structure ---------------------------------------- */
typedef struct
{
  uint8_t x_px;
  uint8_t y_px;
  const unsigned short *data;
}
bsp_lcd_img_t;

typedef struct
{
  uint8_t x_pos;
  uint8_t y_pos;
  bsp_lcd_img_t img;
}
bsp_lcd_t;

/* Private macros ----------------------------------------------------- */
#define IMG_INFO(item, _data, _x_px, _y_px)[item] = { .data = _data, .x_px = _x_px, .y_px = _y_px }
#define ITEM_INFO(item, _x_pos, _y_pos, _x_px, _y_px, _data)[item] = { .x_pos = _x_pos, .y_pos = _y_pos, .img.x_px = _x_px, .img.y_px = _y_px , .img.data = _data }

/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static gc9a01_t m_gc9a01;

static bsp_lcd_img_t BIG_NUM_TABLE[] = 
{
  //         +====+==========+======+======+
  //         |NUM |   Data   | X-Px | Y-Px |
  //         +----+----------+------+------+
     IMG_INFO(0   , num_big_0,    30,    42)
    ,IMG_INFO(1   , num_big_1,    30,    42)
    ,IMG_INFO(2   , num_big_2,    30,    42)
    ,IMG_INFO(3   , num_big_3,    30,    42)
    ,IMG_INFO(4   , num_big_4,    30,    42)
    ,IMG_INFO(5   , num_big_5,    30,    42)
    ,IMG_INFO(6   , num_big_6,    30,    42)
    ,IMG_INFO(7   , num_big_7,    30,    42)
    ,IMG_INFO(8   , num_big_8,    30,    42)
    ,IMG_INFO(9   , num_big_9,    30,    42)
    ,IMG_INFO(10  , num_big_n,    30,    42)
  //         +====+==========+======+======+
};

static bsp_lcd_img_t SMALL_NUM_TABLE[] = 
{
  //         +====+============+======+=====+
  //         |NUM |   Data     | X-Px | Y-x |
  //         +----+------------+------+-----+
     IMG_INFO(0   , num_small_0,    15,   21)
    ,IMG_INFO(1   , num_small_1,    15,   21)
    ,IMG_INFO(2   , num_small_2,    15,   21)
    ,IMG_INFO(3   , num_small_3,    15,   21)
    ,IMG_INFO(4   , num_small_4,    15,   21)
    ,IMG_INFO(5   , num_small_5,    15,   21)
    ,IMG_INFO(6   , num_small_6,    15,   21)
    ,IMG_INFO(7   , num_small_7,    15,   21)
    ,IMG_INFO(8   , num_small_8,    15,   21)
    ,IMG_INFO(9   , num_small_9,    15,   21)
    ,IMG_INFO(10  , num_small_n,    15,   21)
  //         +====+============+======+=====+
};

static bsp_lcd_t ITEMS_TABLE[] = 
{
  //          +====================+=======+=======+======+======+============+
  //          |ITEMS               | X-Pos | Y-Pos | X-Px | Y-Px | Data       |
  //          +--------------------+-------+-------+------+------+------------+
     ITEM_INFO(LCD_SPO2_BG         ,      0,      0,   240,   240, spo2_bg    )
    ,ITEM_INFO(LCD_TEMP_BG         ,      0,      0,   240,   240, temp_bg    )
    ,ITEM_INFO(LCD_BATT_FULL       ,    107,    215,    24,    15, batt_full  )
    ,ITEM_INFO(LCD_BATT_75         ,    107,    215,    24,    15, batt_75    )
    ,ITEM_INFO(LCD_BATT_50         ,    107,    215,    24,    15, batt_50    )
    ,ITEM_INFO(LCD_BATT_25         ,    107,    215,    24,    15, batt_25    )
    ,ITEM_INFO(LCD_BATT_0          ,    107,    215,    24,    15, batt_0     )
    ,ITEM_INFO(LCD_DOT             ,      0,      0,     9,     9, dot        )
    ,ITEM_INFO(LCD_DOT_N           ,      0,      0,     9,     9, dot_n      )
    ,ITEM_INFO(LCD_BIG_DOT         ,      0,      0,    10,     9, big_dot    )
    ,ITEM_INFO(LCD_SMALL_DOT       ,      0,      0,     5,     5, small_dot  )
    ,ITEM_INFO(LCD_BIG_C           ,    160,     85,    34,    27, big_c      )
    ,ITEM_INFO(LCD_BIG_F           ,    160,     85,    34,    27, big_f      )
    ,ITEM_INFO(LCD_BIG_C_F_N       ,    160,     85,    34,    27, big_c_f_n  )
    ,ITEM_INFO(LCD_SMALL_C         ,    140,    185,    18,    14, small_c    )
    ,ITEM_INFO(LCD_SMALL_F         ,    140,    185,    18,    14, small_f    )
    ,ITEM_INFO(LCD_SMALL_C_F_N     ,    140,    185,    18,    14, small_c_f_n)
    ,ITEM_INFO(LCD_SP02_NUM        ,     45,     85,     0,     0, NULL       )
    ,ITEM_INFO(LCD_HEART_RATE_NUM  ,     82,    185,     0,     0, NULL       )
    ,ITEM_INFO(LCD_TEMP_BIG_NUM    ,     28,     85,     0,     0, NULL       )
    ,ITEM_INFO(LCD_TEMP_SMALL_NUM  ,     75,    185,     0,     0, NULL       )
  //          +====================+=======+=======+======+======+============+
};

/* Private function prototypes ---------------------------------------- */
static void m_bsp_lcd_address_set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
static void m_bsp_lcd_write_pixel(uint16_t x, uint16_t y, uint16_t thin, uint16_t color);
static void m_bsp_lcd_draw_image(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, const short unsigned A[]);
static void m_bsp_lcd_fill_square(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
static void m_bsp_lcd_write_char(uint16_t x, uint16_t y, unsigned c, uint16_t color, uint16_t bg, uint8_t size);

static void m_bsp_lcd_display_spo2_progress(uint8_t spo2);

/* Function definitions ----------------------------------------------- */
void bsp_lcd_init(void)
{
  m_gc9a01.delay_ms   = bsp_delay_ms;
  m_gc9a01.gpio_write = bsp_gpio_write;
  m_gc9a01.spi_send   = bsp_spi_write;

  gc9a01_init(&m_gc9a01);

#ifdef TEMPERATURE_BOARD
  // Draw background image
  bsp_lcd_display_image(LCD_TEMP_BG);

  // Draw baterry image
  bsp_lcd_display_image(LCD_BATT_FULL);

  bsp_lcd_display_image(LCD_BIG_C);
  bsp_lcd_display_image(LCD_SMALL_F);

  bsp_lcd_display_big_temp_number(0);
  bsp_lcd_display_small_temp_number(0);
#else
  // Draw background image
  bsp_lcd_display_image(LCD_SPO2_BG);

  // Draw baterry image
  bsp_lcd_display_image(LCD_BATT_FULL);

  bsp_lcd_display_spo2_number(0);
  bsp_lcd_display_heartrate_number(0);
#endif // TEMPERATURE_BOARD
}

/* Public function for project ---------------------------------------- */
void bsp_lcd_display_image(bsp_lcd_item_t item)
{
  m_bsp_lcd_draw_image(ITEMS_TABLE[item].x_pos, ITEMS_TABLE[item].y_pos,
                       ITEMS_TABLE[item].img.x_px + ITEMS_TABLE[item].x_pos,
                       ITEMS_TABLE[item].img.y_px + ITEMS_TABLE[item].y_pos,
                       ITEMS_TABLE[item].img.data);
}

void bsp_lcd_spo2_display_number(bsp_lcd_item_t item, uint8_t num)
{
  uint8_t units, dozens, hundreds;
  uint16_t x_current_position, y_current_position, x_px, y_px;
  bsp_lcd_img_t *TABLE;

  hundreds = (num / 100);
  dozens   = (num % 100) / 10;
  units    = (num % 100) % 10;

  if (item == LCD_SP02_NUM)
  {
    TABLE = BIG_NUM_TABLE;
    m_bsp_lcd_display_spo2_progress(num);
  }
  else if (LCD_HEART_RATE_NUM)
    TABLE = SMALL_NUM_TABLE;

  // Get position at first item
  x_current_position = ITEMS_TABLE[item].x_pos;
  y_current_position = ITEMS_TABLE[item].y_pos;

  // Pixcel equal position of first item plus pixcel of item
  x_px = x_current_position + TABLE[hundreds].x_px;
  y_px = y_current_position + TABLE[hundreds].y_px;

  if (hundreds == 0)
  {
    m_bsp_lcd_draw_image(x_current_position, y_current_position,
                         x_px, y_px, TABLE[10].data);
  }
  else
  {
    m_bsp_lcd_draw_image(x_current_position, y_current_position,
                         x_px, y_px, TABLE[hundreds].data);
  }

  // Get current position and pixcel
  x_current_position = x_px;
  x_px = x_current_position + TABLE[dozens].x_px;

  m_bsp_lcd_draw_image(x_current_position, y_current_position,
                       x_px, y_px, TABLE[dozens].data);

  // Get current position and pixcel
  x_current_position = x_px;
  x_px = x_current_position + TABLE[units].x_px;

  m_bsp_lcd_draw_image(x_current_position, y_current_position,
                       x_px, y_px, TABLE[units].data);
}

void bsp_lcd_temp_display_number(bsp_lcd_item_t item, float num)
{
  uint8_t decimal, units, dot, dozens, hundreds;
  uint16_t x_current_position, y_current_position, x_px, y_px;
  bsp_lcd_img_t *TABLE;

  // Rounding data
  num = num + 0.05;

  hundreds = (num / 100);
  dozens   = ((uint8_t)num % 100) / 10;
  units    = ((uint8_t)num % 100) % 10;
  decimal  = (num - (uint8_t)num) * 10;

  NRF_LOG_INFO( "hundreds: %d", hundreds);
  NRF_LOG_INFO( "dozens: %d", dozens);
  NRF_LOG_INFO( "dozens: %d", dozens);
  NRF_LOG_INFO( "decimal: %d", decimal);

  if (item == LCD_TEMP_BIG_NUM)
  {
    dot   = LCD_BIG_DOT;
    TABLE = BIG_NUM_TABLE;
  }
  else if (LCD_TEMP_SMALL_NUM)
  {
    dot   = LCD_SMALL_DOT;
    TABLE = SMALL_NUM_TABLE;
  }

  // Get position at first item
  x_current_position = ITEMS_TABLE[item].x_pos;
  y_current_position = ITEMS_TABLE[item].y_pos;

  // Pixcel equal position of first item plus pixcel of item
  x_px = x_current_position + TABLE[hundreds].x_px;
  y_px = y_current_position + TABLE[hundreds].y_px;

  if (hundreds == 0)
  {
    m_bsp_lcd_draw_image(x_current_position, y_current_position,
                         x_px, y_px, TABLE[10].data);
  }
  else
  {
    m_bsp_lcd_draw_image(x_current_position, y_current_position,
                         x_px, y_px, TABLE[hundreds].data);
  }

  // Get current position and pixcel for dozens
  x_current_position = x_px;
  x_px = x_current_position + TABLE[dozens].x_px;

  m_bsp_lcd_draw_image(x_current_position, y_current_position,
                       x_px, y_px, TABLE[dozens].data);

  // Get current position and pixcel for units
  x_current_position = x_px;
  x_px = x_current_position + TABLE[units].x_px;

  m_bsp_lcd_draw_image(x_current_position, y_current_position,
                       x_px, y_px, TABLE[units].data);

  // Get current position and pixcel for dot
  x_current_position = x_px;
  x_px = x_current_position + ITEMS_TABLE[dot].img.x_px;

  m_bsp_lcd_draw_image(x_current_position, y_current_position,
                       x_px, y_current_position + ITEMS_TABLE[dot].img.y_px, ITEMS_TABLE[dot].img.data);

  // Get current position and pixcel for decimal
  x_current_position = x_px;
  x_px = x_current_position + TABLE[decimal].x_px;

  m_bsp_lcd_draw_image(x_current_position, y_current_position,
                       x_px, y_px, TABLE[decimal].data);
}

void bsp_lcd_temp_display_celsius_big_num(bool enable)
{
  bsp_lcd_display_image(LCD_BIG_C_F_N);
  bsp_lcd_display_image(LCD_SMALL_C_F_N);

  if (enable)
  {
    bsp_lcd_display_image(LCD_BIG_C);
    bsp_lcd_display_image(LCD_SMALL_F);
  }
  else
  {
    bsp_lcd_display_image(LCD_BIG_F);
    bsp_lcd_display_image(LCD_SMALL_C);
  }
}

/* Public function basic --------------------------------------------- */
void bsp_lcd_fill(uint16_t color)
{
  m_bsp_lcd_address_set(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
  gc9a01_write_cmd(&m_gc9a01, GC9A01_MEMORY_WRITE);

  for (uint16_t i = 0; i < LCD_HEIGHT; i++)
  {
    for (uint16_t j = 0; j < LCD_WIDTH; j++)
    {
      gc9a01_write_data(&m_gc9a01, (uint8_t *)&color, 2);
    }
  }
}

void bsp_lcd_write_string(uint16_t x, uint16_t y, const char c[],
                          uint16_t color, uint16_t bgcolor, uint8_t size)
{
  int i = 0;

  while (c[i] != '\0')
  {
    if (c[i] == '\n')
    {
      y += size * 8;
      x = 0;
    }
    else
    {
      m_bsp_lcd_write_char(x, y, c[i], color, bgcolor, size);

      x += 6 * size;
      i++;

      if (x > (LCD_WIDTH - size * 6))
      {
        y += size * 8;
        x = 0;
      }
    }
  }
}

/* Private function definitions --------------------------------------- */
/* Private function for project --------------------------------------- */
/**
 * @brief         LCD display spo2 progress
 *
 * @param[in]     spo2  Spo2 value
 *
 * @attention     None
 *
 * @return        None
 */
static void m_bsp_lcd_display_spo2_progress(uint8_t spo2)
{
  static uint16_t x_pos, y_pos;
  double degree;
  double val;

  // Value 0  -> 88  -> Circle red
  // Deg 220  -> 168 -> Degrees per value: 0.59

  // Value 88 -> 94  -> Circle violet
  // Deg 168  -> 56  -> Degrees per value: 18.9

  // Value 94 -> 100 -> Circle blue
  // Deg 56   -> -40 -> Degrees per value: 16

  // Remove previous position
  m_bsp_lcd_draw_image(x_pos, y_pos,
                       ITEMS_TABLE[LCD_DOT_N].img.x_px + x_pos,
                       ITEMS_TABLE[LCD_DOT_N].img.y_px + y_pos,
                       ITEMS_TABLE[LCD_DOT_N].img.data);

  // Get degree of the value on the circle
  if (spo2 <= 88)
  {
    degree = 220.0 - (double)(spo2 * 0.59);
  }
  else if (spo2 > 88 && spo2 <= 94)
  {
    spo2   = spo2 - 88;
    degree = 168 - (double)(spo2 * 18.9);
  }
  else
  {
    spo2   = spo2 - 94;
    degree = 56 - (double)(spo2 * 16);
  }

  // (a, b) is the cicle center position (116, -116)
  // x = a + r * cos(t)
  // y = b + r * sin(t)
  // Calculate position depend on the degree on the circle
  val   = PI / 180.0;
  x_pos = 116 + 103 * cos(degree * val);
  y_pos = abs((int)(-116 + 103 * sin(degree * val)));

  // Display current position
  m_bsp_lcd_draw_image(x_pos, y_pos,
                       ITEMS_TABLE[LCD_DOT].img.x_px + x_pos,
                       ITEMS_TABLE[LCD_DOT].img.y_px + y_pos,
                       ITEMS_TABLE[LCD_DOT].img.data);
}

/* Private function basic --------------------------------------------- */
/**
 * @brief         LCD address set
 *
 * @param[in]     x1       X start
 * @param[in]     y1       Y start
 * @param[in]     x2       X end
 * @param[in]     y2       Y end
 *
 * @attention     None
 *
 * @return        None
 */
static void m_bsp_lcd_address_set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  gc9a01_write_cmd(&m_gc9a01, GC9A01_COLUMN_ADDR_SET);
  gc9a01_write_data_byte(&m_gc9a01, x1 >> 8);
  gc9a01_write_data_byte(&m_gc9a01, x1);        // XSTART
  gc9a01_write_data_byte(&m_gc9a01, x2 >> 8);
  gc9a01_write_data_byte(&m_gc9a01, x2);        // XEND

  gc9a01_write_cmd(&m_gc9a01, GC9A01_ROW_ADDR_SET);
  gc9a01_write_data_byte(&m_gc9a01, y1 >> 8);
  gc9a01_write_data_byte(&m_gc9a01, y1);        // YSTART
  gc9a01_write_data_byte(&m_gc9a01, y2 >> 8);
  gc9a01_write_data_byte(&m_gc9a01, y2);        // YEND
}

/**
 * @brief         LCD write pixel
 *
 * @param[in]     x       X position
 * @param[in]     y       Y position
 * @param[in]     thin    Thin
 * @param[in]     color   Color
 *
 * @attention     None
 *
 * @return        None
 */
static void m_bsp_lcd_write_pixel(uint16_t x, uint16_t y, uint16_t thin, uint16_t color)
{
  if ((x >= LCD_WIDTH) || (y >= LCD_HEIGHT))
    return;

  m_bsp_lcd_address_set(x, y, x + thin - 1, y + thin - 1);

  gc9a01_write_cmd(&m_gc9a01, GC9A01_MEMORY_WRITE);

  for (int i = 0; i < (thin * thin); i++)
  {
    gc9a01_write_data(&m_gc9a01, (uint8_t *)&color, 2);
  }
}

/**
 * @brief         LCD draw image
 *
 * @param[in]     x1       X start
 * @param[in]     y1       Y start
 * @param[in]     x2       X end
 * @param[in]     y2       Y end
 * @param[in]     A        Picture bitmap array
 *
 * @attention     None
 *
 * @return        None
 */
static void m_bsp_lcd_draw_image(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, const short unsigned A[])
{
  int k = 0;
  uint16_t data;

  m_bsp_lcd_address_set(x1, y1, x2 - 1, y2 - 1);
  gc9a01_write_cmd(&m_gc9a01, GC9A01_MEMORY_WRITE);

  for (uint16_t i = x1; i < x2; i++)
  {
    for (uint16_t j = y1; j < y2; j++)
    {
      data = (uint16_t)pgm_read_word(A + k);

      gc9a01_write_data_byte(&m_gc9a01, data >> 8);
      gc9a01_write_data_byte(&m_gc9a01, data);
      k++;
    }
  }
}

/**
 * @brief         LCD fill square
 *
 * @param[in]     x1       X start
 * @param[in]     y1       Y start
 * @param[in]     x2       X end
 * @param[in]     y2       Y end
 * @param[in]     color    Color
 *
 * @attention     None
 *
 * @return        None
 */
static void m_bsp_lcd_fill_square(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
  m_bsp_lcd_address_set(x1, y1, x2 - 1, y2 - 1);
  gc9a01_write_cmd(&m_gc9a01, GC9A01_MEMORY_WRITE);

  for (uint16_t i = x1; i < x1; i++)
  {
    for (uint16_t j = y1; j < y1; j++)
    {
      gc9a01_write_data_byte(&m_gc9a01, color >> 8);
      gc9a01_write_data_byte(&m_gc9a01, color);
    }
  }
}

/**
 * @brief         LCD write char
 *
 * @param[in]     x         X position
 * @param[in]     y         Y position
 * @param[in]     c         Char
 * @param[in]     color     Color
 * @param[in]     bg_color  Background color
 * @param[in]     size      Size
 *
 * @attention     None
 *
 * @return        None
 */
static void m_bsp_lcd_write_char(uint16_t x, uint16_t y, unsigned c, uint16_t color, uint16_t bg, uint8_t size)
{
  if ((x >= LCD_WIDTH)          || // Clip right
      (y >= LCD_HEIGHT)         || // Clip bottom
      ((x + 6 * size - 1) < 0)  || // Clip left
      ((y + 8 * size - 1) < 0))    // Clip top
    return;

  for (int8_t i = 0; i < 6; i++)
  {
    uint8_t line;

    if (i == 5)
      line = 0x00;
    else
      line = pgm_read_byte(font_5x7 + (c * 5) + i);

    for (int8_t j = 0; j < 8; j++)
    {
      if (line & 0x1)
        m_bsp_lcd_write_pixel(x + i * size, y + j * size, size, color);
      else if (bg != color)
        m_bsp_lcd_write_pixel(x + i * size, y + j * size, size, bg);

      line >>= 1;
    }
  }
}

/* End of file -------------------------------------------------------- */
