
/**
 * @file       bsp.c
 * @copyright  Copyright (C) 2020 ThuanLe. All rights reserved.
 * @license    This project is released under the ThuanLe License.
 * @version    1.0.0
 * @date       2021-01-23
 * @author     Thuan Le
 * @brief      Board Support Package (BSP)
 * 
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "bsp.h"

/* Private defines ---------------------------------------------------- */
#define TWI_INSTANCE         0
#define SPI_INSTANCE         1

/* Private enumerate/structure ---------------------------------------- */
static nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE);
static nrf_drv_spi_t m_spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
static void m_bsp_i2c_init(void);
static void m_bsp_spi_init(void);
static void m_bsp_gpio_init(void);

/* Function definitions ----------------------------------------------- */
void bsp_hw_init(void)
{
  m_bsp_i2c_init();
  m_bsp_spi_init();
  m_bsp_gpio_init();
}

int bsp_i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t *p_data, uint32_t len)
{
  uint8_t buffer[10];

  memcpy(buffer, &reg_addr, 1);
  memcpy(buffer + 1, p_data, len);

  return nrf_drv_twi_tx(&m_twi, slave_addr, buffer, len + 1, false);
}

int bsp_i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t *p_data, uint32_t len)
{
  nrf_drv_twi_tx(&m_twi, slave_addr, (uint8_t *)&reg_addr, 1, true);

  return nrf_drv_twi_rx(&m_twi, slave_addr, p_data, len);
}

uint32_t bsp_time_now(void)
{
  return app_timer_cnt_get();
}

base_status_t bsp_spi_write(uint8_t *tx_data, uint16_t len)
{
  nrf_drv_spi_transfer(&m_spi, tx_data, len, NULL, 0);
  
  return BS_OK;
}

void bsp_delay_ms(uint32_t ms)
{
  nrf_delay_ms(ms);
}

void bsp_gpio_write(uint8_t pin , uint8_t state)
{
  if (0 == state)
    nrfx_gpiote_out_clear(pin);
  else
    nrfx_gpiote_out_set(pin);
}

/* Private function definitions ---------------------------------------- */
/**
 * @brief         I2C init
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void m_bsp_i2c_init(void)
{
  ret_code_t err_code;

  const nrf_drv_twi_config_t twi_config =
  {
    .scl                = IO_I2C0_SCL,
    .sda                = IO_I2C0_SDA,
    .frequency          = NRF_DRV_TWI_FREQ_400K,
    .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
    .clear_bus_init     = false
  };

  err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
  APP_ERROR_CHECK(err_code);

  nrf_drv_twi_enable(&m_twi);
}

/**
 * @brief         Gpio init
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void m_bsp_gpio_init(void)
{
  ret_code_t err_code;

  err_code = nrf_drv_gpiote_init();
  APP_ERROR_CHECK(err_code);

  // OB1203 pin config
  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
  in_config.pull = NRF_GPIO_PIN_PULLUP;

  err_code = nrf_drv_gpiote_in_init(IO_OB1203_INTR, &in_config, bsp_intr_pin_handler);
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_event_enable(IO_OB1203_INTR, true);

  // Button pin config
  nrf_drv_gpiote_in_config_t btn_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
  btn_config.pull = NRF_GPIO_PIN_PULLUP;

  err_code = nrf_drv_gpiote_in_init(IO_BTN, &btn_config, bsp_intr_handler);
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_event_enable(IO_BTN, true);

  // LCD pin config
  nrf_drv_gpiote_out_config_t out_config = NRFX_GPIOTE_CONFIG_OUT_TASK_TOGGLE(true);
  err_code = nrf_drv_gpiote_out_init(IO_LCD_BACKLIGHT, &out_config);
  err_code = nrf_drv_gpiote_out_init(IO_LCD_POWER, &out_config);
  err_code = nrf_drv_gpiote_out_init(IO_LCD_RST, &out_config);
  err_code = nrf_drv_gpiote_out_init(IO_LCD_DC, &out_config);
  err_code = nrf_drv_gpiote_out_init(IO_LCD_CS, &out_config);
  APP_ERROR_CHECK(err_code);

  nrfx_gpiote_out_set(IO_LCD_RST);
  nrfx_gpiote_out_set(IO_LCD_POWER);
  nrfx_gpiote_out_set(IO_LCD_BACKLIGHT);
}

static void m_bsp_spi_init(void)
{
  ret_code_t err_code;

  nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

  spi_config.ss_pin    = IO_LCD_CS;
  spi_config.mosi_pin  = IO_LCD_SDA;
  spi_config.sck_pin   = IO_LCD_SCL;
  spi_config.frequency = NRF_DRV_SPI_FREQ_8M;
  spi_config.mode      = NRF_DRV_SPI_MODE_0;
  spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;

  err_code = nrf_drv_spi_init(&m_spi, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(err_code);
}

/* End of file -------------------------------------------------------- */
