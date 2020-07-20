/* rtc_timer.c */
#include <peripheral_clk_config.h>
#include <hpl_pm_base.h>
#include <hpl_gclk_base.h>
#include <hpl_tc_base.h>
// #include <hpl_rtc_base.h>
#include <hal_timer.h>
#include "rtc_timer.h"

struct timer_descriptor       TIMER_0;

#if 0
// Leave TIMER_0_init to driver_temp.c
/**
 * \brief Timer initialization function
 *
 * Enables Timer peripheral, clocks and initializes Timer driver
 */
static void TIMER_0_init(void)
{
	hri_mclk_set_APBCMASK_TC0_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, TC0_GCLK_ID, CONF_GCLK_TC0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

	timer_init(&TIMER_0, TC0, _tc_get_timer());
}
#endif

uint32_t rtc_current_count;
#ifdef RTC_USE_MAX_DURATION_REFERENCE
uint16_t rtc_max_state_duration_ref_value;
#endif
static bool rtc_current_count_set;

/**
 * \brief Start timer
 * This is a rewrite of timer_start() that does not enable interrupts and
 * disables the compare function that resets the counter automatically.
 */
int32_t uDACS_timer_start(struct timer_descriptor *const descr)
{
#ifdef USING_RTC
  struct _timer_device *dev;
  uint16_t register_value;
	ASSERT(descr);
  dev = &descr->device;
	if (_timer_is_started(&descr->device)) {
		return ERR_DENIED;
	}
  // Fix settings that timer_init() applied
  // Clear CTRL.MATCHLR bit
  // Disable the CMP0 interrupt

  register_value = hri_rtcmode0_read_CTRL_reg(dev->hw);
  register_value &= ~RTC_MODE0_CTRL_MATCHCLR;
  hri_rtcmode0_write_CTRL_reg(dev->hw, register_value);
  hri_rtcmode0_clear_INTEN_CMP0_bit(dev->hw);


	// _timer_start(&descr->device); copied here and modified to leave IRQ disabled:
	ASSERT(dev && dev->hw);

	// NVIC_EnableIRQ(RTC_IRQn);
	hri_rtcmode0_write_COUNT_COUNT_bf(dev->hw, 0);
	hri_rtcmode0_wait_for_sync(dev->hw);
	hri_rtcmode0_set_CTRL_ENABLE_bit(dev->hw);

	return ERR_NONE;
#else
  return timer_start(descr);
#endif
}

static subbus_cache_word_t rtc_cache[RTC_HIGH_ADDR-RTC_BASE_ADDR+1] = {
  { 0, 0, true, false, false, false, false },
  { 0, 0, true, false, false, false, false },
  { 0, 0, true, false, false, false, false },
  { 0, 0, true, false, false, false, true }
  #ifdef RTC_USE_MAX_DURATION_REFERENCE
  , { 0, 0, true, false, false, false, false }
  #endif
};

static void rtc_reset() {
  // TIMER_0_init(); Done in driver_temp
  uDACS_timer_start(&TIMER_0);
}

/**
 * Reads the RTC counter and saves it to offset 0 and 1 and also saves
 * it in rtc_current_count;
 * Calculates the elapsed time since the previous poll and stores the
 * difference in offset 2.
 * Checks to see if this is a new maximum, and if so, stores it in offset 3.
 */
static void rtc_poll() {
  uint32_t cur_time = hri_rtcmode0_read_COUNT_reg(RTC);
  sb_cache_update32(rtc_cache,RTC_ELAPSED_OFFSET,&cur_time);
  if (rtc_current_count_set) {
    uint16_t dt = cur_time - rtc_current_count;
    sb_cache_update(rtc_cache,RTC_CUR_STATE_DURATION_OFFSET,dt);
    if (dt > rtc_cache[RTC_MAX_STATE_DURATION_OFFSET].cache) {
      sb_cache_update(rtc_cache,RTC_MAX_STATE_DURATION_OFFSET,dt);
      #ifdef RTC_USE_MAX_DURATION_REFERENCE
      sb_cache_update(rtc_cache,RTC_MAX_DURATION_REF_OFFSET,rtc_max_state_duration_ref_value);
      #endif
    }
  } else {
    rtc_current_count_set = true;
  }
  rtc_current_count = cur_time;
}

static void rtc_action(uint16_t offset) {
  if (offset == RTC_MAX_STATE_DURATION_OFFSET)
    sb_cache_update(rtc_cache,RTC_MAX_STATE_DURATION_OFFSET,0);
}

subbus_driver_t sb_rtc = {
  RTC_BASE_ADDR, RTC_HIGH_ADDR, // address range
  rtc_cache,
  rtc_reset,
  rtc_poll,
  rtc_action,
  false
};
