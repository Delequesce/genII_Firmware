#define DT_DRV_COMPAT st_stm32_pwm_custom_fast

#include <errno.h>

#include <soc.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_tim.h>
#include <app/drivers/pwm_stm32_custom_fast.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/dt-bindings/pwm/stm32_pwm.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pwm_stm32_custom_fast, CONFIG_PWM_LOG_LEVEL);

/* L0 series MCUs only have 16-bit timers and don't have below macro defined */
#ifndef IS_TIM_32B_COUNTER_INSTANCE
#define IS_TIM_32B_COUNTER_INSTANCE(INSTANCE) (0)
#endif

/** PWM data. */
struct pwm_stm32_data {
	/** Timer clock (Hz). */
	uint32_t tim_clk;
	/* Reset controller device configuration */
	const struct reset_dt_spec reset;
    void *top_user_data; 
};

/** PWM configuration. */
struct pwm_stm32_config {
	TIM_TypeDef *timer;
	uint32_t prescaler;
	uint32_t countermode;
	struct stm32_pclken pclken;
	const struct pinctrl_dev_config *pcfg;
};

#define TIMER_MAX_CH 4u

/** Channel to LL mapping. */
static const uint32_t ch2ll[TIMER_MAX_CH] = {
	LL_TIM_CHANNEL_CH1, LL_TIM_CHANNEL_CH2,
	LL_TIM_CHANNEL_CH3, LL_TIM_CHANNEL_CH4,
};

/** Channel to compare set function mapping. */
static void (*const set_timer_compare[TIMER_MAX_CH])(TIM_TypeDef *, uint32_t) = {
	LL_TIM_OC_SetCompareCH1, LL_TIM_OC_SetCompareCH2,
	LL_TIM_OC_SetCompareCH3, LL_TIM_OC_SetCompareCH4,
};

/**
 * Obtain LL polarity from PWM flags.
 *
 * @param flags PWM flags.
 *
 * @return LL polarity.
 */
static uint32_t get_polarity(pwm_flags_t flags)
{
	if ((flags & PWM_POLARITY_MASK) == PWM_POLARITY_NORMAL) {
		return LL_TIM_OCPOLARITY_HIGH;
	}

	return LL_TIM_OCPOLARITY_LOW;
}

/**
 * @brief  Check if LL counter mode is center-aligned.
 *
 * @param  ll_countermode LL counter mode.
 *
 * @return `true` when center-aligned, otherwise `false`.
 */
static inline bool is_center_aligned(const uint32_t ll_countermode)
{
	return ((ll_countermode == LL_TIM_COUNTERMODE_CENTER_DOWN) ||
		(ll_countermode == LL_TIM_COUNTERMODE_CENTER_UP) ||
		(ll_countermode == LL_TIM_COUNTERMODE_CENTER_UP_DOWN));
}

/**
 * Obtain timer clock speed.
 *
 * @param pclken  Timer clock control subsystem.
 * @param tim_clk Where computed timer clock will be stored.
 *
 * @return 0 on success, error code otherwise.
 */
static int get_tim_clk(const struct stm32_pclken *pclken, uint32_t *tim_clk)
{
	int r;
	const struct device *clk;
	uint32_t bus_clk, apb_psc;

	clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	r = clock_control_get_rate(clk, (clock_control_subsys_t)pclken,
				   &bus_clk);
	if (r < 0) {
		return r;
	}

#if defined(CONFIG_SOC_SERIES_STM32H7X)
	if (pclken->bus == STM32_CLOCK_BUS_APB1) {
		apb_psc = STM32_D2PPRE1;
	} else {
		apb_psc = STM32_D2PPRE2;
	}
#else
	if (pclken->bus == STM32_CLOCK_BUS_APB1) {
#if defined(CONFIG_SOC_SERIES_STM32MP1X)
		apb_psc = (uint32_t)(READ_BIT(RCC->APB1DIVR, RCC_APB1DIVR_APB1DIV));
#else
		apb_psc = STM32_APB1_PRESCALER;
#endif
	}
#if !defined(CONFIG_SOC_SERIES_STM32C0X) && !defined(CONFIG_SOC_SERIES_STM32F0X) &&                \
	!defined(CONFIG_SOC_SERIES_STM32G0X)
	else {
#if defined(CONFIG_SOC_SERIES_STM32MP1X)
		apb_psc = (uint32_t)(READ_BIT(RCC->APB2DIVR, RCC_APB2DIVR_APB2DIV));
#else
		apb_psc = STM32_APB2_PRESCALER;
#endif
	}
#endif
#endif

#if defined(RCC_DCKCFGR_TIMPRE) || defined(RCC_DCKCFGR1_TIMPRE) || \
	defined(RCC_CFGR_TIMPRE)
	/*
	 * There are certain series (some F4, F7 and H7) that have the TIMPRE
	 * bit to control the clock frequency of all the timers connected to
	 * APB1 and APB2 domains.
	 *
	 * Up to a certain threshold value of APB{1,2} prescaler, timer clock
	 * equals to HCLK. This threshold value depends on TIMPRE setting
	 * (2 if TIMPRE=0, 4 if TIMPRE=1). Above threshold, timer clock is set
	 * to a multiple of the APB domain clock PCLK{1,2} (2 if TIMPRE=0, 4 if
	 * TIMPRE=1).
	 */

	if (LL_RCC_GetTIMPrescaler() == LL_RCC_TIM_PRESCALER_TWICE) {
		/* TIMPRE = 0 */
		if (apb_psc <= 2u) {
			LL_RCC_ClocksTypeDef clocks;

			LL_RCC_GetSystemClocksFreq(&clocks);
			*tim_clk = clocks.HCLK_Frequency;
		} else {
			*tim_clk = bus_clk * 2u;
		}
	} else {
		/* TIMPRE = 1 */
		if (apb_psc <= 4u) {
			LL_RCC_ClocksTypeDef clocks;

			LL_RCC_GetSystemClocksFreq(&clocks);
			*tim_clk = clocks.HCLK_Frequency;
		} else {
			*tim_clk = bus_clk * 4u;
		}
	}
#else
	/*
	 * If the APB prescaler equals 1, the timer clock frequencies
	 * are set to the same frequency as that of the APB domain.
	 * Otherwise, they are set to twice (×2) the frequency of the
	 * APB domain.
	 */
	if (apb_psc == 1u) {
		*tim_clk = bus_clk;
	} else {
		*tim_clk = bus_clk * 2u;
	}
#endif

	return 0;
}



static int pwm_stm32_configure(const struct device *dev, uint32_t channel, pwm_flags_t flags){
	
	uint32_t ll_channel;
	const struct pwm_stm32_config *cfg = dev->config;
	LL_TIM_OC_InitTypeDef oc_init;

	LL_TIM_OC_StructInit(&oc_init);

	ll_channel = ch2ll[channel - 1u];

	oc_init.OCMode = LL_TIM_OCMODE_PWM1;
	//oc_init.OCState = LL_TIM_OCSTATE_ENABLE;
	oc_init.OCPolarity = get_polarity(flags);

	if (LL_TIM_OC_Init(cfg->timer, ll_channel, &oc_init) != SUCCESS) {
		LOG_ERR("Could not initialize timer channel output");
		return -EIO;
	}
	
	LL_TIM_EnableARRPreload(cfg->timer);
	LL_TIM_OC_EnablePreload(cfg->timer, ll_channel);
	
	return 0;
}

static int pwm_stm32_set_period(const struct device *dev, uint32_t period_cycles){
	const struct pwm_stm32_config *cfg = dev->config;
	LL_TIM_SetAutoReload(cfg->timer, period_cycles);
	LL_TIM_GenerateEvent_UPDATE(cfg->timer); // May need to generate an update event for starting counter
	return 0;
}
static int pwm_stm32_set_duty_cycle(const struct device *dev, uint32_t channel, uint32_t pulse_cycles){
	const struct pwm_stm32_config *cfg = dev->config;
	set_timer_compare[channel - 1u](cfg->timer, pulse_cycles);
	return 0;
}

static int pwm_stm32_toggle_channel(const struct device *dev, uint32_t channel, bool en){
	const struct pwm_stm32_config *cfg = dev->config;
	uint32_t ll_channel = ch2ll[channel - 1u];
	if(en){
		
		LL_TIM_CC_EnableChannel(cfg->timer, ll_channel);
		//LL_TIM_GenerateEvent_UPDATE(cfg->timer);
		return 0;
	}
	LL_TIM_CC_DisableChannel(cfg->timer, ll_channel);
	return 0;
}

static const struct custom_pwm_driver_api custom_pwm_stm32_driver_api = {
	.pwm_configure = pwm_stm32_configure,
	.set_period = pwm_stm32_set_period,
	.set_duty_cycle = pwm_stm32_set_duty_cycle,
	.toggle_channel = pwm_stm32_toggle_channel,
};

static int pwm_stm32_init(const struct device *dev)
{
	struct pwm_stm32_data *data = dev->data;
	const struct pwm_stm32_config *cfg = dev->config;

	int r;
	const struct device *clk;
	LL_TIM_InitTypeDef init;

	/* enable clock and store its speed */
	clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	if (!device_is_ready(clk)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	r = clock_control_on(clk, (clock_control_subsys_t)&cfg->pclken);
	if (r < 0) {
		LOG_ERR("Could not initialize clock (%d)", r);
		return r;
	}

	r = get_tim_clk(&cfg->pclken, &data->tim_clk);
	if (r < 0) {
		LOG_ERR("Could not obtain timer clock (%d)", r);
		return r;
	}

	/* Reset timer to default state using RCC */
	(void)reset_line_toggle_dt(&data->reset);

	/* configure pinmux */
	r = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (r < 0) {
		LOG_ERR("PWM pinctrl setup failed (%d)", r);
		return r;
	}

	/* initialize timer */
	LL_TIM_StructInit(&init);

	init.Prescaler = cfg->prescaler;
	init.CounterMode = cfg->countermode;
	init.Autoreload = 0u;
	init.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;

	if (LL_TIM_Init(cfg->timer, &init) != SUCCESS) {
		LOG_ERR("Could not initialize timer");
		return -EIO;
	}

	LL_TIM_EnableCounter(cfg->timer);

	return 0;
}

#define PWM(index) DT_INST_PARENT(index)

#define DT_INST_CLK(index, inst)                                               	\
	{                                                                      		\
		.bus = DT_CLOCKS_CELL(PWM(index), bus),									\
		.enr = DT_CLOCKS_CELL(PWM(index), bits)									\
	}

#define CUSTOM_PWM_DEVICE_INIT(index)                                           \
	static struct pwm_stm32_data pwm_stm32_data_##index = {		       			\
		.reset = RESET_DT_SPEC_GET(PWM(index)),			       					\
	};								       										\
									       										\
	PINCTRL_DT_INST_DEFINE(index);					       						\
                                                                				\
	static const struct pwm_stm32_config pwm_stm32_config_##index = {      		\
		.timer = (TIM_TypeDef *)DT_REG_ADDR(PWM(index)),	       				\
		.prescaler = DT_PROP(PWM(index), st_prescaler),		       				\
		.countermode = DT_PROP(PWM(index), st_countermode),	       				\
		.pclken = DT_INST_CLK(index, timer),                           			\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),		       				\
	};                                                                     		\
                                                                            	\
	DEVICE_DT_INST_DEFINE(index, &pwm_stm32_init, NULL,                    		\
			    &pwm_stm32_data_##index,                           				\
			    &pwm_stm32_config_##index, POST_KERNEL,            				\
			    CONFIG_PWM_INIT_PRIORITY,                          				\
			    &custom_pwm_stm32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CUSTOM_PWM_DEVICE_INIT)
