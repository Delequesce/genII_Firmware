/*
 * Copyright (c) 2016 Intel Corporation.
 * Copyright (c) 2020-2021 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Custom PWM Driver APIs
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_PWM_CUSTOM_FAST_H_
#define ZEPHYR_INCLUDE_DRIVERS_PWM_CUSTOM_FAST_H_

/**
 * @brief PWM Interface with fast updates
 * @defgroup pwm_interface PWM Interface
 * @since 1.0
 * @version 1.0.0
 * @ingroup io_interfaces
 * @{
 */

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys_clock.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/toolchain.h>

#include <zephyr/dt-bindings/pwm/pwm.h>

#ifndef ZEPHYR_INCLUDE_DRIVERS_PWM_H_

/**
 * @name PWM capture configuration flags
 * @anchor PWM_CAPTURE_FLAGS
 * @{
 */

/** @cond INTERNAL_HIDDEN */
/* Bit 0 is used for PWM_POLARITY_NORMAL/PWM_POLARITY_INVERTED */
#define PWM_CAPTURE_TYPE_SHIFT		1U
#define PWM_CAPTURE_TYPE_MASK		(3U << PWM_CAPTURE_TYPE_SHIFT)
#define PWM_CAPTURE_MODE_SHIFT		3U
#define PWM_CAPTURE_MODE_MASK		(1U << PWM_CAPTURE_MODE_SHIFT)
/** @endcond */

/** PWM pin capture captures period. */
#define PWM_CAPTURE_TYPE_PERIOD		(1U << PWM_CAPTURE_TYPE_SHIFT)

/** PWM pin capture captures pulse width. */
#define PWM_CAPTURE_TYPE_PULSE		(2U << PWM_CAPTURE_TYPE_SHIFT)

/** PWM pin capture captures both period and pulse width. */
#define PWM_CAPTURE_TYPE_BOTH		(PWM_CAPTURE_TYPE_PERIOD | \
					 PWM_CAPTURE_TYPE_PULSE)

/** PWM pin capture captures a single period/pulse width. */
#define PWM_CAPTURE_MODE_SINGLE		(0U << PWM_CAPTURE_MODE_SHIFT)

/** PWM pin capture captures period/pulse width continuously. */
#define PWM_CAPTURE_MODE_CONTINUOUS	(1U << PWM_CAPTURE_MODE_SHIFT)

/** @} */

/**
 * @brief Provides a type to hold PWM configuration flags.
 *
 * The lower 8 bits are used for standard flags.
 * The upper 8 bits are reserved for SoC specific flags.
 *
 * @see @ref PWM_CAPTURE_FLAGS.
 */

typedef uint16_t pwm_flags_t;

/**
 * @brief Container for PWM information specified in devicetree.
 *
 * This type contains a pointer to a PWM device, channel number (controlled by
 * the PWM device), the PWM signal period in nanoseconds and the flags
 * applicable to the channel. Note that not all PWM drivers support flags. In
 * such case, flags will be set to 0.
 *
 * @see PWM_DT_SPEC_GET_BY_NAME
 * @see PWM_DT_SPEC_GET_BY_NAME_OR
 * @see PWM_DT_SPEC_GET_BY_IDX
 * @see PWM_DT_SPEC_GET_BY_IDX_OR
 * @see PWM_DT_SPEC_GET
 * @see PWM_DT_SPEC_GET_OR
 */
struct pwm_dt_spec {
	/** PWM device instance. */
	const struct device *dev;
	/** Channel number. */
	uint32_t channel;
	/** Period in nanoseconds. */
	uint32_t period;
	/** Flags. */
	pwm_flags_t flags;
};

/**
 * @brief Static initializer for a struct pwm_dt_spec
 *
 * This returns a static initializer for a struct pwm_dt_spec given a devicetree
 * node identifier and an index.
 *
 * Example devicetree fragment:
 *
 * @code{.dts}
 *    n: node {
 *        pwms = <&pwm1 1 1000 PWM_POLARITY_NORMAL>,
 *               <&pwm2 3 2000 PWM_POLARITY_INVERTED>;
 *        pwm-names = "alpha", "beta";
 *    };
 * @endcode
 *
 * Example usage:
 *
 * @code{.c}
 *    const struct pwm_dt_spec spec =
 *        PWM_DT_SPEC_GET_BY_NAME(DT_NODELABEL(n), alpha);
 *
 *    // Initializes 'spec' to:
 *    // {
 *    //         .dev = DEVICE_DT_GET(DT_NODELABEL(pwm1)),
 *    //         .channel = 1,
 *    //         .period = 1000,
 *    //         .flags = PWM_POLARITY_NORMAL,
 *    // }
 * @endcode
 *
 * The device (dev) must still be checked for readiness, e.g. using
 * device_is_ready(). It is an error to use this macro unless the node exists,
 * has the 'pwms' property, and that 'pwms' property specifies a PWM controller,
 * a channel, a period in nanoseconds and optionally flags.
 *
 * @param node_id Devicetree node identifier.
 * @param name Lowercase-and-underscores name of a pwms element as defined by
 *             the node's pwm-names property.
 *
 * @return Static initializer for a struct pwm_dt_spec for the property.
 *
 * @see PWM_DT_SPEC_INST_GET_BY_NAME
 */
#define PWM_DT_SPEC_GET_BY_NAME(node_id, name)				       \
	{								       \
		.dev = DEVICE_DT_GET(DT_PWMS_CTLR_BY_NAME(node_id, name)),     \
		.channel = DT_PWMS_CHANNEL_BY_NAME(node_id, name),	       \
		.period = DT_PWMS_PERIOD_BY_NAME(node_id, name),	       \
		.flags = DT_PWMS_FLAGS_BY_NAME(node_id, name),		       \
	}

/**
 * @brief Static initializer for a struct pwm_dt_spec from a DT_DRV_COMPAT
 *        instance.
 *
 * @param inst DT_DRV_COMPAT instance number
 * @param name Lowercase-and-underscores name of a pwms element as defined by
 *             the node's pwm-names property.
 *
 * @return Static initializer for a struct pwm_dt_spec for the property.
 *
 * @see PWM_DT_SPEC_GET_BY_NAME
 */
#define PWM_DT_SPEC_INST_GET_BY_NAME(inst, name)			       \
	PWM_DT_SPEC_GET_BY_NAME(DT_DRV_INST(inst), name)

/**
 * @brief Like PWM_DT_SPEC_GET_BY_NAME(), with a fallback to a default value.
 *
 * If the devicetree node identifier 'node_id' refers to a node with a property
 * 'pwms', this expands to <tt>PWM_DT_SPEC_GET_BY_NAME(node_id, name)</tt>. The
 * @p default_value parameter is not expanded in this case. Otherwise, this
 * expands to @p default_value.
 *
 * @param node_id Devicetree node identifier.
 * @param name Lowercase-and-underscores name of a pwms element as defined by
 *             the node's pwm-names property
 * @param default_value Fallback value to expand to.
 *
 * @return Static initializer for a struct pwm_dt_spec for the property,
 *         or @p default_value if the node or property do not exist.
 *
 * @see PWM_DT_SPEC_INST_GET_BY_NAME_OR
 */
#define PWM_DT_SPEC_GET_BY_NAME_OR(node_id, name, default_value)	       \
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, pwms),			       \
		    (PWM_DT_SPEC_GET_BY_NAME(node_id, name)),		       \
		    (default_value))

/**
 * @brief Like PWM_DT_SPEC_INST_GET_BY_NAME(), with a fallback to a default
 *        value.
 *
 * @param inst DT_DRV_COMPAT instance number
 * @param name Lowercase-and-underscores name of a pwms element as defined by
 *             the node's pwm-names property.
 * @param default_value Fallback value to expand to.
 *
 * @return Static initializer for a struct pwm_dt_spec for the property,
 *         or @p default_value if the node or property do not exist.
 *
 * @see PWM_DT_SPEC_GET_BY_NAME_OR
 */
#define PWM_DT_SPEC_INST_GET_BY_NAME_OR(inst, name, default_value)	       \
	PWM_DT_SPEC_GET_BY_NAME_OR(DT_DRV_INST(inst), name, default_value)

/**
 * @brief Static initializer for a struct pwm_dt_spec
 *
 * This returns a static initializer for a struct pwm_dt_spec given a devicetree
 * node identifier and an index.
 *
 * Example devicetree fragment:
 *
 * @code{.dts}
 *    n: node {
 *        pwms = <&pwm1 1 1000 PWM_POLARITY_NORMAL>,
 *               <&pwm2 3 2000 PWM_POLARITY_INVERTED>;
 *    };
 * @endcode
 *
 * Example usage:
 *
 * @code{.c}
 *    const struct pwm_dt_spec spec =
 *        PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(n), 1);
 *
 *    // Initializes 'spec' to:
 *    // {
 *    //         .dev = DEVICE_DT_GET(DT_NODELABEL(pwm2)),
 *    //         .channel = 3,
 *    //         .period = 2000,
 *    //         .flags = PWM_POLARITY_INVERTED,
 *    // }
 * @endcode
 *
 * The device (dev) must still be checked for readiness, e.g. using
 * device_is_ready(). It is an error to use this macro unless the node exists,
 * has the 'pwms' property, and that 'pwms' property specifies a PWM controller,
 * a channel, a period in nanoseconds and optionally flags.
 *
 * @param node_id Devicetree node identifier.
 * @param idx Logical index into 'pwms' property.
 *
 * @return Static initializer for a struct pwm_dt_spec for the property.
 *
 * @see PWM_DT_SPEC_INST_GET_BY_IDX
 */
#define PWM_DT_SPEC_GET_BY_IDX(node_id, idx)				       \
	{								       \
		.dev = DEVICE_DT_GET(DT_PWMS_CTLR_BY_IDX(node_id, idx)),       \
		.channel = DT_PWMS_CHANNEL_BY_IDX(node_id, idx),	       \
		.period = DT_PWMS_PERIOD_BY_IDX(node_id, idx),		       \
		.flags = DT_PWMS_FLAGS_BY_IDX(node_id, idx),		       \
	}

/**
 * @brief Static initializer for a struct pwm_dt_spec from a DT_DRV_COMPAT
 *        instance.
 *
 * @param inst DT_DRV_COMPAT instance number
 * @param idx Logical index into 'pwms' property.
 *
 * @return Static initializer for a struct pwm_dt_spec for the property.
 *
 * @see PWM_DT_SPEC_GET_BY_IDX
 */
#define PWM_DT_SPEC_INST_GET_BY_IDX(inst, idx)				       \
	PWM_DT_SPEC_GET_BY_IDX(DT_DRV_INST(inst), idx)

/**
 * @brief Like PWM_DT_SPEC_GET_BY_IDX(), with a fallback to a default value.
 *
 * If the devicetree node identifier 'node_id' refers to a node with a property
 * 'pwms', this expands to <tt>PWM_DT_SPEC_GET_BY_IDX(node_id, idx)</tt>. The
 * @p default_value parameter is not expanded in this case. Otherwise, this
 * expands to @p default_value.
 *
 * @param node_id Devicetree node identifier.
 * @param idx Logical index into 'pwms' property.
 * @param default_value Fallback value to expand to.
 *
 * @return Static initializer for a struct pwm_dt_spec for the property,
 *         or @p default_value if the node or property do not exist.
 *
 * @see PWM_DT_SPEC_INST_GET_BY_IDX_OR
 */
#define PWM_DT_SPEC_GET_BY_IDX_OR(node_id, idx, default_value)		       \
	COND_CODE_1(DT_NODE_HAS_PROP(node_id, pwms),			       \
		    (PWM_DT_SPEC_GET_BY_IDX(node_id, idx)),		       \
		    (default_value))

/**
 * @brief Like PWM_DT_SPEC_INST_GET_BY_IDX(), with a fallback to a default
 *        value.
 *
 * @param inst DT_DRV_COMPAT instance number
 * @param idx Logical index into 'pwms' property.
 * @param default_value Fallback value to expand to.
 *
 * @return Static initializer for a struct pwm_dt_spec for the property,
 *         or @p default_value if the node or property do not exist.
 *
 * @see PWM_DT_SPEC_GET_BY_IDX_OR
 */
#define PWM_DT_SPEC_INST_GET_BY_IDX_OR(inst, idx, default_value)	       \
	PWM_DT_SPEC_GET_BY_IDX_OR(DT_DRV_INST(inst), idx, default_value)

/**
 * @brief Equivalent to <tt>PWM_DT_SPEC_GET_BY_IDX(node_id, 0)</tt>.
 *
 * @param node_id Devicetree node identifier.
 *
 * @return Static initializer for a struct pwm_dt_spec for the property.
 *
 * @see PWM_DT_SPEC_GET_BY_IDX
 * @see PWM_DT_SPEC_INST_GET
 */
#define PWM_DT_SPEC_GET(node_id) PWM_DT_SPEC_GET_BY_IDX(node_id, 0)

/**
 * @brief Equivalent to <tt>PWM_DT_SPEC_INST_GET_BY_IDX(inst, 0)</tt>.
 *
 * @param inst DT_DRV_COMPAT instance number
 *
 * @return Static initializer for a struct pwm_dt_spec for the property.
 *
 * @see PWM_DT_SPEC_INST_GET_BY_IDX
 * @see PWM_DT_SPEC_GET
 */
#define PWM_DT_SPEC_INST_GET(inst) PWM_DT_SPEC_GET(DT_DRV_INST(inst))

/**
 * @brief Equivalent to
 *        <tt>PWM_DT_SPEC_GET_BY_IDX_OR(node_id, 0, default_value)</tt>.
 *
 * @param node_id Devicetree node identifier.
 * @param default_value Fallback value to expand to.
 *
 * @return Static initializer for a struct pwm_dt_spec for the property.
 *
 * @see PWM_DT_SPEC_GET_BY_IDX_OR
 * @see PWM_DT_SPEC_INST_GET_OR
 */
#define PWM_DT_SPEC_GET_OR(node_id, default_value)			       \
	PWM_DT_SPEC_GET_BY_IDX_OR(node_id, 0, default_value)

/**
 * @brief Equivalent to
 *        <tt>PWM_DT_SPEC_INST_GET_BY_IDX_OR(inst, 0, default_value)</tt>.
 *
 * @param inst DT_DRV_COMPAT instance number
 * @param default_value Fallback value to expand to.
 *
 * @return Static initializer for a struct pwm_dt_spec for the property.
 *
 * @see PWM_DT_SPEC_INST_GET_BY_IDX_OR
 * @see PWM_DT_SPEC_GET_OR
 */
#define PWM_DT_SPEC_INST_GET_OR(inst, default_value)			       \
	PWM_DT_SPEC_GET_OR(DT_DRV_INST(inst), default_value)

#endif /*if pwm is not included*/

/** @cond INTERNAL_HIDDEN */
/**
 * @brief PWM driver API call to configure PWM pin period and pulse width.
 * @see pwm_set_cycles() for argument description.
 */
typedef int (*pwm_configure_t)(const struct device *dev, uint32_t channel, pwm_flags_t flags);
typedef int (*pwm_set_period_t)(const struct device *dev, uint32_t period_cycles);
typedef int (*pwm_set_duty_cycle_t)(const struct device *dev, uint32_t channel, uint32_t pulse_cycles);
typedef int (*pwm_toggle_channel_t)(const struct device *dev, uint32_t channel, bool en);	

/** @brief PWM driver API definition. */
__subsystem struct custom_pwm_driver_api {
	pwm_configure_t pwm_configure;
	pwm_set_period_t set_period;
	pwm_set_duty_cycle_t set_duty_cycle;
	pwm_toggle_channel_t toggle_channel;
};
/** @endcond */

/**
 * @brief Set the period and pulse width for a single PWM output.
 * 
 * @param[in] dev PWM device instance.
 * @param channel PWM channel.
 * @param flags Flags for pin configuration.
 *
 * @retval 0 If successful.
 * @retval -errno Negative errno code on failure.
 */
static inline int fast_pwm_configure(const struct device *dev, uint32_t channel, pwm_flags_t flags){

	const struct custom_pwm_driver_api *api = (const struct custom_pwm_driver_api *)dev->api;

	return api->pwm_configure(dev, channel, flags);
}
static inline int fast_pwm_set_period(const struct device *dev, uint32_t period_cycles){

	const struct custom_pwm_driver_api *api = (const struct custom_pwm_driver_api *)dev->api;

	return api->set_period(dev, period_cycles);
}
static inline int fast_pwm_set_duty_cycles(const struct device *dev, uint32_t channel, uint32_t pulse_cycles){

	const struct custom_pwm_driver_api *api = (const struct custom_pwm_driver_api *)dev->api;

	return api->set_duty_cycle(dev, channel, pulse_cycles);
}
static inline int fast_pwm_toggle_channel(const struct device *dev, uint32_t channel, bool en){

	const struct custom_pwm_driver_api *api = (const struct custom_pwm_driver_api *)dev->api;

	return api->toggle_channel(dev, channel, en);
}
 
#endif