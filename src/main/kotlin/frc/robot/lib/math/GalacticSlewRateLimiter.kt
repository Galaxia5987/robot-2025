// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.lib.math

import edu.wpi.first.math.MathSharedStore
import edu.wpi.first.math.MathUtil

/**
 * A class that limits the rate of change of an input value. Useful for
 * implementing voltage, setpoint, and/or output ramps. A slew-rate limit is
 * most appropriate when the quantity being controlled is a velocity or a
 * voltage; when controlling a position, consider using a [ ] instead.
 */
class GalacticSlewRateLimiter(
    private var m_positiveRateLimit: Double,
    private var m_negativeRateLimit: Double,
    private var m_prevVal: Double
) {
    private var m_prevTime: Double

    /**
     * Creates a new SlewRateLimiter with the given positive and negative rate
     * limits and initial value.
     *
     * @param positiveRateLimit The rate-of-change limit in the positive
     * direction, in units per second. This is expected to be positive.
     * @param negativeRateLimit The rate-of-change limit in the negative
     * direction, in units per second. This is expected to be negative.
     * @param initialValue The initial value of the input.
     */
    init {
        m_prevTime = MathSharedStore.getTimestamp()
    }

    /**
     * Creates a new SlewRateLimiter with the given positive rate limit and
     * negative rate limit of -rateLimit.
     *
     * @param rateLimit The rate-of-change limit, in units per second.
     */
    constructor(rateLimit: Double) : this(rateLimit, -rateLimit, 0.0)

    /**
     * Filters the input to limit its slew rate.
     *
     * @param input The input value whose slew rate is to be limited.
     * @return The filtered value, which will not change faster than the slew
     * rate.
     */
    fun calculate(input: Double): Double {
        val currentTime = MathSharedStore.getTimestamp()
        val elapsedTime = currentTime - m_prevTime
        m_prevVal +=
            MathUtil.clamp(
                input - m_prevVal,
                m_negativeRateLimit * elapsedTime,
                m_positiveRateLimit * elapsedTime
            )
        m_prevTime = currentTime
        return m_prevVal
    }

    /**
     * Returns the value last calculated by the SlewRateLimiter.
     *
     * @return The last value.
     */
    fun lastValue(): Double {
        return m_prevVal
    }

    /**
     * Resets the slew rate limiter to the specified value; ignores the rate
     * limit when doing so.
     *
     * @param value The value to reset to.
     */
    fun reset(value: Double) {
        m_prevVal = value
        m_prevTime = MathSharedStore.getTimestamp()
    }

    fun withLimit(limit: Double): GalacticSlewRateLimiter {
        m_positiveRateLimit = limit
        m_negativeRateLimit = -limit
        return this
    }
}
