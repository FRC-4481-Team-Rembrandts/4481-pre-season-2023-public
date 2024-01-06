package frc.team4481.lib.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A class that limits the rate of change of an input value.
 * It is specifically made to limit the rate of change of the swerve module angle (azimuth).
 * To achieve this, it will take into account angle wrapping between -pi and pi.
 * Additionally, the rate limit can be dynamically adjusted to allow for higher limiting at higher speeds
 */
public class SwerveSlewRateLimiter {
    private double m_positiveRateLimit;
    private double m_negativeRateLimit;
    private double m_prevVal;
    private double m_prevTime;

    /**
     * Creates a new {@code SwerveSlewRateLimiter} with the given positive and negative rate limits and initial
     * value.
     *
     * @param positiveRateLimit The rate-of-change limit in the positive direction, in units per
     *     second. This is expected to be positive.
     * @param negativeRateLimit The rate-of-change limit in the negative direction, in units per
     *     second. This is expected to be negative.
     * @param initialValue The initial value of the input.
     */
    public SwerveSlewRateLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue) {
        m_positiveRateLimit = positiveRateLimit;
        m_negativeRateLimit = negativeRateLimit;
        m_prevVal = initialValue;
        m_prevTime = WPIUtilJNI.now() * 1e-6;
    }
    /**
     * Creates a new {@code SwerveSlewRateLimiter} with the given positive rate limit and negative rate limit of
     * -rateLimit.
     *
     * @param rateLimit The rate-of-change limit, in units per second.
     */
    public SwerveSlewRateLimiter(double rateLimit) {
        this(rateLimit, -rateLimit, 0);
    }

    /**
     * Filters the input to limit its slew rate.
     *
     * @param input The input value whose slew rate is to be limited.
     * @return The filtered value, which will not change faster than the slew rate.
     */
    public double calculate(double input) {
        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime - m_prevTime;
        SmartDashboard.putNumber("SlewRate/current slew rate time", currentTime);

        double delta = MathUtil.angleModulus(input - m_prevVal);

        SmartDashboard.putNumber("SlewRate/delta", delta );
        SmartDashboard.putNumber("SlewRate/input", input);


        //If the angle is too large, the robot is changing direction and the slew rate should not act
        if (Math.abs(delta) > 0.7 * Math.PI){
            m_prevVal = input;
            return m_prevVal;
        }

        m_prevVal +=
                MathUtil.clamp(
                        delta,
                        m_negativeRateLimit * elapsedTime,
                        m_positiveRateLimit * elapsedTime);
        m_prevVal = MathUtil.angleModulus(m_prevVal);
        SmartDashboard.putNumber("SlewRate/prev_val", m_prevVal);
        m_prevTime = currentTime;
        return m_prevVal;
    }

    /**
     * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
     *
     * @param value The value to reset to.
     */
    public void reset(double value) {
        m_prevVal = value;
        m_prevTime = WPIUtilJNI.now() * 1e-6;
    }

    /**
     * Change the rate limit of the skew rate limiter
     *
     * @param limit The rate-of-change limit, in units per
     *              second. This is expected to be positive.
     */
    public void setRateLimit(double limit){
        m_positiveRateLimit = limit;
        m_negativeRateLimit = -limit;
    }

    /**
     * Get the skew rate limit in the positive direction
     * @return positiveRateLimit
     */
    public double getSlewLimit(){
        return m_positiveRateLimit;
    }
}
