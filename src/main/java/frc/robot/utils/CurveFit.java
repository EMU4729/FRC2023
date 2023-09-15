package frc.robot.utils;

import edu.wpi.first.math.MathUtil;

public class CurveFit {
  private double inMin;
  private double inMax;
  public double outAbsMin;
  public double outAbsMax;
  public double pow;
  public double throtEffect = 1;

  public static CurveFit throtFromDriveSettings(double[][] settings) {
    return new CurveFit(settings[0][0], settings[0][1], settings[0][2]);
  }

  public static CurveFit steerFromDriveSettings(double[][] settings) {
    return new CurveFit(settings[1][0], settings[1][1], settings[1][2])
        .setThrotEffect(settings[1][3]);
  }

  /** fits curve and adjusts range of input */
  public CurveFit(double outMin, double outMax, double pow) {
    this(-1, 1, outMin, outMax, pow);
  }

  /** fits curve and adjusts range of input */
  public CurveFit(double inMin, double inMax, double outAbsMin, double outAbsMax, double pow) {
    if (pow <= 0) {
      throw new IllegalArgumentException("pow must be a positive non zero double");
    }
    this.inMin = inMin;
    this.inMax = inMax;
    this.outAbsMin = outAbsMin;
    this.outAbsMax = outAbsMax;
    this.pow = pow;
  }

  public CurveFit setThrotEffect(double throtEffect) {
    this.throtEffect = throtEffect;
    return this;
  }

  /**
   * <pre>
   * clamps the input between inMin and inMax
   * adjust the range to -1 -> 1
   * power ofs the input by pow to fit an exponential curve to the input
   * (pow > 1 = slow start quick end, pow = 1 = linear, pow < 1 = quick start slow
   * end)
   * adjusts range again so that the output jumps straight to outAbsMin and curves
   * up to outAbsMax
   * on either side of zero
   * sign of input respected
   * 
   * <pre>
   */
  public double fit(double input) {
    return fit(input, 1);
  }

  /**
   * <pre>
   * clamps the input between inMin and inMax
   * adjust the range to -1 -> 1
   * power ofs the input by pow to fit an exponential curve to the input
   * (pow > 1 = slow start quick end, pow = 1 = linear, pow < 1 = quick start slow
   * end)
   * adjusts range again so that the output jumps straight to outAbsMin and curves
   * up to outAbsMax
   * on either side of zero
   * sign of input respected
   * 
   * <pre>
   */
  public double fit(double input, double maxOutAdjust) {
    if (input == 0) {
      return 0;
    }
    // simple multiply by the input
    // intended for reduction or increase of turn rate based on forward speed
    maxOutAdjust = (1 - throtEffect) + throtEffect * Math.abs(maxOutAdjust);

    input = MathUtil.clamp(input, inMin, inMax); // clamp input within [inMin, inMax]
    input = ((input - inMin) / (inMax - inMin) - 0.5) * 2; // change range to [-1, 1]
    input = Math.copySign(Math.pow(Math.abs(input), pow), input); // fit power of curve to the input
    // change the range to [-outMax, -outMin], [outMin, outMax]
    // with equivelant ranges either side of 0 and a jump to outMin once out of the
    // deadzone
    input = Math.copySign(Math.abs(input) * (-outAbsMin + outAbsMax * maxOutAdjust) + outAbsMin, input);

    /*
     * rough sketch of the result
     *            ___ outAbsMax
     *           |
     *        __/ ___ outAbsMin
     * ______|______
     * __|
     * /
     * |
     * 
     */
    return input;
  }
}
