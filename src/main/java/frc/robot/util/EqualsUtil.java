package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Twist2d;

public class EqualsUtil {
  public static boolean epsilonEquals(double a, double b) {
    return MathUtil.isNear(a, b, 1e-9);
  }

  /** Extension methods for wpi geometry objects */
  public static class GeomExtensions {
    public static boolean equalsZero(Twist2d twist) {
      return EqualsUtil.epsilonEquals(twist.dx, 0.0)
          && EqualsUtil.epsilonEquals(twist.dy, 0.0)
          && EqualsUtil.epsilonEquals(twist.dtheta, 0.0);
    }
  }
}
