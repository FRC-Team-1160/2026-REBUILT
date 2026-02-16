package frc.robot;

public final class RobotUtils {
    public static double hypot(double a, double b) {
        return Math.sqrt(a*a + b*b);
    }

    public static double getDistance(double x, double y, double z) {
        return  Math.sqrt(x*x + y*y + z*z);
    }

    public static double metersToInches(double x) {
        return x * 39.3701;
    }
}
