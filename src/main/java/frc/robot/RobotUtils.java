package frc.robot;

public final class RobotUtils {
    public static double hypot(double a, double b) {
        return Math.sqrt(a*a + b*b);
    }

    public static double metersToInches(double a) {
        return a*39.3701;
    }

    public static double degreesToRadians(double degrees) {
        return degrees * (Math.PI / 180);
    }
}
