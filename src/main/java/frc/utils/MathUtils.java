package frc.utils;

public class MathUtils {
    public static boolean compareDouble(double a, double b) {
        return Math.abs(a - b) < 0.01;
    }
}
