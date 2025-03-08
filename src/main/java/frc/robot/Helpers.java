package frc.robot;

public class Helpers {
    /**
     * Calculate percent error from current value to target value
     * 
     * @param v1 Original value
     * @param v2 Current value
     */
    public static double percentError(double v1, double v2) {
        return Math.abs(
                (v2 - v1) / (v1));
    }
}
