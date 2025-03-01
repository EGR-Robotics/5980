package frc.robot;

public class Helpers {
    public static double percentError(double v1, double v2) {
        return Math.abs(
                (v2 - v1) / (v1));
    }
}
