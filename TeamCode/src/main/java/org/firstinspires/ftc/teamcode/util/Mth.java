package org.firstinspires.ftc.teamcode.util;

public class Mth {
    public static double clamp(double var, double min, double max) {
        if (var < min) {
            return min;
        }

        return Math.min(var, max);
    }

    public static int clamp(int var, int min, int max) {
        return (int) clamp(var, min, (double) max);
    }
}