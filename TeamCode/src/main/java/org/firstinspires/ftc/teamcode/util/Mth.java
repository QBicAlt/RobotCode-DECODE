package org.firstinspires.ftc.teamcode.util;

public class Mth {
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
