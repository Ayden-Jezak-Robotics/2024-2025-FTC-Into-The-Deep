package org.firstinspires.ftc.teamcode;

public class Constants {

    private static final double DEAD_WHEEL_DIAMETER = 60;

    private static final double TICKS_PER_ROTATION = 8192;
    private static final double WHEEL_CIRCUMFERENCE = (Math.PI * DEAD_WHEEL_DIAMETER) / 25.4;
    private static final double TICKS_PER_INCH = TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE;

    static double ticksPerInch() {
        return TICKS_PER_INCH;
    }
}
