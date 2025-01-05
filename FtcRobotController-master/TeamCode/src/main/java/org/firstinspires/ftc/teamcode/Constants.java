package org.firstinspires.ftc.teamcode;

public class Constants {

    public static final double DEAD_WHEEL_DIAMETER = 60;
    public static final double WHEEL_BASE_WIDTH = 152;

    private static final double DEAD_WHEEL_TICKS_PER_ROTATION = 8192;
    private static final double DEAD_WHEEL_CIRCUMFERENCE = (Math.PI * DEAD_WHEEL_DIAMETER);
    private static final double DEAD_WHEEL_CIRCUMFERENCE_INCH = (Math.PI * DEAD_WHEEL_DIAMETER) / 25.4;

    public static final double DEAD_WHEEL_TICKS_PER_INCH = DEAD_WHEEL_TICKS_PER_ROTATION / DEAD_WHEEL_CIRCUMFERENCE_INCH;
    public static final double DEAD_WHEEL_TICKS_PER_MM = DEAD_WHEEL_TICKS_PER_ROTATION / DEAD_WHEEL_CIRCUMFERENCE;
    public static final double DEAD_WHEEL_MM_PER_TICK = DEAD_WHEEL_CIRCUMFERENCE/DEAD_WHEEL_TICKS_PER_ROTATION;

    public static final double MINIMUM_TIME_IN_SECONDS = 1e-3;
    public static final double CONVERT_TIME_TO_SECONDS = 1000;
    public static final double MINIMUM_DISTANCE = 500;
    public static final double TURN_TOLERANCE = 2;

    public static final double CONVERT_METERS_TO_INCHES = 39.3701;

    public static final double APRIL_TAG_WEIGHT = 0.8;

//    public static final double TURN_SPEED = 0.2;

    public static final double MAX_KP = 0.80;

    public static final double MINIMUM_POWER_OUTPUT_DRIVE = 0.20;
    public static final double MINIMUM_POWER_OUTPUT_TURN = 0.20;

    public static final double MAX_INTEGRAL_XY = 20000;
    public static final double MAX_INTEGRAL_TURN = 100;

    public static final long MAX_AGE_NANOSECONDS = 250000000;

    private Constants() {
        throw new AssertionError("Cannot instantiate constants class");
    }
}

