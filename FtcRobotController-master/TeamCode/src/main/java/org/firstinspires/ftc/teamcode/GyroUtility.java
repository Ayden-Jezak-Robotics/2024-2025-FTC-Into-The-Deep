package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class GyroUtility {

    private BNO055IMU imu;

    GyroUtility(HardwareMap hardwareMap, Telemetry telemetry) {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }

    boolean calibrateIMU() {
        return imu.isGyroCalibrated();
    }

    double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public double normalizeAngle(double angle) {
        double normalizedAngle;
        if (angle > 180) {
            normalizedAngle = angle - 360;
        } else if (angle < -180) {
            normalizedAngle = angle + 360;
        } else {
            normalizedAngle = angle;
        }
        return normalizedAngle;
    }
}
