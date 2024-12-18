package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import com.qualcomm.hardware.bosch.BNO055IMU;

public class GyroUtility {

    private BNO055IMU imu;

    GyroUtility() {
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
    } ;
}
