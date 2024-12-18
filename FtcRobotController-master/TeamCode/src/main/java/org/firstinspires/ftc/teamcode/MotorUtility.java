package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MotorUtility {

    private final DcMotor backLeft, backRight, frontRight, frontLeft;
    private final DcMotor rightDeadWheel, leftDeadWheel, centerDeadWheel;

    MotorUtility() {
        this.frontLeft = initializeMotor("frontLeft", DcMotorSimple.Direction.REVERSE);
        this.frontRight = initializeMotor("frontRight", DcMotorSimple.Direction.FORWARD);
        this.backLeft = initializeMotor("backLeft", DcMotorSimple.Direction.REVERSE);
        this.backRight = initializeMotor("backRight", DcMotorSimple.Direction.FORWARD);

        this.leftDeadWheel = initializeMotor("leftDeadWheel", DcMotorSimple.Direction.REVERSE);
        this.rightDeadWheel = initializeMotor("rightDeadWheel", DcMotorSimple.Direction.FORWARD);
        this.centerDeadWheel = initializeMotor("centerDeadWheel", DcMotorSimple.Direction.REVERSE);
    }

    double getPosition(DeadWheel motor) {

        switch (motor) {
            case Center:
                return centerDeadWheel.getCurrentPosition();
            case Left:
                return  leftDeadWheel.getCurrentPosition();
            case Right:
                return  rightDeadWheel.getCurrentPosition();
            default:
                return 0;
        }
    }

    DcMotor initializeMotor(String name, DcMotorSimple.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }


    void resetDeadWheelEncoders() {
        leftDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    void setMotorPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }

}
