package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "AutoOpTest", group = "Draft")
public class AutoOpTest extends LinearOpMode {
    // Declare motors and sensors

    private MotorUtility motors;
    private GyroUtility gyros;
    private VisionUtility myAprilTagProcessor;


    // Declare PID controllers
    private SanjuPIDController xPIDController, yPIDController, leftPIDController, rightPIDController, centerPIDController, turnPIDController;

    private Point currentPosition = new Point(0, 0);
    private final ElapsedTime timer = new ElapsedTime();
    private static final double STRAIGHT_TOLERANCE = 50;
    private static final double TURN_TOLERANCE = 0.3;
    private static final double TURN_SPEED = 0.2;

    private double deltaTime;

    @Override
    public void runOpMode() throws InterruptedException {

        initializePIDControllers();

        // Declare utilities
        currentPosition = myAprilTagProcessor.getPosition(currentPosition);

        while (!isStopRequested() && !gyros.calibrateIMU()) {
            sleep(100); // Short delay to avoid overloading the CPU
            idle();    // Let other processes run
        }

        waitForStart();

        while (opModeIsActive()) {
            executeMovements();
            break;
        }
    }

    private void executeMovements() {
        moveToPosition(36, 0);
        moveToPosition(-36, 1);
    }

    private void moveToPosition(double targetXInches, double targetYInches) {

        performStrafe(targetXInches);
        checkAndCorrectHeading();
        performStraight(targetYInches);
    }

    private void performStrafe(double targetX) {
        motors.resetDeadWheelEncoders();
        centerPIDController.setTarget(targetX);

        while (opModeIsActive() && Math.abs(centerPIDController.getError()) > STRAIGHT_TOLERANCE) {
            deltaTime = Math.max(timer.milliseconds(), 1e-3);
            double currentPosition = motors.getPosition(DeadWheel.Center);
            timer.reset();
            double output = centerPIDController.calculateOutput(currentPosition, deltaTime);
            motors.setMotorPowers(-output, output, -output, output);
        }
        motors.stopMotors();
    }

    private void performStraight(double targetY) {
        motors.resetDeadWheelEncoders();
        leftPIDController.setTarget(targetY);
        rightPIDController.setTarget(targetY);

        while (opModeIsActive() && !isStraightTargetReached()) {
            deltaTime = Math.max(timer.milliseconds(), 1e-3);
            double leftPosition = motors.getPosition(DeadWheel.Left);
            double rightPosition = motors.getPosition(DeadWheel.Right);
            double leftOutput = leftPIDController.calculateOutput(leftPosition, deltaTime);
            double rightOutput = rightPIDController.calculateOutput(rightPosition, deltaTime);
            timer.reset();
            motors.setMotorPowers(leftOutput, rightOutput, leftOutput, rightOutput);
        }
        motors.stopMotors();
    }

    private void driveToPosition(Point target) {

        currentPosition = myAprilTagProcessor.getPosition(currentPosition);

        while (opModeIsActive()) { //&& !isTargetReached()) {

            double x1 = currentPosition.getX();
            double y1 = currentPosition.getY();

            double x2 = target.getX();
            double y2 = target.getY();

            double deltaX = x1 - x2;
            double deltaY = y1 - y2;

            double errorX = (x2 * Constants.ticksPerInch()) - (x1 * Constants.ticksPerInch());
            double errorY = (y2 * Constants.ticksPerInch()) - (x1 * Constants.ticksPerInch());

//            double pidOutputX = xPIDController.calculateOutput()

//            double distance = Math.sqrt((deltaX * deltaX + deltaY * deltaY));
//            double angle = Math.atan2(deltaY, deltaX);


        }


    }

    private boolean isStraightTargetReached() {
        return Math.abs(leftPIDController.getError()) < STRAIGHT_TOLERANCE &&
                Math.abs(rightPIDController.getError()) < STRAIGHT_TOLERANCE;
    }

    private void checkAndCorrectHeading() {
        double heading = gyros.getHeading();
        while (opModeIsActive() && Math.abs(heading) > TURN_TOLERANCE) {
            double turnPower = heading < 0 ? -TURN_SPEED : TURN_SPEED;
            motors.setMotorPowers(turnPower, -turnPower, turnPower, -turnPower);
            heading = gyros.getHeading();
        }
        motors.stopMotors();
    }

    private void initializePIDControllers() {
        xPIDController = new SanjuPIDController("straight");
        yPIDController = new SanjuPIDController("strafe");

        leftPIDController = new SanjuPIDController("straight");
        rightPIDController = new SanjuPIDController("straight");
        centerPIDController = new SanjuPIDController("strafe");
        turnPIDController = new SanjuPIDController("turn");
    }



}
