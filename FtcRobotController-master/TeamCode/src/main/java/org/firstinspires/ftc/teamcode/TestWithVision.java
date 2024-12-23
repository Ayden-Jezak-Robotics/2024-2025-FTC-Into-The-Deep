package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU; //for gyro

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; //utilized linear mode
import com.qualcomm.robotcore.eventloop.opmode.Autonomous; //send to Autonomous on Driver Hub

import com.qualcomm.robotcore.hardware.DcMotor; //use DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple; //
import com.qualcomm.robotcore.util.ElapsedTime; // use for time
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import android.util.Size;


@Autonomous(name = "TestWithVision", group = "Draft")
public class TestWithVision extends LinearOpMode
{
    //declare Motors
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private DcMotor frontLeft;

    //declare deadWheels
    private DcMotor rightDeadWheel;
    private DcMotor leftDeadWheel;
    private DcMotor centerDeadWheel;

    // reg variables
    private double deltaTime;

    //declare PIDControllers
    private SanjuPIDController centerPIDController, leftPIDController, rightPIDController, turnPIDController;

    // declare Mathy Variables
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();

    // declare motor powers, and wheel distances
    double leftOutput, rightOutput, centerOutput;
    double leftPosition, rightPosition, centerPosition; // in ticks

    //gyro stuff
    private BNO055IMU imu;
    double heading;
    double turnSpeed = 0.2;

    //mathy variables
    // declare Mathy Variables
    private int ticksPerRotation = 8192;
    double wheelCircumference = (Math.PI * 60)/25.4;
    double ticksPerInch = (ticksPerRotation / wheelCircumference);

    double initX = 0; //might wanna change this for the actual match
    double initY = 0; // also this

    @Override
    public void runOpMode() throws InterruptedException
    {
        //method I made below that initialized hardware
        initializeHardware();

        telemetry.addData("Status", "Calibrating IMU...");
        telemetry.update();

        // Wait for calibration to complete or for stop request
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(100); // Short delay to avoid overloading the CPU
            idle();    // Let other processes run
        }

        telemetry.addData("Status", "IMU Calibrated");
        telemetry.update();

        //setting PID Controllers
        leftPIDController = new SanjuPIDController("straight");
        rightPIDController = new SanjuPIDController("straight");
        centerPIDController = new SanjuPIDController("strafe");
        turnPIDController = new SanjuPIDController("turn");

        waitForStart(); //FTC SDK method which wait for the "Play"
        // MAIN PART OF THE CODE!!!!!!!!!!!!
        while (opModeIsActive()) {
            driveToPosition(0,36);
            driveToPosition(36,0);
            driveToPosition(0,-36);
            driveToPosition(-36,0);
            //driveToPosition(36,0);
            //driveToPosition(-36,0);
            //driveTurn(90);
            timer.reset();
            break;
        }

    }

    private void checkHeading()
    {
        heading = imu.getAngularOrientation().firstAngle;
        if (Math.abs(heading) > 1) {
            correctHeading(heading);
        }
    }
    private void correctHeading(double heading) {
        while (opModeIsActive() && Math.abs(heading) > 1) {
            // Determine turn direction and set motor power
            if (heading < 0) {
                // Turn left
                setStraightPower(-turnSpeed, turnSpeed);
            } else {
                // Turn right
                setStraightPower(turnSpeed, -turnSpeed);
            }
            heading = imu.getAngularOrientation().firstAngle;
        }
        stopMotors();
    }


    private void driveStraight(double targetY) //input in ticks
    {
        leftDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftPIDController.setTarget(targetY); // Forward motion is Y
        rightPIDController.setTarget(targetY);
        leftPIDController.setError(targetY); // Forward motion is Y
        rightPIDController.setError(targetY);
        timer.reset();

        while (opModeIsActive() && !isAprilTargetReached()) {
            leftPosition = leftDeadWheel.getCurrentPosition() + initY; //position in ticks
            rightPosition = rightDeadWheel.getCurrentPosition() + initY; //same
            deltaTime = Math.max(timer.milliseconds(), 1e-3); // Avoids setting time to zero, Minimum deltaTime of 1 microsecond
            timer.reset();
            leftOutput = leftPIDController.calculateOutput(leftPosition, deltaTime);
            rightOutput = rightPIDController.calculateOutput(rightPosition, deltaTime);
            setStraightPower(leftOutput, rightOutput);
        }
        stopMotors();
    }

    private void driveStrafe(double targetX) //input in ticks, + strafes to the right
    {
        leftDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        centerPIDController.setTarget(targetX); // Strafing motion is X
        centerPIDController.setError(targetX); // Strafing motion is X
        timer.reset();

        while (opModeIsActive() && !isAprilTargetReached())
        {
            centerPosition = centerDeadWheel.getCurrentPosition() + initX; //position in ticks

            deltaTime = Math.max(timer.milliseconds(), 1e-3); // Avoids setting time to zero, Minimum deltaTime of 1 microsecond
            timer.reset();
            centerOutput = centerPIDController.calculateOutput(centerPosition, deltaTime);

            setStrafePower(centerOutput);
        }
        stopMotors();
    }

    private void driveToPosition(double initialTargetX, double initialTargetY)
    {
        // Reset encoders before starting
        double targetX = initialTargetX * ticksPerInch;
        double targetY = initialTargetY * ticksPerInch;

        driveStrafe(targetX);
        checkHeading();
        leftPosition = leftDeadWheel.getCurrentPosition(); //position in ticks
        rightPosition = rightDeadWheel.getCurrentPosition();
        double average = (leftPosition+rightPosition)/2;
        double newYError = targetY - average;
        telemetry.addData("leftPos",leftPosition);
        telemetry.addData("rightPos", rightPosition);
        telemetry.addData("newYError", newYError);
        telemetry.update();
        sleep(2000);

        driveStraight(newYError);
        leftPosition = leftDeadWheel.getCurrentPosition(); //position in ticks
        rightPosition = rightDeadWheel.getCurrentPosition();
        telemetry.addData("leftPos",leftPosition);
        telemetry.addData("rightPos", rightPosition);
        telemetry.update();
        sleep(2000);
        checkHeading();
        updateCoordinates(targetX, targetY);
    }


    private void updateCoordinates(targetX, targetY)
    {
        if (tagProcessor.getDetections().size() > 0)
            {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                initX = tag.robotPose.getPosition().x;
                initY = tag.robotPose.getPosition().y;
            }
            else
            {
                initX = targetX;
                initY = targetY;
            }
    }
    private void driveTurn(double targetAngle)
    {
        turnPIDController.setTarget(targetAngle); // Strafing motion is X
        turnPIDController.setError(targetAngle); // Strafing motion is X
        timer.reset();

        while (opModeIsActive() && !isTurnTargetReached()) {
            heading = -(imu.getAngularOrientation().firstAngle);

            deltaTime = Math.max(timer.milliseconds(), 1e-3); // Avoids setting time to zero, Minimum deltaTime of 1 microsecond
            timer.reset();

            double power = turnPIDController.calculateOutput(heading, deltaTime);
            // Determine turn direction and set motor power
            setTurnPower(power);
            heading = imu.getAngularOrientation().firstAngle;
            telemetry.addData("target", targetAngle);
            telemetry.addData("heading", heading);
            telemetry.addData("power",power);
            telemetry.update();
        }
        stopMotors();
        telemetry.addData("heading",heading);
        telemetry.update();
        sleep(2000);
    }

    private void setTurnPower(double turnMotorPower)
    {
        frontLeft.setPower(turnMotorPower);
        backLeft.setPower(turnMotorPower);
        frontRight.setPower(-turnMotorPower);
        backRight.setPower(-turnMotorPower);
    }

    private void stopMotors()
    {
        backLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
        frontLeft.setPower(0);
    }

    private void setStrafePower(double centerMotorPower) // automated to go right
    {
        double sign = Math.signum(centerMotorPower);
        backLeft.setPower(-centerMotorPower);
        backRight.setPower(centerMotorPower);
        frontRight.setPower(-centerMotorPower);
        frontLeft.setPower(centerMotorPower);
    }


    private void setStraightPower(double leftMotorPower, double rightMotorPower) //goes the calculated direction
    {
        backLeft.setPower(leftMotorPower);
        backRight.setPower(rightMotorPower);
        frontRight.setPower(rightMotorPower);
        frontLeft.setPower(leftMotorPower);
    }

    // Check if the target distance is reached
    private boolean isStraightTargetReached()
    {
        double leftError = Math.abs(leftPIDController.getError());
        double rightError = Math.abs(rightPIDController.getError());

        return leftError < 50 && rightError < 50; // Tolerance of 50 encoder counts
    }

    private boolean isAprilTargetReached()
    {
        double x;
        double y;
        if (tagProcessor.getDetections().size() > 0)
            {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                x =  tag.robotPose.getPosition().x * ticksPerInch;
                y = tag.robotPose.getPosition().y * ticksPerInch;
                if (Math.abs(initX - x) < 50 && Math.abs(initY - y) < 50)
                {
                    return true;
                }
            }
            else
            {
                double leftError = Math.abs(leftPIDController.getError());
                double rightError = Math.abs(rightPIDController.getError());
                return leftError < 50 && rightError < 50;
            }
    }

    private boolean isStrafeTargetReached()
    {
        double centerError = Math.abs(centerPIDController.getError());

        telemetry.addData("derivative", centerPIDController.getDerivative());
        //telemetry.addData("integral at 3600", centerPIDController.getIntegral());
        telemetry.update();

        return centerError < 50; // Tolerance of 50 encoder counts
        //Later make tolerance a variable
    }

    private boolean isTurnTargetReached()
    {
        double turnError = Math.abs(turnPIDController.getError());

        return turnError < 0.3; // 1 degree
        //Later make tolerance a variable
    }

    private void initializeHardware()
    {
        // Hardware initialization
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        rightDeadWheel = hardwareMap.get(DcMotor.class, "rightDeadWheel");
        leftDeadWheel = hardwareMap.get(DcMotor.class, "leftDeadWheel");
        centerDeadWheel = hardwareMap.get(DcMotor.class, "centerDeadWheel");

        // Set motor directions
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        rightDeadWheel.setDirection(DcMotorSimple.Direction.FORWARD); //Make sure the deadwheels are in the right direction
        leftDeadWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        centerDeadWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset encoders
        rightDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //If we ran using encoder, the robot's speed would be automatically adjusted
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //If we don't specify that we are running WITHOUT encoders, it will set the status to what it previously was, leading to unpredictability
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightDeadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDeadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerDeadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //If we ran using encoder, the robot's speed would be automatically adjusted
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //If we don't specify that we are running WITHOUT encoders, it will set the status to what it previously was, leading to unpredictability
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize the IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);

        //initialize april
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder() //identifies tag, estimates pose
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder() //manages initializations of cameras and runs processors on a seperate thread
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();
    }
}