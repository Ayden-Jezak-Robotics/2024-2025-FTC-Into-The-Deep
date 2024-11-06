package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; //utilized linear mode
import com.qualcomm.robotcore.eventloop.opmode.Autonomous; //send to Autonomous on Driver Hub
import com.qualcomm.robotcore.hardware.DcMotor; //use DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple; //
import com.qualcomm.robotcore.util.ElapsedTime; // use for time

@Autonomous(name = "Test", group = "Draft")
public class Test extends LinearOpMode
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
    private SanjuPIDController centerPIDController, leftPIDController, rightPIDController;

    // declare Mathy Variables
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();
    private int ticksPerRotation = 8192;
    double wheelCircumference = (Math.PI * 60);
    double inchesPerTick = (wheelCircumference / ticksPerRotation) / 2.54;
    double ticksPerInch = (ticksPerRotation / wheelCircumference) * 25.4;

    // declare motor powers, and wheel distances
    double leftOutput, rightOutput, centerOutput;
    double leftPosition, rightPosition, centerPosition; // in ticks

    @Override
    public void runOpMode() throws InterruptedException
    {
        //method I made below that initialized hardware
        initializeHardware();

        //setting PID Controllers
        leftPIDController = new SanjuPIDController("straight");
        rightPIDController = new SanjuPIDController("straight");
        centerPIDController = new SanjuPIDController("strafe");

        waitForStart(); //FTC SDK method which wait for the "Play"
        telemetry.addData("after wait for start",getRuntime());
        telemetry.update();
        // MAIN PART OF THE CODE!!!!!!!!!!!!
        while (opModeIsActive()) {
            telemetry.addData("Present", "Yes");
            telemetry.update();
            driveStraight(36);
            //reset();
        }

    }

    private void driveStraight(double target) //assumes target is zero
    {
        // Reset encoders before starting
        leftDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Reset and set target for PID controller
        leftPIDController.setTarget(target);
        rightPIDController.setTarget(target);

        runtime.reset();//useless right now, but can use later for telemetry
        timer.reset();
        telemetry.addData("in DriveStraight", "yes");
        telemetry.update();
        while (opModeIsActive() && !isStraightTargetReached()) {
            leftPosition = leftDeadWheel.getCurrentPosition(); //position in ticks
            rightPosition = rightDeadWheel.getCurrentPosition(); //same

            deltaTime = Math.max(timer.seconds(), 1e-6); // Avoids setting time to zero, Minimum deltaTime of 1 microsecond
            timer.reset();
            leftOutput = leftPIDController.calculateOutput(leftPosition, deltaTime);
            rightOutput = rightPIDController.calculateOutput(rightPosition, deltaTime);

            setStraightPower(leftOutput, rightOutput);

            // Telemetry for debugging
            telemetry.addData("Left Position", leftPosition);
            telemetry.addData("Right Position", rightPosition);
            telemetry.addData("Left Output", leftOutput);
            telemetry.addData("Right Output", rightOutput);
            telemetry.addData("Runtime", runtime.seconds());
            telemetry.update();
        }

    }

    private void setStraightPower(double leftMotorPower, double rightMotorPower) //goes the calculated direction
    {
        backLeft.setPower(leftMotorPower);
        backRight.setPower(rightMotorPower);
        frontRight.setPower(rightMotorPower);
        frontLeft.setPower(leftMotorPower);
    }

    private void setMotorPower()
    {
        backLeft.setPower(0.2);
        backRight.setPower(0.2);
        frontRight.setPower(0.2);
        frontLeft.setPower(0.2);
    }

    // Check if the target distance is reached
    private boolean isStraightTargetReached()
    {
        telemetry.addData("is?","in");
        telemetry.update();
        double leftError = Math.abs(leftPIDController.getError());
        double rightError = Math.abs(rightPIDController.getError());
        telemetry.addData("leftError",leftError);
        telemetry.addData("rightError", rightError);
        return leftError < 10 && rightError < 10; // Tolerance of 10 encoder counts
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

        rightDeadWheel.setDirection(DcMotorSimple.Direction.REVERSE); //Make sure the deadwheels are in the right direction
        leftDeadWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        centerDeadWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset encoders
        rightDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to run without encoders, so we manually control power
        leftDeadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //but ChatGPT beta says: for active feedback on their position
        rightDeadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerDeadWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}
