package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDUtility {
    private PIDType type;

    private double kP, kI, kD, kF;

    private double initialPosition; // in inches
    private double targetPosition; // in inches
    private double originalError; //in ticks

    //private double aMaxPoint; // in ticks
    private double integralSum;
    private double priorError;
    // Adjust based on your system's needs

    private Telemetry telemetry;

    public PIDUtility(double kP, double kI, double kD, double kF, Telemetry telemetry) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.integralSum = 0;
        this.priorError = 0;
        this.telemetry = telemetry;
    }

    public PIDUtility(PIDType type) {
        this.type = type;
        this.telemetry = telemetry;

        /// DID SOME EXCEL CALCULATIONS AND SUGGEST WE TRY THE COMMENTED VALUES

        switch (type) {
            case STRAIGHT:
                this.kP = 0.00003; // 0.0003
                this.kI = 0.00003; // 0.000015
                this.kD = 0.000001; // 0.000002
                this.kF = 0; //0.2
                break;
            case STRAFE:
                this.kP = 0.00003;
                this.kI = 0;
                this.kD = 0.000001;
                this.kF = 0; //0.22
                break;
            case TURN:
                this.kP = -0.015; //NEW
                this.kI = 0;
                this.kD = 0.01;
                this.kF = 0; //0.15
                break;
            default:
        }

        this.integralSum = 0;
        this.priorError = 0;
    }

    public void setOriginalError(double initialPosition, double targetPosition) {
        this.initialPosition = initialPosition;
        this.targetPosition = targetPosition;

        this.originalError = (targetPosition - initialPosition);

//        if (originalError == 0) {
//            originalError = .006;
//        }

        if (type == PIDType.STRAIGHT || type == PIDType.STRAFE) {
            this.originalError = this.originalError * Constants.DEAD_WHEEL_TICKS_PER_INCH;
        }

        //this.aMaxPoint = originalError / 3;
    }

    public double calculatePower(double currentPosition, double time) // time is in Seconds
    {
        double error;

        if (type == PIDType.STRAIGHT || type == PIDType.STRAFE) {
            error = (targetPosition - currentPosition) * Constants.DEAD_WHEEL_TICKS_PER_INCH;

            if (Math.abs(error) < Constants.MINIMUM_DISTANCE) {
                return 0;
            }
        } 
        else { // For Turn based calculations
            error = targetPosition - currentPosition;

            // Normalize error to the range [-180, 180]

            if (Math.abs(error) < Constants.TURN_TOLERANCE) {
                return 0;
            }
        }

        double deltaTime;

        if (time < Constants.MINIMUM_TIME_IN_SECONDS) {
            deltaTime = Constants.MINIMUM_TIME_IN_SECONDS; // Prevent zero or very small time steps
        } else {
            deltaTime = time;
        }

        integralSum += (error * deltaTime);

        double kProportionalValue = kP * error;

        double kIntegralValue = kI * Range.clip(integralSum, -Constants.MAX_INTEGRAL, Constants.MAX_INTEGRAL);

        double kDerivativeValue = kD * ((error - priorError) / deltaTime);

        double kFeedForwardValue = kF * Math.signum(error);

        priorError = error;

        double baseOutput = kProportionalValue + kIntegralValue + kDerivativeValue;
        double errorCompleted;

        if (type == PIDType.STRAIGHT || type == PIDType.STRAFE) {

            errorCompleted = (currentPosition - initialPosition) * Constants.DEAD_WHEEL_TICKS_PER_INCH;

        }
        else {

            errorCompleted = (currentPosition - initialPosition);

        }

        return baseOutput;
    }
}
