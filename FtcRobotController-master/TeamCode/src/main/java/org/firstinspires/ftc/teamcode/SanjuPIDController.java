package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// Not Working

public class SanjuPIDController {
    private double kP, kI, kD, kF;
    private double derivative;
    private double target; //in ticks
    private double integralSum;
    private double error; // in ticks
    private double lastError;
    private static final double MAX_INTEGRAL = 1000; // Adjust based on your system's needs

    public SanjuPIDController(double kP, double kI, double kD, double kF)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.integralSum = 0;
        this.lastError = 0;
    }

    public SanjuPIDController(String type)
    {
        switch (type) {
            case "straight":
                this.kP = 0.00003;
                this.kI = 0;
                this.kD = 0;
                this.kF = 0.2;
                break;
            case "strafe":
                this.kP = 0.00003;
                this.kI = 0;
                this.kD = 0;
                this.kF = 0.22;
                break;
            case "turn":
                this.kP = 0.003;
                this.kI = 0;
                this.kD = 0;
                this.kF = 0.15;
                break;
            default:
                throw new IllegalArgumentException("Invalid type");
        }

        this.integralSum = 0;
        this.lastError = 0;
    }

    public void setTarget(double target) //set always in inches
    {
        this.target = target;
    }

    public void setError(double newError)
    {
        error = newError;
    }

    public double calculateOutput(double currentPosition, double deltaTime)
    {
        error = target - currentPosition;

        double aMaxPoint = target/4;

        double proportional = kP * error;

        integralSum += (error * deltaTime);
        // Anti-windup especially when distance is large
        integralSum = Range.clip(integralSum, -MAX_INTEGRAL, MAX_INTEGRAL);
        double integral = kI * integralSum;

        double feedforward = kF * Math.signum(error);

        derivative = kD * (error - lastError)/deltaTime;

        lastError = error;

        double baseOutput = proportional + integral + derivative;
        double output;

        if (Math.abs(target) < 4400 && target != 0)
        {
            output = 0.30 * Math.signum(target);
        }
        else if (target <0)
        {
            if (currentPosition > aMaxPoint)
            {
                output = feedforward + (currentPosition/aMaxPoint) * baseOutput;
            }
            else
            {
                output = feedforward + baseOutput;
            }
        }
        else if (target > 0)
        {
            if (currentPosition < aMaxPoint)
            {
                output = feedforward + (currentPosition/aMaxPoint) * baseOutput;
            }
            else
            {
                output = feedforward + baseOutput;
            }
        }
        else
        {
            output = feedforward + baseOutput;
        }

        output = Range.clip(output, -1, 1);

        return output;
    }

    public double getDerivative() {
        return derivative;
    }

    public double updateError(double target, double newLoc)
    {
        error = target - newLoc;
        return error;
    }

    public double getError()
    {
        return error;
    }
    public double getTarget()
    {
        return target;
    }
    public double getkF()
    {
        return kF;
    }
    public void reset()
    {
        integralSum = 0;
        lastError = 0;
    }

}