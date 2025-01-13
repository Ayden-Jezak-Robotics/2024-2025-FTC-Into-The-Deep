package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Autonomous(name = "AutoOpBlueLeft", group = "Draft")
public class AutoOpBlueLeft extends LinearOpMode {

    Position initialPosition = new Position(DistanceUnit.INCH, -24, -64, 0, System.nanoTime());
    double initialHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        
        Robot robot = new Robot(this, hardwareMap, telemetry, initialPosition, initialHeading);

        waitForStart();
        telemetry.addLine("Began");
        telemetry.update();

        robot.moveToPositionAndHeading(new Position(DistanceUnit.INCH, -24, -64, 0, 0), 135);
        robot.moveToPositionAndHeading(new Position(DistanceUnit.INCH, -24, -64, 0, 0), -135);
        robot.moveToPositionAndHeading(new Position(DistanceUnit.INCH, -24, -64, 0, 0), 0);

        /*sleep(2000);
        robot.moveToPositionAndHeading(new Position(DistanceUnit.INCH, -32, -48, 0, 0), 0);
        sleep(2000);
        robot.moveToPositionAndHeading(new Position(DistanceUnit.INCH, -32, -48, 0, 0), 45);
        sleep(2000);
        robot.moveToPositionAndHeading(new Position(DistanceUnit.INCH, -48, -48, 0, 0), 45);
        sleep(2000);
        robot.moveToPositionAndHeading(new Position(DistanceUnit.INCH, -48, -48, 0, 0), 135);
        //robot.moveToPositionAndHeading(new Position(DistanceUnit.INCH, 0, 0, 0, 0), 0);
*/


        //robot.moveToPositionAndHeading(new Position(DistanceUnit.INCH, 24, -12, 0, 0), 90);

    }


}
