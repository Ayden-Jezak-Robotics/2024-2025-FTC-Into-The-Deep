package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoOpSpinTest", group = "Test")
public class AutoOpSpinTest extends LinearOpMode {

    RobotState initialState = new RobotState(0, 0, 0, 0,0,false,false);

    /// In Degrees

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(this, hardwareMap, telemetry, initialState, CameraPosition.RIGHT);

        int numberOfSpins = 10;

        waitForStart();

        robot.moveToNewRobotState(new RobotState(0, 0, (360 * numberOfSpins), 0, 0, false, false));

        robot.stop();
    }
}
