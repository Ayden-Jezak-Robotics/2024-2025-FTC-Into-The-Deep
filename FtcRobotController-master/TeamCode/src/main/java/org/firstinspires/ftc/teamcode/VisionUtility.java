package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;


public class VisionUtility {

    private AprilTagProcessor myAprilTagProcessor;
    private VisionPortal myVisionPortal;

    VisionUtility() {
        Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);

        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

        this.myAprilTagProcessor = new AprilTagProcessor.Builder()
               .setCameraPose(cameraPosition, cameraOrientation)
               .setDrawAxes(true)
               .build();
        this.myVisionPortal = new VisionPortal.Builder()
               .setShowStatsOverlay(true)
               .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
               .addProcessor(myAprilTagProcessor)
               .setCameraResolution(new Size(1280, 720))
               .setStreamFormat(VisionPortal.StreamFormat.YUY2)
               .enableLiveView(true)
               .build();
    }

    Point getPosition(AprilTagProcessor myAprilTagProcessor) {
        double currentY = 0;
        double currentX = 0;
        List<AprilTagDetection> myAprilTagDetections;
        AprilTagDetection myAprilTagDetection;
//        int myAprilTagIDCode;
        myAprilTagDetections = myAprilTagProcessor.getDetections();
        for (AprilTagDetection tagID : myAprilTagDetections) {
            if (tagID.metadata != null) {
//                myAprilTagIDCode = tagID.id;
                currentY = tagID.robotPose.getPosition().y;
                currentX = tagID.robotPose.getPosition().x;
            }
        }

        return new Point(currentX, currentY);
    }
}
