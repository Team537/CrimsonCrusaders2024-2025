package org.firstinspires.ftc.teamcode.Robot.Subsystems.RobotVision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Utilities.CommandSystem.Subsystem;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import android. util. Size;

import java.util.List;

public class VisionLocalizationSubsystem extends Subsystem {

    //The vision portal (contains the camera)
    VisionPortal visionPortal;
    AprilTagProcessor aprilTagProcessor;

    public VisionLocalizationSubsystem(CameraName camera){
        aprilTagProcessor = new AprilTagProcessor.Builder()
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setLensIntrinsics(
                898.5424522397835,
                898.9404753032804,
                596.1380229169472,
                404.5406934385057
            )
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
            .setCameraPose(Constants.Vision.Localization.LOCALIZATION_CAMERA_POSITION,Constants.Vision.Localization.LOCALIZATION_CAMERA_ORIENTATION)
            .build();

        visionPortal = new VisionPortal.Builder()
            .addProcessor(aprilTagProcessor)
            .setCamera(camera)
            .setCameraResolution(new Size(1280,720))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .build();

    }
    //Turning on and off the vision portal when not needed
    public void setVisionPortal(boolean status) {
        if (status) {
            visionPortal.resumeStreaming();
        } else {
            visionPortal.stopStreaming();
        }
    }

    /**
     * returns the robot's Pose as estimated by a camera based on the location of april tags
     */
    public VisionLocalizationPoseResult getPose() {

        //make sure the vision portal is resumed
        setVisionPortal(true);

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        double closestPoseDistance = Double.MAX_VALUE;
        Pose2D pose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);

        for (AprilTagDetection detection : currentDetections) {
            if (detection != null) {
                if (detection.ftcPose != null && detection.robotPose != null) {
                    Robot.getInstance().opMode.telemetry.addLine("Detected Tag " + Integer.toString(detection.id) + " at " + Math.round(detection.ftcPose.x) + ", " + Math.round(detection.ftcPose.y) + ", " + Math.round(detection.ftcPose.z));
                    if (detection.ftcPose.range < closestPoseDistance) {
                        closestPoseDistance = detection.ftcPose.range;
                        pose = new Pose2D(DistanceUnit.INCH, detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, AngleUnit.RADIANS, detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS));
                    }
                }
            }
        }

        return new VisionLocalizationPoseResult(!(Math.abs(closestPoseDistance - Double.MAX_VALUE) < 1.0),pose);
    }

    public class VisionLocalizationPoseResult {

        public boolean foundPose;
        public Pose2D pose;

        public VisionLocalizationPoseResult(boolean foundPose, Pose2D pose) {
            this.foundPose = foundPose;
            this.pose = pose;
        }
    }

}
