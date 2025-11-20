package org.firstinspires.ftc.teamcode.Firmware.Systems;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Resources.AprilTagSystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.ArrayList;
import java.util.List;

public class AprilTag {
    private Telemetry telemetry;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> tagsDetected = new ArrayList<>();
    private int aprilTagID;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = telemetry;
        //...?.
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(736.952533347, 736.952533347, 951.225875883, 540.574797136)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(1280, 960));
        builder.addProcessor(aprilTagProcessor);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        visionPortal = builder.build();
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedID) {
        if (detectedID == null) {
            //why is this here?
            //return;
        }
        if (detectedID.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedID.id, detectedID.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detectedID.ftcPose.x, detectedID.ftcPose.y, detectedID.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedID.ftcPose.pitch, detectedID.ftcPose.roll, detectedID.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectedID.ftcPose.range, detectedID.ftcPose.bearing, detectedID.ftcPose.elevation));
            aprilTagID = detectedID.id;
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detectedID.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detectedID.center.x, detectedID.center.y));
        }
    }

    public void update() {
        tagsDetected = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getTagsDetected() {
        return tagsDetected;
    }

    public AprilTagDetection getSpecific(int id) {
        for (AprilTagDetection detection : tagsDetected) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    /**
     *
     * @param mode (options are: red, blue or obelisk)
     */
    public void locateAprilTags(String mode) {
        if (mode == "red"){
            //Red april tags
            update();
            AprilTagDetection id24 = getSpecific(24);
            displayDetectionTelemetry(id24);
        }
        else if(mode == "blue"){
            //Blue april tags
            update();
            AprilTagDetection id23 = getSpecific(23);
            displayDetectionTelemetry(id23);
        }
        else if(mode == "obelisk") {
            // April tags 20-22 are for the Obelisk
            update();
            AprilTagDetection id20 = getSpecific(20);
            displayDetectionTelemetry(id20);

            update();
            AprilTagDetection id21 = getSpecific(21);
            displayDetectionTelemetry(id21);

            update();
            AprilTagDetection id22 = getSpecific(22);
            displayDetectionTelemetry(id22);
        }
        telemetry.addLine("aprilTagFalse");
        telemetry.update();
    }
    public double getDistance(String mode){
        locateAprilTags(mode);
        return getSpecific(aprilTagID).ftcPose.range;
    }
    public void stop(){
        if (visionPortal !=null){
            visionPortal.close();
        }
    }

    //what is this function??
    public Telemetry getTelemetry() {
        return telemetry;
    }
}