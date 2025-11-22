package org.firstinspires.ftc.teamcode.Firmware.Systems;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
        //The builder class is used to access multiple configurations for the aprilTagProcessor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(736.952533347, 736.952533347, 951.225875883, 540.574797136)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        //The VisionPortal.Builder is used to access the vison processer for April Tags
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(1280, 960));
        builder.addProcessor(aprilTagProcessor);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        visionPortal = builder.build();
    }
    // The displayDetectionTelemetry is used to get telemetry back about what tag it is detecting
    public void displayDetectionTelemetry(AprilTagDetection detectedID) {
        if (detectedID == null) {
            return;
        }
        /*This is used to actually display the telemetry
        If the detection ID is not equal to null then it will display the number of ID it is
        If the detection is null then the detection ID is unknown
         */
        if (detectedID.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedID.id, detectedID.metadata.name));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectedID.ftcPose.range, detectedID.ftcPose.bearing, detectedID.ftcPose.elevation));
            aprilTagID = detectedID.id;
        } else {
            telemetry.addLine("No AprilTags");
        }
    }
    //The update function is used to get tagDetected
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
        if (mode == "red") {
            //Red april tags
            update();
            AprilTagDetection id24 = getSpecific(24);
            displayDetectionTelemetry(id24);
        } else if (mode == "blue") {
            //Blue april tags
            update();
            AprilTagDetection id23 = getSpecific(23);
            displayDetectionTelemetry(id23);
        } else if (mode == "obelisk") {
            update();
            AprilTagDetection id22 = getSpecific(22);
            /*if(id22 != null) {
                aprilTagID = id22.id;
            }*/
            displayDetectionTelemetry(id22);
            // April tags 20-22 are for the Obelisk
            update();
            AprilTagDetection id20 = getSpecific(20);
            displayDetectionTelemetry(id20);

            update();
            AprilTagDetection id21 = getSpecific(21);
            displayDetectionTelemetry(id21);
        }
    }
    public double getDistance(String mode){
        locateAprilTags(mode);
        telemetry.addData("AprilTag",aprilTagID);
        //telemetry.addData("Return Value",getSpecific(aprilTagID).ftcPose.range);
        telemetry.update();
        return 1.0;
        //return getSpecific(aprilTagID).ftcPose.range;
    }
    // the Bearing To Tag is used to turn the robot so it is facing the center of the tag
    public double getBearingToTag(String mode){
        locateAprilTags(mode);
        return getSpecific(aprilTagID).ftcPose.bearing;
    }
    public int getMotifID(){
        //Still need to call detection telemetry for April tag id to be set
        locateAprilTags("obelisk");
        return aprilTagID;
    }
    public void stop(){
        if (visionPortal !=null){
            visionPortal.close();
        }
    }

    //This function is just returning the telemetry
    public Telemetry getTelemetry() {
        return telemetry;
    }
}