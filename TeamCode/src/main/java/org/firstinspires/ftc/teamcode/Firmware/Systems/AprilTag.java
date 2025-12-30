package org.firstinspires.ftc.teamcode.Firmware.Systems;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.ArrayList;
import java.util.List;
@Config
public class AprilTag {
    private Telemetry telemetry;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> tagsDetected = new ArrayList<>();
    public static double RED_OFFSET = -5;
    public static double BLUE_OFFSET = 0.1;

    private double lastAngle = 0.0;
    private double lastDistance= 0.0;
    private int aprilTagID;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = telemetry;
        //The builder class is used to access multiple configurations for the aprilTagProcessor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(589.64467121, 589.64467121, 632.98824788, 477.488107561)
                .setCameraPose(new Position(DistanceUnit.INCH,0,0,0,0),new YawPitchRollAngles(AngleUnit.DEGREES,0,-90,0,0))
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
            aprilTagID = 0;
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
        }else{
            aprilTagID = 0;
        }
    }
    //The update function is used to get tagDetected
    public void update() {
        tagsDetected = aprilTagProcessor.getDetections();
    }

    // The getTagsDetected() function is used to get a list of all detected april tags
    public List<AprilTagDetection> getTagsDetected() {
        return tagsDetected;
    }

    // The getSpecific function is used in order to get the specifics of all detected AprilTags
    public AprilTagDetection getSpecific(int id) {
        for (AprilTagDetection detection : tagsDetected) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

     //@param mode (options are: red, blue or obelisk)

    public void locateAprilTags(String mode) {
        if (mode == "red") {
            //Red april tags
            update();
            AprilTagDetection id24 = getSpecific(24);
            displayDetectionTelemetry(id24);
        } else if (mode == "blue") {
            //Blue april tags
            update();
            AprilTagDetection id20 = getSpecific(20);
            displayDetectionTelemetry(id20);
        } else if (mode == "obelisk") {
            // April tags 20-22 are for the Obelisk
            update();
            AprilTagDetection id21 = getSpecific(21);
            displayDetectionTelemetry(id21);
            if (aprilTagID != 0) return;
            AprilTagDetection id22 = getSpecific(22);
            displayDetectionTelemetry(id22);
            if (aprilTagID != 0) return;
            AprilTagDetection id23 = getSpecific(23);
            displayDetectionTelemetry(id23);
        }
    }

    // the getDistance function calculates the distance between the robot and the april tag
    public double getDistance(String mode){
        locateAprilTags(mode);
        if (aprilTagID == 0) return lastDistance;

        double distance = 0.0;
        double posX = getSpecific(aprilTagID).robotPose.getPosition().x;
        double posY = getSpecific(aprilTagID).robotPose.getPosition().y;
        double goalX = getSpecific(aprilTagID).metadata.fieldPosition.get(0);
        double goalY = getSpecific(aprilTagID).metadata.fieldPosition.get(1);
        distance = Math.sqrt(Math.pow(goalX-posX,2)+Math.pow(goalY-posY,2));
        lastDistance = distance;
        return distance;
    }
    // Locate AprilTags must be called before this!
    public double getRobotX(){
        return getSpecific(aprilTagID).robotPose.getPosition().x;
    }
    public double getRobotY(){
        return getSpecific(aprilTagID).robotPose.getPosition().y;
    }
    public double getRobotAngle(){
        return getSpecific(aprilTagID).robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
    }
    public boolean isTag(String mode){
        locateAprilTags(mode);
        return aprilTagID != 0;
    }



    // the getBearingToTag is used to turn the robot so it is facing the center of the tag
    public double getBearingToTag(String mode){

        locateAprilTags(mode);
        if (aprilTagID == 0) return 0;
        double angle = 0.0;
        switch (mode) {
            case "red":
                angle = getSpecific(aprilTagID).ftcPose.bearing + RED_OFFSET;
                break;
            case "blue":
                angle = getSpecific(aprilTagID).ftcPose.bearing + BLUE_OFFSET;
        }
        return angle;

    }

    // the getMotifID function gets the ID of the motif on the obelisk
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