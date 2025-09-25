package org.firstinspires.ftc.teamcode.Resources;

// huge props to our mentor who also wrote this class in the FIRST SDK, Dryw Wade!

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

// used for figuring out the robots position with the april tags around the field.
public class
AprilTagSystem {
 
  // the position of the camera lens relative to the center of the robot
  private Position cameraPosition = new Position(DistanceUnit.INCH,
          0, -7.25, 4, 0);
  
  // the orientation of the camera lens relative to the orientation of the robot
  private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
          -180, 0, 0, 0);
  
  
  // end of the todo values
  
  // the latest detection the processor has found
  private AprilTagDetection detection;
  
  /**
   * The variable to store our instance of the AprilTag processor.
   */
  private AprilTagProcessor aprilTag;
  
  /**
   * The variable to store our instance of the vision portal.
   */
  private VisionPortal visionPortal;

  /**
   * The amount the tags detection is allowed to be off from the OTOS, this helps if the detection has a really inaccurate reading from things like motion blur and ect...
   */
  private final double ALLOWED_ERROR_DISTANCE = 3.5; // inches
  
  /**
   * Initialize the AprilTag processor.
   * @param hardwareMap the hardware map instance from the first SDK
   */
  public AprilTagSystem(HardwareMap hardwareMap) {
    
    // Create the AprilTag processor.
    aprilTag = new AprilTagProcessor.Builder()
            
            .setCameraPose(cameraPosition, cameraOrientation).build();
    
    // Create the vision portal by using a builder.
    VisionPortal.Builder builder = new VisionPortal.Builder();
    
    builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));
    
    
    builder.addProcessor(aprilTag);
    
    // Build the Vision Portal, using the above settings.
    visionPortal = builder.build();
    
  }
  
  /**
   * used to figure out which april tag to use then
   * calculates the position of the april tags
   * @param robotXCurrent
   * @param robotYCurrent
   */
  public void findAprilTags(double robotXCurrent, double robotYCurrent) {
    // gets all of the april tags we can see
    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
    
    // stores the best detection we have found so far
    AprilTagDetection bestDetection = null;
    
    //TODO: handle the case in which there are no visible tags
    
    // the idea for choosing the best tag to use it's data to figure out which one is
    // closest to the position that we think the robot should be
    for (AprilTagDetection detection : currentDetections) {
      
      // if there is not a tag to compare this to, just set the tag without the calculations.
      if (bestDetection != null) {
  
        // the x and y position variables are condensed so the code below is readable
        double RX = robotXCurrent, RY = robotYCurrent;
        double TX = detection.robotPose.getPosition().x, TY = detection.robotPose.getPosition().y;
        double BX = bestDetection.robotPose.getPosition().x, BY = bestDetection.robotPose.getPosition().y;
        
        // if the distance of the current tag is closer to where to we think the robot is then we make that the new best detection,
        // else we go to the top of the for loop.
        if (calculateDistance(RX,RY,TX,TY) > calculateDistance(RX,RY,BX,BY)) continue;
        if (calculateDistance(RX, RY, TX,TY) > ALLOWED_ERROR_DISTANCE) continue;
      }
      
      this.detection = detection;
      bestDetection = detection;
    }
  }
  
  // the distance formula in a function
  public double calculateDistance(double robotX, double robotY, double projectedX, double projectedY) {
    return Math.sqrt(Math.pow(robotX - projectedX, 2) + Math.pow(robotY + projectedY, 2));
  }
  
  //used to determine if there is an april tag detected so we don't use data that does not exist.
  public boolean hasDetection(){
    if (detection == null){
      return false;
    } else {
      return true;
    }
  }
  
  // returns the tag ID for the current detection
  public int getTagID() {
    return detection.id;
  }
  
  // returns the x of the robot for the current detection
  public double getRobotX() {
      return detection.robotPose.getPosition().x;
  }
  
  // returns the y of the robot for the current detection
  public double getRobotY() {
    return detection.robotPose.getPosition().y;
  }
  
  // returns the z of the robot for the current detection
  public double getRobotZ() {
    return detection.robotPose.getPosition().z;
  }
  
  // returns the yaw rotation of the robot for the current detection
  public double getRobotYaw() {
    return detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
  }
}

