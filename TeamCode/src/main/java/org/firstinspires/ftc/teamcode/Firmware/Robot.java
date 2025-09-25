package org.firstinspires.ftc.teamcode.Firmware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Resources.AprilTagSystem;
import org.firstinspires.ftc.teamcode.Resources.OdometryOTOS;
import org.firstinspires.ftc.teamcode.Resources.PIDController;
import org.firstinspires.ftc.teamcode.Resources.PathFollowing;

import org.firstinspires.ftc.teamcode.Firmware.Systems.MecanumDriveTrain;

//TODO - Refactor this class to be season non-specific and move all specific details to the DecodeBot class and override information in this class.
@Config
public class Robot {
  
  public enum Speed {
    SLOW, FAST
  }
  // used for how fast the turning input is used.
  // the number for maxTurnDegPerSecond is how much the robot can turn for one degree
  public static double maxTurnDegPerSecond = 280;
  public static double pCon = 0.025;

  public static double pConIntake = 0.05;
  public static double dCon = 0;
  
  private double drive;
  private double slide;
  private double turn;
  //TODO Make more permanent system to detect turning
  public boolean turningBoolean;
  public MecanumDriveTrain driveTrain;
  public OdometryOTOS odometry;
  public ElapsedTime runtime = new ElapsedTime();
  //TODO Tune the pConstant and d Constant numbers, these are place holders.
  public PIDController anglePID = new PIDController(pCon, 0, dCon, runtime);
  
  public SpampleArm spampleArm;
  FtcDashboard dashboard;
  private double robotHeading;
  private double lastTimeAngle;
  private boolean CurrentAlign = true;
  private boolean DriverOrientationDriveMode = true;

  private static final double P_CONSTANT_FAST = 0.15;
  private static final double I_CONSTANT_FAST = 0.1;
  private static final double D_CONSTANT_FAST = 0.02;
  private static final double SPEED_MULTIPLIER_FAST = 1;
  private static final double P_CONSTANT_SLOW = 0.5;
  private static final double I_CONSTANT_SLOW = 1;
  private static final double D_CONSTANT_SLOW = 0.02;
  private static final double SPEED_MULTIPLIER_SLOW = 0.5;

  public double derivativeConstantAngle;
  public double proportionalConstantAngle;
  public Telemetry telemetry;
  private boolean resettingImu = false;
  private double AutoStartAngle = 0;

  public LinearOpMode opMode;

  private double currentLoopTime, previousLoopTime;
  private double lastErrorAngle;
  private boolean IsProgramAutonomous;
  public PathFollowing pathFollowing;
 
  public AprilTagSystem aprilTags;

  public Climber climber;

  public void init(HardwareMap hardwareMap, Telemetry telemetry, double robotX, double robotY, double robotAngle, LinearOpMode opMpde, boolean reset, boolean isAuto) {

    this.opMode = opMpde;

    this.telemetry = telemetry;

    driveTrain = new MecanumDriveTrain(hardwareMap, telemetry);

    odometry = new OdometryOTOS(robotX, robotY, robotAngle, telemetry, hardwareMap, reset);

    pathFollowing = new PathFollowing(P_CONSTANT_FAST, P_CONSTANT_FAST, I_CONSTANT_FAST, I_CONSTANT_FAST, D_CONSTANT_FAST, D_CONSTANT_FAST, runtime);
    spampleArm = new SpampleArm(hardwareMap, runtime, reset, isAuto,telemetry);
    anglePID.setTarget(odometry.getRobotAngle());
//    aprilTags = new AprilTagSystem(hardwareMap);

    climber =  new Climber(hardwareMap,telemetry);

  }
  
  // you call this function in a main auto opMode to make the robot move somewhere.
  // This is the foundation that every robot should need but you should more season specific things in the bot class.
  public void autoMoveTo(double targetX, double targetY, double robotAngle, double targetCircle, double Timeout) {
    while (distanceCircle(targetX, targetY) > targetCircle && opMode.opModeIsActive()) {
      pathFollowing.setTargetPosition(targetX,targetY);
      anglePID.setTarget(robotAngle);
      //put all control things that teleop has in here
      anglePID.update(odometry.getRobotAngle());
      pathFollowing.followPath(odometry.getRobotX(), odometry.getRobotY(), odometry.getRobotAngle());
      driveTrain.setDrivePower(pathFollowing.getPowerF(), pathFollowing.getPowerS(), anglePID.getPower(), odometry.getRobotAngle());
      
      spampleArm.updateArm(true);
      telemetry.addLine("AutoMovingTO");
      telemetry.addData("X",odometry.getRobotX());
      telemetry.addData("Y", odometry.getRobotY());
      telemetry.addData("Angle", odometry.getRobotAngle());
      telemetry.update();
    }
  }
  
  public double getDeltaTime() {
    double deltaTime;
    deltaTime = currentLoopTime - previousLoopTime;
    return deltaTime;
  }

  public void updateLoopTime() {
    previousLoopTime = currentLoopTime;
    currentLoopTime = runtime.seconds();

  }

  public void IMUReset() {
    odometry.overridePosition(odometry.getRobotX(),odometry.getRobotY(),0);
    anglePID.setTarget(0);
  }
  
 
  
  // This is the equation used to convert from radians to degrees
  public double radiansToDegrees(double radians) {
    return radians * (180 / Math.PI);
  }
  
  // This is the equation used to convert from degrees to radians
  public double degreesToRadians(double degrees) {
    return degrees * (Math.PI / 180);
  }
  
  public void turnUpdate() {
    if (resettingImu) {
      anglePID.setTarget(odometry.getRobotAngle());
      anglePID.update(odometry.getRobotAngle());
      return;
    }
  }

  
    public double distanceCircle(double x, double y){
      return (Math.sqrt((x - odometry.getRobotX()) * (x - odometry.getRobotX()) + (y - odometry.getRobotY()) * (y - odometry.getRobotY())));
    }


    public void updateRobot(boolean holdPosition, boolean autoSpeedChange, boolean isAuto){

    }

    public void chill(double seconds, boolean holdPosition){
    double startedTime = runtime.seconds();
      while (runtime.seconds() - startedTime < seconds){
        // run things robot specific
      }
    }

    public void setRobotSpeed(Speed robotSpeed){
    switch(robotSpeed) {
      case SLOW:
        driveTrain.setSpeedMultiplier(SPEED_MULTIPLIER_SLOW);
        pathFollowing.setAutoSpeed(P_CONSTANT_SLOW, I_CONSTANT_SLOW, D_CONSTANT_SLOW);
        break;
      case FAST:
        driveTrain.setSpeedMultiplier(SPEED_MULTIPLIER_FAST);
        pathFollowing.setAutoSpeed(P_CONSTANT_FAST, I_CONSTANT_FAST, D_CONSTANT_FAST);
        break;
    }
    }
    public void setTurnPIntake(boolean intakeOn){
      if (intakeOn) {
        anglePID.updateConstants(pConIntake, 0, dCon);
      }else{
        anglePID.updateConstants(pCon, 0, dCon);
      }

    }
  }
