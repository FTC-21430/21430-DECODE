package org.firstinspires.ftc.teamcode.Firmware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Resources.PIDController;
import org.firstinspires.ftc.teamcode.Resources.PathFollowing;

import org.firstinspires.ftc.teamcode.Firmware.Systems.MecanumDriveTrain;

//TODO - Refactor this class to be season non-specific and move all specific details to the DecodeBot class and override information in this class.
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
  public ElapsedTime runtime = new ElapsedTime();
  //TODO Tune the pConstant and d Constant numbers, these are place holders.
  public PIDController anglePID = new PIDController(pCon, 0, dCon, runtime);

  public Telemetry telemetry;
  public LinearOpMode opMode;
  public double P_CONSTANT, I_CONSTANT, D_CONSTANT;


  private double currentLoopTime, previousLoopTime;
  public PathFollowing pathFollowing;
  public void init(HardwareMap hardwareMap, Telemetry telemetry, double robotX, double robotY, double robotAngle, LinearOpMode opMpde, boolean reset, boolean isAuto) {

    this.opMode = opMpde;

    this.telemetry = telemetry;

    driveTrain = new MecanumDriveTrain(hardwareMap, telemetry);

    pathFollowing = new PathFollowing(P_CONSTANT, P_CONSTANT, I_CONSTANT, I_CONSTANT, D_CONSTANT, D_CONSTANT, runtime);
  }
  
  // you call this function in a main auto opMode to make the robot move somewhere.
  // This is the foundation that every robot should need but you should more season specific things in the bot class.
  public void autoMoveTo(double targetX, double targetY, double robotAngle, double targetCircle, double Timeout) {
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

  }

  
  public void turnUpdate() {

  }

  
    public double distanceCircle(double x, double y){
      return 0
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
