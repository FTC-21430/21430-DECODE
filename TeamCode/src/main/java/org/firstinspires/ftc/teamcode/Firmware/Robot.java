package org.firstinspires.ftc.teamcode.Firmware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Firmware.Systems.GobildaPinpointModuleFirmware;
import org.firstinspires.ftc.teamcode.Resources.PIDController;
import org.firstinspires.ftc.teamcode.Resources.PathFollowing;

import org.firstinspires.ftc.teamcode.Firmware.Systems.MecanumDriveTrain;

//TODO - Refactor this class to be season non-specific and move all specific details to the DecodeBot class and override information in this class.

@Config
public class Robot {
  
  public enum Speed {
    SLOW, FAST
  }

  // TODO: Refoctar this part of the code to no longer have last season specific constants (lines 27-30) - Tobin 10/11/2025
  // used for how fast the turning input is used.
  // the number for maxTurnDegPerSecond is how much the robot can turn for one degree
  public static double maxTurnDegPerSecond = 280;
  //TODO: Find values for these to tune PID constants
  //TODO: Also make final once values have been found
  public static double pCon;
  public static double dCon;
  
  private double drive;
  private double slide;
  private double turn;
  public MecanumDriveTrain driveTrain;
  public ElapsedTime runtime = new ElapsedTime();
  public PIDController anglePID = new PIDController(pCon, 0, dCon, runtime);
  public FtcDashboard ftcDashboard;

  public Telemetry telemetry;
  public LinearOpMode opMode;



  private double currentLoopTime, previousLoopTime;
  public PathFollowing pathFollowing;
  public BulkSensorBucket bulkSensorBucket = null;

  public GobildaPinpointModuleFirmware odometry;

  public void init(HardwareMap hardwareMap, Telemetry telemetry, double robotX, double robotY, double robotAngle, LinearOpMode opMode, boolean reset, boolean isAuto) {

    this.opMode = opMode;

    this.telemetry = telemetry;

    odometry = new GobildaPinpointModuleFirmware(hardwareMap, 0,0,reset);

    bulkSensorBucket = new BulkSensorBucket(hardwareMap);
    
    driveTrain = new MecanumDriveTrain(hardwareMap, telemetry);

    //pathFollowing = new PathFollowing(P_CONSTANT, P_CONSTANT, I_CONSTANT, I_CONSTANT, D_CONSTANT, D_CONSTANT, runtime);
    
     bulkSensorBucket.clearCache();

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



  
    public double distanceCircle(double x, double y){
      return 0;
  }


    //ToDo: finish coding updateRobot
  public void updateRobot(boolean holdPosition, boolean autoSpeedChange, boolean isAuto){

    }

    public void chill(double seconds, boolean holdPosition){
    double startedTime = runtime.seconds();
      while (runtime.seconds() - startedTime < seconds){
        // run things robot specific
      }
    }


  }
