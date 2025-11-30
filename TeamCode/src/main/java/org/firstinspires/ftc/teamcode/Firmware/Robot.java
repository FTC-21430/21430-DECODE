package org.firstinspires.ftc.teamcode.Firmware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Firmware.Systems.GobildaPinpointModuleFirmware;
import org.firstinspires.ftc.teamcode.Resources.PathFollowing;
import org.firstinspires.ftc.teamcode.Firmware.Systems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Resources.RotationControl;

//TODO - Refactor this class to be season non-specific and move all specific details to the DecodeBot class and override information in this class.

@Config
public abstract class Robot {
    public enum Speed {
        SLOW, FAST
    }

    // TODO: Refactor this part of the code to no longer have last season specific constants (lines 27-30) - Tobin 10/11/2025
    // used for how fast the turning input is used.
    // the number for maxTurnDegPerSecond is how much the robot can turn for one degree
    public static double maxTurnDegPerSecond = 280;

    //TODO: Find values for these to tune PID constants
    //TODO: Also make final and private once values have been found
    //PID constants
    public static double pCon;
    public static double dCon;

    //TODO: Comment out the purpose of these vars.
    //BC no usages, I cannot look and see usages for ideas of how used
    private double drive;
    private double slide;
    private double turn;

    //Linking classes
    public MecanumDriveTrain driveTrain;
    public GobildaPinpointModuleFirmware odometry;
    public RotationControl rotationControl;
    public FtcDashboard ftcDashboard;
    public Telemetry telemetry;
    public LinearOpMode opMode;
    public PathFollowing pathFollowing;
    public ElapsedTime runtime = new ElapsedTime();
    public BulkSensorBucket bulkSensorBucket = null;

    public boolean aiming = false;
    private double currentLoopTime, previousLoopTime;

  // TODO: Refactor this part of the code to no longer have last season specific constants (lines 27-30) - Tobin 10/11/2025
  // used for how fast the turning input is used.
  // the number for maxTurnDegPerSecond is how much the robot can turn for one degree
  public static double maxTurnDegPerSecond = 280;
  //TODO: Find values for these to tune PID constants
  //TODO: Also make final once values have been found
    //Why are these public? Needed for FTC dashboard?
  public static double pCon;
  public static double dCon;
  private double drive;
  private double slide;
  private double turn;
  public MecanumDriveTrain driveTrain;
  public ElapsedTime runtime = new ElapsedTime();
  public RotationControl rotationControl;
  public FtcDashboard ftcDashboard;
  public Telemetry telemetry;
  public LinearOpMode opMode;
  public boolean aiming = false;
  private double currentLoopTime, previousLoopTime;
  public PathFollowing pathFollowing;
  public BulkSensorBucket bulkSensorBucket = null;
  public GobildaPinpointModuleFirmware odometry;
  
  // you call this function in a main auto opMode to make the robot move somewhere.
  // This is the foundation that every robot should need but you should more season specific things in the bot class.
  
  public double getDeltaTime() {
    double deltaTime;
    deltaTime = currentLoopTime - previousLoopTime;
    return deltaTime;
  }

    public void updateLoopTime() {
        previousLoopTime = currentLoopTime;
        currentLoopTime = runtime.seconds();
    }

    public void updateRobot(boolean holdPosition, boolean autoSpeedChange, boolean isAuto) {
    }

    //The initialization function
    public abstract void init(HardwareMap hardwareMap, Telemetry telemetry, double robotX, double robotY, double robotAngle, LinearOpMode opMode, boolean reset, boolean isAuto, String alliance);

    public void chill(boolean holdPos, double timeout) {
        double startedTime = runtime.seconds();
        while (runtime.seconds() - startedTime < timeout) {
            // run things robot specific
        }
    }
}