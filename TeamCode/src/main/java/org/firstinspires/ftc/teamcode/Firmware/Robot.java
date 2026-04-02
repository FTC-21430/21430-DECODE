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
import org.firstinspires.ftc.teamcode.Resources.SWEEP.SWEEP;

//TODO - Refactor this class to be season non-specific and move all specific details to the DecodeBot class and override information in this class.

@Config
public abstract class Robot {
    //Linking common classes
    public MecanumDriveTrain driveTrain;
    public GobildaPinpointModuleFirmware odometry;
    public RotationControl rotationControl;
    public FtcDashboard ftcDashboard;
    public Telemetry telemetry;
    public LinearOpMode opMode;
    public PathFollowing pathFollowing;
    public SWEEP SWEEP; // Spline-based Waypoint Execution Engine for Path-following XD
    public ElapsedTime runtime = new ElapsedTime();
    public BulkSensorBucket bulkSensorBucket = null;
    private double currentLoopTime, previousLoopTime;

  
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
    public void init(HardwareMap hardwareMap, Telemetry telemetry, double robotX, double robotY, double robotAngle, LinearOpMode opMode, boolean reset, boolean isAuto, String alliance){

    }

    public void chill(boolean holdPos, double timeout) {
        double startedTime = runtime.seconds();
        while (runtime.seconds() - startedTime < timeout) {
            // run things robot specific
        }
    }
}