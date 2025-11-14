package org.firstinspires.ftc.teamcode.Firmware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Firmware.Systems.GobildaPinpointModuleFirmware;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Intake;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Launcher;
import org.firstinspires.ftc.teamcode.Firmware.Systems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Spindexer;
import org.firstinspires.ftc.teamcode.Resources.PathFollowing;
import org.firstinspires.ftc.teamcode.Resources.RotationControl;
import org.firstinspires.ftc.teamcode.Resources.TrajectoryKinematics;

@Config
public class DecodeBot extends Robot{

//    TODO this value must be found through testing as be do not know the constant of kinetic friction between artifacts and the flywheel.
    // the conversion ratio of the speed of the ball's movement to the robots flywheel speed
    public static double velocityMetersToDegrees = 0.03;
    public Launcher launcher = null;
    public Spindexer spindexer = null;
    public Intake intake = null;
  


    public TrajectoryKinematics trajectoryKinematics;

    public String alliance = "red";
    //The PID values are a public because we need to tune it later and public makes it easier to do that
    public static final double P_CONSTANT = 0.15;
    public static final double I_CONSTANT = 0.1;
    public static final double D_CONSTANT = 0.02;

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry, double robotX, double robotY, double robotAngle, LinearOpMode opMode, boolean reset, boolean isAuto,String alliance){
        pathFollowing = new PathFollowing(P_CONSTANT, P_CONSTANT, I_CONSTANT, I_CONSTANT, D_CONSTANT, D_CONSTANT, runtime);
        this.opMode = opMode;

        this.telemetry = telemetry;

        this.alliance = alliance;
        // TODO: change the pod offset values to what they are on the competition robot, currently tuned for software testing bot
        odometry = new GobildaPinpointModuleFirmware(hardwareMap, -14,-6.893,reset);

        trajectoryKinematics = new TrajectoryKinematics(40,30);
        bulkSensorBucket = new BulkSensorBucket(hardwareMap);

        driveTrain = new MecanumDriveTrain(hardwareMap, telemetry);
        launcher = new Launcher(hardwareMap,telemetry);
        intake = new Intake(hardwareMap, telemetry);

        spindexer = new Spindexer(hardwareMap,telemetry);
        rotationControl = new RotationControl(0.3,0.025,0,0.0001,robotAngle);
//



        bulkSensorBucket.clearCache();





    }
    @Override
    //TODO:Call updates for sensors and actuators
    public void updateRobot(boolean holdPosition, boolean autoSpeedChange, boolean isAuto){
        spindexer.updateSpindexer();
        launcher.updateSpeedControl();
    }

    // red or blue
    public void setAlliance(String alliance){
        this.alliance = alliance;
    }
    public void aimBasedOnTags(){
        // TODO: replace the 68.0 values to the output of april tags (these are temp values that mark a general distance from the goal around (0,0)

        // but don't change this one
        double distanceToGoal = 68.0;

        switch (alliance){
            case "red":
                distanceToGoal = 68.0;
                break;
            case "blue":
                distanceToGoal = 68.0;
                break;
        }
//        trajectoryKinematics.calculateTrajectory(distanceToGoal);
//        launcher.setSpeed(trajectoryKinematics.getLaunchMagnitude() * velocityMetersToDegrees);
//        launcher.setLaunchAngle(trajectoryKinematics.getInitialAngle());
    }



    public static double closeSpeed = 1200;
    public static double midSpeed = 1400;
    public static double farSpeed = 1750;
    public static double closeRamp = 56;
    public static double midRamp = 55;
    public static double farRamp = 52;



    /**
     *
     * @param distance can be: "close" or "mid" or "far"
     *
     */
    public void launchFrom(String distance){
        switch (distance){
            case "close":
                launcher.setSpeed(closeSpeed);
                launcher.setLaunchAngle(closeRamp);
                break;
            case "mid":
                launcher.setSpeed(midSpeed);
                launcher.setLaunchAngle(midRamp);
                break;
            case "far":
                launcher.setSpeed(farSpeed);
                launcher.setLaunchAngle(farRamp);
                break;
        }
    }
}
