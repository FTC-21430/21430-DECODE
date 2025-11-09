package org.firstinspires.ftc.teamcode.Firmware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Firmware.Systems.GobildaPinpointModuleFirmware;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Launcher;
import org.firstinspires.ftc.teamcode.Firmware.Systems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Spindexer;
import org.firstinspires.ftc.teamcode.Resources.PathFollowing;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerServoFirmware;
import org.firstinspires.ftc.teamcode.Resources.RotationControl;
import org.firstinspires.ftc.teamcode.Resources.TrajectoryKinematics;

@Config
public class DecodeBot extends Robot{

//    TODO this value must be found through testing as be do not know the constant of kinetic friction between artifacts and the flywheel.
    // the conversion ratio of the speed of the ball's movement to the robots flywheel speed
    public static double velocityMetersToDegrees = 0.03;
    public Launcher launcher = null;
    public Spindexer spindexer = null;


    public TrajectoryKinematics trajectoryKinematics;

    public String alliance = "red";
    //The PID values are a public because we need to tune it later and public makes it easier to do that
    public static final double P_CONSTANT = 0.15;
    public static final double I_CONSTANT = 0.1;
    public static final double D_CONSTANT = 0.02;
    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry, double robotX, double robotY, double robotAngle, LinearOpMode opMode, boolean reset, boolean isAuto){
        pathFollowing = new PathFollowing(P_CONSTANT, P_CONSTANT, I_CONSTANT, I_CONSTANT, D_CONSTANT, D_CONSTANT, runtime);
        this.opMode = opMode;

        this.telemetry = telemetry;

        this.alliance = alliance;
        // TODO: change the pod offset values to what they are on the competition robot, currently tuned for software testing bot
        odometry = new GobildaPinpointModuleFirmware(hardwareMap, 7.2,-18,reset);

        trajectoryKinematics = new TrajectoryKinematics(40,30);
        bulkSensorBucket = new BulkSensorBucket(hardwareMap);

        driveTrain = new MecanumDriveTrain(hardwareMap, telemetry);
        launcher = new Launcher(hardwareMap,telemetry);

        spindexer = new Spindexer(hardwareMap,telemetry);
        spindexer = new Spindexer(hardwareMap);
        rotationControl = new RotationControl(300,0.025,0,0.0001,robotAngle);

        bulkSensorBucket.clearCache();



    }
    @Override
    //TODO:Call updates for sensors and actuators
    public void updateRobot(boolean holdPosition, boolean autoSpeedChange, boolean isAuto){
        spindexer.updateSpindexer();
        launcher.updateSpeedControl();
    }

    public void aimBasedOnTags(){
        // TODO: replace the 0.0 to the output of april tags
        trajectoryKinematics.calculateTrajectory(0.0);
        launcher.setSpeed(trajectoryKinematics.getLaunchMagnitude() * velocityMetersToDegrees);

    }
}
