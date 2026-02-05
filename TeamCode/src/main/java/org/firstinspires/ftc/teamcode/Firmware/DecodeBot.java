package org.firstinspires.ftc.teamcode.Firmware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Firmware.Systems.AprilTag;
import org.firstinspires.ftc.teamcode.Firmware.Systems.GobildaPinpointModuleFirmware;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Intake;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Launcher;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Lifter;
import org.firstinspires.ftc.teamcode.Firmware.Systems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Spindexer;
import org.firstinspires.ftc.teamcode.Resources.PathFollowing;
import org.firstinspires.ftc.teamcode.Resources.RotationControl;
import org.firstinspires.ftc.teamcode.Resources.TrajectoryKinematics;

@Config
public abstract class DecodeBot extends Robot{

//TODO this value must be found through testing as be do not know the constant of kinetic friction between artifacts and the flywheel.
    // the conversion ratio of the speed of the ball's movement to the robots flywheel speed
    public static double velocityMetersToDegrees = 0.03;
    //Connecting necessary classes for decode bot's functions
    public Launcher launcher = null;
    public Spindexer spindexer = null;
    public Intake intake = null;
    public AprilTag aprilTags = null;
    public TrajectoryKinematics trajectoryKinematics;
    public Lifter lifter = null;
    //Setting the alliance
    //TODO: This is only used in decode bot, should we make private?
    public String alliance = "red";

  
    public OperatorStateMachine operatorStateMachine = null;
    //The PID values are a public because we need to tune it later and public makes it easier to do that
    public static final double P_CONSTANT = 0.14;
    public static final double I_CONSTANT = 0.11;
    public static final double D_CONSTANT = 0.031;
    public static double P_ANGLE = 0.035;
    public static double I_ANGLE = 0.0005;
    public static double D_ANGLE = 0.0001;
    public static double yOffset = 2.78;
    public static double xOffset = 4.9574;
    public static long cameraExposure = 10;
    private boolean isAuto;

//    public static double xOffset = -3.125;
//    public static double yOffset = -7;

    public DecodeBot(HardwareMap hardwareMap, Telemetry telemetry, double robotX, double robotY, double robotAngle, LinearOpMode opMode, boolean reset, boolean isAuto, String alliance, Gamepad gamepad2){
        pathFollowing = new PathFollowing(P_CONSTANT, P_CONSTANT, I_CONSTANT, I_CONSTANT, D_CONSTANT, D_CONSTANT, runtime);
        this.opMode = opMode;
        this.telemetry = telemetry;
        this.alliance = alliance;
        this.isAuto = isAuto;

        //Creating the classes as objects for future use
        odometry = new GobildaPinpointModuleFirmware(hardwareMap, xOffset,yOffset,reset);
        trajectoryKinematics = new TrajectoryKinematics(isAuto);
        bulkSensorBucket = new BulkSensorBucket(hardwareMap);
        driveTrain = new MecanumDriveTrain(hardwareMap, telemetry, this.alliance);
        launcher = new Launcher(hardwareMap,telemetry, trajectoryKinematics);
        intake = new Intake(hardwareMap, telemetry);
        spindexer = new Spindexer(hardwareMap,telemetry,reset,isAuto);
//        lifter = new Lifter(hardwareMap, telemetry);
    rotationControl = new RotationControl(0.3,P_ANGLE,I_ANGLE,D_ANGLE,robotAngle,telemetry);
        aprilTags = new AprilTag();

        aprilTags.init(hardwareMap,telemetry,cameraExposure);
        bulkSensorBucket.clearCache();
        // for the last parameter of the operatorStateMachine Constructor, note that this:: means to provide a runnable reference as the value. This way, The operator state machine can run the function without needing to 'have' a DecodeBot,
        // which would completely break the intended structure of our repository.
        operatorStateMachine = new OperatorStateMachine(launcher,spindexer,intake,telemetry,this::setLauncherBasedOnTags,gamepad2, trajectoryKinematics);
    }

    //the function used to move to a spot on the field during auto
    public void autoMoveTo(double targetX, double targetY, double robotAngle, double targetCircle){
        pathFollowing.setTargetPosition(targetX,targetY);
        pathFollowing.setFollowTolerance(targetCircle);
        rotationControl.setTargetAngle(robotAngle);
        driveTrain.fieldCentricDriving(false);
        while(!pathFollowing.isWithinTargetTolerance(odometry.getRobotX(),odometry.getRobotY())&&opMode.opModeIsActive()){
            updateRobot(false,false,false);
            pathFollowing.followPath(odometry.getRobotX(),odometry.getRobotY(),odometry.getRobotAngle());
            operatorStateMachine.updateStateMachine();
            driveTrain.setDrivePower(pathFollowing.getPowerS(),pathFollowing.getPowerF(),rotationControl.getOutputPower(odometry.getRobotAngle()),odometry.getRobotAngle());
            telemetry.update();
            bulkSensorBucket.clearCache();

        }
    }
    //The function that stops the robot, bc robot.stop is something diff
    @Override
    public void chill(boolean holdPos, double timeout){
        double startTime = runtime.seconds();
        while (runtime.seconds() < startTime + timeout && opMode.opModeIsActive()){
            updateRobot(false,false,false);
            if (holdPos){
                pathFollowing.followPath(odometry.getRobotX(),odometry.getRobotY(),odometry.getRobotAngle());
                driveTrain.setDrivePower(pathFollowing.getPowerS(),pathFollowing.getPowerF(),rotationControl.getOutputPower(odometry.getRobotAngle()),odometry.getRobotAngle());

            }
            operatorStateMachine.updateStateMachine();
            telemetry.update();
            bulkSensorBucket.clearCache();
        }
    }

    @Override
    //TODO:Call updates for sensors and actuators
    //Updates all necessary classes together to compact code in teleop/auto
    public void updateRobot(boolean holdPosition, boolean autoSpeedChange, boolean isAuto){
        intake.updateIntake();
        odometry.updateOdometry();
//        operatorStateMachine.updateStateMachine();
        aprilTags.clearCache();
    }

    // red or blue
    public void setAlliance(String alliance){
        this.alliance = alliance;
        driveTrain.setAlliance(alliance);
    }

    public void updateOdometryOnTags(boolean hardUpdate){
        if (aprilTags.updateAprilValues(odometry.getRobotX(),odometry.getRobotY(),odometry.getRobotAngle(),hardUpdate,alliance)){
            odometry.overridePosition(aprilTags.getRobotX(), aprilTags.getRobotY(), aprilTags.getRobotAngle());
            rotationControl.setTargetAngle(aprilTags.getRobotAngle()-aprilTags.getRotationError());
        }
    }
    public void aimAtGoal(){
        double bearingToGoal = trajectoryKinematics.getBearingToTag(alliance, isAuto, odometry.getRobotX(),odometry.getRobotY());
        rotationControl.setTargetAngle(bearingToGoal);
    }
    public void setLauncherBasedOnTags(){
        double distanceToGoal = aprilTags.getDistance(alliance,odometry.getRobotX(), odometry.getRobotY());
        telemetry.addData("alliance", alliance);
        telemetry.addData("distance", distanceToGoal);
        trajectoryKinematics.calculateTrajectory(distanceToGoal);
        launcher.setLaunchAngle(trajectoryKinematics.getInitialAngle());
        launcher.setSpeed(trajectoryKinematics.getLaunchMagnitude());
    }

    public static double closeSpeed = 1200;
    public static double midSpeed = 1400;
    public static double farSpeed = 1750;
    public static double closeRamp = 56;
    public static double midRamp = 55;
    public static double farRamp = 52;

    /**
     * @param distance - String input can be "close", "mid", or "far"
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