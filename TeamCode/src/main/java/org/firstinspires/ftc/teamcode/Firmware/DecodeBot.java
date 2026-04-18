package org.firstinspires.ftc.teamcode.Firmware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Firmware.Systems.AprilTag;
import org.firstinspires.ftc.teamcode.Firmware.Systems.GobildaPinpointModuleFirmware;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Intake;
import org.firstinspires.ftc.teamcode.Firmware.Systems.LED;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Launcher;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Lifter;
import org.firstinspires.ftc.teamcode.Firmware.Systems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Spindexer;
import org.firstinspires.ftc.teamcode.Resources.PathFollowing;
import org.firstinspires.ftc.teamcode.Resources.RotationControl;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.SWEEP;
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
    public LED led = null;
    //Setting the alliance
    //TODO: This is only used in decode bot, should we make private?
    public String alliance = "red";

  
    public OperatorStateMachine operatorStateMachine = null;
    //The PID values are a public because we need to tune it later and public makes it easier to do that
    public static final double P_CONSTANT = 0.14;
    public static final double I_CONSTANT = 0.11;
    public static final double D_CONSTANT = 0.031;
    public static double P_ANGLE = 0.021;
    public static double I_ANGLE = 0.0005;
    public static double D_ANGLE = 0.0004;
    public static double yOffset = 2.78;
    public static double xOffset = 4.9574;
    public static long cameraExposure = 10;
    private boolean isAuto;
    public static double closeSpeed = 1200;
    public static double midSpeed = 1400;
    public static double farSpeed = 1750;
    public static double closeRamp = 56;
    public static double midRamp = 55;
    public static double farRamp = 52;

    public static double aprilTagUpdateSpeed = 2;

    // These odometry pod position value are just for the software testing bot

    public static double SWEEP_P = 0.20;
    public static double SWEEP_I = 0.0;
    public static double SWEEP_D = 0.02;
    private boolean SWEEPAimingAtGoal = false;

    public int motifId = 0;


    public DecodeBot(HardwareMap hardwareMap, Telemetry telemetry, double robotX, double robotY, double robotAngle, LinearOpMode opMode, boolean resetSpindexer, boolean resetOdemetry, boolean isAuto, String alliance, Gamepad gamepad2){
        pathFollowing = new PathFollowing(P_CONSTANT, P_CONSTANT, I_CONSTANT, I_CONSTANT, D_CONSTANT, D_CONSTANT, runtime);
        this.opMode = opMode;
        this.telemetry = telemetry;
        this.alliance = alliance;
        this.isAuto = isAuto;

        //Creating the classes as objects for future use
        odometry = new GobildaPinpointModuleFirmware(hardwareMap, telemetry,xOffset,yOffset,resetOdemetry);
        trajectoryKinematics = new TrajectoryKinematics(isAuto, telemetry);
        bulkSensorBucket = new BulkSensorBucket(hardwareMap);
        driveTrain = new MecanumDriveTrain(hardwareMap, telemetry, this.alliance);
        launcher = new Launcher(hardwareMap,telemetry, trajectoryKinematics);
        intake = new Intake(hardwareMap, telemetry);
        spindexer = new Spindexer(hardwareMap,telemetry,resetSpindexer,isAuto);
        lifter = new Lifter(hardwareMap, telemetry);
        rotationControl = new RotationControl(0.3,P_ANGLE,I_ANGLE,D_ANGLE,robotAngle,telemetry);
        aprilTags = new AprilTag();
        led = new LED(hardwareMap);

        aprilTags.init(hardwareMap,telemetry,cameraExposure);
        bulkSensorBucket.clearCache();
        this.SWEEP = new SWEEP(this, 0.9, SWEEP_P,SWEEP_I,SWEEP_D);
        // for the last parameter of the operatorStateMachine Constructor, note that this:: means to provide a runnable reference as the value. This way, The operator state machine can run the function without needing to 'have' a DecodeBot,
        // which would completely break the intended structure of our repository.
        operatorStateMachine = new OperatorStateMachine(launcher,spindexer,intake,telemetry,this::setLauncherBasedOnTags,gamepad2, trajectoryKinematics, this);
    }

    //the function used to move to a spot on the field during auto
    public void autoMoveTo(double targetX, double targetY, double robotAngle, double targetCircle){
        pathFollowing.setTargetPosition(targetX,targetY);
        pathFollowing.setFollowTolerance(targetCircle);
        rotationControl.setTargetAngle(robotAngle);
        driveTrain.fieldCentricDriving(false);
        while(!pathFollowing.isWithinTargetTolerance(odometry.getRobotX(),odometry.getRobotY())&&opMode.opModeIsActive()){
            odometry.updateOdometry();
            launcher.revFlywheel();
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
        if (shouldScan){
            motifId = aprilTags.getMotifID();
        }
        intake.updateIntake();
        lifter.update();
        aprilTags.clearCache();
        led.setLed(spindexer.getNumberOfArtifacts());
        led.setAllianceColor(alliance);
        led.update();
        if (motifId != 0) {
            telemetry.addData("Motif ID", motifId);
        }
    }

    // red or blue
    public void setAlliance(String alliance){
        this.alliance = alliance;
        driveTrain.setAlliance(alliance);
    }

    public void updateTrajectories(){
        trajectoryKinematics.updateVelocities(odometry.getVelocityX(),odometry.getVelocityY());
        trajectoryKinematics.calculateTrajectory(trajectoryKinematics.getDistance(alliance, odometry.getRobotX(),odometry.getRobotY()), launcher.getFlywheelError());
    }
    public void updateOdometryOnTags(boolean hardUpdate){
        odometry.updateOdometry();
        double velocity = Math.hypot(odometry.getVelocityX(), odometry.getVelocityY());
        if (velocity > aprilTagUpdateSpeed && !hardUpdate) {
            return;
        }
        if (aprilTags.updateAprilValues(odometry.getRobotX(),odometry.getRobotY(),odometry.getRobotAngle(),hardUpdate,alliance)){
            odometry.overridePosition(aprilTags.getRobotX(), aprilTags.getRobotY(), aprilTags.getRobotAngle());
            rotationControl.setTargetAngle(aprilTags.getRobotAngle()-aprilTags.getRotationError());
        }
    }
    public void aimAtGoal(){
        trajectoryKinematics.updateVelocities(odometry.getVelocityX(),odometry.getVelocityY());
        double bearingToGoal = trajectoryKinematics.getBearingToTag(alliance, isAuto, odometry.getRobotX(),odometry.getRobotY());
        rotationControl.setTargetAngle(bearingToGoal);
    }
    public void setLauncherBasedOnTags(){
        trajectoryKinematics.updateVelocities(odometry.getVelocityX(),odometry.getVelocityY());
        double distanceToGoal = trajectoryKinematics.getDistance(alliance,odometry.getRobotX(),odometry.getRobotY());

        trajectoryKinematics.calculateTrajectory(distanceToGoal, launcher.getFlywheelError());
        launcher.setLaunchAngle(trajectoryKinematics.getInitialAngle());
        launcher.revFlywheel();
    }
    private boolean shouldScan = false;
    public void scanMotif(){
        shouldScan = !shouldScan;

    }
    public static double parkPosX = 28;
    public static double parkPosY = -40;
    public void park(){
        pathFollowing.setAutoConstants(P_CONSTANT+0.3,I_CONSTANT,D_CONSTANT);
        double angle = 90;
        double tempParkPosX = parkPosX;
        double tempParkPosY = parkPosY;
        switch (alliance){
            case "red":
                tempParkPosY *= -1;
                angle *= -1;
                break;
            case "blue":
                tempParkPosY *= 1;
                angle *= 1;
                break;
        }
        driveTrain.fieldCentricDriving(false);
        pathFollowing.setTargetPosition(tempParkPosX,tempParkPosY);
        rotationControl.setTargetAngle(angle);
        pathFollowing.followPath(odometry.getRobotX(),odometry.getRobotY(),odometry.getRobotAngle());
        driveTrain.setDrivePower(pathFollowing.getPowerS(),pathFollowing.getPowerF(),rotationControl.getOutputPower(odometry.getRobotAngle()),odometry.getRobotAngle());
        driveTrain.fieldCentricDriving(true);
    }



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

    /**
     * Optional support method for the SWEEP Library to choose to aim at the goal. Use robot actions to say you want the aiming override, you can either disable it or enable it.
     */
    public void SWEEPAiming(){
        if (SWEEPAimingAtGoal){
            aimAtGoal();
        }
    }
    public boolean shouldSWEEPAimAtGoal(){
        return SWEEPAimingAtGoal;
    }

    /**
     * Set whether or not SWEEP movement should have the robot yaw locked onto the Goal's position. REMEMBER TO SET ALLIANCE!!!!
     * @param aiming true for aiming
     */
    public void setSweepAimingAtGoal(boolean aiming){
        SWEEPAimingAtGoal = aiming;
    }
}