package org.firstinspires.ftc.teamcode.Firmware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Intake;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Launcher;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Spindexer;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor.COLORS;
import org.firstinspires.ftc.teamcode.Resources.TrajectoryKinematics;

import java.util.ArrayList;
import java.util.List;

@Config
// Controls the automation for the mechanisms on the robot. Mainly, the launcher, spindexer, and intake.
public class OperatorStateMachine {

    // All states the operator is at
    public enum State {
        IDLE,
        PREPPING,
        LAUNCH,
        INTAKE

    }

    // The idle speed the flywheel will be set too when not actively shooting
    // The launcher instance
    private Launcher launcher;
    // The Spindexer Instance
    private Spindexer spindexer;
    // The Intake Instance
    private Intake intake;
    // Telemetry Instance from main op-mode
    private Telemetry telemetry;
    // The Decodebot class is what creates this class, but passes itself alongside the other classes so that we can access its aiming functions
    private Runnable setLauncherBasedOnTags;
    // The current state of the robot
    private State currentState = State.IDLE;

    // A queue that should hold up to three colors that we will shoot. in this case, Purple will launch Purple, Green will launch Green, and NONE will just shoot what ever is next
    private List<COLORS> launchQueue = new ArrayList<>();
    public static double sortingTimeout = 0.45;
    private Gamepad gamepad2 = null;
    private ElapsedTime launchTimer = null;
    private ElapsedTime preppingTimer = null;
    private TrajectoryKinematics trajectoryKinematics = null;
    private String alliance;
    private DecodeBot bot;


    // Will Trigger the transition from one state to the next

    /**
     * The constructor for this class, Stores all of the instances of the components of the robot
     * Params are self explanatory
     * @param launcher
     * @param spindexer
     * @param intake
     * @param telemetry
     * @param setLauncherBasedOnTags - One function we need from DecodeBot.java but this is just a refernce to call method, not anything else in DecodeBot!
     */
    public OperatorStateMachine(Launcher launcher, Spindexer spindexer, Intake intake, Telemetry telemetry, Runnable setLauncherBasedOnTags, Gamepad gamepad2, TrajectoryKinematics trajectoryKinematics, DecodeBot bot){
        this.launcher = launcher;
        this.spindexer = spindexer;
        this.intake = intake;
        this.telemetry = telemetry;
        this.setLauncherBasedOnTags = setLauncherBasedOnTags;
        this.gamepad2 = gamepad2;
        this.launchTimer = new ElapsedTime();
        this.preppingTimer = new ElapsedTime();
        this.alliance = alliance;
        this.trajectoryKinematics = trajectoryKinematics;
        this.bot = bot;
        addToQueue(COLORS.NONE);
        addToQueue(COLORS.NONE);
        addToQueue(COLORS.NONE);
    }
    public void moveToState(State state){
        State lastState = currentState;
        currentState = state;
        switch (lastState){
            case IDLE:
                switch (state){
                    case IDLE:
                        break;
                    case INTAKE:
                        idleToIntake();
                        break;
                    case PREPPING:
                        idleToPrepping();
                        break;
                    case LAUNCH:
                        idleToLaunch();
                        break;
                }
                break;

            case PREPPING:
                switch (state){
                    case IDLE:
                        preppingToIdle();
                        break;
                    case INTAKE:
                        preppingToIntake();
                        break;
                    case LAUNCH:
                        preppingToLaunch();
                        break;
                    case PREPPING:
                        refreshPrep();
                        break;
                }
                break;

            case LAUNCH:
                switch (state){
                    case IDLE:
                        launchToIdle();
                        break;
                    case INTAKE:
                        launchToIntake();
                        break;
                    case PREPPING:
                        launchToPrepping();
                        break;
                    case LAUNCH:
                        break;
                }
                break;

            case INTAKE:
                switch (state){
                    case IDLE:
                        intakeToIdle();
                        break;
                    case INTAKE:
                        break;
                    case PREPPING:
                        intakeToPrepping();
                        break;
                    case LAUNCH:
                        intakeToLaunch();
                        break;
                }
                break;


        }

    }

    // Will call the corresponding update function to the current state
    public void updateStateMachine(){
//        for (COLORS color : spindexer.indexColors){
//            telemetry.addData("A Color", color);
//        }
//        telemetry.addData("spin target", spindexer.getTarget());
        if (gamepad2.left_trigger > 0.4){
            intake.setIntakePower(0.4);
        }
        switch (currentState){

            case IDLE:
                idleState();
                break;
            case INTAKE:
                intakeState();
                break;
            case PREPPING:
                preppingState();
                break;
            case LAUNCH:
                launchState();
                break;

        }
    }

    /**
     * Adds a color to the launching queue. Will clear old values over the limit of 3
     * @param color the color to be added
     */
    public void addToQueue(COLORS color){
        launchQueue.add(color);
        if (launchQueue.size() > 3){
            launchQueue.remove(0);
        }
    }
    public void clearQueue(){
        for (int i = 0; i < 3; i++){
            addToQueue(COLORS.NONE);
        }
    }
    /**
     * The idle state update method
     * retracts and slows down launcher, updates spindexer
     */
    private void idleState(){
        if (!(gamepad2.left_trigger >= 0.4)&&!gamepad2.square){
            intake.setIntakePower(0.3);
        } else if (gamepad2.square) {
            intake.setIntakePower(0);
        }
        launcher.setSpeed(1200);
        launcher.retractRamp();
        launcher.update();
        spindexer.updateSpindexer();
    }

    /**
     * The intake state update method
     * Handles all logic for intaking new artifacts and storing their color
     */


    private int ballSampling = 0;
    private int switchSampling = 0;
    public static int ballSamplingThreshold = 1;
    public static int switchSamplingThreshold = 6;
    private void intakeState (){
        if (!(gamepad2.left_trigger >= 0.4) && !gamepad2.square){
            intake.setIntakePower(-1);
        }else if(gamepad2.square){
            intake.setIntakePower(0);
            launcher.setSpeed(0);
        }

        if (spindexer.getColorInIntake() != COLORS.NONE && spindexer.isAtRest() && (spindexer.getIntakeSwitch()) || ballSampling >= ballSamplingThreshold || switchSampling > switchSamplingThreshold){
            spindexer.storeColorAtIndex();
            spindexer.moveToNextIndex();
            ballSampling = 0;
            switchSampling = 0;
        } else{
            if (spindexer.isAtRest() && spindexer.getColorInIntake() != COLORS.NONE) {
                ++ballSampling;
            }else{
                ballSampling = 0;
            }
            if (spindexer.isAtRest() && spindexer.getIntakeSwitch()){
                ++switchSampling;
            }else{
                switchSampling = 0;
            }
        }

        if (spindexer.isFull() && spindexer.isAtRest()){
            moveToState(State.PREPPING);
        }

        launcher.update();
        spindexer.updateSpindexer();

    }

    public static double preppingTimeout = 0.7;
    private void preppingState(){
        trajectoryKinematics.updateVelocities(bot.odometry.getVelocityX(),bot.odometry.getVelocityY());
        trajectoryKinematics.calculateTrajectory(trajectoryKinematics.getDistance(bot.alliance,bot.odometry.getRobotX(),bot.odometry.getRobotY()), launcher.getFlywheelError());
        launcher.setLaunchAngle(trajectoryKinematics.getInitialAngle());
        launcher.setSpeed(trajectoryKinematics.getLaunchMagnitude());
        launcher.update();
        spindexer.updateSpindexer();
        if (queuedLaunch && preppingTimer.seconds() >= preppingTimeout){
            moveToState(State.LAUNCH);
            queuedLaunch = false;
        }
    }
    private boolean queuedLaunch = false;
    private void prep(){

        preppingTimer.reset();
        while (launchQueue.size() < 3){
            addToQueue(COLORS.NONE);
        }

        COLORS[] launchSequence = new COLORS[3];
        for (int i = 0; i < 3; i++){
            COLORS color = launchQueue.get(i);
            launchSequence[i] = color;
        }

        spindexer.prepLaunch(launchSequence);
    }
    public static double singleLaunchIncrement = 130;
    public static double fullLaunchIncrement = 400;
    public static double launchJamThreshold = 0.01;
    public static int launchJamSampleThresh = 24;
    private int launchJamSampling = 0;
    private int shotsRemaining = 0;
    private boolean launchStalled = false;
    private boolean spinning = false;

    /**
     * The launch state update method
     * Handles the logic for shooting the balls in the right order
     */
    private void launchState(){
        if (!(gamepad2.left_trigger >= 0.4) && !gamepad2.square){
            intake.setIntakePower(-0.1);
        }else if(gamepad2.square){
            intake.setIntakePower(0);
        }
        setLauncherBasedOnTags.run();
        trajectoryKinematics.updateVelocities(bot.odometry.getVelocityX(),bot.odometry.getVelocityY());
        trajectoryKinematics.calculateTrajectory(trajectoryKinematics.getDistance(bot.alliance,bot.odometry.getRobotX(),bot.odometry.getRobotY()), launcher.getFlywheelError());
        launcher.setLaunchAngle(trajectoryKinematics.getInitialAngle());
        launcher.setSpeed(trajectoryKinematics.getLaunchMagnitude());

        // check if we should be sorting our shots
        boolean shouldSort = false;
        for (int i = 0; i < 3; i++){
            if (launchQueue.get(i) != COLORS.NONE){
                shouldSort = true;
            }
        }

        if (spindexer.isAtRest() && shotsRemaining > 0 && !launchStalled && launcher.isUpToSpeed()){
            spinning = false;
            if (shouldSort){
                launchStalled = true;
                launchTimer.reset();

            } else {
                spindexer.eject(fullLaunchIncrement);
                shotsRemaining = 0;
            }
            spinning = true;
        }

        if (spinning){
            if (Math.abs(spindexer.getVelocity()) < launchJamThreshold){
                launchJamSampling++;
            }
            if (launchJamSampling >= launchJamSampleThresh){
                launchJamSampling = 0;
                queuedLaunch = true;
                moveToState(State.PREPPING);

            }
        }

        if (launchStalled && launchTimer.seconds() >= sortingTimeout){
            launchStalled = false;
            spindexer.eject(singleLaunchIncrement);
            shotsRemaining -= 1;
            spinning = true;
        }

        if (!spinning && shotsRemaining <= 0){
            moveToState(State.IDLE);
        }

        if (spindexer.isAtRest() && !launchStalled){
            spinning = false;
        }

        launcher.update();
        spindexer.updateSpindexer();
    }

    /**
     * Transition from idle to intake
     */
    private void idleToIntake(){
        spindexer.setIndexOffset(Spindexer.INDEX_TYPE.INTAKE);
        intake.turnOn();
        intake.openGate();
    }
    /**
     * Transition from launch to idle
     */
    private void launchToIdle(){
        clearQueue();
        spindexer.setIndexOffset(Spindexer.INDEX_TYPE.INTAKE);
        spindexer.setSpindexerPos(0);
        launcher.setGatePosition(false);
        spindexer.setIndexOffset(Spindexer.INDEX_TYPE.NONE);
        for (int i = 0; i < 2; i++) {
            spindexer.clearColor(i);
        }
        intake.turnOff();
        intake.openGate();
    }
    /**
     * Transition from intake to idle
     */
    private void intakeToIdle(){
        spindexer.setIndexOffset(Spindexer.INDEX_TYPE.NONE);
        intake.turnOff();
        intake.closeGate();
    }
    /**
     * Transition from intake to launch
     */
    private void intakeToLaunch(){
        intake.turnOff();
        intake.closeGate();
        spindexer.setIndexOffset(Spindexer.INDEX_TYPE.LAUNCH);
        queuedLaunch = true;
        moveToState(State.PREPPING);
    }
    /**
     * Transition from launch to intake
     */
    private void launchToIntake(){
        launcher.setGatePosition(false);
        spindexer.setIndexOffset(Spindexer.INDEX_TYPE.INTAKE);
        for (int i = 0; i < 2; i++) {
            spindexer.clearColor(i);
        }
        intake.turnOn();
        intake.openGate();
    }
    /**
     * Transition from idle to launch
     */
    private void idleToLaunch(){
        spindexer.setIndexOffset(Spindexer.INDEX_TYPE.LAUNCH);
        queuedLaunch = true;
        moveToState(State.PREPPING);
    }
    private void preppingToLaunch(){
        launcher.setGatePosition(true);
        shotsRemaining = 3;
    }
    private void preppingToIdle(){
        // nothing really needed
    }
    private void preppingToIntake(){
        spindexer.setIndexOffset(Spindexer.INDEX_TYPE.INTAKE);
        intake.turnOn();
        intake.openGate();
    }

    private void idleToPrepping(){
        prep();
    }
    private void launchToPrepping(){
        launcher.setGatePosition(false);
        prep();
    }
    private void intakeToPrepping(){
        intake.turnOff();
        intake.closeGate();
        prep();
    }
    private void refreshPrep(){
        prep();
    }

    public List<COLORS> getLaunchQueue(){
        return launchQueue;
    }
    public State getCurrentState(){
        return currentState;
    }
}