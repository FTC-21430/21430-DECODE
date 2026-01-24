package org.firstinspires.ftc.teamcode.Firmware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Intake;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Launcher;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Spindexer;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor.COLORS;

import java.util.ArrayList;
import java.util.List;

@Config
// Controls the automation for the mechanisms on the robot. Mainly, the launcher, spindexer, and intake.
public class OperatorStateMachine {

    // All states the operator is at
    public enum State {
        IDLE,
        LAUNCH,
        INTAKE
    }

    // The idle speed the flywheel will be set too when not actively shooting
    public static double idleSpeed = 1000;
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
    // Logic to ensure that a launch is completed before it starts the next launch
    private boolean prepping = false;
    private boolean launched = false;
    public static double launchingTimeout = 0.1;
    public static double sortingTimeout = 0.5;
    private ElapsedTime runtime = null;
    private Gamepad gamepad2 = null;
    private ElapsedTime launchTimer = null;
    private double launcherTimeOut = 0;
    private boolean launchTimeOuting = false;

    /**
     * The constructor for this class, Stores all of the instances of the components of the robot
     * Params are self explanatory
     * @param launcher
     * @param spindexer
     * @param intake
     * @param telemetry
     * @param setLauncherBasedOnTags - One function we need from DecodeBot.java but this is just a refernce to call method, not anything else in DecodeBot!
     */
    public OperatorStateMachine(Launcher launcher, Spindexer spindexer, Intake intake, Telemetry telemetry, Runnable setLauncherBasedOnTags, Gamepad gamepad2){
        this.launcher = launcher;
        this.spindexer = spindexer;
        this.intake = intake;
        this.telemetry = telemetry;
        this.setLauncherBasedOnTags = setLauncherBasedOnTags;
        this.runtime = new ElapsedTime();
        this.gamepad2 = gamepad2;
        this.launchTimer = new ElapsedTime();
    }

    // Will Trigger the transition from one state to the next
    public void moveToState(State state){
        switch (currentState){
            case IDLE:
                switch (state){
                    case IDLE:
                        break;
                    case INTAKE:
                        idleToIntake();
                        break;
                    case LAUNCH:
                        idleToLaunch();
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
                    case LAUNCH:
                        intakeToLaunch();
                        break;
                }
            break;
        }
        currentState = state;
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

    /**
     * The idle state update method
     * retracts and slows down launcher, updates spindexer
     */
    private void idleState(){
        if (!(gamepad2.left_trigger >= 0.4)){
            intake.setIntakePower(0.3);
        }
        launcher.retractRamp();
        launcher.setSpeed(idleSpeed);
        launcher.update();
        spindexer.updateSpindexer();
    }

    /**
     * The intake state update method
     * Handles all logic for intaking new artifacts and storing their color
     */


    private int ballSampling = 0;
    private void intakeState (){
//        telemetry.addData("holdingIntake", holdingIntake);
//        telemetry.addData("time", runtime.seconds());

        if (!(gamepad2.left_trigger >= 0.4)){
            intake.setIntakePower(-1);
        }

        if (spindexer.getColorInIntake() != COLORS.NONE && spindexer.isAtRest() && (spindexer.getIntakeSwitch()) || ballSampling >= 4){
            spindexer.storeColorAtIndex();
            spindexer.moveToNextIndex();
            ballSampling = 0;
        } else if (spindexer.isAtRest() && spindexer.getColorInIntake() != COLORS.NONE) {
            ++ballSampling;
        }else{
            ballSampling = 0;
        }

        if (spindexer.isFull() && spindexer.isAtRest()){
            moveToState(State.IDLE);
        }

        launcher.update();
        spindexer.updateSpindexer();

    }

    private double currentLaunchTimeout = 0;
    /**
     * The launch state update method
     * Handles the logic for shooting the balls in the right order
     */
    private void launchState(){
        if (!(gamepad2.left_trigger >= 0.4)){
            intake.setIntakePower(0.1);
        }
//        telemetry.addData("speed", launcher.getSpeed());
//        telemetry.addData("up to speed", launcher.isUpToSpeed());
//        telemetry.addData("get target speed", launcher.getTargetSpeed());
        setLauncherBasedOnTags.run();

        if (!launchQueue.isEmpty() && !prepping && !launched){
            COLORS toPrep = launchQueue.remove(0);
            spindexer.prepColor(toPrep);
            prepping = true;
            launcher.setGatePosition(true);
            currentLaunchTimeout = toPrep != COLORS.NONE? sortingTimeout:launchingTimeout;

        }


        // When prepped and launcher is ready, eject and clear the stored color


        if (prepping && spindexer.isAtRest() && launcher.isUpToSpeed() && launcher.rampReady()){
            spindexer.eject();
            prepping = false;
            launched = true;
            launchTimer.reset();

            // Clear the color from the spindexer and remove it from the queue so we don't re-prep it

            int clearedIndex = spindexer.getCurrentIndexInLaunch() -1;
            if (clearedIndex < 0){
                moveToState(State.IDLE);
                spindexer.setColorIndexing(COLORS.NONE,COLORS.NONE,COLORS.NONE);
            }
            spindexer.clearColor(clearedIndex);
        }

        // Wait for ejector to fully retract before allowing next cycle
        if (launched && !spindexer.isEjectorOut() && launchTimer.seconds() >= launchingTimeout){
            launched = false;
            launcher.setGatePosition(false);
        }

        // If nothing left to launch and nothing in progress, go idle
        if (launchQueue.isEmpty() && !prepping && !launched && !spindexer.isEjectorOut()){
            if (!launchTimeOuting) {
                runtime.reset();
                launchTimeOuting = true;
            } else {
                launcherTimeOut = runtime.seconds();
            }
            if (launcherTimeOut == 0.5) {
                moveToState(State.IDLE);
                launcher.setGatePosition(false);
            }
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
    }
    /**
     * Transition from launch to idle
     */
    private void launchToIdle(){
        for (int i = 0; i < 3; i++){
            addToQueue(COLORS.NONE);
        }
        launcher.setGatePosition(false);
        spindexer.setIndexOffset(Spindexer.INDEX_TYPE.NONE);
        intake.turnOff();
        prepping = false;
    }
    /**
     * Transition from intake to idle
     */
    private void intakeToIdle(){
        spindexer.setIndexOffset(Spindexer.INDEX_TYPE.NONE);
        intake.turnOff();
    }
    /**
     * Transition from intake to launch
     */
    private void intakeToLaunch(){
        intake.turnOff();
        spindexer.setIndexOffset(Spindexer.INDEX_TYPE.LAUNCH);
        prepping = false;
    }
    /**
     * Transition from launch to intake
     */
    private void launchToIntake(){
        launcher.setGatePosition(false);
        spindexer.setIndexOffset(Spindexer.INDEX_TYPE.INTAKE);
        intake.turnOn();
        prepping = false;
    }
    /**
     * Transition from idle to launch
     */
    private void idleToLaunch(){
        spindexer.setIndexOffset(Spindexer.INDEX_TYPE.LAUNCH);
        prepping = false;
    }
    public List<COLORS> getLaunchQueue(){
        return launchQueue;
    }
    public State getCurrentState(){
        return currentState;
    }

}