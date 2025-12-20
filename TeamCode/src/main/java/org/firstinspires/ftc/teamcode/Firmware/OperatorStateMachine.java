package org.firstinspires.ftc.teamcode.Firmware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    private boolean launching = false;

    /**
     * The constructor for this class, Stores all of the instances of the components of the robot
     * Params are self explanatory
     * @param launcher
     * @param spindexer
     * @param intake
     * @param telemetry
     * @param setLauncherBasedOnTags - One function we need from DecodeBot.java but this is just a refernce to call method, not anything else in DecodeBot!
     */
    public OperatorStateMachine(Launcher launcher, Spindexer spindexer, Intake intake, Telemetry telemetry, Runnable setLauncherBasedOnTags){
        this.launcher = launcher;
        this.spindexer = spindexer;
        this.intake = intake;
        this.telemetry = telemetry;
        this.setLauncherBasedOnTags = setLauncherBasedOnTags;
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
        launcher.retractRamp();
        launcher.setSpeed(idleSpeed);
        launcher.updateSpeedControl();
        spindexer.updateSpindexer();
    }

    /**
     * The intake state update method
     * Handles all logic for intaking new artifacts and storing their color
     */
    private void intakeState(){
        telemetry.addData("intakeSwitch", spindexer.getIntakeSwitch());
        if (spindexer.getIntakeSwitch()&&spindexer.isAtRest()){
            spindexer.storeColorAtIndex();
            spindexer.moveToNextIndex();
        }
        if (spindexer.isFull()){
            moveToState(State.IDLE);
        }

        launcher.updateSpeedControl();
        spindexer.updateSpindexer();

    }

    /**
     * The launch state update method
     * Handles the logic for shooting the balls in the right order
     */
    private void launchState(){
        telemetry.addData("speed", launcher.getSpeed());
        telemetry.addData("up to speed", launcher.isUpToSpeed());
        telemetry.addData("get target speed", launcher.getTargetSpeed());
        setLauncherBasedOnTags.run();

        if (!launchQueue.isEmpty() && !launching){
            spindexer.prepColor(launchQueue.get(launchQueue.size()-1));
            launching = true;
        }

        if (launching && spindexer.isAtRest() && launcher.isUpToSpeed()){
            spindexer.eject();
            spindexer.clearColor(spindexer.getCurrentIndexInLaunch());
        }
        if (launching && !spindexer.isEjectorOut() && spindexer.isAtRest()){
            launching = false;
        }

        if (launchQueue.isEmpty() && !launching){
            moveToState(State.IDLE);
        }

        launcher.updateSpeedControl();
        spindexer.updateSpindexer();
    }

    /**
     * Transition from idle to intake
     */
    private void idleToIntake(){
        intake.turnOn();
    }
    /**
     * Transition from launch to idle
     */
    private void launchToIdle(){
        for (int i = 0; i < 3; i++){
            addToQueue(COLORS.NONE);
        }
        intake.turnOff();
        launching = false;
    }
    /**
     * Transition from intake to idle
     */
    private void intakeToIdle(){
        intake.turnOff();
    }
    /**
     * Transition from intake to launch
     */
    private void intakeToLaunch(){
        intake.turnOff();
        launching = false;
    }
    /**
     * Transition from launch to intake
     */
    private void launchToIntake(){
        intake.turnOn();
        launching = false;
    }
    /**
     * Transition from idle to launch
     */
    private void idleToLaunch(){
        launching = false;
    }
    public List<COLORS> getLaunchQueue(){
        return launchQueue;
    }
    public State getCurrentState(){
        return currentState;
    }

}