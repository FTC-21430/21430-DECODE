package org.firstinspires.ftc.teamcode.Firmware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Intake;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Launcher;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Spindexer;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor.COLORS;

import java.util.ArrayList;
import java.util.List;

public class OperatorStateMachine {

    public enum State {
        IDLE,
        LAUNCH,
        INTAKE
    }
    private double idleSpeed = 1000;
    private Launcher launcher;
    private Spindexer spindexer;
    private Intake intake;
    private Telemetry telemetry;
    private DecodeBot bot;
    private State currentState = State.IDLE;
    private List<COLORS> launchQueue = new ArrayList<>();
    private boolean launching = false;

    public OperatorStateMachine(Launcher launcher, Spindexer spindexer, Intake intake, Telemetry telemetry, DecodeBot bot){
        this.launcher = launcher;
        this.spindexer = spindexer;
        this.intake = intake;
        this.telemetry = telemetry;
        this.bot = bot;
    }

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

    public void addToQueue(COLORS color){
        launchQueue.add(color);
        if (launchQueue.size() > 3){
            launchQueue.remove(0);
        }
    }
    public void setLaunchQueue(COLORS one, COLORS two, COLORS three){
        launchQueue.set(0,one);
        launchQueue.set(1,two);
        launchQueue.set(2,three);
    }
    private void idleState(){
        launcher.retractRamp();
        launcher.setSpeed(idleSpeed);
        launcher.updateSpeedControl();
        spindexer.updateSpindexer();
    }
    private void intakeState(){

        if (spindexer.getIntakeSwitch()){
            spindexer.storeColorAtIndex();
            spindexer.moveToNextIndex();
        }
        if (spindexer.isFull()){
            moveToState(State.IDLE);
        }

        launcher.updateSpeedControl();
        spindexer.updateSpindexer();

    }
    private void launchState(){
        bot.aimBasedOnTags();

        if (!launchQueue.isEmpty() && !launching && launcher.isUpToSpeed()){
            spindexer.prepColor(launchQueue.get(0));
            launching = true;
        }

        if (launching && spindexer.isAtRest()){
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

    private void idleToIntake(){
        intake.turnOn();
    }
    private void launchToIdle(){
        intake.turnOff();
        launching = false;
    }
    private void intakeToIdle(){
        intake.turnOff();
    }
    private void intakeToLaunch(){
        intake.turnOff();
        launching = false;
    }
    private void launchToIntake(){
        intake.turnOn();
        launching = false;
    }
    private void idleToLaunch(){
        launching = false;
    }

}
