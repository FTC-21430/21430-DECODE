package org.firstinspires.ftc.teamcode.Firmware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Launcher;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Spindexer;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class OperatorStateMachine {

    public enum State {
        IDLE,
        LAUNCH,
        INTAKE
    }

    private Launcher launcher;
    private Spindexer spindexer;
    private Telemetry telemetry;
    private State currentState = State.IDLE;
    private List<SpindexerColorSensor.COLORS> launchQueue = new ArrayList<>(Collections.nCopies(3, SpindexerColorSensor.COLORS.NONE));
    
    public OperatorStateMachine(Launcher launcher, Spindexer spindexer, Telemetry telemetry){
        this.launcher = launcher;
        this.spindexer = spindexer;
        this.telemetry = telemetry;
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
    }

    private void idleToLaunch(){

    }
    private void idleToIntake(){

    }
    private void launchToIdle(){

    }
    private void intakeToIdle(){

    }
    private void intakeToLaunch(){

    }
    private void launchToIntake(){

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
    private void idleState(){
        spindexer.updateSpindexer();
    }
    private void intakeState(){
        spindexer.updateSpindexer();

    }
    private void launchState(){
        spindexer.updateSpindexer();
    }
}
