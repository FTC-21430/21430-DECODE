package org.firstinspires.ftc.teamcode.Firmware.Systems;

// Written by Tobin

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Controls the launcher system, including the flywheel speed.
 */
public class Launcher {
    // Flywheel subsystem instance
    private final Flywheel FLYWHEEL;

    // Launcher Ramp subsystem instance
    private final LauncherRamp RAMP;

    // Launcher gate subsystem - controls the physical gate/door that releases rings.
    // The Gate subsystem provides simple open/close commands and a short movement timer
    // so callers can know whether the gate is still moving (useful when sequencing shots).
    private final LauncherGate GATE;

    //the speed at which the flywheel remains when there is nothing to do :`(
    public double idleSpeed = 1000;

    /**
     * Constructs a Launcher with the given hardware map and telemetry.
     * @param hardwareMap the hardware map to use
     * @param telemetry the telemetry to use
     */
    public Launcher(HardwareMap hardwareMap, Telemetry telemetry){

        // PID constants for flywheel speed control (values can be overridden with a different function) used only for flywheel initialization
        final double FLYWHELLSPEEDCONTROLP = 300;
        final double FLYWHELLSPEEDCONTROLI = 1;
        final double FLYWHELLSPEEDCONTROLD = 10;

        // Initialize the flywheel with PID constants
        FLYWHEEL = new Flywheel(hardwareMap, telemetry, new ElapsedTime(), FLYWHELLSPEEDCONTROLP, FLYWHELLSPEEDCONTROLI, FLYWHELLSPEEDCONTROLD);
        // Set the accuracy threshold for speed control (in degrees per second)
        FLYWHEEL.setAccuracyThreshold(50);

        // Initialize the ramp and gate. The gate is a simple servo controller
        // used to momentarily open and close the launcher release mechanism.
        // We keep a reference so OpModes can command open/close and query whether
        // the gate has finished moving.
        RAMP = new LauncherRamp(hardwareMap);
        GATE = new LauncherGate(hardwareMap,telemetry);
        GATE.closeGate();
        RAMP.retract();
    }

    /**
     * Sets the flywheel target speed.
     * @param degreesPerSecond the target speed in degrees per second
     */
    public void setSpeed(double degreesPerSecond){
        FLYWHEEL.setTargetSpeed(degreesPerSecond);
    }

    /**
     * Gets the current flywheel speed.
     * @return the current speed in degrees per second
     */
    public double getSpeed(){
        return FLYWHEEL.getCurrentSpeed();
    }

    public double getIdleSpeed(){
        return idleSpeed;
    }

    /**
     * Gets the target flywheel speed.
     * @return the target speed in degrees per second
     */
    public double getTargetSpeed(){
        return FLYWHEEL.getTargetSpeed();
    }

    /**
     * Checks if the flywheel is at the target speed.
     * @return true if at speed, false otherwise
     */
    public boolean isUpToSpeed(){
        return FLYWHEEL.isAtSpeed();
    }

    /**
     * Must be called every loop iteration in order to keep subsystems updated.
     * <p>Includes a call to the gate update so the gate's internal movement timer
     * is advanced; this allows callers to check `gateMoving()` reliably after
     * commanding `setGatePosition(...)`.</p>
     */
    public void update(){
        // Update gate timing first so its movement state is current for this loop.
        GATE.updateGate();
        FLYWHEEL.updateSpeedControl();
    }

    /**
     * Set the angle of the ramp.
     * @param angle Degrees Up from Horizontal in the launcher direction. Will get clipped to allowed range if value is outside of mechanical limits.
     */
    public void setLaunchAngle(double angle){
        RAMP.setLaunchAngle(angle);
    }

    /**
     * Bring the ramp as close to robot or as low a launch angle as possible
     */
    public void retractRamp(){
        RAMP.retract();
    }

    /**
     * Steepest preset launch angle
     */
    public void fullyExtendRamp(){
        RAMP.extendFull();
    }

    /**
     * preset directly in the middle of all possible launch angles
     */
    public void halfExtendRamp(){
        RAMP.midAngle();
    }
    public boolean rampReady(){
        return RAMP.isReady();
    }
    /**
     * Command the gate to open or close.
     *
     * <p>Opening/closing the gate will start a short internal movement timer inside
     * the {@link LauncherGate} so callers can poll {@link #gateMoving()} to wait until
     * the motion has completed. This method simply delegates to the gate subsystem.</p>
     *
     * @param open true to open the gate, false to close it
     */
    public void setGatePosition(boolean open){
        if (open){
            GATE.openGate();
        }else{
            GATE.closeGate();
        }
    }

    /**
     * Returns whether the gate is currently moving.
     *
     * @return true when the gate is in motion (i.e. a recent open/close command
     *         is still within the gate movement timeout), false when the gate is stopped
     *         and considered settled.
     */
    public boolean gateMoving(){
        return !GATE.isStopped();
    }
}