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

        // Initialize the ramp and have it move to starting configuration of retracted
        RAMP = new LauncherRamp(hardwareMap);
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
     * must be called every loop iteration in order to keep the wheel up to speed
     */
    public void updateSpeedControl(){
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
}