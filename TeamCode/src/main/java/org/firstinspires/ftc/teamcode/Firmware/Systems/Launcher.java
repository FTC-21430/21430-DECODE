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
    private final Flywheel flywheel;

    // Launcher Ramp subsystem instance
    private final LauncherRamp ramp;

    /**
     * Constructs a Launcher with the given hardware map and telemetry.
     * @param hardwareMap the hardware map to use
     * @param telemetry the telemetry to use
     */
    public Launcher(HardwareMap hardwareMap, Telemetry telemetry){

        // PID constants for flywheel speed control (values can be overridden with a different function) used only for flywheel initialization
        final double flywheelSpeedControlP = 300;
        final double flywheelSpeedControlI = 1;
        final double flywheelSpeedControlD = 10;

        // Initialize the flywheel with PID constants
        flywheel = new Flywheel(hardwareMap, telemetry, new ElapsedTime(),
                flywheelSpeedControlP, flywheelSpeedControlI, flywheelSpeedControlD);
        // Set the accuracy threshold for speed control (in degrees per second)
        flywheel.setAccuracyThreshold(20);

        // Initialize the ramp and have it move to starting configuration of retracted
        ramp = new LauncherRamp(hardwareMap);
        ramp.retract();
    }

    /**
     * Sets the flywheel target speed.
     * @param degreesPerSecond the target speed in degrees per second
     */
    public void setSpeed(double degreesPerSecond){
        flywheel.setTargetSpeed(degreesPerSecond);
    }

    /**
     * Gets the current flywheel speed.
     * @return the current speed in degrees per second
     */
    public double getSpeed(){
        return flywheel.getCurrentSpeed();
    }

    /**
     * Gets the target flywheel speed.
     * @return the target speed in degrees per second
     */
    public double getTargetSpeed(){
        return flywheel.getTargetSpeed();
    }

    /**
     * Checks if the flywheel is at the target speed.
     * @return true if at speed, false otherwise
     */
    public boolean isUpToSpeed(){
        return flywheel.isAtSpeed();
    }

    /**
     * must be called every loop iteration in order to keep the wheel up to speed
     */
    public void updateSpeedControl(){
        flywheel.updateSpeedControl();
    }

    /**
     * Set the angle of the ramp.
     * @param angle Degrees Up from Horizontal in the launcher direction. Will get clipped to allowed range if value is outside of mechanical limits.
     */
    public void setLaunchAngle(double angle){
        ramp.setLaunchAngle(angle);
    }

    /**
     * Bring the ramp as close to robot or as low a launch angle as possible
     */
    public void retractRamp(){
        ramp.retract();
    }

    /**
     * Steepest preset launch angle
     */
    public void fullyExtendRamp(){
        ramp.extendFull();
    }

    /**
     * preset directly in the middle of all possible launch angles
     */
    public void halfExtendRamp(){
        ramp.midAngle();
    }

}