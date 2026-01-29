package org.firstinspires.ftc.teamcode.Firmware.Systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Controls the launcher gate servo.
 *
 * <p>This class provides a small state machine for moving a gate servo used in the launcher.
 * It tracks whether the servo is currently moving (isStopped) and uses an {@link ElapsedTime}
 * timer plus a configurable timeout to consider the movement finished. Call {@link #updateGate()}
 * regularly from your main loop to update the internal stopped state.</p>
 */
public class IntakeGate {
    // Hardware
    private Servo servo = null;
    private ElapsedTime timer = null;
    private Telemetry telemetry = null;

    /** Servo position used when the gate is closed. Range: 0.0 - 1.0 */
    public static double closedPos = 1;

    /** Servo position used when the gate is open. Range: 0.0 - 1.0 */
    public static double openPos = 0.892;

    /**
     * Time in seconds to wait after commanding a motion before considering the gate stopped.
     * Typically a small value like 0.02 - 0.1 seconds depending on your servo and motion.
     */
    public static double movementTimeout = 0.03;

    // True when the gate is not currently moving.
    private boolean isStopped = true;

    /**
     * Create a LauncherGate controller.
     *
     * @param hardwareMap the robot HardwareMap used to fetch the "gate" servo
     * @param telemetry   telemetry instance for optional logging (may be null)
     */
    public IntakeGate(HardwareMap hardwareMap, Telemetry telemetry) {
        servo = hardwareMap.get(Servo.class, "intakeGate");
        timer = new ElapsedTime();
        this.telemetry = telemetry;
    }

    /**
     * Update internal state; should be called repeatedly from your main loop.
     *
     * <p>If the gate was commanded to move and the movement timeout has elapsed, this method
     * marks the gate as stopped. This method does not move hardware — it only updates state.</p>
     */
    public void updateGate() {
        telemetry.addData("gateStopped", isStopped);
        if (!isStopped && timer.seconds() >= movementTimeout) {
            isStopped = true;
        }
    }

    /**
     * Returns true when the gate is not currently moving.
     *
     * @return true if the gate is considered stopped
     */
    public boolean isStopped() {
        return isStopped;
    }

    /**
     * Command the gate to the open position.
     * Resets the movement timer so {@link #updateGate()} will track the motion duration.
     */
    public void openGate() {
        isStopped = false;
        timer.reset();
        servo.setPosition(openPos);
    }

    /**
     * Command the gate to the closed position.
     * Resets the movement timer so {@link #updateGate()} will track the motion duration.
     */
    public void closeGate() {
        isStopped = false;
        timer.reset();
        servo.setPosition(closedPos);
    }
}