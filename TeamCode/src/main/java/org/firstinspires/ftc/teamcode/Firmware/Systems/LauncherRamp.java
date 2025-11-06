package org.firstinspires.ftc.teamcode.Firmware.Systems;

// Written by Tobin

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Resources.ServoPlus;

/**
 * Firmware to control the ramp of the launcher. Uses the ServoPlus resource to ensure easy logic.
 * TODO: Test and double check all of the constants, they were guessed based on the CAD and the specs of a Gobilda Torque Servo as found here https://www.gobilda.com/2000-series-dual-mode-servo-25-2-torque on 11/6/2025 - Tobin
 */
public class LauncherRamp {

    // servo instance
    private ServoPlus rampServo = null;

    // the full range of motion of the servo - note that this is usually a little different than what it is on the spec sheet, double check values.
    private final double servoROM = 300; // degrees

    // The range of motion the ramp can move, from completely retracted to farthest mechanical limit
    private final double rampROM = 23.6; // degrees

    // The gear ratio between the launcher hood and the servo gear
    private final double servoToRampRatio = (double) 1 / 6;

    // The minimum angle of the ramp up from Horizontal
    private final double minRampAngle = 6.97;

    // The maximum angle of the ramp up from Horizontal
    private final double maxRampAngle = minRampAngle + rampROM;


    /**
     * Construct function for this class, gets the servo reference
     * @param hardwareMap used to get the hardware reference for the ramp servo
     */
    public LauncherRamp(HardwareMap hardwareMap){
        // All values are passed to the servo assuming that the servo's zero is the launcher's min.
        rampServo = new ServoPlus(hardwareMap.get(Servo.class, "ramp"),servoROM * servoToRampRatio,rampAngleToServo(minRampAngle), rampAngleToServo(maxRampAngle));
    }

    /**
     * Sets the launch angle of the servo. will ensure correct range of input and convert angle to servo position.
     * @param angleUpFromHorizontal degrees above Horizontal the tangent line of the hood should be.
     */
    public void setLaunchAngle(double angleUpFromHorizontal){
        angleUpFromHorizontal = Range.clip(angleUpFromHorizontal, minRampAngle, maxRampAngle);
        rampServo.setServoPos(rampAngleToServo(angleUpFromHorizontal));
    }

    /**
     * lowest launch angle, resting position
     */
    public void retract(){
        rampServo.setServoPos(rampAngleToServo(minRampAngle));
    }

    /**
     * Highest launch angle
     */
    public void extendFull(){
        rampServo.setServoPos(rampAngleToServo(maxRampAngle));
    }

    /**
     * right in between the limits of launch angles
     */
    public void midAngle(){
        rampServo.setServoPos(rampAngleToServo((maxRampAngle - minRampAngle)/2));
    }

    /**
     * converts degrees over horizontal to something the servo can use by subtracting the offset that is the ramp's min angle.
     * @param angle degrees over horizontal
     * @return 1/6 the angle the servo needs to be at to align hood to angle.
     */
    private double rampAngleToServo(double angle){
        return angle - minRampAngle;
    }


}
