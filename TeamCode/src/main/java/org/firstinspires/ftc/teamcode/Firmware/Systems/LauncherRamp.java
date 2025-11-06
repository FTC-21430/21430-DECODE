package org.firstinspires.ftc.teamcode.Firmware.Systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Resources.ServoPlus;

public class LauncherRamp {

    private ServoPlus rampServo = null;

    private final double servoROM = 300; // degrees

    private final double rampROM = 23.6; // degrees

    private final double servoToRampRatio = (double) 1 / 6;

    private final double minRampAngle = 6.97;
    private final double maxRampAngle = minRampAngle + rampROM;



    public LauncherRamp(HardwareMap hardwareMap){
        rampServo = new ServoPlus(hardwareMap.get(Servo.class, "ramp"),servoROM * servoToRampRatio,rampAngleToServo(minRampAngle), rampAngleToServo(maxRampAngle));
    }

    public void setLaunchAngle(double angleUpFromHorizontal){
        angleUpFromHorizontal = Range.clip(angleUpFromHorizontal, minRampAngle, maxRampAngle);
        rampServo.setServoPos(angleUpFromHorizontal);
    }

    public void retract(){
        rampServo.setServoPos(rampAngleToServo(minRampAngle));
    }
    public void extendFull(){
        rampServo.setServoPos(rampAngleToServo(maxRampAngle));
    }
    public void midAngle(){
        rampServo.setServoPos(rampAngleToServo((maxRampAngle - minRampAngle)/2));
    }

    private double rampAngleToServo(double angle){
        return angle - minRampAngle;
    }


}
