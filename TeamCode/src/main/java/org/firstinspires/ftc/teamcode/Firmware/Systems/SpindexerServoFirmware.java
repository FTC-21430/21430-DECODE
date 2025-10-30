package org.firstinspires.ftc.teamcode.Firmware.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Resources.ServoPlus;

public class SpindexerServoFirmware {
    private final Servo spindexerServo;
    private final double[] positions;

    private final double passPWM;
    private int encoderPosition;

    private int positionTolerance = 4;

    private DcMotor spindexerEncoderMotorInstance;
    public SpindexerServoFirmware(HardwareMap hardwareMap, boolean spinClockwise, double position1, double position2, double position3, String encoderConfigAddress){
        this.positions = new double[] {position1,position2,position3};
        spindexerServo = hardwareMap.get(Servo.class, "spindexer servo");
        spindexerEncoderMotorInstance = hardwareMap.get(DcMotor.class, encoderConfigAddress);
        if (spinClockwise){
            passPWM = -1;
        } else{
            passPWM = 1;
        }
    }

    public void setSpindexerPosition(int position){

    }

    public void setPositionTolerance(int toleranceTicks){
        positionTolerance = toleranceTicks;
    }
    public boolean isAtTargetSlot(){
        return false; //TODO: 
    }

    private double encoderTicksToDegrees(int ticks){
        return (double)ticks; // TODO: insert regression function here.
    }

    private double degreesToServoPWM(double degrees){
        return 0.0; //TODO: make this regression function
    }

}
