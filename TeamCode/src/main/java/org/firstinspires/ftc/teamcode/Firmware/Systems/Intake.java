package org.firstinspires.ftc.teamcode.Firmware.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    //This just sets the intake hardware map to null so it just initalize it
    private HardwareMap hardwareMap = null;
    //This initalize the DcMotor for the intake
    private DcMotor intake =null;
//This function just runs the encoder in the intake
    public Intake(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //This just sets the power of the intake
    public void setIntakePower (double power){
        intake.setPower(power);
    }
    public void turnOn(){
        setIntakePower(-1);
    }
    public void turnOff(){
        setIntakePower(0);
    }
}
