package org.firstinspires.ftc.teamcode.Firmware.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Resources.PIDController;

public class Flywheel {

    private HardwareMap hardwareMap = null;
    private Telemetry telemetry = null;
    private ElapsedTime runtime = null;
    private double targetSpeed = 0.0; // Meters per Second
    private double lastSpeed = 0.0; // The speed of the flywheel from the last iteration
    private double currentSpeed = 0.0; // Meters per Second
    private double currentAcceleration = 0.0; // Meters per Second per Second
    private PIDController speedController = null;
    private DcMotorEx flywheel = null;
    private final double wheelCircumference = 96*Math.PI; // mm
    private final double mmPerM = 1/1000.0; // There are 1000 mm per M
    private final double metersPerDegree = (1.0/360) * wheelCircumference * mmPerM;
    private double lastTime = 0.0;
    private int lastPos =0;
    private double accuracyThreshold;
    public Flywheel(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime runtime, double P, double I, double D){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.runtime = runtime;
        speedController = new PIDController(P,I,D,runtime);
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMotorEnable();
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setAccuracyThreshold(double threshold){
        accuracyThreshold = threshold;
    }

    public double getCurrentSpeed(){
        return currentSpeed;
    }
    public double getTargetSpeed()
    {
        return targetSpeed;
    }
    public double getCurrentAcceleration(){
        return currentAcceleration;
    }

    public void updatePIDConst(double P, double I, double D){
        speedController.updateConstants(P,I,D);
    }

    public void updateSpeedControl(){
        currentSpeed = flywheel.getVelocity(AngleUnit.DEGREES); // Degrees per Second - from DcMotorEx SDK class
        currentAcceleration = (currentSpeed-lastSpeed)/getDeltaTime();

        speedController.update(currentSpeed);
        flywheel.setPower(speedController.getPower());

        telemetry.addData("DeltaTime: ", getDeltaTime());
        telemetry.addData("current position: ", flywheel.getCurrentPosition());
        telemetry.addData("output power: ", speedController.getPower());

        lastSpeed = currentSpeed;
        lastTime = runtime.seconds();
        lastPos = flywheel.getCurrentPosition();



    }

    public void setTargetSpeed(double degreesPerSecond){
        targetSpeed = degreesPerSecond;
        speedController.setTarget(targetSpeed);
    }
    public boolean isAtSpeed(){
        return Math.abs(currentSpeed - targetSpeed)<accuracyThreshold;
    }
    private double getDeltaTime(){
        return runtime.seconds() - lastTime;
    }
    private double getDeltaPosition(){
        return (flywheel.getCurrentPosition() - lastPos) * (360/28.0); //returns Degrees - getCurrentPosition and lastPos is in raw encoder ticks
    }
}
