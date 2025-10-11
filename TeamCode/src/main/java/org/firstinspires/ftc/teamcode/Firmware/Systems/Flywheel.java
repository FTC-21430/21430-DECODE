package org.firstinspires.ftc.teamcode.Firmware.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Resources.PIDController;

import java.util.concurrent.TimeUnit;

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
    private final double mmPerM = 1/1000; // There are 1000 mm per M
    private final double degreesToMeters = (1/360) * wheelCircumference * mmPerM;
    private double lastTime = 0.0;
    private double accuracyThreshold;
    public Flywheel(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime runtime, double P, double I, double D){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.runtime = runtime;
        speedController = new PIDController(P,I,D,runtime);
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setAccuracyThreshold(double threshold){
        accuracyThreshold = threshold;
    }

    public void updatePIDConst(double P, double I, double D){
        speedController.updateConstants(P,I,D);
    }

    public void updateSpeedControl(){
        currentSpeed = flywheel.getVelocity(AngleUnit.DEGREES)*degreesToMeters; // converts to Meters per Second
        currentAcceleration = (currentSpeed-lastSpeed)/getDeltaTime();
        speedController.setTarget(targetSpeed);
        speedController.update(currentSpeed);
        flywheel.setPower(speedController.getPower());
        lastSpeed = currentSpeed;
        lastTime = runtime.time(TimeUnit.SECONDS);
    }

    public void setTargetSpeed(double metersPerSecond){
        targetSpeed = metersPerSecond;
    }
    public boolean isAtSpeed(){
        return Math.sqrt(Math.pow((currentSpeed - targetSpeed),2))<accuracyThreshold;
    }
    private double getDeltaTime(){
        return runtime.time(TimeUnit.SECONDS) - lastTime;
    }
}
