package org.firstinspires.ftc.teamcode.Firmware.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.comp.Todo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.ArrayList;
import java.util.List;

public class MecanumDriveTrain {
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    double speedMultiplier = 1;
    public boolean fieldCentricDriving = false;
    private double avgDrivePower = 0;

    private Telemetry telemetry;
    /**
     * constructor for mecanum drive train
     * -assigns motors from hardware map
     * -sets directions
     * -sets zero power behavior to brake
     * @param hardwareMap hardware map of robot
     */
    public MecanumDriveTrain(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        motorFL = hardwareMap.get(DcMotor.class, "fl");
        motorFR = hardwareMap.get(DcMotor.class, "fr");
        motorBL = hardwareMap.get(DcMotor.class, "bl");
        motorBR = hardwareMap.get(DcMotor.class, "br");

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // F R F R for software testing bot, competition bot has F R R F


        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //Allows fieldCentricDriving to be turned on and off
    public void fieldCentricDriving(boolean enabled){
        fieldCentricDriving = enabled;
    }

    /**
     * Sets the speed multiplier
     *
     * @param speedMultiplier speed multiplier (between 0 and 1)
     */
    public void setSpeedMultiplier(double speedMultiplier) {

        // ensure valid speed multiplier
        if (speedMultiplier < 0 || speedMultiplier > 1) {
            speedMultiplier = 1;
        }

        // sets speed multiplier
        this.speedMultiplier = speedMultiplier;
    }

    public double getSpeedMultiplier(){
        return speedMultiplier;
    }

    /**
     * Calculates Field Centic Driving (code that allows the driver controls to
     * stay constant and match the driver's position).
     * Translates the forward and sideways power based on the angle robot is facing.
     * @param forwardPower
     * @param sidewaysPower
     * @param robotHeading
     * @return
     */
    private List<Double> calculateFieldCentricDriving(double forwardPower, double sidewaysPower,double robotHeading){
        double fwdPower = forwardPower * Math.cos(-AngleUnit.DEGREES.toRadians(robotHeading)) + sidewaysPower * Math.sin(-AngleUnit.DEGREES.toRadians(robotHeading));
        double sidePower = -forwardPower * Math.sin(-AngleUnit.DEGREES.toRadians(robotHeading)) + sidewaysPower * Math.cos(-AngleUnit.DEGREES.toRadians(robotHeading));
        List<Double> power = new ArrayList<Double>();
        power.add(fwdPower);
        power.add(sidePower);
        return power;
    }

    /**
     * sets the motor powers
     *
     * @param forwardPower  amount it goes forward
     * @param sidewaysPower amount it goes sideways
     * @param turnPower     amount it turns
     * @param robotHeading current heading of the robot in degrees
     */
    public void setDrivePower(double forwardPower, double sidewaysPower, double turnPower, double robotHeading) {

        List<Double> transformedMovementVectors = calculateFieldCentricDriving(forwardPower,sidewaysPower, robotHeading);
        if (fieldCentricDriving) {
            forwardPower = transformedMovementVectors.get(0);
            sidewaysPower = transformedMovementVectors.get(1);
        }

        motorFL.setPower(Range.clip(forwardPower + sidewaysPower - turnPower, -1.0, 1.0) * speedMultiplier);
        motorFR.setPower(Range.clip(forwardPower + sidewaysPower + turnPower, -1.0, 1.0) * speedMultiplier);
        motorBL.setPower(Range.clip(forwardPower - sidewaysPower - turnPower, -1.0, 1.0) * speedMultiplier);
        motorBR.setPower(Range.clip(forwardPower - sidewaysPower + turnPower, -1.0, 1.0) * speedMultiplier);

        //Finds average Drive Power
        //Divide by 2 to give us closer values to the use case
        //TODO: why do we need this here?
        avgDrivePower = (Math.abs(forwardPower) + Math.abs(sidewaysPower) + Math.abs(turnPower))/2;
    }
    public double getAvgDrivePower(){
        return avgDrivePower;
    }
}