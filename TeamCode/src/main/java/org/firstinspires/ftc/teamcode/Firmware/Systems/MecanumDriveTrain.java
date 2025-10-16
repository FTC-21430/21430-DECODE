package org.firstinspires.ftc.teamcode.Firmware.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDriveTrain {

    public double mediumSpeed = 0.6;

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    public final double mediumSpeedMultiplier = 0.6;
    double speedMultiplier = 1;
    public boolean fieldCentricDriving = false;
    private Telemetry telemetry;

    private double avgDrivePower = 0;
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

        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
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
     * sets the motor powers
     *
     * @param forwardPower  amount it goes forward
     * @param sidewaysPower amount it goes sideways
     * @param turnPower     amount it turns
     */
    private double calculateFieldCentricDriving(double forwardPower, double sidewaysPower, double turnPower,double robotHeading){
        return (forwardPower* Math.cos(-AngleUnit.DEGREES.toRadians(robotHeading)) + sidewaysPower * Math.sin(-AngleUnit.DEGREES.toRadians(robotHeading)));
    }
    public void setDrivePower(double forwardPower, double sidewaysPower, double turnPower, double robotHeading) {
        // Field Centric Driving aligns all robot movements with the player's perspective from the field, rather than the robot's
        // Added math equation to change from degrees to radians on the robot
        if (fieldCentricDriving) {
            double temp = calculateFieldCentricDriving(forwardPower,sidewaysPower,turnPower, robotHeading);
            sidewaysPower = -forwardPower * Math.sin(-AngleUnit.DEGREES.toRadians(robotHeading)) + sidewaysPower * Math.cos(-AngleUnit.DEGREES.toRadians(robotHeading));
            forwardPower = temp;
        }
        
        motorFL.setPower(Range.clip(forwardPower + sidewaysPower - turnPower, -1.0, 1.0) * speedMultiplier);
        motorFR.setPower(Range.clip(forwardPower - sidewaysPower + turnPower, -1.0, 1.0) * speedMultiplier);
        motorBL.setPower(Range.clip(forwardPower - sidewaysPower - turnPower, -1.0, 1.0) * speedMultiplier);
        motorBR.setPower(Range.clip(forwardPower + sidewaysPower + turnPower, -1.0, 1.0) * speedMultiplier);

        //divided by 2 on purpose, I think it will help give us closer values to the use case
        avgDrivePower = (Math.abs(forwardPower) + Math.abs(sidewaysPower) + Math.abs(turnPower))/2;

    }
    public double getAvgDrivePower(){
        return avgDrivePower;
    }
}


