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
    public boolean fieldCentricDriving = true;
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
        motorFL = hardwareMap.get(DcMotor.class, "FL");
        motorFR = hardwareMap.get(DcMotor.class, "FR");
        motorBL = hardwareMap.get(DcMotor.class, "BL");
        motorBR = hardwareMap.get(DcMotor.class, "BR");

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

    /**
     * used to change whether or not you use feedback from encoders
     * to keep the motors at the correct speed
     *
     * @param velocityMode the boolean that tells if you want to use "RUN_USING_ENCODER" mode
     */
    public void setVelocityMode(boolean velocityMode) {

        if (velocityMode) {

            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        } else {

            motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
    }

    public void setFieldCentricDriving(boolean fcd){
        fieldCentricDriving = fcd;
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
    public void setDrivePower(double forwardPower, double sidewaysPower, double turnPower, double robotHeading) {
        // Field Centric Driving aligns all robot movements with the player's perspective from the field, rather than the robot's
        // Added math equation to change from degrees to radians on the robot
        if (fieldCentricDriving) {
            double temp = forwardPower * Math.cos(-AngleUnit.DEGREES.toRadians(robotHeading)) + sidewaysPower * Math.sin(-AngleUnit.DEGREES.toRadians(robotHeading));
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


