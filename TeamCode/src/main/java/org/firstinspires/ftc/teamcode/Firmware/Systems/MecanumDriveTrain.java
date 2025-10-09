package org.firstinspires.ftc.teamcode.Firmware.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDriveTrain {

    // motor objects
    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;
    private boolean fieldCentricDriving = true;
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
        motorFL = hardwareMap.get(DcMotor.class, "FL");
        motorFR = hardwareMap.get(DcMotor.class, "FR");
        motorBL = hardwareMap.get(DcMotor.class, "BL");
        motorBR = hardwareMap.get(DcMotor.class, "BR");

    // We do not plan to use the encoders but we reset them just in case
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

    public void setDrivePower(double forwardPower, double sidewaysPower, double turnPower, double robotHeading) {
        // Field Centric Driving aligns all robot movements with the player's perspective from the field, rather than the robot's
        // Added math equation to change from degrees to radians on the robot
        if (fieldCentricDriving) {
            double temp = forwardPower * Math.cos(-AngleUnit.DEGREES.toRadians(robotHeading)) + sidewaysPower * Math.sin(-AngleUnit.DEGREES.toRadians(robotHeading));
            sidewaysPower = -forwardPower * Math.sin(-AngleUnit.DEGREES.toRadians(robotHeading)) + sidewaysPower * Math.cos(-AngleUnit.DEGREES.toRadians(robotHeading));
            forwardPower = temp;
        }
        
        motorFL.setPower(Range.clip(forwardPower + sidewaysPower - turnPower, -1.0, 1.0));
        motorFR.setPower(Range.clip(forwardPower - sidewaysPower + turnPower, -1.0, 1.0));
        motorBL.setPower(Range.clip(forwardPower - sidewaysPower - turnPower, -1.0, 1.0));
        motorBR.setPower(Range.clip(forwardPower + sidewaysPower + turnPower, -1.0, 1.0));

    }
}