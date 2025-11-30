package org.firstinspires.ftc.teamcode.Firmware.Systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Resources.PIDFController;
import org.firstinspires.ftc.teamcode.Resources.ServoPlus;
@Config
// The lift is the mechanism on the robot that protects against defense and raises up the robot to score two robots in the base zone at the end of the match.
public class Lift {
    private Telemetry telemetry;
    private DcMotor leftLift = null;
    private DcMotor rightLift = null;
    private PIDFController leftLiftController = null;
    private PIDFController rightLiftController = null;

    private ServoPlus leftLiftServo = null;
    private ServoPlus rightLiftServo = null;

    private ServoPlus[] servos;
    private DcMotor[] lifts;

    //TODO: tune these values, I guessed them completely - Tobin 11/29/2025
    public static double holdingConstant = 0.01;
    public static double pConstant = 0.3;
    public static double iConstant = 0.001;
    public static double dConstant = 0.025;

    private double maxExtension = (180*3)/(2.54 * 10); // the lifts are 3 cascading Misumi SAR 230s. Those slides have a stroke of 180 mms. Converted to Inches.
    private double endSafetySpacing = 0.5; // Extra distance from the absolute end to make us all feel good about the robot now ripping itself to shreds!

    // TODO: Find these values from printing out the encoder values for one of the lift motors and comparing difference to the amount the lift traveled
    private double encoderTickToInch = 1.0;
    private double inchToEncoderTick = 1.0;

    private double leftPawlUp = 0.0;
    private double leftPawlDown = 1.0;
    private double rightPawlUp = 0.0;
    private double rightPawlDown = 1.0;
    public Lift(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;

        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");

        //TODO: when the servos are choosen for the racket and pawls here, enter in the range of motion
        leftLiftServo = new ServoPlus(hardwareMap.get(Servo.class, "leftLiftServo"),200,0,200);
        rightLiftServo = new ServoPlus(hardwareMap.get(Servo.class, "rightLiftServo"), 200,0,200);


        servos = new ServoPlus[] { leftLiftServo, rightLiftServo };
        lifts = new DcMotor[] {leftLift,rightLift};

        for (DcMotor lift: lifts){
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        leftLiftController = new PIDFController(pConstant, iConstant, dConstant, holdingConstant, new ElapsedTime());
        rightLiftController = new PIDFController(pConstant, iConstant, dConstant, holdingConstant, new ElapsedTime());


    }


    public void update(){
        leftLiftController.update(encoderTicksToInch(leftLift.getCurrentPosition()));
        rightLiftController.update(encoderTicksToInch(rightLift.getCurrentPosition()));

        leftLift.setPower(leftLiftController.getPower());
        rightLift.setPower(rightLiftController.getPower());
    }
    public double getCurrentExtension(){
        double exLeft = encoderTicksToInch(leftLift.getCurrentPosition());
        double exRight = encoderTicksToInch(rightLift.getCurrentPosition());
        return (exLeft+exRight)/2.0;
    }

    public void setExtension(double inches){
        // safety check for if the target value is within mechanical bounds.
        if (inches < 0.0 || inches > (maxExtension-endSafetySpacing)){
            return;
        }

        leftLiftController.setTarget(inches);
        rightLiftController.setTarget(inches);
    }

    public void retract(){
        setExtension(0);
    }
    public void defense(){
        setExtension(1.7);
    }
    public void fullLift(){
        setExtension(maxExtension-endSafetySpacing);
    }
    public void lockRackets(){
        leftLiftServo.setServoPos(leftPawlDown);
        rightLiftServo.setServoPos(rightPawlDown);
    }
    public void liftRackets(){
        leftLiftServo.setServoPos(leftPawlUp);
        rightLiftServo.setServoPos(rightPawlUp);
    }
    public void printTelemetryData(){
        telemetry.addData("Current Lift Extension", getCurrentExtension());
        telemetry.addData("leftLiftPower", leftLiftController.getPower());
        telemetry.addData("rightLiftPower", rightLiftController.getPower());
        telemetry.addData("currentLiftDifference", encoderTicksToInch(Math.abs(leftLift.getCurrentPosition()-rightLift.getCurrentPosition())));
    }
    private double encoderTicksToInch(int tick){
        return tick * encoderTickToInch;
    }

    private int inchesToEncoderTick(double inch) {
        return (int) (inch * inchToEncoderTick);
    }

}
