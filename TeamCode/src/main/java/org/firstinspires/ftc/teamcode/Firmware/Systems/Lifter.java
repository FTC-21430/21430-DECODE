package org.firstinspires.ftc.teamcode.Firmware.Systems;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Resources.PIDFController;

@Config
public class Lifter {

    //TODO: Comments
    private HardwareMap hardwareMap = null;
    private Telemetry telemetry = null;

    //TODO: Find and tune values
    //TODO: Find height values. These are not even guestimates! RANDOM NUMBERS!
    private double MIN_HEIGHT = 0;
    private double MAX_HEIGHT = 21.5; // inches
    public static double DEFENCE_HEIGHT = 0.1;
    public static double HOMING_HEIGHT = 1;
    public static double FLOOR_CONTACT_HEIGHT = 1;
    public static double MAX_ENCODER = 2020; // ticks, formula from Gobilda motor spec sheet. The full stroke length in revolutions is 4.5 = (21.26 stroke length) / (4.725 belt circumference)
    public static double pCon = 0.1;
    public static double iCon = 0;
    public static double dCon = 0;
    public static double fCon = 0.4;
    public static double powerLimit = 1.0;

    private boolean latched = false;
    //TODO: we should make the homing logic work
    private boolean homing = false;

    private PIDFController leftLiftController = null;
    private PIDFController rightLiftController = null;
    Servo liftRightLatch;
    Servo liftLeftLatch;
    DcMotor liftRight;
    DcMotor liftLeft;
    private Servo[] servos;
    private DcMotor[] lifts;
    private DigitalChannel LiftLimitSwitch1;
    private DigitalChannel LiftLimitSwitch2;



    public Lifter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        liftLeft = hardwareMap.get(DcMotor.class, "liftL");
        liftRight = hardwareMap.get(DcMotor.class, "liftR");
        liftLeftLatch = hardwareMap.get(Servo.class, "leftLiftServo");
        liftRightLatch = hardwareMap.get(Servo.class, "rightLiftServo");
        LiftLimitSwitch1 = hardwareMap.get(DigitalChannel.class, "liftLimitSwitch1");
        LiftLimitSwitch2 = hardwareMap.get(DigitalChannel.class, "liftLimitSwitch2");
        servos = new Servo[]{liftLeftLatch, liftRightLatch};
        lifts = new DcMotor[]{liftLeft, liftRight};
        for (DcMotor lift : lifts) {
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        leftLiftController = new PIDFController(pCon, iCon, dCon, fCon, new ElapsedTime());
        rightLiftController = new PIDFController(pCon, iCon, dCon, fCon, new ElapsedTime());

        //Potentially reverse in the future, TODO:Check
        liftLeft.setDirection(REVERSE);
        liftRight.setDirection(FORWARD);
        unlockLatches();
        latched = false;
    }
    public void setLiftPosition(double position){
        if (position>MAX_HEIGHT){
            position = MAX_HEIGHT;
        }
        if (position<MIN_HEIGHT){
            position = MIN_HEIGHT;
        }

        leftLiftController.setTarget(position);
        rightLiftController.setTarget(position);
    }

    private void homed(){
        homing = false;
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLiftController.setTarget(0);
        rightLiftController.setTarget(0);
    }

    public void update(){

        if (homing){
            if (getLiftPosition() <1.2) {
                double loweringSpeed = -0.003;
                liftLeft.setPower(loweringSpeed);
                liftRight.setPower(loweringSpeed);
                checkHomingSwitches();
            }else{
                leftLiftController.update(ticksToInches(liftLeft.getCurrentPosition()));
                rightLiftController.update(ticksToInches(liftRight.getCurrentPosition()));
                if (leftHomingSwitchesPressed()){
                    liftLeft.setPower(0);
                    liftRight.setPower(0);
                }else{
                    double powerL = leftLiftController.getPower();
                    double powerR = leftLiftController.getPower();
                    liftLeft.setPower(powerL);
                    liftRight.setPower(powerR);
                }
            }
        }else {

            if (getLiftPosition() < FLOOR_CONTACT_HEIGHT){
                leftLiftController.updatePIDFConstants(pCon,iCon,dCon,0);
            }else{
                leftLiftController.updatePIDFConstants(pCon,iCon,dCon,fCon);
            }

            leftLiftController.update(ticksToInches(liftLeft.getCurrentPosition()));
            rightLiftController.update(ticksToInches(liftRight.getCurrentPosition()));

            double powerL = leftLiftController.getPower();
            double powerR = leftLiftController.getPower();
            if (leftHomingSwitchesPressed()){
              powerL = powerL<0? 0 : powerL;
              powerR = powerR<0? 0 : powerR;
            }
            telemetry.addData("liftSwitch", leftHomingSwitchesPressed());
            telemetry.addData("powerL", powerL);
            telemetry.addData("powerR",powerR);

            liftLeft.setPower(powerL);
            liftRight.setPower(powerR);

        }
    }
    public double ticksToInches(int ticks){

        double ticks2Inches = MAX_HEIGHT/MAX_ENCODER;
//        double ticks2Inches = 1.0;
        return ticks * ticks2Inches;
    }
    public double inchesToTicks(double inches){

        double inches2Ticks = MAX_ENCODER/MAX_HEIGHT;
        return inches * inches2Ticks;
    }
    public void lockLatches(){
        double leftLockPos = 0.45;
        double rightLockPos = 0.55;
        liftLeftLatch.setPosition(leftLockPos);
        liftRightLatch.setPosition(rightLockPos);
        latched = true;
}
    public void unlockLatches(){
        if (!latched) {
            //Find values
            double leftUnlockPos = 0.68;
            double rightUnlockPos = 0.4;
            liftLeftLatch.setPosition(leftUnlockPos);
            liftRightLatch.setPosition(rightUnlockPos);
        }
    }
    public boolean leftHomingSwitchesPressed(){
        return LiftLimitSwitch1.getState() || LiftLimitSwitch2.getState();
    }
    public void checkHomingSwitches(){
        if (leftHomingSwitchesPressed()){
            homed();
        }

    }
    public void retract(){
     setLiftPosition(MIN_HEIGHT);
    }
    public void lift(){
        setLiftPosition(MAX_HEIGHT);
    }
    public void defence(){
        setLiftPosition(DEFENCE_HEIGHT);
    }
    public void home(){
        homing = true;
        setLiftPosition(HOMING_HEIGHT);
    }
    public double getLiftPosition(){
        telemetry.addData("left", liftLeft.getCurrentPosition());
        telemetry.addData("right",liftRight.getCurrentPosition());
        int position = (liftLeft.getCurrentPosition() + liftRight.getCurrentPosition())/2;
        return ticksToInches(position);
    }
    public void updateConstants(){
        leftLiftController.updatePIDFConstants(pCon,iCon,dCon,fCon);
        rightLiftController.updatePIDFConstants(pCon,iCon,dCon,fCon);
    }
}