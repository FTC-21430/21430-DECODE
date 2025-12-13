package org.firstinspires.ftc.teamcode.Firmware.Systems;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Resources.PIDFController;
import org.firstinspires.ftc.teamcode.Resources.ServoPlus;

public class Lifter {

    //TODO: Comments
    private HardwareMap hardwareMap = null;
    private Telemetry telemetry = null;

    //TODO: Find and tune values
    //TODO: Find height values. These are not even guestimates! RANDOM NUMBERS!
    private double MIN_HEIGHT = 0;
    private double MAX_HEIGHT = 1;
    private double DEFENCE_HEIGHT = 0.1;
    private double pCon;
    private double iCon;
    private double dCon;
    private double fCon;
    private boolean unlached = false;
    //TODO: we should make the homing logic work
    private boolean homing = false;

    private PIDFController leftLiftController = null;
    private PIDFController rightLiftController = null;
    ServoPlus liftRightLatch;
    ServoPlus liftLeftLatch;
    DcMotor liftRight;
    DcMotor liftLeft;
    private ServoPlus[] servos;
    private DcMotor[] lifts;
    private DigitalChannel leftLiftLimitSwitch1;
    private DigitalChannel leftLiftLimitSwitch2;
    private DigitalChannel rightLiftLimitSwitch1;
    private DigitalChannel rightLiftLimitSwitch2;


    public Lifter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        liftLeft = hardwareMap.get(DcMotor.class, "liftL");
        liftRight = hardwareMap.get(DcMotor.class, "liftR");
        liftLeftLatch = new ServoPlus(hardwareMap.get(Servo.class, "leftLiftServo"), 200, 0, 200);
        liftRightLatch = new ServoPlus(hardwareMap.get(Servo.class, "rightLiftServo"), 200, 0, 200);
        leftLiftLimitSwitch1 = hardwareMap.get(DigitalChannel.class, "liftLimitSwitchL1");
        rightLiftLimitSwitch1 = hardwareMap.get(DigitalChannel.class, "liftLimitSwitchR1");
        leftLiftLimitSwitch2 = hardwareMap.get(DigitalChannel.class, "liftLimitSwitchL2");
        rightLiftLimitSwitch2 = hardwareMap.get(DigitalChannel.class, "liftLimitSwitchR2");
        servos = new ServoPlus[]{liftLeftLatch, liftRightLatch};
        lifts = new DcMotor[]{liftLeft, liftRight};
        for (DcMotor lift : lifts) {
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        leftLiftController = new PIDFController(pCon, iCon, dCon, fCon, new ElapsedTime());
        rightLiftController = new PIDFController(pCon, iCon, dCon, fCon, new ElapsedTime());

        //Potentially reverse in the future, TODO:Check
        liftLeft.setDirection(FORWARD);
        liftRight.setDirection(FORWARD);
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
    public void update(){
        leftLiftController.update(ticksToInches(liftLeft.getCurrentPosition()));
        rightLiftController.update(ticksToInches(liftRight.getCurrentPosition()));
        liftLeft.setPower(leftLiftController.getPower());
        liftRight.setPower(rightLiftController.getPower());
        checkHomingSwitches();
        home();
    }
    public double ticksToInches(int ticks){
        //TODO: Actually make this work
        return 0.0;
    }
    public double inchesToTicks(double inches){
        //TODO: Actually make this work
        return 0.0;
    }
    public void lockLatches(){
        if (!unlached) {//Find values
            double leftLockPos = 0;
            double rightLockPos = 0;
            liftLeftLatch.setServoPos(leftLockPos);
            liftRightLatch.setServoPos(rightLockPos);
        }
    }
    public void unlockLatches(){
        //Find values
        double leftUnlockPos = 0;
        double rightUnlockPos = 0;
        liftLeftLatch.setServoPos(leftUnlockPos);
        liftRightLatch.setServoPos(rightUnlockPos);
        unlached = true;
    }
    public boolean leftHomingSwitchesPressed(){
        return leftLiftLimitSwitch1.getState() || leftLiftLimitSwitch2.getState();
    }
    public boolean rightHomingSwitchesPressed(){
        return rightLiftLimitSwitch1.getState() || rightLiftLimitSwitch2.getState();
    }
    public void checkHomingSwitches(){
        if (leftHomingSwitchesPressed()){
            liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (rightHomingSwitchesPressed()){
            liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        setLiftPosition(1);
    }
    public double getLiftPosition(){
        int position = (liftLeft.getCurrentPosition() + liftRight.getCurrentPosition())/2;
        return ticksToInches(position);
    }
}