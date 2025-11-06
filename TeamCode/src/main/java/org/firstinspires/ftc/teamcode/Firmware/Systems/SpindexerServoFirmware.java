package org.firstinspires.ftc.teamcode.Firmware.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Current class assumptions:
 *The servo has a "dead spot"
 */
public class SpindexerServoFirmware {
    private final Servo spindexerServo;
    // the degree values for each slot
    private final double[] slots;
    private final double direction;

    // The servo degree that is the current target
    private double targetPosition = 0;

    // how many degrees of tolerance there will be for the isAtTarget() Function to return true
    private int positionTolerance = 4;

    // Warp speed exit tolerance - The servo will always spin in one direction at full continuous speed until
    // we get close enough to the target position that the servo will be in range (not in the gap area outside of its it's range)
    // At this point, we will directly address the servo PWM to the position that we are trying to stop at.
    // This is the solution to being able to always turn one way and also use the limited range servo features of this servo.


    private final DcMotor spindexerEncoderMotorInstance;


    public SpindexerServoFirmware(HardwareMap hardwareMap, boolean spinClockwise, double slot1, double slot2, double slot3, String encoderConfigAddress){
        this.slots = new double[] {slot1,slot2,slot3};
        spindexerServo = hardwareMap.get(Servo.class, "spindexer servo");
        spindexerEncoderMotorInstance = hardwareMap.get(DcMotor.class, encoderConfigAddress);

        if (spinClockwise){
            direction = -1;
        } else{
            direction = 1;
        }
    }

    public void update(){
        final double warpSpeedExitTolerance = 15;
        double encoderPosition = getEncoderPosition();
        if (Math.abs(encoderPosition - targetPosition) <= warpSpeedExitTolerance){
            spindexerServo.setPosition(degreesToServoPWM(targetPosition));
        } else {
            spindexerServo.setPosition(direction);
        }
    }

    public void setSpindexerPosition(double position){
        if (position > 360){
            position = position % 360;
        }
        position = clipPositionToRange(position);
        targetPosition = position;
        update();
    }

    // valid for slots 1-3
    public void setSpindexerSlot(int slot){
        slot = slot % 3;
        setSpindexerPosition(slots[slot-1]);
    }

    public void setPositionTolerance(int toleranceTicks){
        positionTolerance = toleranceTicks;
    }
    public boolean isAtTarget(){
        return Math.abs(getEncoderPosition()-targetPosition) < positionTolerance;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getEncoderPosition(){
        return this.encoderTicksToDegrees(spindexerEncoderMotorInstance.getCurrentPosition());
    }
    public void calibrationPosition(){
        spindexerServo.setPosition(degreesToServoPWM(0));
    }
    public void resetEncoderPosition(){
        spindexerEncoderMotorInstance.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerEncoderMotorInstance.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private double encoderTicksToDegrees(int ticks){
        // According to product page [https://www.revrobotics.com/rev-11-1271/] There are 8192 ticks per full revolution in this encoder. 360 degrees in a rotation, and the spindexer rotation to the encoder is 1:1.
        double rotationsPerTick = 1.0/8192.0;
        double degreesPerRotation = 360;
        return ((double)ticks * rotationsPerTick * degreesPerRotation);
    }

    private double degreesToServoPWM(double degrees){


//        Assuming that the product page for the SWYFT Balance servo (the one used here) is correct: Full range is 270.
//        We want 15 degrees of space on each side of our main target points ( (270-(120 * 2) )/ 2

        double rangeSpacing = 15; // degrees

        double servoRange = 270;

        // ensure the input of this function returns the correct range of values and

        degrees = Range.clip(degrees, 0, servoRange - rangeSpacing) + rangeSpacing;

        double degreesToPWM = (double) 1 / servoRange;

        double originalPWM = degrees * degreesToPWM ;

        // TODO: find these values using the servo tuning testing op-mode!
        double minimumPWM = 0.05;
        double maximumPWM = 0.95;

        return Range.clip(originalPWM,minimumPWM,maximumPWM);
    }
    private double clipPositionToRange( double position){
        final double controlledRangeMinDegree = 10;
        final double controlledRangeMaxDegree = 250;
        return Range.clip(position, controlledRangeMinDegree, controlledRangeMaxDegree);
    }

}
