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
    private double encoderPosition; // in degrees

    // The servo degree that is the current target
    private double targetPosition = 0;

    // how many degrees of tolerance there will be for the isAtTarget() Function to return true
    private int positionTolerance = 4;

    // Warp speed exit tolerance - The servo will always spin in one direction at full continuous speed until
    // we get close enough to the target position that the servo will be in range (not in the gap area outside of its it's range)
    // At this point, we will directly address the servo PWM to the position that we are trying to stop at.
    // This is the solution to being able to always turn one way and also use the limited range servo features of this servo.

    private double warpSpeedExitTolerance = 15;
    private DcMotor spindexerEncoderMotorInstance;

    private double controlledRangeMinDegree = 10;
    private double controlledRangeMaxDegree = 250;
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
        encoderPosition = getEncoderPosition();
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
        if (slot > 0 && slot < 4)
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
    private double encoderTicksToDegrees(int ticks){
        return ticks; // TODO: insert regression function here.
    }

    private double degreesToServoPWM(double degrees){
        return 0.0; //TODO: make this regression function
    }

    private double clipPositionToRange( double position){
        return Range.clip(position, controlledRangeMinDegree, controlledRangeMaxDegree);
    }
}
