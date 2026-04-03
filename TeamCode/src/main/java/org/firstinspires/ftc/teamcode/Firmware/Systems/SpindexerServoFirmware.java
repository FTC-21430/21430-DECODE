package org.firstinspires.ftc.teamcode.Firmware.Systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config

/**
 * SpindexerServoFirmware class controls the spindexer servo and its associated encoder.
 * It manages the servo's movement to specific positions and ensures accurate calibration.
 */
public class SpindexerServoFirmware {
    private final Servo spindexerServo; // Servo controlling the spindexer.
    private final double[] slots; // Degree values for each slot.
    // This is continuous-servo power, not a target angle. the tuned values matter more than the names here.
    public static double direction; // Direction of servo movement (-1 for clockwise, 1 for counterclockwise).

    // The servo degree that is the current target
    private double targetPosition = 0;

    // how many degrees of tolerance there will be for the isAtTarget() Function to return true
    public static int positionTolerance = 38;
    // what the PWM signal is for the zero position of the servo
    public static double pwmAtZeroDegrees = 0.71;

    // offset shifts the logical frame for intake / launch / idle without changing encoder zero.
    private double positionOffset = 0.0;

    private double lastRawPosition;
    private boolean isJamed = false;

    public static double jamThreshold = 0.1;

    private double lastTarget;

    private double attemptedTarget;

    private ElapsedTime deltaRuntime;
    private ElapsedTime jamRuntime;
    private Telemetry telemetry;
    private int jamCount = 0;

    private final DcMotor spindexerEncoderMotorInstance; // Encoder motor instance for position tracking.

    /**
     * Constructor initializes the spindexer servo and encoder.
     * @param hardwareMap Hardware map to retrieve servo and encoder instances.
     * @param spinClockwise Direction of servo movement (true for clockwise).
     * @param slot1 Degree value for slot 1.
     * @param slot2 Degree value for slot 2.
     * @param slot3 Degree value for slot 3.
     * @param encoderConfigAddress Configuration address for the encoder motor.
     */
    public SpindexerServoFirmware(HardwareMap hardwareMap, boolean spinClockwise, double slot1, double slot2, double slot3, String encoderConfigAddress, Telemetry telemetry){
        this.slots = new double[] {slot1,slot2,slot3};
        spindexerServo = hardwareMap.get(Servo.class, "spindexer");
        spindexerEncoderMotorInstance = hardwareMap.get(DcMotor.class, encoderConfigAddress);
        spindexerEncoderMotorInstance.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexerServo.setDirection(Servo.Direction.FORWARD);
        // Set direction based on spinClockwise parameter.
        direction = spinClockwise ? 0.17 : 0.83;
        deltaRuntime = new ElapsedTime();
        jamRuntime = new ElapsedTime();
        this.telemetry = telemetry;
    }

    // Warp speed exit tolerance - The servo will always spin in one direction at full continuous speed until
    // we get close enough to the target position that the servo will be in range (not in the gap area outside of its it's range)
    // At this point, we will directly address the servo PWM to the position that we are trying to stop at.
    // This is the solution to being able to always turn one way and also use the limited range servo features of this servo.
    public static double warpSpeedExitTolerance = 65; // Tolerance for exiting warp speed.
    private double encoderPosition = 0;
    private double rawEncoderPosition = 0;
    public static double jamFreedTimeout = 0.18;
    public static int jamsAmount = 16;
    private boolean hasReachedTarget = false;
    private double deltaTime = 0;
    public static double spinningEndThreshold = 20; //degrees
    /**
     * Updates the servo position based on the target position and tolerance.
     */
    public void update(){
        updateDeltaTime();
        sensorUpdate();

        if (spinning){
            // SPPPIIIIIINNNN!!!!
            // launch spin ignores precise hold on purpose; the goal here is throughput, not exact intermediate positioning.
            spindexerServo.setPosition(direction);
            // subtract measured encoder travel so battery sag changes time, not total spin distance.
            remainingSpin -= Math.abs(deltaTime * getMovementVelocity());
            if (remainingSpin <= spinningEndThreshold){
                remainingSpin = 0;
                spinning = false;
                // after ejecting, return to the normal prep/positioning direction.
                setDirection(true);
            }
            return; // do not enter precise control
        }

        if (isJamed) {
            if (jamRuntime.seconds() >= jamFreedTimeout){
                // retry the original target after sitting on the backed-up position for a short time.
                targetPosition = attemptedTarget;
                hasReachedTarget = false;
                isJamed = false;
            }
        } else {
            if (checkForJam()) {
                jamCount += 1;
            }else{
                jamCount = 0;
            }
            if (jamCount > jamsAmount) {
                isJamed = true;
                jamCount = 0;
                attemptedTarget = targetPosition;
                // jam recovery is just "undo the last move" and try again later.
                targetPosition = lastTarget;
                jamRuntime.reset();
            }
        }
        telemetry.addData("jamCount", jamCount);
        telemetry.addData("isJamed", isJamed);
        telemetry.addData("movement", getMovementVelocity());
        telemetry.addData("isAtTarget", isAtTarget());




        // once inside the handoff zone, switch from one-way spinning to direct position hold.
        if (isWithinPreciseControl()){
            spindexerServo.setPosition(degreesToServoPWM(targetPosition));
            hasReachedTarget = true;
            telemetry.addData("SPIN output", degreesToServoPWM(targetPosition));
        } else {
            if (!hasReachedTarget){
                spindexerServo.setPosition(direction);
                telemetry.addData("SPIN output", direction);
            }
        }
    }
    private boolean isWithinPreciseControl(){
        return getAngleDisplacement(Math.abs(encoderPosition), targetPosition) <= warpSpeedExitTolerance;
    }
    private void sensorUpdate(){
        lastRawPosition = rawEncoderPosition;
        rawEncoderPosition = getRawEncoderPosition();
        encoderPosition = getEncoderPosition();
    }

    private boolean checkForJam(){
        // only call it a jam when we should still be cruising at full speed.
        boolean isCloseToTarget = getAngleDisplacement(encoderPosition, targetPosition) < positionTolerance;
        double velocity = getMovementVelocity();
        boolean isTooSlow = Math.abs(velocity) < jamThreshold;
        // returns true only if we should be moving at full speed and we are not moving how we should be.
        return !isCloseToTarget && isTooSlow;
    }
    
    public double getMovementVelocity(){
        double delta = getDeltaTime();
        // correct delta time if it is zero to avoid divide by zero issues.

        double velocity = (rawEncoderPosition - lastRawPosition)/delta; // angular velocity in deg / sec
        return velocity;
    }
    public void setDirection(boolean clockwise){
        direction = clockwise? 0.17:0.83;
    }
    /**
     * Sets the spindexer to a specific position in degrees.
     * @param position Target position in degrees.
     */
    public void setSpindexerPosition(double position){
        hasReachedTarget = false;

        // callers give logical angles; the current mode offset is applied here.
        position += positionOffset;
        position = position % 360; // Wraps position within 360 degrees.
        position = clipPositionToRange(position);

        lastTarget = targetPosition;
        targetPosition = position;
        // TODO: double check that this is the correct direciton, I forgot which frame of reference this rotation is measured from (I remember from the intake side but it prolly should be launch side ¯\_(ツ)_/¯)
        update();
    }


    private double remainingSpin = 0;
    public boolean spinning = false;
    public void launchSpin(double degrees){
        // spin still sets a final target so the servo holds the post-launch resting angle when the burst ends.
        hasReachedTarget = false;
        double position = targetPosition + degrees;
        position = position % 360; // Wraps position within 360 degrees.
        position = clipPositionToRange(position);

        lastTarget = targetPosition;
        targetPosition = position;

        setDirection(true); // turn to shoot
        remainingSpin = degrees;
        spinning = true;

        update();
    }

    /**
     * Sets the spindexer to a predefined slot position.
     * @param slot Slot number (1-3). one outside of this range will loop back to the opposite index
     */
    public void setSpindexerSlot(int slot){
        if (slot < 1) slot = 3;
        if (slot > 3) slot = 1;
        direction = 0.83;
        setSpindexerPosition(slots[slot-1]);
    }

    public void setSpindexerOffset(double degrees){
        // preserve the current logical target when changing frames so we do not jump to a different slot.
        targetPosition -= (positionOffset-degrees);
        positionOffset = degrees;

    }

    /**
     * Sets the tolerance for position accuracy.
     * @param toleranceTicks Tolerance in encoder ticks.
     */
    public void setPositionTolerance(int toleranceTicks){
        positionTolerance = toleranceTicks;
    }

    /**
     * Checks if the spindexer is at the target position within the tolerance.
     * @return True if at target, false otherwise.
     */
    public boolean isAtTarget(){

        return getAngleDisplacement(encoderPosition,targetPosition) < positionTolerance && !isJamed && !spinning;
    }

    /**
     * Gets the current target position of the spindexer.
     * @return Target position in degrees.
     */
    public double getTargetPosition() {
        // report back in the logical frame expected by Spindexer, not the offset internal frame.
        return targetPosition - positionOffset;
    }

    /**
     * Retrieves the current position of the encoder in degrees.
     * @return Encoder position in degrees.
     */
    public double getEncoderPosition(){
        double pos = ((this.encoderTicksToDegrees(spindexerEncoderMotorInstance.getCurrentPosition())%360) + 360) % 360;
        return pos;
    }
    private double getRawEncoderPosition(){
        double pos = this.encoderTicksToDegrees(spindexerEncoderMotorInstance.getCurrentPosition());
        return pos;
    }

    /**
     * Moves the spindexer to the calibration position (0 degrees).
     */
    public void calibrationPosition(){
        // this only drives to the known pose; the outer class decides when it is safe to zero the encoder.
        spindexerServo.setPosition(pwmAtZeroDegrees);
        targetPosition = 0;
    }

    /**
     * Resets the encoder position to zero.
     */
    public void resetEncoderPosition(){
        spindexerEncoderMotorInstance.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerEncoderMotorInstance.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sensorUpdate();
        lastRawPosition = 0;
    }

    /**
     * Converts encoder ticks to degrees.
     * @param ticks Encoder ticks.
     * @return Equivalent position in degrees.
     */
    private double encoderTicksToDegrees(int ticks){
        // According to product page [https://www.revrobotics.com/rev-11-1271/] There are 8192 ticks per full revolution in this encoder. 360 degrees in a rotation, and the spindexer rotation to the encoder is 1:1.
        final double rotationsPerTick = 1.0/8192.0; // Encoder resolution.
        final double degreesPerRotation = 360; // Degrees in one rotation.
        return ticks * rotationsPerTick * degreesPerRotation;
    }

    /**
     * Converts degrees to servo PWM values.
     * @param degrees Target position in degrees.
     * @return Equivalent PWM value.
     */
    private double degreesToServoPWM(double degrees){

//        Assuming that the product page for the SWYFT Balance servo (the one used here) is correct: Full range is 270.
//        We want 15 degrees of space on each side of our main target points ( (270-(120 * 2) )/ 2

        final double rangeSpacing = 43; // Spacing in degrees for safety.
        final double servoRange = 318; // Full range of the servo in degrees.

        // Adjust degrees to fit within the servo's range.
        degrees = Range.clip(degrees, -rangeSpacing, servoRange - rangeSpacing);

        final double minimumPWM = 0.223; // Minimum PWM value.
        final double maximumPWM = 0.785; // Maximum PWM value.
        final double degreesToPWM =  -(maximumPWM-minimumPWM)/ servoRange; // Conversion factor.
        double resultingPWM = degrees * degreesToPWM + pwmAtZeroDegrees;
        return Range.clip(resultingPWM,minimumPWM,maximumPWM);
    }

    /**
     * Clips the position to a controlled range.
     * @param position Target position in degrees.
     * @return Clipped position within the valid range.
     */
    private double clipPositionToRange( double position){
        final double controlledRangeMinDegree = -1000; // Minimum valid degree.
        final double controlledRangeMaxDegree = 1000; // Maximum valid degree.
        return Range.clip(position, controlledRangeMinDegree, controlledRangeMaxDegree);
    }

    private void updateDeltaTime(){
        double time = deltaRuntime.seconds();
        deltaRuntime.reset();
        time = time <= 0? 0.0001:time;
        deltaTime = time;

    }

    /**
     * get milliseconds since this function was last called
     * @return
     */
    private double getDeltaTime(){
        return deltaTime;
    }
    public boolean getIfJammed(){
        return isJamed;

    }
    private double getAngleDisplacement(double a, double b){
        // shortest wrapped distance only. sign is ignored on purpose.
        a = Math.abs(a);
        b = Math.abs(b);
        double rawError = a - b;
        final double degreesInRotation = 360;
        double normalizedError = Math.abs((rawError+180) % degreesInRotation -180);
        return normalizedError;
    }
    public boolean isOverSlot(){
        //TODO Make this happen, we are depricating this feature to do TIME CRUNCH!!!
        return true;
    }
    public double getRemainingSpin(){
        return remainingSpin;
    }

}