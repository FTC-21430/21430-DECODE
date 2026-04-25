package org.firstinspires.ftc.teamcode.Firmware.Systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor.COLORS;
import java.util.Arrays;

/**
 * Spindexer class manages the spindexer mechanism, including its servo and ejector.
 * It handles movement between slots, ejection, and calibration.
 */
@Config
public class Spindexer {
    private final SpindexerServoFirmware PADDLE_SERVO; // Firmware for controlling the spindexer servo.

    private final SpindexerColorSensor COLOR_SENSOR;
    private DigitalChannel intakeLimitSwitchOne = null;
    private DigitalChannel intakeLimitSwitchTwo = null;

    // Stored by intake index, not by launch order.
    public COLORS[] indexColors = {
      COLORS.GREEN,
      COLORS.PURPLE,
      COLORS.PURPLE
    };

    private final ElapsedTime RUNTIME; // Timer for managing ejection and calibration timeouts.
    private final int SLOT_INCREMENT = 120; // Degrees between slots.
    private boolean calibrating = false; // Indicates if the spindexer is in calibration mode.
    private double calibrationTimeout = 0.6; // Timeout duration for calibration in seconds.
    private Telemetry telemetry; // telemetry instance stored from constructor, helps for debugging and quick testing. Not required for base function but is still useful
    public static double intakeOffSet = -8;
    public static double launchOffSet = -5;
    public static double idleOffSet = 0;

    public enum INDEX_TYPE{
        INTAKE,
        LAUNCH,
        NONE
    }


    /**
     * Constructor initializes the spindexer components.
     * @param hardwareMap Hardware map to retrieve hardware instances.
     */
    public Spindexer(HardwareMap hardwareMap, Telemetry telemetry, boolean reset, boolean autonomous) {
        PADDLE_SERVO = new SpindexerServoFirmware(hardwareMap, true, 0, 120, 240, "intake",telemetry);
        RUNTIME = new ElapsedTime();
        this.telemetry = telemetry;
        COLOR_SENSOR = new SpindexerColorSensor(hardwareMap, "colorSensor1","colorSensor2");

        intakeLimitSwitchOne = hardwareMap.get(DigitalChannel.class, "intakeLimitSwitchOne");
        intakeLimitSwitchTwo = hardwareMap.get(DigitalChannel.class, "intakeLimitSwitchTwo");
        intakeLimitSwitchOne.setMode(DigitalChannel.Mode.INPUT);
        intakeLimitSwitchTwo.setMode(DigitalChannel.Mode.INPUT);
        if (reset){
            recalibrateSpindexerPosition();
            while (RUNTIME.seconds() < 0.3){
                telemetry.addLine("SPINDEXER INIT");
                PADDLE_SERVO.calibrateSpinny();
                telemetry.update();
            }
            PADDLE_SERVO.zeroPosition();
        }

    }

    private int stoppedSampling = 0;

    /**
     * Updates the spindexer state, handling ejection and calibration timeouts.
     */
    public void updateSpindexer() {
        telemetry.addData("CurrentIntake0", indexColors[0]);
        telemetry.addData("CurrentIntake1", indexColors[1]);
        telemetry.addData("CurrentIntake2", indexColors[2]);
        if (!calibrating) {

        } else {
            if (RUNTIME.seconds() >= calibrationTimeout) {
                calibrating = false; // Ends calibration mode after timeout.
                PADDLE_SERVO.resetEncoderPosition();
            }
        }

            PADDLE_SERVO.update(); // Updates the spindexer servo position.
        // rest is based on repeated target hits, not raw velocity. this is a small debounce for state logic.
        if (PADDLE_SERVO.isAtTarget()){
            ++stoppedSampling;
        }else{
            stoppedSampling = 0;
        }
    }

    public void eject(double degrees){
        PADDLE_SERVO.launchSpin(degrees);
    }

    public void setIndexOffset(INDEX_TYPE type){
        // offsets change which physical feature lines up with a slot without changing the chosen slot itself.
        switch (type) {
            case INTAKE:
                PADDLE_SERVO.setSpindexerOffset(intakeOffSet);
                break;
            case LAUNCH:
                PADDLE_SERVO.setSpindexerOffset(launchOffSet);
                break;
            case NONE:
                PADDLE_SERVO.setSpindexerOffset(idleOffSet);
                break;
        }
    }
    public void prepLaunch(COLORS[] launchSequence){
        int launchIndex = getSortedIndex(launchSequence, getCurrentIndexInLaunch());
        // prep should be the non-eject direction so the passive ejector folds in instead of pushing a ball out.
        setPaddleDirection(false);
        setIndexOffset(INDEX_TYPE.LAUNCH);
        moveIndexToLaunch(launchIndex);
    }
    public double getVelocity(){
        return PADDLE_SERVO.getMovementVelocity();
    }

    /**
     * Algorithm for determining the best launch order, will return the index that should move to the launch position. NOTE: reference to the launch slot of the spindexer, while other parts use the intake!!!
     * @param launchSequence an array of three colors, can handle all types: PURPLE, GREEN, NONE
     * @param currentIdx the current spindexer index that is in the launch position to have the order prioritize the index we are already in,
     *                   for the edge case of not actually having a perfect match such as [G,G,P] where the target is [G,P,P]
     * @return the index that should be in the LAUNCH position. spinning in the launching direction will cause the correct order to leave the launcher. (high throughput rates will likely cause the order to be lost inside the goal.
     */
     private int getSortedIndex(COLORS[] launchSequence, int currentIdx){
         // best count is the 'high score' for the pattern matching, the best Index stores which position is currently the best
        int bestCount = 0, bestIndex = -1;
        // starts from currentIdx so ties prefer the smallest movement from where we already are.
        for (int i = currentIdx; i < currentIdx+3; i++){
            // the iteration counts, match and none. Because none balls would not actually shoot a ball, Nones are skipped and instead only counts the next index for a match. matchCount is how many matches this index has
            int matchCount = 0 , noneCount = 0;
            // j is the single color in the pattern, and we iterate through all three.
            for (int j = 0; j < 3; j++){
                // gets the color at the index of the launch slot, the +1 ensures that we get the correct artifact.
                COLORS color = indexColors[(i+j+1)%3];
                // check if the color is a none, if it is skip it and signal to the next iteration that we skipped one.
                if (color == COLORS.NONE){
                    noneCount++;
                    continue;
                }
                if (j-noneCount >= 0) {
                    // check if there is a match the the target launch order, MOD 3 to ensure correct wrapping. noneCount is used to know if we skipped an index without shooting another color.
                    if (color == launchSequence[(j - noneCount) % 3]) {
                        matchCount++;
                    }
                }
            }
            // check if the latest was the best, if it is less than or equal to than we use the earlier one because it is closer to our current position
            if (matchCount > bestCount){
                bestCount = matchCount;
                bestIndex = i % 3; // MOD 3 for keeping inside the 0-2 range
                if (bestCount == 3){ // in the edge case that we find the perfect match, there is no reason to continue searching.
                    break;
                }
            }
        }

        if (bestIndex == -1){
            // all NONE or no useful match: keep the current launch slot instead of inventing a new one.
            return currentIdx % 3;
        }

        // returns the best result. If all Nones were passed or some other strange launch sequence
         bestIndex += 1;
        if (bestIndex < 0) bestIndex = 2;
        if (bestIndex > 2) bestIndex = 0;
        return bestIndex;
     }



//    zero is index 1 at intake, positive moves counterclockwise facing intake, so 120 will be index 1 at launcher
    public void moveIndexToLaunch(int index){
        // launch indexing is intentionally not the same frame as intake indexing.
        index = index % 3;
        double pos = (index * SLOT_INCREMENT) % 360;
        PADDLE_SERVO.setSpindexerPosition(pos);
    }
    public void moveIndexToIntake(int index){
        if (index < 1 || index > 3){
            return;
        }
        double pos = ((index-1) * SLOT_INCREMENT);
        PADDLE_SERVO.setSpindexerPosition(pos);
    }

    public void storeColorAtIndex(){
        if (getCurrentIndexInIntake() == -1){
            return;
        }
        indexColors[getCurrentIndexInIntake()-1] = COLOR_SENSOR.getDetectedColor();
    }
    public COLORS getColorInIntake(){
        return COLOR_SENSOR.getDetectedColor();
    }
    public void clearColor(int index){
        indexColors[index] = COLORS.NONE;
    }
    public void setColorIndexing(COLORS index1, COLORS index2, COLORS index3){
        indexColors = new COLORS[]{
                index1,index2,index3
        };
    }


    /**
     * Recalibrates the spindexer position to the starting point.
     */
    public void recalibrateSpindexerPosition(){
        PADDLE_SERVO.calibrationPosition(); // Moves the spindexer to the calibration position.
        calibrating = true; // Sets the spindexer to calibration mode.
        RUNTIME.reset(); // Resets the timer for calibration timeout.
    }

    public static int stoppedSamplingThreshold = 2;
    /**
     * Checks if the spindexer is at rest (not moving).
     * @return True if at rest, false otherwise.
     */
    public boolean isAtRest(){
        if (PADDLE_SERVO.getRemainingSpin() > 0) return false;
        return stoppedSampling >= stoppedSamplingThreshold && !PADDLE_SERVO.spinning;
    }
    /**
     * Gets the current index of the spindexer in the intake position.
     * @return Index number (1-3) or -1 if not at a valid slot.
     */
    public int getCurrentIndexInIntake(){
        // this uses the commanded target frame, not the live encoder frame.
        switch ((int) Math.round(PADDLE_SERVO.getTargetPosition()/120)){
            case 0:
                return 1;
            case 1:
                return 2;
            case 2:
                return 3;
        }
        return -1;
    }

    /**
     * Gets the current index of the spindexer in the launch position.
     * @return Index number (1-3) or -1 if not at a valid slot.
     */
    public int getCurrentIndexInLaunch(){
        // launch slot numbering is rotated relative to intake because the exit hole is on a different face.
        switch ((int) Math.round(PADDLE_SERVO.getTargetPosition()/120)){
            case 0:
                return 3;
            case 1:
                return 1;
            case 2:
                return 2;
            default:
                return 1;
        }
    }

    /**
     * Moves the spindexer to the next index position.
     */
    public void moveToNextIndex(){
        setPaddleDirection(false);
       int pos = (getCurrentIndexInIntake()-1)%3; // Calculates the next index position.
       PADDLE_SERVO.setSpindexerSlot(pos); // Moves the spindexer to the calculated position.
    }

    /**
     * Commands the spindexer paddle to turn to a specific degree of rotation
     * @param degree - degrees counter clockwise from the init position
     */
    public void setSpindexerPos(double degree){
        PADDLE_SERVO.setSpindexerPosition(degree);
    }

    /**
     * Returns the current rotation that is provided by the spindexer encoder
     * @return degrees of rotation
     */
    public double getEncoderPosition(){
        return PADDLE_SERVO.getEncoderPosition();
    }

    public boolean getIntakeSwitch(){
        return intakeLimitSwitchOne.getState() || intakeLimitSwitchTwo.getState();
    }
    public boolean isFull(){
     return Arrays.stream(indexColors).noneMatch(c -> c == COLORS.NONE);
    }
    public double getTarget(){
        return PADDLE_SERVO.getTargetPosition();
    }
    private void setPaddleDirection(boolean clockwise){
        PADDLE_SERVO.setDirection(clockwise);
    }

    public int getNumberOfArtifacts(){
        int count = 0;
        for (COLORS slot : indexColors){
            if (slot != COLORS.NONE){
                count ++;
            }
        }
        return count;
    }
}

