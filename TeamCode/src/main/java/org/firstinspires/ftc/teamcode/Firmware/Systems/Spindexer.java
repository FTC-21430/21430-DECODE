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

    public COLORS[] indexColors = {
      COLORS.NONE,
      COLORS.NONE,
      COLORS.NONE
    };

    private final ElapsedTime RUNTIME; // Timer for managing ejection and calibration timeouts.
    private boolean ejecting = false; // Indicates if the spindexer is currently ejecting.
    public static double ejectionTimeout = 0.3; // Timeout duration for ejection in seconds.
    private final int SLOTH_INCREMENT = 120; // Degrees between slots.
    private final Servo EJECTOR_SERVO; // Servo for controlling the ejector mechanism.
    private double ejectorOutPos = 0.7; // Position of the ejector when pushed out.
    private double ejectorInPos = 0.42; // Position of the ejector when retracted.
    private boolean calibrating = false; // Indicates if the spindexer is in calibration mode.
    private double calibrationTimeout = 1.2; // Timeout duration for calibration in seconds.
    private Telemetry telemetry; // telemetry instance stored from constructor, helps for debugging and quick testing. Not required for base function but is still useful
    private boolean ejectorOut = false;
    /**
     * Constructor initializes the spindexer components.
     * @param hardwareMap Hardware map to retrieve hardware instances.
     */
    public Spindexer(HardwareMap hardwareMap, Telemetry telemetry, boolean reset) {
        PADDLE_SERVO = new SpindexerServoFirmware(hardwareMap, true, 0, 120, 240, "intake",telemetry);
        RUNTIME = new ElapsedTime();
        this.telemetry = telemetry;
        // Range of motion for the ServoPlus class is in inches for linear movement.
        EJECTOR_SERVO = hardwareMap.get(Servo.class, "ejector");
        COLOR_SENSOR = new SpindexerColorSensor(hardwareMap, "colorSensor");
        recalibrateSpindexerPosition();

        intakeLimitSwitchOne = hardwareMap.get(DigitalChannel.class, "intakeLimitSwitchOne");
        intakeLimitSwitchTwo = hardwareMap.get(DigitalChannel.class, "intakeLimitSwitchTwo");
        intakeLimitSwitchOne.setMode(DigitalChannel.Mode.INPUT);
        intakeLimitSwitchTwo.setMode(DigitalChannel.Mode.INPUT);
    }

    /**
     * Updates the spindexer state, handling ejection and calibration timeouts.
     */
    public void updateSpindexer() {
        if (!calibrating) {
            if (RUNTIME.seconds() >= ejectionTimeout) {
                moveEjector(false); // Retracts the ejector after ejection timeout.
            }
            if (RUNTIME.seconds() >= ejectionTimeout*2){
                ejectorOut = false;
            }
        } else {
            if (RUNTIME.seconds() >= calibrationTimeout) {
                calibrating = false; // Ends calibration mode after timeout.
                PADDLE_SERVO.resetEncoderPosition();
            }
        }
            telemetry.addData("Encoder", getEncoderPosition());
            telemetry.addData("target", PADDLE_SERVO.getTargetPosition());
            PADDLE_SERVO.update(); // Updates the spindexer servo position.
    }

    public void prepColor(COLORS color){
        if (color == COLORS.NONE) {
            int launchIndex = getCurrentIndexInLaunch();
            if (launchIndex == -1) return;
            if (indexColors[launchIndex - 1] == COLORS.NONE){
                int purple = getIndexWithColor(COLORS.PURPLE);
                if (purple != -1){
                    moveIndexToLaunch(purple);
                } else {
                    int green = getIndexWithColor(COLORS.GREEN);
                    if (green != -1){
                        moveIndexToLaunch(green);
                    }
                }
            }
        } else {
            int idx = getIndexWithColor(color);
            if (idx != -1) {
                moveIndexToLaunch(idx);
            }
        }
    }
//    zero is index 1 at intake, positive moves counterclockwise facing intake, so 120 will be index 1 at launcher
    public void moveIndexToLaunch(int index){
        if (index < 1 || index > 3){
            return;
        }
        if (ejecting) return;
        double pos = (index * SLOTH_INCREMENT) % 360;
        PADDLE_SERVO.setSpindexerPosition(pos);
    }

    public int getIndexWithColor(COLORS color){
        for (int i = 0; i < indexColors.length; i++){
            if (indexColors[i] == color){
                return i + 1;
            }
        }
        return -1; // if we do not find the color
    }
    public void moveIndexToIntake(int index){
        if (index < 1 || index > 3){
            return;
        }
        if (ejecting) return;
        double pos = ((index-1) * SLOTH_INCREMENT);
        PADDLE_SERVO.setSpindexerPosition(pos);
    }

    public boolean isEjectorOut(){
        return ejectorOut;
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
        indexColors[index-1] = COLORS.NONE;
    }

    /**
     * Ejects the current item if the spindexer is at a valid slot.
     */
    public void eject(){
        if (PADDLE_SERVO.isAtTarget() && PADDLE_SERVO.getTargetPosition() % SLOTH_INCREMENT == 0){
            moveEjector(true); // Pushes the ejector out for ejection.
            RUNTIME.reset(); // Resets the timer for ejection timeout.
        }
    }

    /**
     * Recalibrates the spindexer position to the starting point.
     */
    public void recalibrateSpindexerPosition(){
        moveEjector(false); // Ensures the ejector is retracted.
        PADDLE_SERVO.calibrationPosition(); // Moves the spindexer to the calibration position.
        calibrating = true; // Sets the spindexer to calibration mode.
        RUNTIME.reset(); // Resets the timer for calibration timeout.
    }
    /**
     * Checks if the spindexer is at rest (not moving).
     * @return True if at rest, false otherwise.
     */
    public boolean isAtRest(){
        return PADDLE_SERVO.isAtTarget();
    }

    /**
     * Gets the current index of the spindexer in the intake position.
     * @return Index number (1-3) or -1 if not at a valid slot.
     */
    public int getCurrentIndexInIntake(){
        switch ((int) PADDLE_SERVO.getTargetPosition()){
            case 0:
                return 1;
            case 120:
                return 2;
            case 240:
                return 3;
        }
        return -1;
    }

    /**
     * Gets the current index of the spindexer in the launch position.
     * @return Index number (1-3) or -1 if not at a valid slot.
     */
    public int getCurrentIndexInLaunch(){
        switch ((int) PADDLE_SERVO.getTargetPosition()){
            case 0:
                return 3;
            case 120:
                return 1;
            case 240:
                return 2;
        }
        return -1;
    }

    /**
     * Moves the spindexer to the next index position.
     */
    public void moveToNextIndex(){
        if (ejecting) return;
       int pos = getCurrentIndexInIntake() + 1; // Calculates the next index position.
       PADDLE_SERVO.setSpindexerSlot(pos); // Moves the spindexer to the calculated position.
    }

    /**
     * Moves the ejector to the specified position (out or in).
     * @param pushOut True to push the ejector out, false to retract it.
     */
    private void moveEjector(boolean pushOut){
        if (pushOut){
            EJECTOR_SERVO.setPosition(ejectorOutPos); // Sets the ejector to the out position.
            ejecting = true; // Marks the spindexer as ejecting.
            ejectorOut = true;
        }else{
            EJECTOR_SERVO.setPosition(ejectorInPos); // Sets the ejector to the in position.
            ejecting = false; // Marks the spindexer as not ejecting.
        }
    }

    /**
     * Commands the spindexer paddle to turn to a specific degree of rotation
     * @param degree - degrees counter clockwise from the init position
     */
    public void setSpindexerPos(double degree){
        if (ejecting) return;
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
}