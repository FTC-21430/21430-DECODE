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

    private final SpindexerServoFirmware paddleServo; // Firmware for controlling the spindexer servo.

    private final SpindexerColorSensor colorSensor;
    private DigitalChannel intakeLimitSwitchOne = null;
    private DigitalChannel intakeLimitSwitchTwo = null;

    public COLORS[] indexColors = {
      COLORS.NONE,
      COLORS.NONE,
      COLORS.NONE
    };

    private final ElapsedTime runtime; // Timer for managing ejection and calibration timeouts.
    private boolean ejecting = false; // Indicates if the spindexer is currently ejecting.
    public static double ejectionTimeout = 0.3; // Timeout duration for ejection in seconds.
    private final int slotIncrement = 120; // Degrees between slots.
    private final Servo ejectorServo; // Servo for controlling the ejector mechanism.
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
        paddleServo = new SpindexerServoFirmware(hardwareMap, true, 0, 120, 240, "intake",telemetry);
        runtime = new ElapsedTime();
        this.telemetry = telemetry;
        // Range of motion for the ServoPlus class is in inches for linear movement.
        ejectorServo = hardwareMap.get(Servo.class, "ejector");
        colorSensor = new SpindexerColorSensor(hardwareMap, "colorSensor");
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
            if (runtime.seconds() >= ejectionTimeout) {
                moveEjector(false); // Retracts the ejector after ejection timeout.
            }
            if (runtime.seconds() >= ejectionTimeout*2){
                ejectorOut = false;
            }
        } else {
            if (runtime.seconds() >= calibrationTimeout) {
                calibrating = false; // Ends calibration mode after timeout.
                paddleServo.resetEncoderPosition();
            }
        }
            telemetry.addData("Encoder", getEncoderPosition());
            telemetry.addData("target", paddleServo.getTargetPosition());
            paddleServo.update(); // Updates the spindexer servo position.
    }

    public void prepColor(COLORS color){
        if (color == COLORS.NONE) {
            if (indexColors[getCurrentIndexInLaunch()-1] == COLORS.NONE){
                if (getIndexWithColor(COLORS.PURPLE) != -1){
                    moveIndexToLaunch(getIndexWithColor(COLORS.PURPLE));
                }else if (getIndexWithColor(COLORS.GREEN) != -1){
                    moveIndexToLaunch(getIndexWithColor(COLORS.GREEN));
                }
            }
        }else {
            moveIndexToLaunch(getIndexWithColor(color));
        }
    }
//    zero is index 1 at intake, positive moves counterclockwise facing intake, so 120 will be index 1 at launcher
    public void moveIndexToLaunch(int index){
        if (index < 1 || index > 3){
            return;
        }
        if (ejecting) return;
        double pos = (index * slotIncrement) % 360;
        paddleServo.setSpindexerPosition(pos);
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
        double pos = ((index-1) * slotIncrement);
        paddleServo.setSpindexerPosition(pos);
    }

    public boolean isEjectorOut(){
        return ejectorOut;
    }

    public void storeColorAtIndex(){
        if (getCurrentIndexInIntake() == -1){
            return;
        }
        indexColors[getCurrentIndexInIntake()-1] = colorSensor.getDetectedColor();
    }
    public COLORS getColorInIntake(){
        return colorSensor.getDetectedColor();
    }
    public void clearColor(int index){
        indexColors[index-1] = COLORS.NONE;
    }

    /**
     * Ejects the current item if the spindexer is at a valid slot.
     */
    public void eject(){
        if (paddleServo.isAtTarget() && paddleServo.getTargetPosition() % slotIncrement == 0){
            moveEjector(true); // Pushes the ejector out for ejection.
            runtime.reset(); // Resets the timer for ejection timeout.
        }
    }

    /**
     * Recalibrates the spindexer position to the starting point.
     */
    public void recalibrateSpindexerPosition(){
        moveEjector(false); // Ensures the ejector is retracted.
        paddleServo.calibrationPosition(); // Moves the spindexer to the calibration position.
        calibrating = true; // Sets the spindexer to calibration mode.
        runtime.reset(); // Resets the timer for calibration timeout.
    }
    /**
     * Checks if the spindexer is at rest (not moving).
     * @return True if at rest, false otherwise.
     */
    public boolean isAtRest(){
        return paddleServo.isAtTarget();
    }

    /**
     * Gets the current index of the spindexer in the intake position.
     * @return Index number (1-3) or -1 if not at a valid slot.
     */
    public int getCurrentIndexInIntake(){
        if (paddleServo.getTargetPosition() % 120 == 0) {
            return ((int) paddleServo.getTargetPosition() / slotIncrement) + 1;
        } else {
            return -1; // Indicates the spindexer is not at a valid slot.
        }
    }

    /**
     * Gets the current index of the spindexer in the launch position.
     * @return Index number (1-3) or -1 if not at a valid slot.
     */
    public int getCurrentIndexInLaunch(){
        if (paddleServo.getTargetPosition()%120 == 0){
            return ((int)paddleServo.getTargetPosition() / slotIncrement) + 2;
        }else{
            return -1; // Indicates the spindexer is not at a valid slot.
        }
    }

    /**
     * Moves the spindexer to the next index position.
     */
    public void moveToNextIndex(){
        if (ejecting) return;
       int pos = getCurrentIndexInIntake() + 1; // Calculates the next index position.
       paddleServo.setSpindexerSlot(pos); // Moves the spindexer to the calculated position.
    }

    /**
     * Moves the ejector to the specified position (out or in).
     * @param pushOut True to push the ejector out, false to retract it.
     */
    private void moveEjector(boolean pushOut){
        if (pushOut){
            ejectorServo.setPosition(ejectorOutPos); // Sets the ejector to the out position.
            ejecting = true; // Marks the spindexer as ejecting.
            ejectorOut = true;
        }else{
            ejectorServo.setPosition(ejectorInPos); // Sets the ejector to the in position.
            ejecting = false; // Marks the spindexer as not ejecting.
        }
    }

    /**
     * Commands the spindexer paddle to turn to a specific degree of rotation
     * @param degree - degrees counter clockwise from the init position
     */
    public void setSpindexerPos(double degree){
        if (ejecting) return;
        paddleServo.setSpindexerPosition(degree);
    }

    /**
     * Returns the current rotation that is provided by the spindexer encoder
     * @return degrees of rotation
     */
    public double getEncoderPosition(){
        return paddleServo.getEncoderPosition();
    }

    public boolean getIntakeSwitch(){
        return intakeLimitSwitchOne.getState() || intakeLimitSwitchTwo.getState();
    }
    public boolean isFull(){
     return Arrays.stream(indexColors).noneMatch(c -> c == COLORS.NONE);
    }
}