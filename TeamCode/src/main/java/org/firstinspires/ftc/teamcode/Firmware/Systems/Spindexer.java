package org.firstinspires.ftc.teamcode.Firmware.Systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor.COLORS;
import org.firstinspires.ftc.teamcode.Resources.ServoPlus;

public class Spindexer {

    private final SpindexerServoFirmware paddleServo;

//    private final SpindexerColorSensor colorSensor; - Not needed for scrimmage - Tobin 11/6

//    private COLORS[] indexColors = { - Not needed for scrimmage - Tobin 11/6
//      COLORS.NONE,
//      COLORS.NONE,
//      COLORS.NONE
//    };

    private final ElapsedTime runtime;
    private boolean ejecting = false;
    private double ejectionTimeout = 0.6;
    private final int slotIncrement = 120; // there is a new slot every 120 degrees

    private final ServoPlus racketServo;
    private double racketOutPos = 0.0;
    private double racketInPos = 0.0;
    private boolean calibrating = false;
    private double calibrationTimeout = 0.7;
    public Spindexer(HardwareMap hardwareMap){
        paddleServo = new SpindexerServoFirmware(hardwareMap,false,0,120,240,"intake");
        runtime = new ElapsedTime();
//        range of motion does not just need to be in degrees for the ServoPLus class. In this case, we are using inches because this should be linear.
        racketServo = new ServoPlus(hardwareMap.get(Servo.class,"racket"),5,0,5);
//        colorSensor = new SpindexerColorSensor(hardwareMap, "spindexerColorSensor"); - Not needed for scrimmage, Tobin 11/6
    }

    public void updateSpindexer(){
        if (!calibrating){
            if (runtime.seconds() >= ejectionTimeout){
                moveRacket(false);
            }
        }else{
            if (runtime.seconds() >= calibrationTimeout){
                calibrating = false;
                paddleServo.resetEncoderPosition();
            }
        }
            paddleServo.update();
    }

//    public void prepColor(COLORS color){ - Not needed for scrimmage - Tobin 11/6
//        if (color == COLORS.NONE) {
//            return;
//        }
//
//        // figure out what index the correct color is in
//        // move that index to launch pos
//    }
//    zero is index 1 at intake, positive moves counterclockwise facing intake, so 120 will be index 1 at launcher
//    public void moveIndexToLaunch(int index){ - Not needed for scrimmage - Tobin 11/6
//        if (index < 1 || index > 3){
//            return;
//        }
//        if (ejecting) return;
//        double pos = (index * slotIncrement) % 360;
//        paddleServo.setSpindexerPosition(pos);
//    }
//
//    public void moveIndexToIntake(int index){ - Not needed for scrimmage - Tobin 11/6
//        if (index < 1 || index > 3){
//            return;
//        }
//        if (ejecting) return;
//        double pos = ((index-1) * slotIncrement);
//        paddleServo.setSpindexerPosition(pos);
//    }
    public void eject(){
        if (paddleServo.isAtTarget() && paddleServo.getTargetPosition() % slotIncrement == 0){
            moveRacket(true);
            runtime.reset();
        }
    }

    public void recalibrateSpindexerPosition(){
        moveRacket(false);
        paddleServo.calibrationPosition();
        calibrating = true;
        runtime.reset();
    }
    public boolean isAtRest(){
        return paddleServo.isAtTarget();
    }

    // returns -1 if it is not at a slot.
    public int getCurrentIndexInIntake(){
        if (paddleServo.getTargetPosition()%120 == 0){
            return ((int)paddleServo.getTargetPosition() / slotIncrement) + 1;
        }else{
            return -1;
        }
    }

    // returns -1 if it is not at a slot.
    public int getCurrentIndexInLaunch(){
        if (paddleServo.getTargetPosition()%120 == 0){
            return ((int)paddleServo.getTargetPosition() / slotIncrement) + 2;
        }else{
            return -1;
        }
    }
    public void moveToNextIndex(){
       double pos = getCurrentIndexInIntake() + 1;
       paddleServo.setSpindexerPosition(pos);
    }


    private void moveRacket(boolean pushOut){
        if (pushOut){
            racketServo.setServoPos(racketOutPos);
            ejecting = true;
        }else{
            racketServo.setServoPos(racketInPos);
            ejecting = false;
        }
    }


}
