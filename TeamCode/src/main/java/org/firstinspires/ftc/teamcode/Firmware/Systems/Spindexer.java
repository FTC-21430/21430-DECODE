package org.firstinspires.ftc.teamcode.Firmware.Systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor.COLORS;
import org.firstinspires.ftc.teamcode.Resources.ServoPlus;

public class Spindexer {

    private final SpindexerServoFirmware paddleServo;

    private final SpindexerColorSensor colorSensor;

    private COLORS[] indexColors = {
      COLORS.NONE,
      COLORS.NONE,
      COLORS.NONE
    };

    private final ElapsedTime runtime;
    private boolean ejecting = false;
    private double ejectionTimeout = 0.6;
    private final int slotIncrement = 120; // there is a new slot every 120 degrees

    private final ServoPlus racketServo;
    private double racketOutPos = 0.0;
    private double racketInPos = 0.0;
    public Spindexer(HardwareMap hardwareMap){
//        TODO: ensure that the slot values and the config address name are correct
        paddleServo = new SpindexerServoFirmware(hardwareMap,false,0,90,180,"intake");
        runtime = new ElapsedTime();
//        range of motion does not just need to be in degrees for the ServoPLus class. In this case, we are using inches because this should be linear.
        racketServo = new ServoPlus(hardwareMap.get(Servo.class,"racket"),5,0,5);
        colorSensor = new SpindexerColorSensor(hardwareMap, "spindexerColorSensor");
    }

    public void updateSpindexer(){
        if (runtime.seconds() >= ejectionTimeout){
            moveRacket(false);
        }
        paddleServo.update();
    }

    public void prepColor(COLORS color){
        if (color == COLORS.NONE) {
            return;
        }

        // figure out what index the correct color is in
        // move that index to launch pos
    }
//    zero is index 1 at intake, positive moves counterclockwise facing intake, so 120 will be index 1 at launcher
    public void prepIndexForLaunch(int index){
        if (index < 1 || index > 3){
            return;
        }
        if (ejecting) return;
        double pos = (index * slotIncrement) % 360;
        paddleServo.setSpindexerPosition(pos);
    }

    public void moveIndexToIntake(int index){
        if (index < 1 || index > 3){
            return;
        }
        if (ejecting) return;
        double pos = ((index-1) * slotIncrement);
        paddleServo.setSpindexerPosition(pos);
    }
    public void eject(){
        if (paddleServo.isAtTarget() && paddleServo.getTargetPosition() % slotIncrement == 0){
            moveRacket(true);
            runtime.reset();
        }
    }

    public boolean isAtRest(){
        return paddleServo.isAtTarget();
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
    private int getCurrentIndexInIntake(){
        if (paddleServo.getTargetPosition()%120 == 0){
            
        }else{
            return -1;
        }
    }
}
