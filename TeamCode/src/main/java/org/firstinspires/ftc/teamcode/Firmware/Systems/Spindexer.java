package org.firstinspires.ftc.teamcode.Firmware.Systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor.COLORS;

public class Spindexer {

    private final SpindexerServoFirmware servo;

    public Spindexer(HardwareMap hardwareMap){
//        TODO: ensure that the slot values and the config address name are correct
        servo = new SpindexerServoFirmware(hardwareMap,false,0,90,180,"intake");
    }

    public void prepColor(COLORS color){
        if (color == COLORS.NONE) {
            return;
        }
        // figure out what index the correct color is in
        // move that index to launch pos
    }
    public void eject(){
        if ()
    }
}
