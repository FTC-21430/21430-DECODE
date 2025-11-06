package org.firstinspires.ftc.teamcode.Firmware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Firmware.Systems.GobildaPinpointModuleFirmware;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Launcher;
import org.firstinspires.ftc.teamcode.Firmware.Systems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Firmware.Systems.Spindexer;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerServoFirmware;

public class DecodeBot extends Robot{

    public Launcher launcher = null;
    public Spindexer spindexer = null;


    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry, double robotX, double robotY, double robotAngle, LinearOpMode opMode, boolean reset, boolean isAuto){








        



        spindexer = new Spindexer(hardwareMap);
    }
    @Override
    //TODO:Call updates for sensors and actuators
    public void updateRobot(boolean holdPosition, boolean autoSpeedChange, boolean isAuto){
        spindexer.updateSpindexer();
        launcher.updateSpeedControl();
    }
}
