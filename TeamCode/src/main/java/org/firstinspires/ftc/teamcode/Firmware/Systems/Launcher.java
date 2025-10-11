package org.firstinspires.ftc.teamcode.Firmware.Systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class Launcher {
    private HardwareMap hardwareMap = null;
    private Telemetry telemetry = null;
//    TODO: make the flywheel object here private, currently public for initial testing - 10/11/25 Tobin R
    public Flywheel flywheel = null;

//    again, public for debugging using FTC dashboard
    public static double flywheelSpeedControlP, flywheelSpeedControlI,flywheelSpeedControlD;
    public Launcher(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        flywheel = new Flywheel(hardwareMap, telemetry, new ElapsedTime(),flywheelSpeedControlP,flywheelSpeedControlI,flywheelSpeedControlD);
    }
}
