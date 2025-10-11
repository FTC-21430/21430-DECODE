package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Firmware.DecodeBot;


// Creates robot object. All OpModes inherit this.
abstract public class GeneralOpMode extends LinearOpMode {
    public DecodeBot robot;

    public ElapsedTime runtime = new ElapsedTime();


    // normal functions
    public void initialize(boolean reset, boolean isAuto) {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new DecodeBot();
        robot.init(hardwareMap,telemetry,0,0,0,this,true,false);
        
    }
    

    // Arm state machine logic is here because all Op-modes need to use it.
    

    
}
