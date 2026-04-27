package org.firstinspires.ftc.teamcode.Opmodes.Testing;

// Written by Tobin

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor;
import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;

@Config
//@Disabled
@TeleOp
public class ColorSensorTesting extends LinearOpMode {

    public static float gain = 80;

    @Override
    public void runOpMode() throws InterruptedException {

        SpindexerColorSensor sensor = new SpindexerColorSensor(hardwareMap, "colorSensor1", "colorSensor2");
        waitForStart();
        while (opModeIsActive()){

            telemetry.addData("Hue", sensor.getRawData()[0]);
            telemetry.addData("Saturation", sensor.getRawData()[1]);
            telemetry.addData("Value", sensor.getRawData()[2]);
            sensor.setGain(gain);
            telemetry.addData("Current Detected Color", sensor.getDetectedColor());
            telemetry.update();

        }
    }
}