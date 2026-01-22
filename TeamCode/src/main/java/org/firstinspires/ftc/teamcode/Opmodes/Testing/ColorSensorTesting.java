package org.firstinspires.ftc.teamcode.Opmodes.Testing;

// Written by Tobin

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor;
import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;

@Config
@TeleOp
public class ColorSensorTesting extends BaseTeleOp {

    public static float gain = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,false);
        SpindexerColorSensor sensor = new SpindexerColorSensor(hardwareMap, "colorSensor");
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("RGB", sensor.getRawData()[0]);
            sensor.setGain(gain);
            telemetry.addData("Current Detected Color", sensor.getDetectedColor());
            telemetry.update();
            robot.bulkSensorBucket.clearCache();
        }
    }
}