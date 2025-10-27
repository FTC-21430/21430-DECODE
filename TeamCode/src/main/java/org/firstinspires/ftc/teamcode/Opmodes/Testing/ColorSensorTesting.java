package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Firmware.Systems.SpindexerColorSensor;
import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;
@Config
@TeleOp
public class ColorSensorTesting extends BaseTeleOp {

    public static float gain = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,false);
        SpindexerColorSensor sensor = new SpindexerColorSensor(hardwareMap, "colorSensor1");
        waitForStart();
        while (opModeIsActive()){
            sensor.setGain(gain);
            telemetry.addData("Current Detected Color", sensor.getDetectedColor());
            telemetry.update();
            robot.bulkSensorBucket.clearCache();
        }
    }
}
