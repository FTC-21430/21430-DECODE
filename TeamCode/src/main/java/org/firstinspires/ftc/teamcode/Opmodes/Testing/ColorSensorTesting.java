package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Firmware.Systems.ColorSensorFirmware;
import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;
@Config
@TeleOp
public class ColorSensorTesting extends BaseTeleOp {

    public static float gain = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,false);
        ColorSensorFirmware sensor = new ColorSensorFirmware(hardwareMap,telemetry, "colorSensor1",1);
        waitForStart();
        while (opModeIsActive()){
            sensor.setGain(gain);
            telemetry.addData("rawSensorData - H:", sensor.getRawData()[0]);
            telemetry.addData("rawSensorData - S:", sensor.getRawData()[1]);
            telemetry.addData("rawSensorData - V:", sensor.getRawData()[2]);
            telemetry.addData("Current Detected Color", sensor.getDetectedColor());
            telemetry.update();
            robot.bulkSensorBucket.clearCache();
        }
    }
}
