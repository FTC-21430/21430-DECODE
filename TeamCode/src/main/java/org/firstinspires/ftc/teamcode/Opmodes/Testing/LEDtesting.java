package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;

@TeleOp
public class LEDtesting extends BaseTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,true,false);
        while (opModeIsActive()){
            robot.led.discoParty();
            robot.updateRobot(false,false,false);
            telemetry.addData("Led color", robot.led.color);
            telemetry.update();
            robot.bulkSensorBucket.clearCache();
        }
    }
}
