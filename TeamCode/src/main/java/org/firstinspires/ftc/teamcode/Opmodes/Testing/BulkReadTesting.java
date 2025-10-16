package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;

@TeleOp
public class BulkReadTesting extends BaseTeleOp {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,false);
        waitForStart();
        while (opModeIsActive()){

            robot.bulkSensorBucket.clearCache();
        }
    }
}
