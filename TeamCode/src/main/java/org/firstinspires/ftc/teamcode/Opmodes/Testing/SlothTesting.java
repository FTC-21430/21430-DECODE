package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;

@TeleOp
public class SlothTesting extends BaseTeleOp {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Sloth Test 4");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){

        }
    }
}
