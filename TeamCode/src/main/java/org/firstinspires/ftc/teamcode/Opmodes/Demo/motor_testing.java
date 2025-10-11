package org.firstinspires.ftc.teamcode.Opmodes.Demo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;

@TeleOp
public class motor_testing extends BaseTeleOp {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,false);
        waitForStart();
        while(opModeIsActive()) {
            robot.driveTrain.setDrivePower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 0);
        }
    }
}
