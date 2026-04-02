package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Firmware.Systems.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;

@TeleOp
public class DriveTrainTesting extends BaseTeleOp {

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    // The main code that runs during init
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveTrain driveTrain = new MecanumDriveTrain(hardwareMap, telemetry, "red");
        driveTrain.fieldCentricDriving(false);
        waitForStart();
        while(opModeIsActive()) {
            driveTrain.setDrivePower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x,0);
        }
    }
}
