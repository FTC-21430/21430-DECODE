package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;
@TeleOp
public class IntakeTesting extends BaseTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true, false);
        waitForStart();
        while(opModeIsActive()){
            robot.bulkSensorBucket.clearCache();

            if(gamepad1.b){
                robot.intake.setIntakePower(-1);
            }else if (gamepad1.a) {
                robot.intake.setIntakePower(-0.5);
            }else {
                    robot.intake.setIntakePower(0);
                }
            }

        }
    }

