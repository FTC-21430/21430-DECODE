package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

// Intent of the class is to allow you to ftc dashboard to the robot and then you can fine control servos and find needed values quickly.

// Uncomment line 11 to reveal the position on FTC dashboard.
//@Config

// Disable this class by default, but when you want to mess with it, then comment out line 14
@Disabled
public class ServoTuningTesting extends LinearOpMode {
    private Servo servo;
    public static double position = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Change this value to match the config value of the servo you want to tune
        String address = "servo";

        // get the servo object from the config files
        servo = hardwareMap.get(Servo.class, address);

        // wait for the start button to be pressed on DS
        waitForStart();
        while (opModeIsActive()){

            //Change value with all FTC legal controllers
            if (gamepad1.dpadUpWasPressed()){
                position += 0.01;
            }
            if (gamepad1.dpadDownWasPressed()){
                position -= 0.01;
            }

            // Set the position
            servo.setPosition(position);
        }
    }
}
