package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Firmware.Systems.AprilTag;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class AprilTagTesting extends OpMode {

    AprilTag aprilTag = new AprilTag();

    @Override
    public void init(){
        aprilTag.init(hardwareMap, telemetry);
    }
    @Override
    public void loop(){
        aprilTag.update();
        AprilTagDetection id20 = aprilTag.getSpecific(20);
        aprilTag.displayDetectionTelemetry(id20);

        aprilTag.update();
        AprilTagDetection id21 = aprilTag.getSpecific(21);
        aprilTag.displayDetectionTelemetry(id20);

        aprilTag.update();
        AprilTagDetection id22 = aprilTag.getSpecific(22);
        aprilTag.displayDetectionTelemetry(id20);

        aprilTag.update();
        AprilTagDetection id23 = aprilTag.getSpecific(23);
        aprilTag.displayDetectionTelemetry(id20);

        aprilTag.update();
        AprilTagDetection id24 = aprilTag.getSpecific(24);
        aprilTag.displayDetectionTelemetry(id20);
    }
}
