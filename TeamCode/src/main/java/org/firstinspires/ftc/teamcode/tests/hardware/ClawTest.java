package org.firstinspires.ftc.teamcode.tests.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Claw Test", group = "tests")
@Config
public class ClawTest extends OpMode {
    public static double clawPos = 1;

    Servo claw;
    CRServo leftWrist, rightWrist;

    @Override
    public void init() {
        leftWrist = hardwareMap.crservo.get("leftWrist");
        rightWrist = hardwareMap.crservo.get("rightWrist");
        claw = hardwareMap.servo.get("claw");

        leftWrist.setPower(0.0);
        rightWrist.setPower(0.0);
        claw.setPosition(clawPos);
    }

    @Override
    public void loop() {
        double rotate = gamepad1.left_stick_x;
        double offset = gamepad1.left_stick_y;
        if (Math.abs(offset) < 0.15) offset = 0;

        offset = 0;

        leftWrist.setPower(offset + rotate);
        rightWrist.setPower(rotate - offset);

        claw.setPosition(clawPos);

        telemetry.addData("x axis", gamepad1.left_stick_x);
        telemetry.addData("y axis", gamepad1.left_stick_y);
    }
}
