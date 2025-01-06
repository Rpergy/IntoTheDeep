package org.firstinspires.ftc.teamcode.tests.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(group="tests", name="Intake Test")
public class IntakeTest extends OpMode {
    CRServo leftIntake, rightIntake;
    DcMotor leftSlide, rightSlide;

    int position;

    @Override
    public void init() {
        leftIntake = hardwareMap.crservo.get("leftIntake");
        rightIntake = hardwareMap.crservo.get("rightIntake");

        leftSlide = hardwareMap.dcMotor.get("leftSlide");
        rightSlide = hardwareMap.dcMotor.get("rightSlide");

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setPower(1.0);
        rightSlide.setPower(1.0);

        leftSlide.setTargetPosition(position);
        rightSlide.setTargetPosition(position);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        position = 0;

//        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
//        if (gamepad1.cross) {
//            leftSlide.setPower(1.0);
////            leftIntake.setPower(1.0);
////            rightIntake.setPower(-1.0);
//        }
//        else if (gamepad1.square) {
//            rightSlide.setPower(1.0);
////            leftIntake.setPower(-1.0);
////            rightIntake.setPower(1.0);
//        }
//        else {
//            leftSlide.setPower(0.0);
//            rightSlide.setPower(0.0);
//        }

        if(gamepad1.left_bumper) position += 1;
        else if (gamepad1.right_bumper) position -= 1;

        leftSlide.setTargetPosition(position);
        rightSlide.setTargetPosition(position);

        telemetry.addData("target pos", position);
        telemetry.addData("pos L", leftSlide.getCurrentPosition());
        telemetry.addData("pos R", rightSlide.getCurrentPosition());
        telemetry.update();
    }
}
