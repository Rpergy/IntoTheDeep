package org.firstinspires.ftc.teamcode.tests.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group="tests", name="Deposit Test")
public class DepositTest extends OpMode {
    DcMotor leftDeposit, rightDeposit;

    int position;

    @Override
    public void init() {

        leftDeposit = hardwareMap.dcMotor.get("leftDeposit");
        rightDeposit = hardwareMap.dcMotor.get("rightDeposit");

        leftDeposit.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDeposit.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDeposit.setPower(1.0);
        rightDeposit.setPower(1.0);

        leftDeposit.setTargetPosition(position);
        rightDeposit.setTargetPosition(position);

        leftDeposit.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDeposit.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        position = 0;
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

        leftDeposit.setTargetPosition(position);
        rightDeposit.setTargetPosition(position);

        telemetry.addData("target pos", position);
        telemetry.addData("pos L", leftDeposit.getCurrentPosition());
        telemetry.addData("pos R", rightDeposit.getCurrentPosition());
        telemetry.update();
    }
}
