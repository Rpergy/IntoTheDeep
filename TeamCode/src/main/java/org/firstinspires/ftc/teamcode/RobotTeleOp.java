package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;

@TeleOp(name="TeleOp")
public class RobotTeleOp extends OpMode {
    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        Actuation.drive(gamepad1.left_stick_y, -gamepad1.right_stick_x, gamepad1.left_stick_x);

        Actuation.toggleIntakeExtend(gamepad1.triangle);
        Actuation.toggleIntakeDown(gamepad1.circle);
        Actuation.toggleIntake(gamepad1.cross);

        Actuation.toggleDepositExtension(gamepad2.triangle);
        Actuation.toggleDespoitWrist(gamepad2.circle);
        Actuation.toggleDepositor(gamepad2.cross);

        AutoMovement.updatePosition();
        AutoMovement.displayPosition();
        telemetry.addData("Pose", AutoMovement.robotPose);
    }
}
