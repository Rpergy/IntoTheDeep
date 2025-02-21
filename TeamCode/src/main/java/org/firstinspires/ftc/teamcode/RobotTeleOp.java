package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@TeleOp(name="TeleOp")
public class RobotTeleOp extends OpMode {
    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
        AutoMovement.setStartPos(new Pose(-36, 65, Math.toRadians(-90)));
    }

    @Override
    public void loop() {
        Actuation.drive(-gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

        Actuation.basketExtension(gamepad2.triangle);
        Actuation.chamberExtension(gamepad2.square);
        Actuation.retractExtension(gamepad2.cross);
        Actuation.intakeExtension(gamepad2.circle);

        Actuation.adjustExtension(10 * gamepad2.left_trigger);
        Actuation.adjustExtension(-10 * gamepad2.right_trigger);

        Actuation.tiltObservation(gamepad1.square);
        Actuation.tiltBasketDeposit(gamepad1.triangle);
        Actuation.tiltChamberDeposit(gamepad1.dpad_right);
        Actuation.tiltSubmersible(gamepad1.cross);
        Actuation.retractSubmersible(gamepad1.circle);

        Actuation.toggleClaw(gamepad1.right_bumper);

        AutoMovement.updatePosition();
        AutoMovement.displayPosition();
        telemetry.addData("Pose", AutoMovement.robotPose);
    }
}
