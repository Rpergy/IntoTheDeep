package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.SampleLocation;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;

@TeleOp(name="TeleOp")
public class RobotTeleOp extends OpMode {
    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
        AutoMovement.setStartPos(new Pose(-36, 65, Math.toRadians(-90)));
    }

    @Override
    public void loop() {
        Actuation.teleDrive(gamepad1.left_stick_button, false, -gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

        Actuation.basketExtension(gamepad2.triangle);
        Actuation.chamberExtension(gamepad2.square);
        Actuation.retractExtension(gamepad2.cross);
        Actuation.intakeExtension(gamepad2.circle);

        Actuation.adjustExtension(25 * gamepad2.left_trigger);
        Actuation.adjustExtension(-25 * gamepad2.right_trigger);

        Actuation.adjustTilt(35 * gamepad1.left_trigger);
        Actuation.adjustTilt(-35 * gamepad1.right_trigger);

        if (gamepad1.left_bumper) {
            Point change = SampleLocation.findSample(Actuation.extend.getCurrentPosition());
            Pose newPos = AutoMovement.robotPose.augment(new Pose(0, -change.x, 0));
            AutoMovement.moveTowards(newPos, 0.7, 0.5);
            if (change.y > 0 && change.y < 4000) {
                Actuation.setExtension((int)change.y);
            }
        }

        Actuation.tiltObservation(gamepad1.square);
        Actuation.tiltBasketDeposit(gamepad1.triangle);
        Actuation.tiltChamberDeposit(gamepad1.dpad_right);
        Actuation.tiltSubmersible(gamepad1.cross);

        Actuation.toggleClaw(gamepad1.right_bumper || gamepad2.right_bumper);
        Actuation.powerWrist(-gamepad2.left_stick_y/2.0, gamepad2.right_stick_x/2.0);

        AutoMovement.updatePosition();
        AutoMovement.displayPosition();
        telemetry.addData("Pose", AutoMovement.robotPose);
    }
}
