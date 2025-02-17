package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.FieldConstants;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.ArrayList;

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

        AutoMovement.updatePosition();
        AutoMovement.displayPosition();
        telemetry.addData("Pose", AutoMovement.robotPose);
    }
}
