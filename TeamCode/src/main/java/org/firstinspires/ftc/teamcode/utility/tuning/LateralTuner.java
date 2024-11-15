package org.firstinspires.ftc.teamcode.utility.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.RobotMovement;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.ArrayList;

@Config
@TeleOp(name="Lateral Tuner", group="tuning")
public class LateralTuner extends OpMode {
    public static double measuredAngle = Math.PI;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        AutoMovement.updatePosition();

        Actuation.drive(0, gamepad1.right_stick_x, 0);

        telemetry.addData("Heading", AutoMovement.robotPose.heading);
        telemetry.addData("diff", measuredAngle / AutoMovement.robotPose.heading);
        telemetry.addData("new lateral multiplier", ActuationConstants.Drivetrain.lateral_multiplier / (measuredAngle/AutoMovement.robotPose.heading));
        telemetry.update();
    }
}