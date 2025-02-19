package org.firstinspires.ftc.teamcode.tests.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@TeleOp(name="Position Viewer", group = "tests")
@Config
public class PositionViewer extends OpMode {
    public static double x, y, heading;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
        x = 0;
        y = 0;
        heading = 0;

        AutoMovement.setStartPos(new Pose(x, y, Math.toRadians(heading)));
    }

    @Override
    public void loop() {
        AutoMovement.setStartPos(new Pose(x, y, Math.toRadians(heading)));
        AutoMovement.displayPosition();
    }
}
