package org.firstinspires.ftc.teamcode.tests.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.RobotMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;
import java.util.ArrayList;

@Autonomous(name="path test", group="tests")
public class PathTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);
        waitForStart();
        Trajectory move = new Trajectory()
                .lineTo(new Pose(90, 0, Math.toRadians(0)))
                .lineTo(new Pose(90, -80, Math.toRadians(180)))
                .lineTo(new Pose(0, -90, Math.toRadians(90)))
                .lineTo(new Pose(0, 0, Math.toRadians(0)));

        while(opModeIsActive()) {
            move.run();
        }
    }
}
