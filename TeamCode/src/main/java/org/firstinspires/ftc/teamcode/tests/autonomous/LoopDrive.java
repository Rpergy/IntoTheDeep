package org.firstinspires.ftc.teamcode.tests.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.FieldConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@Autonomous(name="loop drive", group = "tests")
public class LoopDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);
        AutoMovement.setStartPos(new Pose(-40, 65, Math.toRadians(-90)));

        waitForStart();
        Trajectory circle = new Trajectory()
                .lineTo(new Pose(-40, -50, Math.toRadians(0)))
                .lineTo(new Pose(45, -50, Math.toRadians(90)))
                .lineTo(new Pose(45, 50, Math.toRadians(90)))
                .lineTo(new Pose(-45, 55, Math.toRadians(-90)));

        while(opModeIsActive()) {
            circle.run();
        }
    }
}
