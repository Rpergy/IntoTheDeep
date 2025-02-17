package org.firstinspires.ftc.teamcode.tests.autonomous;

import com.acmerobotics.dashboard.config.Config;
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
        AutoMovement.setStartPos(new Pose(0, 0, Math.toRadians(0)));

        waitForStart();
        Trajectory circle = new Trajectory()
                .lineTo(new Pose(0, 0, Math.toRadians(90)))
                .lineTo(new Pose(0, 0, Math.toRadians(180)))
                .lineTo(new Pose(0, 0, Math.toRadians(270)))
                .lineTo(new Pose(0, 0, Math.toRadians(360)));

        Trajectory line = new Trajectory()
                .lineTo(new Pose(40, 0, 0))
                .lineTo(new Pose(0, 0, 0));

        while(opModeIsActive()) {
            Actuation.drive(0, ActuationConstants.Autonomous.turnSpeed, 0);
//            circle.run();
//            line.run();
        }
    }
}
