package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.FieldConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@Autonomous (name = "Right")
public class RightRed extends LinearOpMode {
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);
        AutoMovement.setStartPos(FieldConstants.Red.rightStart);

        Actuation.setExtension(ActuationConstants.Extend.init);
        Actuation.setClaw(ActuationConstants.Claw.closed);
        Actuation.powerWrist(1.0, 0.0);
        sleep(10);
        Actuation.powerWrist(0.0, 0.0);

        Trajectory depositPreload = new Trajectory()
                .action(() -> Actuation.setExtension(1150))
                .action(() -> Actuation.setTilt(600))
                .action(() -> Actuation.powerWrist(0.5, 0))
                .action(() -> sleep(300))
                .action(() -> Actuation.powerWrist(0, 0))
                .lineTo(FieldConstants.Red.rightShort)
                .action(() -> sleep(1000))
                .action(() -> Actuation.setTilt(1500))
                .action(() -> sleep(750))
                .action(() -> Actuation.setClaw(ActuationConstants.Claw.open))
                .action(() -> Actuation.setExtension(ActuationConstants.Extend.init))
                .lineTo(FieldConstants.Red.rightShort.augment(new Pose(0, -14, 0)));

        Trajectory pushSamples = new Trajectory()
                .lineTo(new Pose(34, -40, Math.toRadians(90)))
                .lineTo(new Pose(48, -14, Math.toRadians(90)))
                .lineTo(new Pose(48, -49, Math.toRadians(90)), 0.8, 0.7)
                .lineTo(new Pose(34, -14, Math.toRadians(90)), 0.8, 0.7)
                .lineTo(new Pose(48, -14, Math.toRadians(90)), 0.8, 0.7)
                .lineTo(new Pose(59, -14, Math.toRadians(90)), 0.8, 0.7)
                .lineTo(new Pose(59, -49, Math.toRadians(90)), 0.8, 0.7)
                .lineTo(new Pose(59, -14, Math.toRadians(90)), 0.8, 0.7)
                .action(() -> Actuation.setTilt(ActuationConstants.Tilt.basketDeposit))
                .lineTo(new Pose(63, -49, Math.toRadians(90)), 0.8, 0.7);

//        Trajectory intakeSpecimen = new Trajectory()
//                .lineTo(new Pose(34, -20, Math.toRadians(-90)))
//                .action(() -> sleep(1500))
//                .action(() -> Actuation.setTilt(ActuationConstants.Tilt.intakeSpecimen))
//                .action(() -> sleep(10000));

        waitForStart();

        depositPreload.run();
        pushSamples.run();
//        intakeSpecimen.run();

//        while (opModeIsActive()) {
//            telemetry.addData("x", AutoMovement.robotPose.x);
//            telemetry.addData("y", AutoMovement.robotPose.y);
//            telemetry.addData("h", Math.toDegrees(AutoMovement.robotPose.heading));
//            telemetry.update();
//
//            AutoMovement.updatePosition();
//            AutoMovement.displayPosition();
//        }
    }
}
