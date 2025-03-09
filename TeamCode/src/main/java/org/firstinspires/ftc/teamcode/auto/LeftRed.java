package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.FieldConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@Autonomous(name="Left")
public class LeftRed extends LinearOpMode {
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);
        AutoMovement.setStartPos(FieldConstants.Red.leftStart);

        Actuation.setExtension(ActuationConstants.Extend.init);
        Actuation.setClaw(ActuationConstants.Claw.closed);
        Actuation.powerWrist(1.0, 0.0);
        sleep(10);
        Actuation.powerWrist(0.0, 0.0);

        Trajectory depositLineup1 = new Trajectory()
                .action(() -> Actuation.setTilt(ActuationConstants.Tilt.basketDeposit + 120))
                .lineTo(FieldConstants.Red.baskets, 0.65, 0.7);

        Trajectory deposit = new Trajectory()
                .action(() -> Actuation.powerWrist(0.5, 0))
                .action(() -> sleep(300))
                .action(() -> Actuation.powerWrist(0.0, 0.0))
                .action(() -> Actuation.setExtension(ActuationConstants.Extend.highBasket))
                .action(() -> sleep(2200))
                .action(() -> Actuation.setClaw(ActuationConstants.Claw.open))
                .action(() -> sleep(500))
                .action(() -> Actuation.setExtension(1300))
                .lineTo(FieldConstants.Red.baskets.augment(new Pose(7, 7, Math.toRadians(-45))))
                .action(() -> sleep(400))
                .action(() -> Actuation.powerWrist(-0.5, 0))
                .action(() -> sleep(100))
                .action(() -> Actuation.powerWrist(0.0, 0.0))
                .action(() -> sleep(100));

        Trajectory firstCycle = new Trajectory()
                .action(() -> Actuation.setTilt(ActuationConstants.Tilt.intakeSetup))
                .lineTo(FieldConstants.Red.neutralSamples)
                .action(() -> Actuation.setExtension(1300))
                .action(() -> sleep(200))
                .action(() -> Actuation.setTilt(ActuationConstants.Tilt.intake))
                .action(() -> sleep(500))
                .action(() -> Actuation.setClaw(ActuationConstants.Claw.closed))
                .action(() -> Actuation.setTilt(ActuationConstants.Tilt.intakeSetup))
                .action(() -> sleep(800))
                .action(() -> Actuation.setExtension(ActuationConstants.Extend.init))
                .action(() -> sleep(1000));

        Trajectory depositLineup2 = new Trajectory()
                .action(() -> Actuation.setTilt(ActuationConstants.Tilt.basketDeposit + 120))
                .lineTo(FieldConstants.Red.baskets.augment(new Pose(-0.75, 2.0, 0)));

        Trajectory secondCycle = new Trajectory()
                .action(() -> Actuation.setTilt(ActuationConstants.Tilt.intakeSetup))
                .lineTo(FieldConstants.Red.neutralSamples.augment(new Pose(-10.0, 0, Math.toRadians(2.5))))
                .action(() -> Actuation.setExtension(1390))
                .action(() -> sleep(1000))
                .action(() -> Actuation.setTilt(ActuationConstants.Tilt.intake))
                .action(() -> sleep(500))
                .action(() -> Actuation.setClaw(ActuationConstants.Claw.closed))
                .action(() -> Actuation.setTilt(ActuationConstants.Tilt.intakeSetup))
                .action(() -> sleep(800))
                .action(() -> Actuation.setExtension(ActuationConstants.Extend.init))
                .action(() -> sleep(1000));

        Trajectory depositLineup3 = new Trajectory()
                .action(() -> Actuation.setTilt(ActuationConstants.Tilt.basketDeposit + 120))
                .lineTo(FieldConstants.Red.baskets.augment(new Pose(-.5, 2, 0)));

        Trajectory park = new Trajectory()
                .lineTo(new Pose(-49, -10, 0), 0.75, 0.7)
                .action(() -> Actuation.setTilt(ActuationConstants.Tilt.chamberDeposit))
                .action(() -> Actuation.setExtension(1300))
                .action(() -> Actuation.powerWrist(1.0, 0))
                .action(() -> sleep(200))
                .action(() -> Actuation.powerWrist(0.0, 0))
                .lineTo(new Pose(-25, -8, 0));

        Actuation.setTilt(ActuationConstants.Tilt.init);
        waitForStart();

//        while (opModeIsActive()) {
//            telemetry.addData("x", AutoMovement.robotPose.x);
//            telemetry.addData("y", AutoMovement.robotPose.y);
//            telemetry.addData("h", Math.toDegrees(AutoMovement.robotPose.heading));
//            telemetry.update();
//
//            AutoMovement.updatePosition();
//            AutoMovement.displayPosition();
//        }

        depositLineup1.run();
        deposit.run();
        firstCycle.run();
        depositLineup2.run();
        deposit.run();
        secondCycle.run();
        depositLineup3.run();
        deposit.run();
        park.run();
    }
}
