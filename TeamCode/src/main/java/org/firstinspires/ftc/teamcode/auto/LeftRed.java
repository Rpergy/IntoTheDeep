package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.FieldConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@Autonomous(name="Left Red", group="red")
public class LeftRed extends LinearOpMode {
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);
        AutoMovement.setStartPos(FieldConstants.Red.leftStart);

        int angle = 0;

        Trajectory depositPreload = new Trajectory()
                .action(() -> Actuation.setTilt(ActuationConstants.Tilt.basketDeposit))
                .lineTo(FieldConstants.Red.baskets)
                .action(() -> Actuation.setExtension(ActuationConstants.Extend.highBasket))
                .action(() -> sleep(2500))
                .lineTo(FieldConstants.Red.baskets.augment(new Pose(-2.5, -2.5, 0)))
                .action(() -> sleep(500))
                .action(() -> Actuation.setClaw(ActuationConstants.Claw.open))
                .action(() -> Actuation.setExtension(ActuationConstants.Extend.init))
                .lineTo(FieldConstants.Red.baskets.augment(new Pose(2, 2, 0)));

        Trajectory cycleTransition = new Trajectory()
                .lineTo(FieldConstants.Red.neutralSamples.augment(new Pose(0, 0, Math.toRadians(angle))))
                .action(() -> Actuation.setTilt(ActuationConstants.Tilt.intakeSetup));

        Trajectory cycleSample = new Trajectory()
                .action(() -> sleep(1500))
                .action(() -> Actuation.setTilt(ActuationConstants.Tilt.intake))
                .action(() -> sleep(500))
                .action(() -> Actuation.setClaw(ActuationConstants.Claw.closed))
                .action(() -> Actuation.setExtension(ActuationConstants.Extend.init))
                .action(() -> sleep(1500))
                .action(() -> Actuation.setTilt(ActuationConstants.Tilt.basketDeposit))
                .action(() -> sleep(200));

        Trajectory park = new Trajectory()
                .lineTo(new Pose(-40, -12, 0), 0.7, 0.7)
                .action(() -> Actuation.setTilt(0))
                .lineTo(FieldConstants.Red.leftLong)
                .action(() -> sleep(1000));

        waitForStart();
        depositPreload.run();
//
//        cycleTransition.run();
//        Actuation.setExtension(1000);
//        cycleSample.run();
//        depositPreload.run();
//        angle = 30;
//
//        cycleTransition.run();
//        Actuation.setExtension(1250);
//        cycleSample.run();
//        depositPreload.run();
//        angle = 50;
//
//        cycleTransition.run();
//        Actuation.setExtension(1500);
//        cycleSample.run();
//        depositPreload.run();

        park.run();
    }
}
