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

        Trajectory deposit = new Trajectory()
                .action(() -> Actuation.setTilt(ActuationConstants.Tilt.basketDeposit))
                .lineTo(FieldConstants.Red.baskets)
                .action(() -> Actuation.setFlip(ActuationConstants.Claw.flipBasketDeposit))
                .action(() -> Actuation.setExtension(ActuationConstants.Extend.highBasket))
                .action(() -> sleep(2000))
                .action(() -> Actuation.setFlip(0.5))
                .action(() -> Actuation.setClaw(ActuationConstants.Claw.open))
                .action(() -> sleep(500))
                .action(() -> Actuation.setFlip(ActuationConstants.Claw.flipBasketDeposit))
                .action(() -> Actuation.setExtension(ActuationConstants.Extend.init))
                .action(() -> sleep(2000));

        Trajectory firstCycle = new Trajectory()
                .action(() -> Actuation.setTilt(ActuationConstants.Tilt.intakeSetup))
                .action(() -> Actuation.setFlip(ActuationConstants.Claw.flipIntake))
                .lineTo(FieldConstants.Red.neutralSamples)
                .action(() -> Actuation.setExtension(1350))
                .action(() -> sleep(1000))
                .action(() -> Actuation.setTilt(ActuationConstants.Tilt.intake))
                .action(() -> sleep(500))
                .action(() -> Actuation.setClaw(ActuationConstants.Claw.closed))
                .action(() -> Actuation.setTilt(ActuationConstants.Tilt.intakeSetup))
                .action(() -> sleep(800))
                .action(() -> Actuation.setExtension(ActuationConstants.Extend.init))
                .action(() -> sleep(1000));

        Trajectory secondCycle = new Trajectory()
                .action(() -> Actuation.setTilt(ActuationConstants.Tilt.intakeSetup))
                .action(() -> Actuation.setFlip(ActuationConstants.Claw.flipIntake))
                .lineTo(FieldConstants.Red.neutralSamples.augment(new Pose(-9.5, 0, 0)))
                .action(() -> Actuation.setExtension(1350))
                .action(() -> sleep(1000))
                .action(() -> Actuation.setTilt(ActuationConstants.Tilt.intake))
                .action(() -> sleep(500))
                .action(() -> Actuation.setClaw(ActuationConstants.Claw.closed))
                .action(() -> Actuation.setTilt(ActuationConstants.Tilt.intakeSetup))
                .action(() -> sleep(800))
                .action(() -> Actuation.setExtension(ActuationConstants.Extend.init))
                .action(() -> sleep(1000));

        Trajectory thirdCycle = new Trajectory()
                .action(() -> Actuation.setTilt(ActuationConstants.Tilt.intakeSetup))
                .action(() -> Actuation.setFlip(ActuationConstants.Claw.flipIntake))
                .lineTo(FieldConstants.Red.neutralSamples.augment(new Pose(-9.5, 1, Math.toRadians(40))))
                .action(() -> Actuation.setExtension(1350))
                .action(() -> sleep(1000));
//                .action(() -> Actuation.setTilt(ActuationConstants.Tilt.intake))
//                .action(() -> sleep(500))
//                .action(() -> Actuation.setClaw(ActuationConstants.Claw.closed))
//                .action(() -> Actuation.setTilt(ActuationConstants.Tilt.intakeSetup))
//                .action(() -> sleep(800))
//                .action(() -> Actuation.setExtension(ActuationConstants.Extend.init))
//                .action(() -> sleep(1000));

        Actuation.setTilt(ActuationConstants.Tilt.init);
        waitForStart();
        deposit.run();
        firstCycle.run();
        deposit.run();
        secondCycle.run();
        deposit.run();
        thirdCycle.run();
    }
}
