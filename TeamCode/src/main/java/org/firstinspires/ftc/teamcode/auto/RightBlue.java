package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.FieldConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@Autonomous(name="right blue", group = "blue")
public class RightBlue extends LinearOpMode {
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);
        AutoMovement.setStartPos(FieldConstants.Blue.rightStart);

        waitForStart();
        Trajectory depositSpecimen = new Trajectory()
                .lineTo(FieldConstants.Blue.rightShort)
                .action(Actuation::autoDeposit)
                .action(() -> sleep(500))
                .action(() -> Actuation.depositExtend(ActuationConstants.Deposit.highChamber))
                .lineTo(FieldConstants.Blue.rightShort.augment(new Pose(0, 5, 0)))
                .action(() -> sleep(500))
                .action(() -> Actuation.depositExtend(ActuationConstants.Deposit.lowChamber))
                .action(() -> Actuation.setDepositor(ActuationConstants.Deposit.open))
                .lineTo(FieldConstants.Blue.rightShort)
                .action(() -> Actuation.depositExtend(ActuationConstants.Deposit.min))
                .action(Actuation::autoDepositTransfer);

        Trajectory intakeSubSample = new Trajectory();
        Trajectory deliverSample = new Trajectory()
                .lineTo(FieldConstants.Blue.deliverPoint)
                .action(Actuation::autoIntake)
                .action(() -> Actuation.setIntakeClaw(ActuationConstants.Intake.closed))
                .action(Actuation::autoIntakeTransfer)
                .action(Actuation::toggleBlueDeliver);
        Trajectory intakeGroundSample = new Trajectory()
                .lineTo(FieldConstants.Blue.allianceSamples)
                .action(Actuation::autoIntake)
                .action(() -> Actuation.setIntakeClaw(ActuationConstants.Intake.open))
                .action(Actuation::autoIntakeTransfer);
        Trajectory intakeSpecimen = new Trajectory()
                .lineTo(FieldConstants.Blue.pickupPoint)
                .action(Actuation::autoIntake)
                .action(() -> Actuation.setIntakeClaw(ActuationConstants.Intake.open))
                .action(Actuation::autoIntakeTransfer)
                .action(() -> Actuation.setDepositor(ActuationConstants.Deposit.closed))
                .action(() -> Actuation.setIntakeClaw(ActuationConstants.Intake.closed));

        Trajectory park = new Trajectory()
                .lineTo(FieldConstants.Blue.leftLong)
                .action(() -> Actuation.depositExtend(ActuationConstants.Deposit.lowChamber));

        depositSpecimen.run();
        intakeSubSample.run();
        deliverSample.run();
        intakeGroundSample.run();
        deliverSample.run();
        intakeSpecimen.run();
        for(int i = 0; i < 3; i++) {
            depositSpecimen.run();
            if (i != 2) {
                intakeSubSample.run();
                deliverSample.run();
                intakeSpecimen.run();
            }
        }
        park.run();
    }
}
