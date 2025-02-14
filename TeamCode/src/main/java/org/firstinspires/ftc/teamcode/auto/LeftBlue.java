package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.FieldConstants;
import org.firstinspires.ftc.teamcode.utility.autonomous.Trajectory;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@Autonomous(name="left blue", group = "blue")
public class LeftBlue extends LinearOpMode {
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, telemetry);
        AutoMovement.setStartPos(FieldConstants.Blue.leftStart);

        waitForStart();
        Trajectory depositPreload = new Trajectory()
                .lineTo(FieldConstants.Blue.leftShort)
//                .action(Actuation::autoDeposit)
                .action(() -> sleep(500))
//                .action(() -> Actuation.depositExtend(ActuationConstants.Deposit.highChamber))
                .lineTo(FieldConstants.Blue.leftShort.augment(new Pose(0, 0, 0)))
                .action(() -> sleep(500))
//                .action(() -> Actuation.depositExtend(ActuationConstants.Deposit.lowChamber))
//                .action(() -> Actuation.setDepositor(ActuationConstants.Deposit.open))
                .lineTo(FieldConstants.Blue.leftShort);
//                .action(() -> Actuation.depositExtend(ActuationConstants.Deposit.min))
//                .action(Actuation::autoDepositTransfer);

        Trajectory deliverSample = new Trajectory();
        Trajectory intakeSpecimen = new Trajectory();

        depositPreload.run();

    }
}
