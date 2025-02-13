package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utility.Actuation;
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

        Trajectory depositPreload = new Trajectory()
                .lineTo(FieldConstants.Red.baskets)
                .action(() -> sleep(200));

        Trajectory cycleSamples = new Trajectory()
                .lineTo(FieldConstants.Red.neutralSamples)
                .action(() -> sleep(200))
                .lineTo(FieldConstants.Red.baskets)
                .action(() -> sleep(200))
                .lineTo(FieldConstants.Red.neutralSamples.augment(new Pose(0, 0, Math.toRadians(30))))
                .action(() -> sleep(200))
                .lineTo(FieldConstants.Red.baskets)
                .action(() -> sleep(200))
                .lineTo(FieldConstants.Red.neutralSamples.augment(new Pose(0, 0, Math.toRadians(50))))
                .action(() -> sleep(200))
                .lineTo(FieldConstants.Red.baskets);

        Trajectory park = new Trajectory()
                .lineTo(new Pose(-40, -12, 0))
                .lineTo(FieldConstants.Red.leftLong);

        waitForStart();
        depositPreload.run();
        cycleSamples.run();
        park.run();
    }
}
