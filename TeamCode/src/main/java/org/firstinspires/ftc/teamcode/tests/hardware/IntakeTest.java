package org.firstinspires.ftc.teamcode.tests.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;

@TeleOp(group="tests", name="Intake Test")
@Config
public class IntakeTest extends OpMode {
    public static double clawPos, armPos, rotPos, wristPos;
    public static int slidesPos;

    int position;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
        clawPos = ActuationConstants.Intake.open;
        armPos = ActuationConstants.Intake.armInit;
        rotPos = ActuationConstants.Intake.vertical;
        wristPos = ActuationConstants.Intake.wristInit;
        slidesPos = 0;
    }

    @Override
    public void loop() {
        Actuation.setIntakeArm(armPos);
        Actuation.setIntakeWrist(wristPos);
        Actuation.setIntakeRotate(rotPos);
        Actuation.setIntakeClaw(clawPos);
        Actuation.intakeExtend(slidesPos);
    }
}
