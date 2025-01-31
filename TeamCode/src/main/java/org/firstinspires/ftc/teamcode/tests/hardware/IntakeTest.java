package org.firstinspires.ftc.teamcode.tests.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;

@TeleOp(group="tests", name="Intake Test")
@Config
public class IntakeTest extends OpMode {
    public static double clawPos, armPos, rotPos, wristPos;

    int position;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
        clawPos = ActuationConstants.Intake.open;
        armPos = ActuationConstants.Intake.armTransfer;
        rotPos = ActuationConstants.Intake.horizontal;
        wristPos = ActuationConstants.Intake.wristTransfer;
    }

    @Override
    public void loop() {
        Actuation.setIntakeArm(armPos);
        Actuation.setIntakeWrist(wristPos);
        Actuation.setIntakeRotate(rotPos);
        Actuation.setIntake(clawPos);
    }
}
