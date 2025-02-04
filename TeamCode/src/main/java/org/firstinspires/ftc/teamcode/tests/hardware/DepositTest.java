package org.firstinspires.ftc.teamcode.tests.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utility.Actuation;

@TeleOp(group="tests", name="Deposit Test")
@Config
public class DepositTest extends OpMode {
    public static float wristPos, flipPos, depositorPos;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        Actuation.setDepositWrist(wristPos);
        Actuation.setDepositFlip(flipPos);
        Actuation.setDepositor(depositorPos);
    }
}
