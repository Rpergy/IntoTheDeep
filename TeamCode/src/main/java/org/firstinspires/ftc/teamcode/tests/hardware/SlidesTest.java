package org.firstinspires.ftc.teamcode.tests.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.Actuation;

@TeleOp(name="Slides Test", group="tests")
@Config
public class SlidesTest extends OpMode {
    public static int intakeLen;
    public static int depositLen;

    @Override
    public void init() {
        Actuation.setup(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        Actuation.intakeExtend(intakeLen);
//        Actuation.depositExtend(depositLen);
    }
}
