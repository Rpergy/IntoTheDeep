package org.firstinspires.ftc.teamcode.tests.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;

@TeleOp(group="tests", name="Intake Test")
@Config
public class IntakeTest extends OpMode {
    public DcMotor armTilt, extend;

    public static int tiltPos, extendPos;

    @Override
    public void init() {
        armTilt = hardwareMap.dcMotor.get("armTilt");
        extend = hardwareMap.dcMotor.get("extend");

        armTilt.setPower(1.0);
        armTilt.setTargetPosition(tiltPos);
        armTilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//hi ryan, this is a motivational message to keep you encouraged while programming
        extend.setPower(1.0);
        extend.setTargetPosition(extendPos);
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        extend.setTargetPosition(extendPos);
        armTilt.setTargetPosition(tiltPos);
    }
}
