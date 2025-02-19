package org.firstinspires.ftc.teamcode.tests.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.Actuation;
import org.firstinspires.ftc.teamcode.utility.ActuationConstants;

@TeleOp(group="tests", name="Intake Test")
@Config
public class IntakeTest extends OpMode {
    public DcMotorEx armTilt, extend;
    public Servo claw, tilt;
    public static double clawPos = 0.0;
    public static double clawTiltPos = 0.0;
    public static int tiltPos = 0;
    public static int extendPos = 0;

    public static double p = 6.0;
    public static double i = 1.0;
    public static double d = 0.75;
    public static double f = 0.0;

    PIDFCoefficients original;

    FtcDashboard dashboard;

    @Override
    public void init() {
        armTilt = (DcMotorEx)hardwareMap.dcMotor.get("armTilt");
        extend = (DcMotorEx)hardwareMap.dcMotor.get("extend");

        armTilt.setPower(0.6);
        armTilt.setTargetPosition(tiltPos);
        armTilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armTilt.setDirection(DcMotorSimple.Direction.REVERSE);

        extend.setPower(1.0);
        extend.setTargetPosition(extendPos);
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setDirection(DcMotorSimple.Direction.REVERSE);

        original = armTilt.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients newPIDF = new PIDFCoefficients(p, i, d, f);

        armTilt.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);

        claw = hardwareMap.servo.get("claw");
        tilt = hardwareMap.servo.get("clawTilt");

        dashboard = FtcDashboard.getInstance();

        //hi ryan, this is a motivational message to keep you encouraged while programming
    }

    @Override
    public void loop() {
        extend.setTargetPosition(extendPos);
        armTilt.setTargetPosition(tiltPos);

        claw.setPosition(clawPos);
        tilt.setPosition(clawTiltPos);

        PIDFCoefficients current = armTilt.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients newPIDF = new PIDFCoefficients(p, i, d, f);

        if (!current.equals(newPIDF))
            armTilt.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("extend pos", extend.getCurrentPosition());
        packet.put("position", armTilt.getCurrentPosition());
        packet.put("target", armTilt.getTargetPosition());
        packet.put("P", current.p);
        packet.put("I", current.i);
        packet.put("D", current.d);
        packet.put("F", current.f);

        dashboard.sendTelemetryPacket(packet);
    }
}
