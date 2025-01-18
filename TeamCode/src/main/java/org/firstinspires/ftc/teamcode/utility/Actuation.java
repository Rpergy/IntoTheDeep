package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.util.Arrays;

public class Actuation {
    public static boolean slowMode = false;
    public static boolean fieldCentric = false;

    private static boolean fieldCentricToggle = false;
    private static boolean slowModeToggle = false;

    private static boolean intakeExtendToggle = false;
    private static boolean intakeExtend = false;
    private static boolean intakeClawToggle = false;
    private static boolean intakeClaw = false;
    private static boolean intakeDownToggle = false;
    private static boolean isIntakeDown = false;

    private static boolean depositExtendToggle = false;
    private static boolean depositExtended = false;
    private static boolean depositWristToggle = false;
    private static boolean depositWristPos = false;
    private static boolean depositClawToggle = false;
    private static boolean depositClaw = false;

    public static DcMotor frontLeft, frontRight, backLeft, backRight;

    public static DcMotor leftDeposit, rightDeposit;
    public static DcMotor leftIntake, rightIntake;

    public static Servo intake, intakeWrist, intakeDown;
    public static Servo depositWrist, depositor;

//    private static RevBlinkinLedDriver leds;

    private static FtcDashboard dashboard;

    private static Telemetry telemetry;

    public static void setup(HardwareMap map, Telemetry tel) {
        AutoMovement.setup(map, telemetry);

        telemetry = tel;

//        leds = map.get(RevBlinkinLedDriver.class, "leds");
//        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

        frontLeft  = map.get(DcMotor.class, "frontLeft");
        frontRight = map.get(DcMotor.class, "frontRight");
        backLeft = map.get(DcMotor.class, "backLeft");
        backRight = map.get(DcMotor.class, "backRight");

//        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        if (map.dcMotor.contains("leftIntake")) {
            leftIntake = map.dcMotor.get("leftIntake");

            leftIntake.setPower(1.0);
            leftIntake.setTargetPosition(ActuationConstants.Intake.min);
            leftIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (map.dcMotor.contains("rightIntake")) {
            rightIntake = map.dcMotor.get("rightIntake");

            rightIntake.setPower(1.0);
            rightIntake.setTargetPosition(ActuationConstants.Intake.min);
            rightIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (map.servo.contains("intake")) {
            intake = map.servo.get("intake");
            intake.setPosition(ActuationConstants.Intake.open);
        }
        if (map.servo.contains("intakeDown")) {
            intakeDown = map.servo.get("intakeDown");
            intakeDown.setPosition(ActuationConstants.Intake.up);
        }
        if (map.servo.contains("intakeWrist")) {
            intakeWrist = map.servo.get("intakeWrist");
            intakeWrist.setPosition(ActuationConstants.Intake.horizontal);
        }


        if (map.dcMotor.contains("leftDeposit")) {
            leftDeposit = map.dcMotor.get("leftDeposit");

            leftDeposit.setPower(1.0);
            leftDeposit.setTargetPosition(ActuationConstants.Deposit.min);
            leftDeposit.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (map.dcMotor.contains("rightDeposit")) {
            rightDeposit = map.dcMotor.get("rightDeposit");

            rightDeposit.setPower(1.0);
            rightDeposit.setTargetPosition(ActuationConstants.Deposit.min);
            rightDeposit.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (map.servo.contains("depositWrist")) {
            depositWrist = map.servo.get("depositWrist");
            depositWrist.setPosition(ActuationConstants.Deposit.transfer);
        }
        if (map.servo.contains("depositor")) {
            depositor = map.servo.get("depositor");
            depositor.setPosition(ActuationConstants.Deposit.open);
        }

        dashboard = FtcDashboard.getInstance();
    }

    public static void drive(double move, double turn, double strafe) {
        frontLeft.setPower(move - turn - strafe);
        frontRight.setPower(move + turn + strafe);
        backLeft.setPower(move - turn + strafe);
        backRight.setPower(move + turn - strafe);
    }

    public static void teleDrive(boolean toggleSlowMode, boolean toggleFieldCentric, double move, double turn, double strafe) {
        if (toggleSlowMode && !slowModeToggle) slowMode = !slowMode;
        if (toggleFieldCentric && !fieldCentricToggle) fieldCentric = !fieldCentric;

        double multip = (slowMode) ? 0.5 : 1.0;
        if (fieldCentric) {
            double newMove = strafe*Math.sin(-AutoMovement.robotPose.heading)+move*Math.cos(-AutoMovement.robotPose.heading);
            double newStrafe = strafe*Math.cos(-AutoMovement.robotPose.heading)-move*Math.sin(-AutoMovement.robotPose.heading);

            frontLeft.setPower((newMove-turn-newStrafe) * multip);
            backLeft.setPower((newMove+turn-newStrafe) * multip);
            frontRight.setPower((newMove+turn+newStrafe) * multip);
            backRight.setPower((newMove-turn+newStrafe) * multip);
        }
        else {
            frontLeft.setPower((move-strafe-turn) * multip);
            backLeft.setPower((move+strafe-turn) * multip);
            frontRight.setPower((move+strafe+turn) * multip);
            backRight.setPower((move-strafe+turn) * multip);
        }

        slowModeToggle = toggleSlowMode;
        fieldCentricToggle = toggleFieldCentric;
    }
    public static void intakeExtend(int pos) {
        leftIntake.setTargetPosition(pos);
        rightIntake.setTargetPosition(pos);
    }

    public static void toggleIntakeExtend(boolean input) {
        if (input && intakeExtendToggle) {
            intakeExtend = !intakeExtend;

            if (intakeExtend) intakeExtend(ActuationConstants.Intake.max);
            else intakeExtend(ActuationConstants.Intake.min);
        }
        intakeExtendToggle = input;
    }

    public static void setIntakeWrist(double pos) {
        intakeWrist.setPosition(pos);
    }

    public static void setIntakeDown(double pos) {
        intakeDown.setPosition(pos);
    }

    public static void toggleIntakeDown(boolean input) {
        if (input && intakeDownToggle){
            isIntakeDown = !isIntakeDown;

            if (isIntakeDown) setIntakeDown(ActuationConstants.Intake.down);
            else setIntakeDown(ActuationConstants.Intake.up);
        }
        intakeDownToggle = input;
    }

    public static void setIntake(double pos) {
        intake.setPosition(pos);
    }

    public static void toggleIntake(boolean input) {
        if (input && intakeClawToggle) {
            intakeClaw = !intakeClaw;

            if (intakeClaw) setIntake(ActuationConstants.Intake.closed);
            else setIntake(ActuationConstants.Intake.open);
        }
        intakeClawToggle = input;
    }

    public static void depositExtend(int pos) {
        leftIntake.setTargetPosition(pos);
        rightIntake.setTargetPosition(pos);
    }

    public static void toggleDepositExtension(boolean input) {
        if (input && depositExtendToggle){
            depositExtended = !depositExtended;

            if (depositExtended) depositExtend(ActuationConstants.Deposit.max);
            else depositExtend(ActuationConstants.Deposit.min);
        }
        depositExtendToggle = input;
    }

    public static void setDepositWrist(double pos) {
        depositWrist.setPosition(pos);
    }

    public static void toggleDespoitWrist(boolean input) {
        if (input && depositWristToggle) {
            depositWristPos = !depositWristPos;

            if (depositWristPos) setDepositWrist(ActuationConstants.Deposit.deposit);
            else setDepositWrist(ActuationConstants.Deposit.transfer);
        }
        depositWristToggle = input;
    }

    public static void setDepositor(double pos) {
        depositor.setPosition(pos);
    }

    public static void toggleDepositor(boolean input) {
        if (input && depositClawToggle) {
            depositClaw = !depositClaw;

            if (depositClaw) setDepositor(ActuationConstants.Deposit.closed);
            setDepositor(ActuationConstants.Deposit.open);
        }
        depositClawToggle = input;
    }

//    public static void setLeds(RevBlinkinLedDriver.BlinkinPattern pattern) {
//        leds.setPattern(pattern);
//    }
}
