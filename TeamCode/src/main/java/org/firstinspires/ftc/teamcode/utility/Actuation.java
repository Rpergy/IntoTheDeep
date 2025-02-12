package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.FieldConstants;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

public class Actuation {
    public static boolean slowMode = false;
    public static boolean fieldCentric = false;

    private static boolean fieldCentricToggle = false;
    private static boolean slowModeToggle = false;

    private static boolean intakeExtendToggle = false;
    private static boolean intakeExtend = false;
    private static boolean intakeClawToggle = false;
    private static boolean intakeClawOpen = false;
    private static boolean intakeRotateToggle = false;
    private static boolean intakeVertical = false;
    private static boolean intakeWristToggle = false;
    private static boolean intakeWristDown = false;
    private static boolean intakeArmToggle = false;
    private static boolean intakeArmDown = false;


    private static boolean depositExtendToggle = false;
    private static boolean depositExtended = false;
    private static boolean depositWristToggle = false;
    private static boolean depositWristPos = false;
    private static boolean depositFlipToggle = false;
    private static boolean depositFlipped = false;
    private static boolean depositClawToggle = false;
    private static boolean depositClaw = false;

    public static DcMotor frontLeft, frontRight, backLeft, backRight;

    public static DcMotor leftDeposit, rightDeposit;
    public static DcMotor leftIntake, rightIntake;

    public static Servo intakeClaw, intakeWrist, intakeRotate, intakeArm;
    public static Servo depositWrist, depositFlip, depositor;

    public static ColorSensor color;

//    private static RevBlinkinLedDriver leds;

    private static FtcDashboard dashboard;

    private static Telemetry telemetry;

    public static void setup(HardwareMap map, Telemetry tel) {
        AutoMovement.setup(map, telemetry);

        telemetry = tel;

//        leds = map.get(RevBlinkinLedDriver.class, "leds");
//        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

        if (map.dcMotor.contains("frontLeft")) {
            frontLeft = map.get(DcMotor.class, "frontLeft");
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (map.dcMotor.contains("frontRight")) {
            frontRight = map.get(DcMotor.class, "frontRight");
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (map.dcMotor.contains("backLeft")) {
            backLeft = map.get(DcMotor.class, "backLeft");
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (map.dcMotor.contains("backRight")) {
            backRight = map.get(DcMotor.class, "backRight");
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

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
            rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        if (map.servo.contains("intake")) {
            intakeClaw = map.servo.get("intake");
            intakeClaw.setPosition(ActuationConstants.Intake.closed);
        }

        if (map.servo.contains("intakeRotate")) {
            intakeRotate = map.servo.get("intakeRotate");
            intakeRotate.setPosition(ActuationConstants.Intake.vertical);
        }

        if (map.servo.contains("intakeWrist")) {
            intakeWrist = map.servo.get("intakeWrist");
            intakeWrist.setPosition(ActuationConstants.Intake.wristInit);
        }

        if (map.servo.contains("intakeArm")) {
            intakeArm = map.servo.get("intakeArm");
            intakeArm.setPosition(ActuationConstants.Intake.armInit);
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
            depositWrist.setPosition(ActuationConstants.Deposit.wristTransfer);
        }
        if (map.servo.contains("depositFlip")) {
            depositFlip = map.servo.get("depositFlip");
            depositFlip.setPosition(ActuationConstants.Deposit.flipTransfer);
        }
        if (map.servo.contains("depositor")) {
            depositor = map.servo.get("depositor");
            depositor.setPosition(ActuationConstants.Deposit.closed);
        }

        dashboard = FtcDashboard.getInstance();
    }

    public static void drive(double move, double turn, double strafe) {
        frontLeft.setPower(move + turn + strafe);
        frontRight.setPower(move - turn - strafe);
        backLeft.setPower(move + turn - strafe);
        backRight.setPower(move - turn + strafe);
    }

    public static void teleDrive(boolean toggleSlowMode, boolean toggleFieldCentric, double move, double turn, double strafe) {
        if (toggleSlowMode && !slowModeToggle) slowMode = !slowMode;
        if (toggleFieldCentric && !fieldCentricToggle) fieldCentric = !fieldCentric;

        double multip = (slowMode) ? 0.5 : 1.0;
        if (fieldCentric) {
            double newMove = strafe*Math.sin(-AutoMovement.robotPose.heading)+move*Math.cos(-AutoMovement.robotPose.heading);
            double newStrafe = strafe*Math.cos(-AutoMovement.robotPose.heading)-move*Math.sin(-AutoMovement.robotPose.heading);

            frontLeft.setPower((newMove+turn+newStrafe) * multip);
            backLeft.setPower((newMove-turn+newStrafe) * multip);
            frontRight.setPower((newMove+turn-newStrafe) * multip);
            backRight.setPower((newMove-turn-newStrafe) * multip);
        }
        else {
            frontLeft.setPower((move+strafe+turn) * multip);
            backLeft.setPower((move-strafe+turn) * multip);
            frontRight.setPower((move-strafe-turn) * multip);
            backRight.setPower((move+strafe-turn) * multip);
        }

        slowModeToggle = toggleSlowMode;
        fieldCentricToggle = toggleFieldCentric;
    }

    // INTAKE EXTENSION
    public static void intakeExtend(int pos) {
        pos = Math.min(ActuationConstants.Intake.max, pos);
        leftIntake.setTargetPosition(pos);
        rightIntake.setTargetPosition(pos);
    }

    public static void toggleIntakeExtend(boolean input) {
        if (input && !intakeExtendToggle) {
            intakeExtend = !intakeExtend;

            if (intakeExtend) {
                intakeExtend(ActuationConstants.Intake.max);
                setIntakeArm(ActuationConstants.Intake.armInbound);
                setIntakeWrist(ActuationConstants.Intake.wristIntake);
            }
            else {
                intakeExtend(ActuationConstants.Intake.min);
                setIntakeArm(ActuationConstants.Intake.armTransfer);
                setIntakeWrist(ActuationConstants.Intake.wristTransfer);
            }
        }
        intakeExtendToggle = input;
    }

    // INTAKE WRIST
    public static void setIntakeWrist(double pos) {
        intakeWrist.setPosition(pos);
    }
    public static void toggleIntakeWrist(boolean input) {
        if (input && intakeWristToggle){
            intakeWristDown = !intakeWristDown;

            if (intakeWristDown) setIntakeWrist(ActuationConstants.Intake.wristIntake);
            else setIntakeWrist(ActuationConstants.Intake.wristTransfer);
        }
        intakeWristDown = input;
    }

    // INTAKE ROTATE
    public static void setIntakeRotate(double pos) {
        intakeRotate.setPosition(pos);
    }

    public static void toggleIntakeRotate(boolean input) {
        if (input && intakeRotateToggle){
            intakeVertical = !intakeVertical;

            if (intakeVertical) setIntakeRotate(ActuationConstants.Intake.vertical);
            else setIntakeRotate(ActuationConstants.Intake.horizontal);
        }
        intakeRotateToggle = input;
    }

    // INTAKE ARM
    public static void setIntakeArm(double pos) { intakeArm.setPosition(pos); }

    public static void toggleIntakeArm(boolean input) {
        if (input && intakeArmToggle){
            intakeArmDown = !intakeArmDown;

            if (intakeArmDown) setIntakeArm(ActuationConstants.Intake.armIntake);
            else setIntakeArm(ActuationConstants.Intake.armTransfer);
        }
        intakeArmToggle = input;
    }

    // INTAKE CLAW
    public static void setIntakeClaw(double pos) {
        intakeClaw.setPosition(pos);
    }

    public static void toggleIntake(boolean input) {
        if (input && intakeClawToggle) {
            intakeClawOpen = !intakeClawOpen;

            if (intakeClawOpen) setIntakeClaw(ActuationConstants.Intake.closed);
            else setIntakeClaw(ActuationConstants.Intake.open);
        }
        intakeClawToggle = input;
    }

    // DEPOSIT EXTENSION
    public static void depositExtend(int pos) {
        leftDeposit.setTargetPosition(pos);
        rightDeposit.setTargetPosition(pos);
    }

    public static void toggleDepositExtension(boolean input) {
        if (input && depositExtendToggle){
            depositExtended = !depositExtended;

            if (depositExtended) depositExtend(ActuationConstants.Deposit.max);
            else depositExtend(ActuationConstants.Deposit.min);
        }
        depositExtendToggle = input;
    }

    // DEPOSIT WRIST
    public static void setDepositWrist(double pos) {
        depositWrist.setPosition(pos);
    }

    public static void toggleDepositWrist(boolean input) {
        if (input && depositWristToggle) {
            depositWristPos = !depositWristPos;

            if (depositWristPos) setDepositWrist(ActuationConstants.Deposit.wristDeposit);
            else setDepositWrist(ActuationConstants.Deposit.wristTransfer);
        }
        depositWristToggle = input;
    }

    // DEPOSIT FLIP
    public static void setDepositFlip(double pos) {
        depositFlip.setPosition(pos);
    }

    public static void toggleDepositFlip(boolean input) {
        if (input && depositFlipToggle) {
            depositFlipped = !depositFlipped;

            if (depositFlipped) setDepositFlip(ActuationConstants.Deposit.flipDeposit);
            else setDepositFlip(ActuationConstants.Deposit.flipTransfer);
        }
        depositFlipToggle = input;
    }

    // DEPOSIT CLAW
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

    // collection of methods that line up the deposit for cleaner autonomous usage
    public static void autoDeposit() { // sets to deposit pos
        setDepositFlip(ActuationConstants.Deposit.flipDeposit);
        setDepositWrist(ActuationConstants.Deposit.wristDeposit);
    }

    public static void autoDepositTransfer() { // sets to transfer pos
        setDepositFlip(ActuationConstants.Deposit.flipTransfer);
        setDepositWrist(ActuationConstants.Deposit.wristTransfer);
    }

    // collection of methods that line up the intake for cleaner autonomous usage
    public static void autoIntake() { // sets to intake pos
        setIntakeRotate(ActuationConstants.Intake.vertical);
        setIntakeArm(ActuationConstants.Intake.armIntake);
        setIntakeWrist(ActuationConstants.Intake.wristIntake);
    }

    public static void autoIntakeTransfer() { // sets to transfer pos
        setIntakeRotate(ActuationConstants.Intake.vertical);
        setIntakeArm(ActuationConstants.Intake.armTransfer);
        setIntakeWrist(ActuationConstants.Intake.wristTransfer);
    }

    public static boolean scanSamples(Pose targetColor, int step) {
        int extension = ActuationConstants.Intake.min;
        Pose reading = new Pose(color.red(), color.blue(), color.green());
        while (!reading.withinRange(targetColor, 10)) {
            if (extension >= ActuationConstants.Intake.max) {
                return false;
            }
            extension += step;
            intakeExtend(extension);
            reading = new Pose(color.red(), color.blue(), color.green());
        }
        return true;
    }

    // Toggles between p1 and p2 dropoff/pickup for autonomous
    public static void toggleBlueDeliver() {
        if (FieldConstants.Blue.deliverPoint.equals(FieldConstants.Blue.observation1)) {
            FieldConstants.Blue.deliverPoint = FieldConstants.Blue.observation2;
            FieldConstants.Blue.pickupPoint = FieldConstants.Blue.observation1;
        }
        else {
            FieldConstants.Blue.deliverPoint = FieldConstants.Blue.observation1;
            FieldConstants.Blue.pickupPoint = FieldConstants.Blue.observation2;
        }
    }
    public static void toggleRedDeliver() {
        if (FieldConstants.Red.deliverPoint.equals(FieldConstants.Red.observation1)) {
            FieldConstants.Red.deliverPoint = FieldConstants.Red.observation2;
            FieldConstants.Red.pickupPoint = FieldConstants.Red.observation1;
        }
        else {
            FieldConstants.Red.deliverPoint = FieldConstants.Red.observation1;
            FieldConstants.Red.pickupPoint = FieldConstants.Red.observation2;
        }
    }

//    public static void setLeds(RevBlinkinLedDriver.BlinkinPattern pattern) {
//        leds.setPattern(pattern);
//    }
}
