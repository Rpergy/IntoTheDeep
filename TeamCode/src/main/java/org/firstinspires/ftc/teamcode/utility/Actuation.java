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

    private static boolean clawOpen = false;
    private static boolean clawToggle = false;

    private static boolean intake = true;
    private static boolean intakeToggle = false;

    public static DcMotor frontLeft, frontRight, backLeft, backRight;

    public static DcMotor extend, tilt;

    public static ColorSensor color;

    public static Servo flip, claw;

//    private static RevBlinkinLedDriver leds;

    private static FtcDashboard dashboard;

    public static Telemetry telemetry;

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

        if (map.dcMotor.contains("armTilt")) {
            tilt = map.dcMotor.get("armTilt");
            tilt.setPower(1.0);
            tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tilt.setTargetPosition(ActuationConstants.Mechanical.tiltInit);
        }

        if (map.dcMotor.contains("extend")) {
            extend = map.dcMotor.get("extend");
            extend.setPower(1.0);
            extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extend.setTargetPosition(ActuationConstants.Mechanical.slidesInit);
        }

        if (map.servo.contains("flip")) {
            flip = map.servo.get("flip");
            flip.setPosition(ActuationConstants.Mechanical.flipInit);
        }
        if (map.servo.contains("claw")) {
            claw = map.servo.get("claw");
            claw.setPosition(ActuationConstants.Mechanical.closedClaw);
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

    // EXTEND
    public static void setExtension(int pos) {
        extend.setTargetPosition(pos);
    }

    // TILT
    public static void setTilt(int pos) {
        tilt.setTargetPosition(pos);
    }

    // CLAW
    public static void setClaw(double pos) {
        claw.setPosition(pos);
    }

    public static void setFlip(double pos) {
        flip.setPosition(pos);
    }

    public static void toggleClaw(boolean input) {
        if (input && clawToggle) {
            clawOpen = !clawOpen;

            if (clawOpen)
                setClaw(ActuationConstants.Mechanical.openClaw);
            else
                setClaw(ActuationConstants.Mechanical.closedClaw);
        }

        clawToggle = input;
    }

    public static void toggleIntake(boolean input) {
        if (input && intakeToggle) {
            intake = !intake;

            if (intake) {
                setTilt(ActuationConstants.Mechanical.tiltIntake);
                setFlip(ActuationConstants.Mechanical.flipIntake);
            }
            else {
                setTilt(ActuationConstants.Mechanical.tiltDeposit);
                setFlip(ActuationConstants.Mechanical.flipDeposit);
            }
        }
        intakeToggle = input;
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
