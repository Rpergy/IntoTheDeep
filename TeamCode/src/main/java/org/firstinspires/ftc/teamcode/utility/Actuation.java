package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.autonomous.FieldConstants;

public class Actuation {
    public static boolean slowMode = false;
    public static boolean fieldCentric = false;

    private static boolean fieldCentricToggle = false;
    private static boolean slowModeToggle = false;

    private static boolean clawOpen = false;
    private static boolean clawToggle = false;

    private static boolean basketToggle, chamberToggle = false;
    private static int basketState, chamberState = 0;

    private static boolean intakeState, intakeExtensionToggle = false;

    private static boolean submersibleToggle, submersibleState = false;

    public static DcMotor frontLeft, frontRight, backLeft, backRight;

    public static DcMotorEx extend, tilt;

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
            tilt = (DcMotorEx)map.dcMotor.get("armTilt");
            tilt.setPower(1.0);
            tilt.setTargetPosition(ActuationConstants.Tilt.init);
            tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            tilt.setDirection(DcMotorSimple.Direction.REVERSE);

            PIDFCoefficients newPIDF = new PIDFCoefficients(ActuationConstants.armPID.p, ActuationConstants.armPID.i, ActuationConstants.armPID.d, 0.0);
            tilt.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);
        }

        if (map.dcMotor.contains("extend")) {
            extend = (DcMotorEx)map.dcMotor.get("extend");
            extend.setPower(1.0);
            extend.setTargetPosition(ActuationConstants.Extend.init);
            extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extend.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        if (map.servo.contains("clawTilt")) {
            flip = map.servo.get("clawTilt");
            flip.setPosition(ActuationConstants.Claw.flipInit);
        }
        if (map.servo.contains("claw")) {
            claw = map.servo.get("claw");
            claw.setPosition(ActuationConstants.Claw.closed);
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
        if (input && !clawToggle) {
            clawOpen = !clawOpen;

            if (clawOpen)
                setClaw(ActuationConstants.Claw.open);
            else
                setClaw(ActuationConstants.Claw.closed);
        }

        clawToggle = input;
    }

    public static void basketExtension(boolean input) {
        if (input && !basketToggle) {
            basketState += 1;
            if (basketState == 3) basketState = 0;

            if (basketState == 0) {
                setExtension(ActuationConstants.Extend.init);
            }
            else if (basketState == 1) {
                setExtension(ActuationConstants.Extend.lowBasket);
            }
            else if (basketState == 2) {
                setExtension(ActuationConstants.Extend.highBasket);
            }
        }
        basketToggle = input;
    }

    public static void chamberExtension(boolean input) {
        if (input && !chamberToggle) {
            chamberState += 1;
            if (chamberState == 3) chamberState = 0;

            if (chamberState == 0) {
                setExtension(ActuationConstants.Extend.init);
            }
            else if (chamberState == 1) {
                setExtension(ActuationConstants.Extend.lowChamber);
            }
            else if (chamberState == 2) {
                setExtension(ActuationConstants.Extend.highChamber);
            }
        }
        chamberToggle = input;
    }

    public static void intakeExtension(boolean input) {
        if (input && !intakeExtensionToggle) {
            intakeState = !intakeState;

            chamberState = 0;
            basketState = 0;

            if (intakeState) {
                setExtension(ActuationConstants.Extend.intake);
            }
            else {
                setExtension(ActuationConstants.Extend.init);
            }
        }
        intakeExtensionToggle = input;
    }

    public static void retractExtension(boolean input) {
        if (input) {
            chamberState = 0;
            basketState = 0;
            setExtension(ActuationConstants.Extend.init);
        }
    }

    public static void adjustExtension(double rate) {
        setExtension(extend.getTargetPosition() + (int)rate);
    }

    public static void tiltObservation(boolean input) {
        if (input) {
            submersibleState = false;
            setTilt(ActuationConstants.Tilt.intakeSpecimen);
            setFlip(ActuationConstants.Claw.flipIntakeSpecimen);
        }
    }

    public static void tiltBasketDeposit(boolean input) {
        if (input) {
            submersibleState = false;
            setTilt(ActuationConstants.Tilt.basketDeposit);
            setFlip(ActuationConstants.Claw.flipBasketDeposit);
        }
    }

    public static void tiltChamberDeposit(boolean input) {
        if (input) {
            submersibleState = false;
            setTilt(ActuationConstants.Tilt.chamberDeposit);
            setFlip(ActuationConstants.Claw.flipChamberDeposit);
        }
    }

    public static void tiltSubmersible(boolean input) {
        if (input && !submersibleToggle) {
            submersibleState = !submersibleState;

            if (submersibleState) {
                setTilt(ActuationConstants.Tilt.intakeSetup);
                setFlip(ActuationConstants.Claw.flipIntake);
            }
            else {
                setTilt(ActuationConstants.Tilt.intake);
            }
        }
    }

    public static void retractSubmersible(boolean input) {
        if (input) {
            submersibleState = false;
            chamberState = 0;
            basketState = 0;

            setTilt(ActuationConstants.Tilt.intakeSetup);
            setExtension(ActuationConstants.Extend.init);
        }
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
