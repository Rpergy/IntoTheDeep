package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

public class ActuationConstants {
    @Config
    public static class Autonomous {
        public static double moveSpeed = 0.7;
        public static double turnSpeed = 0.8;
        public static double followDistance = 10;

        public static double turnAccelMult = 2.3;
        public static double moveAccelMult = 0.6;
    }

    @Config
    public static class Colors {
        public static Pose red = new Pose(0, 0, 0);
        public static Pose blue = new Pose(0, 0, 0);
        public static Pose yellow = new Pose(0, 0, 0);
    }

    @Config
    public static class armPID {
        public static double p = 5.8;
        public static double i = 1.0;
        public static double d = 0.75;
    }

    @Config
    public static class Extend {
        public static int init = 100;
        public static int lowBasket = 2000;
        public static int highBasket = 3500;
        public static int lowChamber = 0;
        public static int highChamber = 1800;
        public static int intake = 1800;
    }

    @Config
    public static class Tilt {
        public static int init = 1700;
        public static int intakeSetup = 2000;
        public static int intake = 2450;
        public static int intakeSpecimen = 1700;
        public static int basketDeposit = 800;
        public static int chamberDeposit = 1100;
        public static int hang = 200;
    }

    @Config
    public static class Claw {
        public static double flipInit = 0.8;
        public static double flipIntake = 0.0;
        public static double flipIntakeSpecimen = 0.35;
        public static double flipBasketDeposit = 0.5;
        public static double flipChamberDeposit = 0.3;

        public static double open = 0.15;
        public static double closed = 0.0;
    }

    @Config
    public static class Drivetrain {
        public static final double ticksPerRev = 2000;

        public static double centerMultiplier = 0.3922; // responsible for move
        public static double lateralMultiplier = 2.5171; // responsible for turn
        public static double perpendicularMultiplier = -0.3923; // responsible for strafe

        public static double wheel_circ = 10.05; // cm
        public static double track_width = 11.25 * lateralMultiplier; // inches distance between drive wheels (test robot: 11.024)
        public static double forward_offset = 15.0; // inches distance from center of robot to perp wheel (test robot: -5.906)

        public static double scale = wheel_circ / ticksPerRev;
    }
}
