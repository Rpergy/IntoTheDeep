package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

public class ActuationConstants {
    @Config
    public static class Autonomous {
        public static double moveSpeed = 0.6;
        public static double turnSpeed = 0.8;
        public static double followDistance = 10;

        public static double turnAccelMult = 2.3;
        public static double moveAccelMult = 0.5;
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
        public static int highBasket = 4800;
        public static int lowChamber = 0;
        public static int highChamber = 1600;
        public static int intake = 1800;

        public static int max = 3200;
    }

    @Config
    public static class Tilt {
        public static int init = 1250;
        public static int intakeSetup = 1700;
        public static int intake = 2400;
        public static int intakeSpecimen = 1850;
        public static int basketDeposit = 400;
        public static int chamberDeposit = 1100;
        public static int hang = 200;
    }

    @Config
    public static class Claw {
        public static double open = 1.0;
        public static double closed = 0.65;
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
