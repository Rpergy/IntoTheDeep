package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;

public class ActuationConstants {
    @Config
    public static class Autonomous {
        public static double moveSpeed = 0.6;
        public static double turnSpeed = 0.65;
        public static double followDistance = 10;
        //omkar is mucho gay
        public static double minTurnSpeed = 0.04;
        public static double turnAccelMult = 2.25;
        public static double moveAccelMult = 0.75;
        public static double strafeAccelMult = 0.75;
    }

    @Config
    public static class Intake {
        // extension
        public static int max = 1000;
        public static int min = 0;

        // claw
        public static double open = 0.0;
        public static double closed = 0.0;

        // linear actuator
        public static double up = 0.0;
        public static double down = 0.0;

        // wrists
        public static double horizontal = 0.0;
        public static double vertical = 0.0;
    }

    @Config
    public static class Deposit {
        // extension
        public static int max = 1000;
        public static int min = 0;

        // claw
        public static double open = 0.0;
        public static double closed = 0.0;

        // wrist
        public static double transfer = 0.0;
        public static double deposit = 0.0;
    }

    @Config
    public static class Drivetrain {
        public static final double ticksPerRev = 2000;

        public static double centerMultiplier = 0.3926; // responsible for move
        public static double lateralMultiplier = 2.5464; // responsible for turn
        public static double perpendicularMultiplier = -0.3975; // responsible for strafe

        public static double wheel_circ = 10.05; // cm
        public static double track_width = 11.25 * lateralMultiplier; // inches distance between drive wheels (test robot: 11.024)
        public static double forward_offset = 15.0; // inches distance from center of robot to perp wheel (test robot: -5.906)

        public static double scale = wheel_circ / ticksPerRev;
    }
}
