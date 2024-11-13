package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;

public class ActuationConstants {
    @Config
    public static class Autonomous {
        public static double moveSpeed = 0.8;
        public static double turnSpeed = 0.85;
        public static double followDistance = 10;
        //omkar is gay
        public static double minTurnSpeed = 0.06;
        public static double turnAccelMult = 2.5;
        public static double moveAccelMult = 0.75;
        public static double strafeAccelMult = 0.75;
    }

    @Config
    public static class Drivetrain {
        public static final double ticksPerRev = 2000;

        public static double centerMultiplier = 0.3926; // responsible for move
        public static double lateral_multiplier = 2.5464; // responsible for turn
        public static double perpendicularMultiplier = -0.3975; // responsible for strafe

        public static double wheel_circ = 10.05; // cm
        public static double track_width = 11.25 * lateral_multiplier; // inches distance between drive wheels (test robot: 11.024)
        public static double forward_offset = -7.25; // inches distance from center of robot to perp wheel (test robot: -5.906)

        public static double scale = wheel_circ / ticksPerRev;
    }
}
