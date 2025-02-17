package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

public class ActuationConstants {
    @Config
    public static class Autonomous {
        public static double moveSpeed = 0.7;
        public static double turnSpeed = 0.8;
        public static double followDistance = 10;
        //omkar is mucho gay
        public static double minTurnSpeed = 0.04;
        public static double turnAccelMult = 2.3;
        public static double moveAccelMult = 0.7;
        public static double strafeAccelMult = 0.7;
    }

    @Config
    public static class Colors {
        public static Pose red = new Pose(0, 0, 0);
        public static Pose blue = new Pose(0, 0, 0);
        public static Pose yellow = new Pose(0, 0, 0);
    }

    @Config
    public static class Mechanical {
        public static int slidesInit = 0;
        public static int slidesDeposit = 0;

        public static int tiltInit = 0;
        public static int tiltIntake = 0;
        public static int tiltDeposit = 0;
        public static int tiltHang = 0;

        public static double flipInit = 0.0;
        public static double flipIntake = 0.0;
        public static double flipDeposit = 0.0;

        public static double openClaw = 0.0;
        public static double closedClaw = 0.0;
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
