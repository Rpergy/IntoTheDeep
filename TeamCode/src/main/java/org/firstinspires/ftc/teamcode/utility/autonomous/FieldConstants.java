package org.firstinspires.ftc.teamcode.utility.autonomous;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

import java.lang.reflect.Field;
import java.util.Dictionary;
import java.util.Hashtable;

@Config
public class FieldConstants {
    Point middle = new Point(0, 0);

    public static class Blue {
        public static Pose leftStart = new Pose(0,0,0);
        public static Pose rightStart = new Pose(0,0,0);

        public static Pose leftShort = new Pose(0,0,0);
        public static Pose leftLong = new Pose(0,0,0);
        public static Pose rightShort = new Pose(0,0,0);
        public static Pose rightLong = new Pose(0,0,0);

        public static Pose observation = new Pose(0,0,0);
        public static Pose baskets = new Pose(0,0,0);

        public static Pose neutralSamples = new Pose(0,0,0);
        public static Pose allianceSamples = new Pose(0,0,0);
    }

    public static class Red {
        public static Pose leftStart = new Pose(0,0,0);
        public static Pose rightStart = new Pose(0,0,0);

        public static Pose leftShort = new Pose(0,0,0);
        public static Pose leftLong = new Pose(0,0,0);
        public static Pose rightShort = new Pose(0,0,0);
        public static Pose rightLong = new Pose(0,0,0);

        public static Pose observation = new Pose(0,0,0);
        public static Pose baskets = new Pose(0,0,0);

        public static Pose neutralSamples = new Pose(0,0,0);
        public static Pose allianceSamples = new Pose(0,0,0);
    }
}