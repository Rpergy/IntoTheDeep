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
        public static Pose leftStart = new Pose(36,62, Math.toRadians(-90));
        public static Pose rightStart = new Pose(-12,62, Math.toRadians(-90));

        public static Pose leftShort = new Pose(6,35, Math.toRadians(-90));
        public static Pose leftLong = new Pose(25,12, Math.toRadians(180));
        public static Pose rightShort = new Pose(-6,35, Math.toRadians(-90));
        public static Pose rightLong = new Pose(-25,12,0);

        public static Pose observation1 = new Pose(-40,60, Math.toRadians(135));
        public static Pose observation2 = new Pose(-58, 50, Math.toRadians(90));
        public static Pose deliverPoint = new Pose(observation1);
        public static Pose pickupPoint = new Pose(observation2);

        public static Pose baskets = new Pose(57,57, Math.toRadians(45));

        public static Pose neutralSamples = new Pose(49,42, Math.toRadians(-90));
        public static Pose allianceSamples = new Pose(-49,42, Math.toRadians(-90));
    }

    public static class Red {
        public static Pose leftStart = new Pose(-36,-62, Math.toRadians(90));
        public static Pose rightStart = new Pose(12,-62, Math.toRadians(90));

        public static Pose leftShort = new Pose(-6,-35, Math.toRadians(90));
        public static Pose leftLong = new Pose(-25,-10,0);
        public static Pose rightShort = new Pose(5,-41.5, Math.toRadians(90));
        public static Pose rightLong = new Pose(25,-12,Math.toRadians(180));

        public static Pose observation1 = new Pose(40,-60, Math.toRadians(-45));
        public static Pose observation2 = new Pose(58, -50, Math.toRadians(-90));
        public static Pose deliverPoint = new Pose(observation1);
        public static Pose pickupPoint = new Pose(observation2);
        public static Pose baskets = new Pose(-53.5,-52.5, Math.toRadians(-135));

        public static Pose neutralSamples = new Pose(-48.4,-41.5,Math.toRadians(90));
        public static Pose allianceSamples = new Pose(28,-33,Math.toRadians(45));

        public static Pose specimenPickup = new Pose(40, -53, Math.toRadians(-90));
    }
}