package org.firstinspires.ftc.teamcode.utility.autonomous;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import java.io.IOException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.CvType;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

public class SampleLocation extends OpenCvPipeline {
    public static OpenCvCamera webcam;
    private static Mat frame;
    private static final Point resolution = new Point(640, 360);
    private static final Point clawPos = new Point(320, 180);
    public static String desiredColor = "red";
    private static String redChange = "red1";
    private static boolean upP = false, downP = false,
            triP = false, croP = false,
            circP = false, sqrP = false,
            leftP = false, rightP = false,
            mode = false;

    private static final Point topLeftCrop = new Point(540, 260);
    private static final Point bottomRightCrop = new Point(100, 100);
    private static Scalar minRed1 = new Scalar(0, 50, 0);
    private static Scalar maxRed1 = new Scalar(10, 255, 255);
    private static Scalar minRed2 = new Scalar(150, 100, 100);
    private static Scalar maxRed2 = new Scalar(255, 255, 255);
    private static Scalar minBlue = new Scalar(110, 30, 35);
    private static Scalar maxBlue = new Scalar(130, 255, 255);
    private static Scalar minYellow = new Scalar(10, 140, 120);
    private static Scalar maxYellow = new Scalar(25, 255, 255);

    @Override
    public Mat processFrame(Mat input) {
        Mat rotated = new Mat();
        Core.rotate(input, rotated, Core.ROTATE_90_COUNTERCLOCKWISE);
        frame = input;
        return input;
    }
    public static org.firstinspires.ftc.teamcode.utility.dataTypes.Point findSample(double slidesLength) {
        Mat out = frame.clone();
        Imgproc.cvtColor(out, out, Imgproc.COLOR_RGB2HSV);

        List<Point> centroids = new ArrayList<>();
        List<Double> rotations = new ArrayList<>();

        // Locate contours (returns original image with contours drawn)
        Mat contours = locateContours(out, centroids, rotations, desiredColor);

        // Find the closest point to the target (clawPos)
        List<Object> closest = closestPoint(centroids, rotations);

        Point closestPoint = (Point) closest.get(0);
        closestPoint = toMovements(closestPoint); // convert pixel coordinates to robot (x) movement (in) & slides (y) movement (in)
        closestPoint.y = slidesInToMotor(closestPoint.y-3.54331);
        double closestRot = (double) closest.get(1) / 180;

        return new org.firstinspires.ftc.teamcode.utility.dataTypes.Point(closestPoint.x, closestPoint.y);
    }
    private static double slidesInToMotor(double in) {
        return 200*in-2400;
    }
    private static Point toMovements(Point p) {
        p = new Point(p.x-180, p.y-280);
        Point ret = new Point(0, 0);
        ret.x = p.x*5/360;
        ret.y = p.y*7.75/640;
        return ret;
    }
    private static Mat locateContours(Mat image, List<Point> centers, List<Double> rotations, String color) {

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat(), mask = new Mat();

        Mat hsv = image.clone();

        // Median Blur (Reduces Noise)
        Imgproc.medianBlur(hsv, hsv, 3);

        // HSV Masking
        if (color.equals("red")) {
            Mat mask2 = new Mat();
            Core.inRange(hsv, minRed1, maxRed1, mask);
            Core.inRange(hsv, minRed2, maxRed2, mask2);
            Core.bitwise_or(mask, mask2, mask);
        } else if (color.equals("blue")) {
            Core.inRange(hsv, minBlue, maxBlue, mask);
        } else if (color.equals("yellow")) {
            Core.inRange(hsv, minYellow, maxYellow, mask);
        } else {
            telemetry.addLine("Invalid desired color");
        }

        mask = applyWatershed(mask);
        Imgproc.threshold(mask, mask, 0, 255, Imgproc.THRESH_BINARY);
        mask.convertTo(mask, CvType.CV_8UC1);

        // Locating Contours
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        locateContour(contours, centers, rotations, hsv);

        return hsv;
    }
    public static Mat applyWatershed(Mat mask) {
        Mat kernel = Mat.ones(3, 3, CvType.CV_8U);
        Mat opening = new Mat();
        Imgproc.morphologyEx(mask, opening, Imgproc.MORPH_OPEN, kernel, new Point(-1, -1), 3);

        Mat sureBg = new Mat();
        Imgproc.dilate(opening, sureBg, kernel, new Point(-1, -1), 2);

        Mat distTransform = new Mat();
        Imgproc.distanceTransform(opening, distTransform, Imgproc.DIST_L2, 5);
        Mat sureFg = new Mat();
        Core.normalize(distTransform, distTransform, 0, 1, Core.NORM_MINMAX);
        Imgproc.threshold(distTransform, sureFg, 0.6 * Core.minMaxLoc(distTransform).maxVal, 255, Imgproc.THRESH_BINARY);

        return sureFg;
    }
    public static void locateContour(List<MatOfPoint> contours, List<Point> centers, List<Double> rotations, Mat output_image) {

        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            RotatedRect rect = Imgproc.minAreaRect(contour2f);
            Point[] box = new Point[4];
            rect.points(box);
            for (int j = 0; j < 4; j++) {
                Imgproc.line(output_image, box[j], box[(j + 1) % 4], new Scalar(0, 255, 0), 2);
            }

            Mat line = new Mat();
            Imgproc.fitLine(contour, line, Imgproc.DIST_L2, 0, 0.01, 0.01);
            float[] data = new float[4];
            line.get(0, 0, data);
            double vx = data[0];
            double vy = data[1];

            double angle = Math.atan2(vy, vx);
            angle = Math.toDegrees(angle);

            if (angle < 0) {
                angle += 180;
            } else if (angle > 180) {
                angle -= 180;
            }

            Moments moments = Imgproc.moments(contour);
            if (moments.get_m00() != 0) {
                int cx = (int) (moments.get_m10() / moments.get_m00());
                int cy = (int) (moments.get_m01() / moments.get_m00());
                Imgproc.circle(output_image, new Point(cx, cy), 5, new Scalar(0, 0, 255), -1);
                if (topLeftCrop.x <= cx && cx <= bottomRightCrop.x && bottomRightCrop.y <= cy && cy <= topLeftCrop.y) {
                    centers.add(new Point(cx, cy));
                    rotations.add(angle);
                }
            }
        }
    }
    private static List<Object> closestPoint(List<Point> centroids, List<Double> rotations) {
        double closeDist = Integer.MAX_VALUE;
        Point closePoint = new Point(0, 0);
        double closestRot = 0;
        for (int i = 0; i < centroids.size(); i++) {
            if (Math.sqrt(Math.pow(centroids.get(i).x - clawPos.x, 2) + Math.pow(centroids.get(i).y - clawPos.y, 2)) < closeDist) {
                closeDist = Math.sqrt(Math.pow(centroids.get(i).x - clawPos.x, 2) + Math.pow(centroids.get(i).y - clawPos.y, 2));
                closePoint = centroids.get(i);
                closestRot = rotations.get(i);
            }
        }

        ArrayList<Object> ret = new ArrayList<>();

        ret.add(closePoint);
        ret.add(closestRot);

        return ret;
    }
    private static String updateThresholds(String color, Gamepad gamepad1) {

        int dh = 0, ds = 0, dv = 0;

        if (!upP && gamepad1.dpad_up) {
            dh++;
        } else if (!downP && gamepad1.dpad_down) {
            dh--;
        } else if (!triP && gamepad1.triangle) {
            ds++;
        } else if (!croP && gamepad1.cross) {
            ds--;
        } else if (!circP && gamepad1.circle) {
            dv++;
        } else if (!sqrP && gamepad1.square) {
            dv--;
        } else if (!leftP && gamepad1.dpad_left) {
            mode = !mode;
        } else if (!rightP && gamepad1.dpad_right) {
            if (desiredColor.equals("yellow")) {
                desiredColor = "red";
                redChange = "red1";
            } else if (desiredColor.equals("red")) {
                if (redChange.equals("red1")) {
                    redChange = "red2";
                } else {
                    desiredColor = "blue";
                }
            } else {
                desiredColor = "yellow";
            }
        }

        if (color.equals("red")) {
            if (!mode) {
                if (redChange.equals("red2")) {
                    minRed2 = new Scalar(minRed2.val[0] + dh, minRed2.val[1] + ds, minRed2.val[2] + dv);
                } else {
                    minRed1 = new Scalar(minRed1.val[0] + dh, minRed1.val[1] + ds, minRed1.val[2] + dv);
                }
            } else {
                if (redChange.equals("red2")) {
                    maxRed2 = new Scalar(maxRed2.val[0] + dh, maxRed2.val[1] + ds, maxRed2.val[2] + dv);
                } else {
                    maxRed1 = new Scalar(maxRed1.val[0] + dh, maxRed1.val[1] + ds, maxRed1.val[2] + dv);
                }
            }
        } else if (color.equals("blue")) {
            if (!mode) {
                minBlue = new Scalar(minBlue.val[0] + dh, minBlue.val[1] + ds, minBlue.val[2] + dv);
            } else {
                maxBlue = new Scalar(maxBlue.val[0] + dh, maxBlue.val[1] + ds, maxBlue.val[2] + dv);
            }
        } else if (color.equals("yellow")) {
            if (!mode) {
                minYellow = new Scalar(minYellow.val[0] + dh, minYellow.val[1] + ds, minYellow.val[2] + dv);
            } else {
                maxYellow = new Scalar(maxYellow.val[0] + dh, maxYellow.val[1] + ds, maxYellow.val[2] + dv);
            }
        }
        upP = gamepad1.dpad_up;
        downP = gamepad1.dpad_down;
        triP = gamepad1.triangle;
        croP = gamepad1.cross;
        circP = gamepad1.circle;
        sqrP = gamepad1.square;
        leftP = gamepad1.dpad_left;
        rightP = gamepad1.dpad_right;

        return "minRed1=" + minRed1 + "\nmaxRed1=" + maxRed1 +
                "\nminRed2=" + minRed2 + "\nmaxRed2=" + maxRed2 +
                "\nminBlue=" + minBlue + "\nmaxBlue=" + maxBlue +
                "\nminYellow=" + minYellow + "\nmaxYellow=" + maxYellow +
                "\nColor=" + desiredColor + ((mode) ? "\nMode=maximum" : "\nMode=minimum") +
                "RedType=" + redChange + " (Only if color is on red)";
    }
    private static Point bound(Point point) {
        double x = point.x;
        double y = point.y;
        if (x < 0) {
            x = 0;
        } else if (x > resolution.y - 1) {
            x = resolution.y - 1;
        }
        if (y < 0) {
            y = 0;
        } else if (y > resolution.x - 1) {
            y = resolution.x - 1;
        }
        return new Point(x, y);
    }
}
