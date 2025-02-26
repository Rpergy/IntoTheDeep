package org.firstinspires.ftc.teamcode.utility.dataTypes;

import androidx.annotation.NonNull;

public class Pose {
    public double x, y, heading;

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose(Point point, double heading) {
        this.x = point.x;
        this.y = point.y;
        this.heading = heading;
    }

    public Pose(Pose newPose) {
        this.x = newPose.x;
        this.y = newPose.y;
        this.heading = newPose.heading;
    }

    public Point toPoint() {
        return new Point(x, y);
    }

    /**
     * Checks if another pose is within [-range, range] of the parent point
     * @param p Comparison pose
     * @param range Range of comparison
     * @return if p is within [-range, range] of pose
     */
    public boolean withinRange(Pose p, double range) {
        return (p.x >= x - range && p.x <= x + range) && (p.y >= y - range && p.y <= y + range);
    }

    public Pose augment(Pose newPose) {
        return new Pose(x + newPose.x, y + newPose.y, heading + newPose.heading);
    }

    @NonNull
    @Override
    public String toString() {
        return "X: " + x + ", Y: " + y + ", H: " + heading;
    }

    public boolean equals(Pose other) {
        return (x == other.x) && (y == other.y) && (heading == other.heading);
    }
}
