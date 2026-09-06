package org.firstinspires.ftc.teamcode.hardware.localization;

public class Pose2D {
    public double x;       // inches (or mm, just be consistent)
    public double y;
    public double heading; // radians

    public Pose2D(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public static Pose2D copy(Pose2D p) {
        return new Pose2D(p.x, p.y, p.heading);
    }

    public Pose2D plus(Pose2D delta) {
        return new Pose2D(x + delta.x, y + delta.y, normalizeAngle(heading + delta.heading));
    }

    public static double normalizeAngle(double angleRad) {
        while (angleRad > Math.PI) angleRad -= 2 * Math.PI;
        while (angleRad < -Math.PI) angleRad += 2 * Math.PI;
        return angleRad;
    }

    @Override
    public String toString() {
        return String.format("(x=%.2f, y=%.2f, hdg=%.1f°)", x, y, Math.toDegrees(heading));
    }
}