package org.firstinspires.ftc.teamcode.tests.localization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.MathFunctions;
import org.firstinspires.ftc.teamcode.utility.autonomous.AutoMovement;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Point;
import org.firstinspires.ftc.teamcode.utility.dataTypes.Pose;

@TeleOp(name = "DodgeMovement", group="tests")
@Config
public class DodgeMovement extends OpMode {
    FtcDashboard dashboard;

    public static double falloff = 50.0;
    public static int density = 10;

    @Override
    public void init() {
        AutoMovement.setup(hardwareMap, telemetry);
        AutoMovement.setStartPos(new Pose(36, 62, 0));

        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        for (double x = -72; x < 144; x+= 144.0/density) {
            for (double y = -72; y < 144; y += 144.0/density) {
                double magnitude = falloff/MathFunctions.distance(new Point(0, 0), new Point(x, y));
                packet.fieldOverlay().strokeLine(x, y, x, y+magnitude);
            }
        }

        dashboard.sendTelemetryPacket(packet);
    }
}
