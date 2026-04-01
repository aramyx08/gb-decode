package org.firstinspires.ftc.teamcode.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utilities.TelemetryDebug;

import java.util.List;

@TeleOp (name = "Demo TeleOp (Blue)")
public class DemoTeleOp extends OpMode {
    private Follower follower;
    private Intake intake;
    private Kicker kicker;
    private Turret turret;
    private Shooter shooter;
    private TelemetryDebug debug;

    @Override
    public void init() {
        // Sets Bulk Reading to auto
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));
        follower.update();

        debug = new TelemetryDebug();
        intake = new Intake(hardwareMap);
        kicker = new Kicker(hardwareMap, debug);
        turret = new Turret(hardwareMap, false, debug);
        shooter = new Shooter(hardwareMap, false, debug);
    }

    @Override
    public void start() {
        follower.startTeleOpDrive();
    }

    @Override
    public void loop() {
        follower.update();
        kicker.update();

        if (follower.getPose() != null) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y, // Forward/Back
                    -gamepad1.left_stick_x, // Strafe
                    -gamepad1.right_stick_x * 0.6, // Turn
                    true // TRUE = Robot Centric
            );
        }

        if (gamepad1.left_trigger_pressed) {
            intake.intake();
        } else {intake.idle();}

        if (gamepad1.right_bumper) {
            shooter.accelerate();
            turret.aim();
        } else {
            shooter.idle();
            turret.idle();
        }

        if (gamepad1.right_trigger_pressed) {
            kicker.startSequence();
        }

        for (TelemetryDebug.watcher w : debug.watchers){
            telemetry.addData(w.getName(), w.getValue());
        }
    }
}
