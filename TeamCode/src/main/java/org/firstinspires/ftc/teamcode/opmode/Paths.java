package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.subsystem.LauncherOuttakeFuckingThing;

import java.util.function.Function;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.extensions.pedro.PedroComponent;

public class Paths {
    // GPP
    public static final Command GPP = new LambdaCommand().setStart(() -> {

        Follower follower = PedroComponent.follower();
        follower.followPath(
                follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 8.000), new Pose(44.658, 36.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .addPath(
                        new BezierLine(new Pose(44.658, 36.000), new Pose(14.127, 35.544))
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(new Pose(14.127, 35.544), new Pose(36.000, 107.089))
                )
                .setTangentHeadingInterpolation()
                .build()
        );
    }).setStop(bool -> LauncherOuttakeFuckingThing.INSTANCE.setTargetRpm(LauncherOuttakeFuckingThing.LAUNCH_RPM));

    // PGP
    public static final Command PGP = new LambdaCommand().setStart(() -> {
        Follower follower = PedroComponent.follower();
        follower.followPath(follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 8.000), new Pose(44.886, 59.696))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .addPath(
                        new BezierLine(new Pose(44.886, 59.696), new Pose(14.354, 59.468))
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(new Pose(14.354, 59.468), new Pose(22.329, 59.468))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addPath(
                        new BezierLine(new Pose(22.329, 59.468), new Pose(35.772, 106.633))
                )
                .setTangentHeadingInterpolation()
                .build()
        );
    }).setStop(bool -> LauncherOuttakeFuckingThing.INSTANCE.setTargetRpm(LauncherOuttakeFuckingThing.LAUNCH_RPM));

    // PPG
    public static final Command PPG = new LambdaCommand().setStart(() -> {
        Follower follower = PedroComponent.follower();
        follower.followPath(follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 8.000), new Pose(43.063, 83.848))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .addPath(
                        new BezierLine(new Pose(43.063, 83.848), new Pose(19.595, 84.076))
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(new Pose(19.595, 84.076), new Pose(24.380, 84.076))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .addPath(
                        new BezierLine(new Pose(24.380, 84.076), new Pose(35.316, 107.089))
                )
                .setTangentHeadingInterpolation()
                .build()
        );
    }).setStop(bool -> LauncherOuttakeFuckingThing.INSTANCE.setTargetRpm(LauncherOuttakeFuckingThing.LAUNCH_RPM));
}
