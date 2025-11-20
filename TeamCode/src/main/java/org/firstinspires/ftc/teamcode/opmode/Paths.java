package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import java.util.function.Function;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.extensions.pedro.PedroComponent;

public class Paths {
    // GPP
    public static final Command GPP = new LambdaCommand().setStart(() -> {
        Follower follower = PedroComponent.follower();
        follower.followPath(follower.pathBuilder().build());
    });

    // PGP
    public static final Command PGP = new LambdaCommand().setStart(() -> {
        Follower follower = PedroComponent.follower();
        follower.followPath(follower.pathBuilder().build());
    });

    // PPG
    public static final Command PPG = new LambdaCommand().setStart(() -> {
        Follower follower = PedroComponent.follower();
        follower.followPath(follower.pathBuilder().build());
    });
}
