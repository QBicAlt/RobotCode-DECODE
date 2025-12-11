package org.firstinspires.ftc.teamcode.subsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;
public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake() {}

    public final MotorEx intakeOne = new MotorEx("intake_one");
    public final MotorEx intakeTwo = new MotorEx("intake_two");
    public final ServoEx indexer = new ServoEx("indexer");

    public final Command intakeOnePowerFull = new SetPower(intakeOne, 1).requires(this);
    public final Command intakeTwoPower8 = new SetPower(intakeTwo, .8).requires(this);
    public final Command intakeTwoPowerFull = new SetPower(intakeTwo, 1).requires(this);

    public final Command intakeTwoZero = new SetPower(intakeTwo, 0).requires(this);
    public final Command intakeOneZero = new SetPower(intakeOne, 0).requires(this);

    public final Command indexerIn = new SetPosition(indexer, .04).requires(this);
    public final Command indexerOut = new SetPosition(indexer, .77).requires(this);

    @Override
    public void initialize() {
    }

    @Override
    public void periodic() {

    }
}


