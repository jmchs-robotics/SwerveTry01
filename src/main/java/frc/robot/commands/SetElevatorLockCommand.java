package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorLockCommand extends Command{
    private final ElevatorSubsystem elevator;
    private final boolean shouldLock;

    public SetElevatorLockCommand(ElevatorSubsystem elevator, boolean shouldLock) {
        this.elevator = elevator;
        this.shouldLock = shouldLock;
    }

    protected void initialize(){
        if (shouldLock)
            elevator.lock();
        else
            elevator.unlock();
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}
