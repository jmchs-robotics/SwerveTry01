package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ToggleElevatorLockCommand extends Command{
    private final ElevatorSubsystem elevator;

    public ToggleElevatorLockCommand(ElevatorSubsystem elevator ) {
        this.elevator = elevator;
    }

    protected void initialize(){
        elevator.lock();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    protected void end(){
        elevator.unlock();
    }

    protected void interrupted(){
        end();
    }
}
