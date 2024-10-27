package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCmd extends Command {

  private final Intake intake;
  private final Supplier<Double> vacuum;
  private final Supplier<Double> feed;
  

  public IntakeCmd(Intake intake, Supplier<Double> vacuum, Supplier<Double> feed ) {
    this.intake = intake;
    this.vacuum = vacuum;
    this.feed = feed;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (vacuum.get() > 0) {
      intake.vacuum(vacuum.get());
    } else {
      intake.feed(feed.get());
    }
  }
  
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
