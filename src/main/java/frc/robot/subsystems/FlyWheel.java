package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlyWheel extends SubsystemBase{
    private final TalonFX leftWheel;
    private final TalonFX rightWheel;

    private final int leftWheelID = 14; //ID's of TalonFX through CANBUS
    private final int rightWheelID = 1; //ID's of TalonFX through CANBUS

    private volatile double targetSpeed;

    private final double MAX_SPEED = 511.998;
    private final double MIN_SPEED = -512;

    private boolean wheelTargetHit = false;

    private final double INPUT_THRESHOLD = 0.001;

    public FlyWheel(){
        leftWheel = new TalonFX(leftWheelID);
        rightWheel = new TalonFX(rightWheelID);

        leftWheel.setNeutralMode(NeutralModeValue.Coast);
        rightWheel.setNeutralMode(NeutralModeValue.Coast);

        

    }

    public void shoot(double power)
    {
        
        leftWheel.set(-power*0.8);
        rightWheel.set(power*0.81);//puts spin on ring
    }

    public void stop()
    {
        leftWheel.set(0);
        rightWheel.set(0);
    }

    public double getLeftSpeed()
    {
        return leftWheel.getVelocity().getValueAsDouble(); //range {-512, 511.998} rotations per second
    }

    public double getRightSpeed()
    {
        return rightWheel.getVelocity().getValueAsDouble(); //range {-512, 511.998} rotations per second
    }

    public Command getShootCommand()
    {
        return new StartEndCommand(() -> shoot(1), () -> stop(), this);
    }
}
