package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    private double turningPower;

    private Thread aimThread;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        
        SparkMaxConfig turningMotorConfig = new SparkMaxConfig();
        turningMotorConfig
            .smartCurrentLimit(30)//Limits the current to 8 Amps
            .inverted(turningMotorReversed)
            .idleMode(IdleMode.kCoast);

        turningMotor.configure(turningMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
        driveMotorConfig
            .smartCurrentLimit(30)
            .inverted(driveMotorReversed)
            .idleMode(IdleMode.kCoast);

        driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        
        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();
        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        angle = (angle + 2*Math.PI) % (2*Math.PI);
        if (angle > Math.PI){
            angle = angle - 2 * Math.PI;
        }
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        //desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        //WRITE YOUR OWN OPTIMIZE

        /*angleError = desiredState.angle.getDegrees()-getAbsoluteEncoderRad();//getState().angle.getDegrees();
        if(Math.abs(angleError)<=90){
            driveMotor.set(desiredState.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
            turningMotor.set(turningPidController.calculate(getAbsoluteEncoderRad(), desiredState.angle.getRadians()));
        }//want -135, at 44
        else{
            driveMotor.set(-desiredState.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
            turningMotor.set(turningPidController.calculate(getAbsoluteEncoderRad(), Math.abs(desiredState.angle.getRadians())-180));
        }
        SmartDashboard.putNumber("Angle Error", angleError);*/
        /*SmartDashboard.putNumber("Desired State Before", desiredState.angle.getDegrees()); //prints for Back Right
        SmartDashboard.putNumber("Desired State After", desiredState.angle.getDegrees());
        SmartDashboard.putNumber("Desired Velocity Before", desiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("Desired Velocity After", desiredState.speedMetersPerSecond);*/
        driveMotor.set(desiredState.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        //delay the following line below by one second to prevent oversampling
        turningMotor.set(turningPidController.calculate(getAbsoluteEncoderRad(), desiredState.angle.getRadians()));
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
