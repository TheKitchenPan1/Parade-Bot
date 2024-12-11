package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

public class ShooterIOSparkMax implements ShooterIO {
  public static final int SHOOTER_ID = 1; // TODO: Move to schematic
  private final CANSparkMax shooter;

  private final RelativeEncoder shooterEncoder;
  double rotsToRads = Units.rotationsToRadians(1);

  public ShooterIOSparkMax() {
    shooter = new CANSparkMax(SHOOTER_ID, MotorType.kBrushless);
    shooterEncoder = shooter.getEncoder();
    shooter.setInverted(false);
    shooter.setIdleMode(IdleMode.kBrake);
    shooterEncoder.setVelocityConversionFactor(rotsToRads / 60);
    shooterEncoder.setPositionConversionFactor(rotsToRads);
    shooter.setSmartCurrentLimit(35);
  }

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.VelocityRadPerSec = shooterEncoder.getVelocity();
    inputs.AppliedVolts = shooter.getBusVoltage() * shooter.getAppliedOutput();
    inputs.CurrentAmps = shooter.getOutputCurrent();
    inputs.AppliedVolts = shooter.getAppliedOutput() * shooter.getBusVoltage();
  }

  public void setShooterVoltage(double volts) {
    shooter.setVoltage(volts);
  }

  public void setShooterVelocity(double velocity) {
    shooter.set(velocity);
  }
}
