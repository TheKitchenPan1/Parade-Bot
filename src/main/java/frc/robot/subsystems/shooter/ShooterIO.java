package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public class ShooterIOInputs {
    // information/vars we need
    public double VelocityRadPerSec = 0.0;
    public double AppliedVolts = 0.0;
    public double CurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(ShooterIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  default void setVoltage(double Volts) {}

  default void setShooterVelocity(double velocity) {}
}
