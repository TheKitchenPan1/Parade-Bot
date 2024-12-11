package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  /** Creates a new shooter. */
  private final ShooterIO io;

  private boolean PIDMode = false;
  private double currentVelocitySetpoint;
  private SimpleMotorFeedforward shooterFeedforward =
      new SimpleMotorFeedforward(ShooterIOConstants.SHOOTER_KS, ShooterIOConstants.SHOOTER_KV);
  private PIDController shooterFeedback =
      new PIDController(ShooterIOConstants.SHOOTER_KP, 0, ShooterIOConstants.SHOOTER_KD);

  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
    shooterFeedback.setTolerance(ShooterIOConstants.SHOOTER_TOLERANCE);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    if (PIDMode) {
      io.setVoltage(
          shooterFeedforward.calculate(currentVelocitySetpoint)
              + shooterFeedback.calculate(inputs.VelocityRadPerSec, currentVelocitySetpoint));
    }
    Logger.recordOutput("Shooter/setPointVelocity", shooterFeedback.getSetpoint());
    Logger.recordOutput("Shooter/error", shooterFeedback.getVelocityError());
  }
  /**
   * @param velocitySetpoint the velocity that the shooter should be set to
   * @return A command that sets the PIDMode to true, and then sets to PID setpoint to that of the
   *     inputted velocitySetpoint
   */
  public Command shoot(double velocitySetpoint) {
    return Commands.run(
        () -> {
          PIDMode = true;
          currentVelocitySetpoint = velocitySetpoint;
        },
        this);
  }

  /**
   * @param percent the volts that the shooter should be set to
   * @return A command that sets the shooter voltage to that of the inputed volts
   */
  public Command manualShoot(double percent) {
    return Commands.run(
        () -> {
          io.setShooterVelocity(percent);
          PIDMode = false;
        },
        this);
  }

  public Command manualShootVolts(double volts) {
    return Commands.run(
        () -> {
          io.setVoltage(volts);
          PIDMode = false;
        },
        this);
  }

  public Command stop() {
    return Commands.run(
        () -> {
          io.setVoltage(0.0);
          PIDMode = false;
        },
        this);
  }
  /**
   * @return if the shooter is at the speed required to shoot by checking if the shooters speed is
   *     that of the setpoint of the PID.
   */
  public boolean ShooterSpeedIsReady() {
    return inputs.VelocityRadPerSec >= ShooterIOConstants.SHOOTER_READY_VELOCITY_RAD_PER_SEC;
  }

  public boolean atVelocity(double velocitySetpoint) {
    return inputs.VelocityRadPerSec >= velocitySetpoint - 0.05;
  }
}
