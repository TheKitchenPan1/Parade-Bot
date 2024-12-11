package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.util.Units;
// import frc.robot.Schematic;

public class IndexerIOSparkMax implements IndexerIO {
  private final CANSparkMax indexer;
  private final RelativeEncoder indexerEncoder;
  private final SparkLimitSwitch indexerLimitSwitch;
  private static final int INDEXER_ID = 1; // TODO: move to schematic

  public IndexerIOSparkMax() {
    indexer = new CANSparkMax(INDEXER_ID, MotorType.kBrushless);
    indexer.setIdleMode(IdleMode.kBrake);
    indexer.setInverted(false);
    indexerEncoder = indexer.getEncoder();
    indexerLimitSwitch = indexer.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    indexerEncoder.setVelocityConversionFactor(
        Units.rotationsPerMinuteToRadiansPerSecond(1) * IndexerIOConstants.INDEXER_GEAR_RATIO);
    indexerEncoder.setPositionConversionFactor(
        Units.rotationsToRadians(1) * IndexerIOConstants.INDEXER_GEAR_RATIO);

    indexer.setSmartCurrentLimit(40);
  }

  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexerVelocityRadPerSec = indexerEncoder.getVelocity();
    inputs.indexerLimitSwitchPressed = indexerLimitSwitch.isPressed();
    inputs.indexerAppliedVolts = indexer.getAppliedOutput() * indexer.getBusVoltage();
    inputs.indexerCurrentAmps = indexer.getOutputCurrent();
  }

  public void setIndexerVoltage(double volts) {
    indexer.setVoltage(volts);
  }

  public void setIndexerPercentVelocity(double percent) {
    indexer.set(percent);
  }
}
