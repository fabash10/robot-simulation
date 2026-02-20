package frc.robot.elevator;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    private final TalonFX motor = new TalonFX(ElevatorConst.MOTOR_ID);
    private double targetHeightFraction = 0.0;

    public ElevatorSubsystem() {
        motor.getConfigurator().apply(ElevatorConfig.motorConfig);
    }

    public void moveHeightFraction(double heightFraction) {
        motor.setControl(new MotionMagicVoltage(MathUtil.clamp(heightFraction, 0, 1)));
        targetHeightFraction = MathUtil.clamp(heightFraction, 0.0, 1.0);
        motor.setControl(new MotionMagicVoltage(targetHeightFraction));
    }

    public double getHeightFraction() {
        return motor.getPosition().getValueAsDouble();
    }

    public Distance getHeight() {
        return Meters.of(
                getHeightFraction()
                                * (ElevatorConst.MAX_HEIGHT.in(Meters)
                                        - ElevatorConst.MIN_HEIGHT.in(Meters))
                        + ElevatorConst.MIN_HEIGHT.in(Meters));
        // 0.0 * 1.56 + 0.17 = 0.17
        // 1.0 * 1.73 +.17 = 1.9
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Height Fraction", () -> getHeightFraction(), null);
        builder.addDoubleProperty("Height in Meters", () -> getHeight().in(Meters), null);
        builder.addDoubleProperty(
                "Target Height Fraction",
                () -> targetHeightFraction,
                (heightFraction) -> moveHeightFraction(heightFraction));
    }
}
