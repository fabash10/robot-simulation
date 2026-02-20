package frc.robot.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final CANcoder canCoder = new CANcoder(ArmConst.ENCODER_ID);
    private final TalonFX motor = new TalonFX(ArmConst.MOTOR_ID);
    private Angle targetAngle = Rotations.of(0);
    public ArmSubsystem(){
        motor.getConfigurator().apply(ArmConfig.motorConfig);
        canCoder.getConfigurator().apply(ArmConfig.encoderConfig);

    }
    public void moveAngle(Angle angle){
        targetAngle = Rotations.of(MathUtil.clamp(angle.in(Rotations), 0, 1));
        motor.setControl(new MotionMagicVoltage(targetAngle));
    }
    public Angle getAngle(){
        return canCoder.getPosition().getValue();
    }
    @Override
    public void initSendable(SendableBuilder builder){
        
        builder.addDoubleProperty("Angle in degrees", ()-> getAngle().in(Degrees), null);
        builder.addDoubleProperty("Target angle", 
        ()-> targetAngle.in(Degrees), 
        (angle)-> moveAngle(Degrees.of(angle)));
        

    }

    }
