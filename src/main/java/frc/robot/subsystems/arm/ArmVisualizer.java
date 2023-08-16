package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/* Helper class for logging arm state */
public class ArmVisualizer {
    Mechanism2d mechanism;
    MechanismRoot2d mechanismRoot;
    MechanismLigament2d arm;

    public ArmVisualizer() {
        Color color = Color.kOrange;

        mechanism = new Mechanism2d(2, 1.5);
        mechanismRoot = mechanism.getRoot("Arm", 0.8, 0.75);
        arm = mechanismRoot.append(new MechanismLigament2d("Arm", 0.5334, 0, 6, new Color8Bit(color)));
    }

    public void update(double armAngle) {
        arm.setAngle(Units.rotationsToDegrees(armAngle));
        Logger.getInstance().recordOutput("ArmMechanism2d", mechanism);
    }
}
