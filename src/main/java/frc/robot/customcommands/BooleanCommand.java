package frc.robot.customcommands;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class BooleanCommand extends Command {
    BooleanSupplier supp;
    public BooleanCommand(BooleanSupplier supp) {
        super();
        this.supp = supp;
    }
    @Override
    public boolean isFinished() {
        return supp.getAsBoolean();
    }
}