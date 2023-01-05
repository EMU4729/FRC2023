package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PiBridge {
    private static Optional<PiBridge> inst = Optional.empty();
    
    /** Type: Number */
    public final NetworkTableEntry x;
    /** Type: Number */
    public final NetworkTableEntry y;
    /** Type: Number */
    public final NetworkTableEntry dist;

    private PiBridge() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("Vision");
        x = table.getEntry("x");
        y = table.getEntry("y");
        dist = table.getEntry("dist");
        inst.startClientTeam(4729);
    }

    public static PiBridge getInstance() {
        if (!inst.isPresent()) {
            inst = Optional.of(new PiBridge());
        }
        return inst.get();
    }
}
