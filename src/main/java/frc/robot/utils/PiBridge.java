package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * This handles most networking with the raspberry pi - not in use right now
 * though
 */
public class PiBridge {
    private static Optional<PiBridge> inst = Optional.empty();

    private PiBridge() {
        NetworkTableInstance ntinst = NetworkTableInstance.getDefault();

        ntinst.startClient4("fluffy");
        ntinst.setServerTeam(4729);
        ntinst.startDSClient();
    }

    public static PiBridge getInstance() {
        if (!inst.isPresent()) {
            inst = Optional.of(new PiBridge());
        }
        return inst.get();
    }
}
