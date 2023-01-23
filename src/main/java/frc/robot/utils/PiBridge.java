package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.networktables.NetworkTableInstance;

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
