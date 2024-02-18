package frc.robot.subsystems.indexer;

public class IndexerDynamics {
    public record IndexerState(double indexerVelocityMetersPerSecond) {
        public static IndexerState STARTING_STATE = new IndexerState(0);
        public static IndexerState TRAVEL_STATE = new IndexerState(0);

        @Override
        public boolean equals(Object obj) {
            if (obj instanceof IndexerState) {
                return 
                    Math.abs(
                        indexerVelocityMetersPerSecond
                         - ((IndexerState) obj).indexerVelocityMetersPerSecond)
                      <= 1e-5;
            }
            return false;
        }
    }
}
