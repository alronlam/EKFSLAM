package driver;

import desktop.imu.IMUReadingsBatch;
import dummies.features.FeatureUpdate;

public interface ControllerInterface {

	public void predict(IMUReadingsBatch imuBatch);

	public void update(FeatureUpdate featureUpdate);
}
