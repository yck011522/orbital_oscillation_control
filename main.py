import threading
import time
from pose_estimator import PoseEstimator
from visualization_thread import visualization_thread
from controller import Controller
from sensor_utils import sensor_data_stream

USE_LIVE_SENSOR = False

if __name__ == "__main__":
    # Start Pose Estimation
    if USE_LIVE_SENSOR:
        pose_est = PoseEstimator(sensor_data_stream())
    else:
        pose_est = PoseEstimator("sensor_recording/glass01.csv")
    pose_est.start()

    # Start Controller (prints status)
    controller = Controller(pose_est)
    controller.start()

    # Start Visualization Thread
    vis_thread = threading.Thread(target=visualization_thread, args=(pose_est,))
    vis_thread.start()

    # Wait until everything is done
    controller.join()
    vis_thread.join()
