import threading
import time
from pose_estimator import PoseEstimator
from visualization_thread import PoseVisualizer
from controller import Controller
from sensor_utils import sensor_data_stream

USE_LIVE_SENSOR = True
# USE_LIVE_SENSOR = False

if __name__ == "__main__":
    # Start Pose Estimation
    if USE_LIVE_SENSOR:
        pose_estimator = PoseEstimator(sensor_data_stream())
    else:
        pose_estimator = PoseEstimator("sensor_recording/glass02.csv")
    pose_estimator.start()

    # Start Controller (prints status)
    controller = Controller(pose_estimator)
    controller.start()

    # Start Visualization Thread
    visualizer = PoseVisualizer(pose_estimator, controller)
    visualizer.start()

    while (True):
        # print(f"Pose Frequency: {pose_estimator.freq_estimator.smoothed_freq :.2f} Hz, Controller Frequency: {controller.freq_estimator.smoothed_freq :.2f} Hz")
        # print(f"Pose Estimator State: {pose_estimator.get_phase()}")
        time.sleep(0.05)
    # Wait until everything is done
    controller.join()
    visualizer.join()

