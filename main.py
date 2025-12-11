import time
from pose_estimator import PoseEstimator
from visualization_thread import PoseVisualizer
from controller import Controller
from sensor_utils import sensor_data_stream

# USE_LIVE_SYSTEM = True
USE_LIVE_SYSTEM = False

if __name__ == "__main__":
    # Start Pose Estimation
    if USE_LIVE_SYSTEM:
        pose_estimator = PoseEstimator(sensor_data_stream())
    else:
        pose_estimator = PoseEstimator("sensor_recording/glass01.csv")
    pose_estimator.start()

    # Start Controller

    controller = Controller(pose_estimator, live_output=USE_LIVE_SYSTEM)
    controller.start()

    # Start Visualization Thread
    visualizer = PoseVisualizer(pose_estimator, controller)
    visualizer.start()

    # Wait until everything is done
    # controller.join()
    # visualizer.join()

    while True:
        # print(f"Pose Frequency: {pose_estimator.freq_estimator.smoothed_freq :.2f} Hz, Controller Frequency: {controller.freq_estimator.smoothed_freq :.2f} Hz")
        state = pose_estimator.get_latest_state()
        time.sleep(0.2)
        print(f"speed : {state.get('velocity',None)}, phase: {state.get('phase',None)}")
        # print(f"speed : {pose_estimator.state.get('velocity',None)}")
        # print(
        #     f"Visualization Thread Frequency: {visualizer.freq_estimator.smoothed_freq:.2f} Hz, "
        #     f"Controller Frequency: {controller.freq_estimator.smoothed_freq:.2f} Hz, "
        #     f"Pose Estimator Frequency: {pose_estimator.freq_estimator.smoothed_freq:.2f} Hz"
        # )
