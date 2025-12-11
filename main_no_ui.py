import threading
import time
from pose_estimator import PoseEstimator
from visualization_thread import PoseVisualizer
from controller import Controller
from sensor_utils import sensor_data_stream

USE_LIVE_SYSTEM = True
# USE_LIVE_SYSTEM = False

if __name__ == "__main__":
    # Start Pose Estimation
    if USE_LIVE_SYSTEM:
        pose_estimator = PoseEstimator(sensor_data_stream())
    else:
        pose_estimator = PoseEstimator("sensor_recording/glass02.csv")
    pose_estimator.start()

    # Start Controller

    controller = Controller(pose_estimator, live_output=USE_LIVE_SYSTEM)
    controller.start()

    # Start Visualization Thread
    # visualizer = PoseVisualizer(pose_estimator, controller)
    # visualizer.start()

    # Wait until everything is done
    # controller.join()
    # visualizer.join()

    while True:
        # print(f"Pose Frequency: {pose_estimator.freq_estimator.smoothed_freq :.2f} Hz, Controller Frequency: {controller.freq_estimator.smoothed_freq :.2f} Hz")
        # print(f"Pose Estimator State: {pose_estimator.get_phase()}")
        time.sleep(0.1)
        state = pose_estimator.get_latest_state()
        arc_center_x = state.get('arc_center_x',None)
        arc_center_y = state.get('arc_center_y',None)
        arc_center_deviation = None
        if arc_center_x is not None and arc_center_y is not None:
            arc_center_deviation = (arc_center_x**2 + arc_center_y**2)**0.5
        # Format numbers to 2 decimals
        print(f"Object State: {state.get('motion_state',None)}, control_state: {controller.get_state_text()}, speed : {state.get('velocity',None)}, arc_center_deviation: {arc_center_deviation}")
        # print(
        #     f"Visualization Thread Frequency: {visualizer.freq_estimator.smoothed_freq:.2f} Hz, "
        #     f"Controller Frequency: {controller.freq_estimator.smoothed_freq:.2f} Hz, "
        #     f"Pose Estimator Frequency: {pose_estimator.freq_estimator.smoothed_freq:.2f} Hz"
        # )
