import time
import numpy as np
import cv2
from pose_estimator import PoseEstimator


SAFETY_CIRCLE_RADIUS_MM = 200
SCALE = 2  # pixels per mm (800 px represents 200 mm)


def visualization_thread(
    pose_estimator: PoseEstimator, canvas_height=800, canvas_width=1600
):
    polar_size = (canvas_height, canvas_width // 2)
    timeplot_size = (canvas_height, canvas_width // 2)
    time_window = 10.0  # seconds

    while not pose_estimator.is_finished():
        with pose_estimator.lock:
            history = list(pose_estimator.history)
            shared_state = pose_estimator.state.copy()

        if len(history) < 2 or "cop_x" not in shared_state:
            time.sleep(0.01)
            continue

        t_now = history[-1]["timestamp"]
        t0 = t_now - time_window

        polar_img = np.ones((*polar_size, 3), dtype=np.uint8) * 255
        time_img = np.ones((*timeplot_size, 3), dtype=np.uint8) * 255

        ## === LEFT PANEL: Polar View ===
        center = (polar_size[1] // 2, polar_size[0] // 2)

        # Define colors
        OUTSIDE_SAFETY = (0, 0, 50)
        DARK_GREY = (50, 50, 50)
        LIGHT_GREY = (230, 230, 230)
        RING_MINOR = (180, 180, 180)
        RING_MAJOR = (120, 120, 120)
        TEXT_COLOR = (0, 0, 0)
        FONT = cv2.FONT_HERSHEY_SIMPLEX

        # Draw background (outside area dark grey)
        polar_img[:] = OUTSIDE_SAFETY

        # Draw filled safety circle (light grey)
        mask = np.zeros_like(polar_img, dtype=np.uint8)
        cv2.circle(
            mask, center, int(SAFETY_CIRCLE_RADIUS_MM * SCALE), (255, 255, 255), -1
        )

        overlay = np.full_like(polar_img, LIGHT_GREY)
        cv2.copyTo(overlay, mask[:, :, 0], polar_img)

        # Draw minor concentric rings (10 mm intervals)
        for i in range(1, 30):
            r_px = int(i * 10 * SCALE)
            cv2.circle(polar_img, center, r_px, RING_MINOR, 1)

        # Draw major concentric rings (50 mm intervals)
        for i in range(1, 10):
            radius_mm = i * 50
            r_px = int(radius_mm * SCALE)
            cv2.circle(polar_img, center, r_px, RING_MAJOR, 2)

            # === Draw label at top ===
            label = f"{radius_mm} mm"
            (text_w, text_h), _ = cv2.getTextSize(label, FONT, 0.5, 1)
            label_pos = (center[0] - text_w // 2, center[1] - r_px - 10)

            # Draw background rectangle for label
            rect_top_left = (label_pos[0] - 4, label_pos[1] - text_h - 4)
            rect_bottom_right = (label_pos[0] + text_w + 4, label_pos[1] + 4)
            cv2.rectangle(polar_img, rect_top_left, rect_bottom_right, LIGHT_GREY, -1)

            # Draw text
            cv2.putText(
                polar_img, label, label_pos, FONT, 0.5, TEXT_COLOR, 1, cv2.LINE_AA
            )

        # Draw safety circle again (outline on top)
        cv2.circle(
            polar_img, center, int(SAFETY_CIRCLE_RADIUS_MM * SCALE), (100, 100, 255), 3
        )

        # === Draw trail of past COP positions ===
        t_now = shared_state["timestamp"]
        trail = [s for s in history if t_now - s["timestamp"] <= 10.0]

        for s in trail:
            age = t_now - s["timestamp"]
            alpha = 1.0 - age / 10.0  # fade from 1.0 to 0.0
            alpha = max(0.0, min(1.0, alpha))

            color = tuple(
                [
                    int(DARK_GREY[i] * alpha + LIGHT_GREY[i] * (1 - alpha))
                    for i in range(3)
                ]
            )

            x_mm = s["cop_x"]
            y_mm = s["cop_y"]
            px = int(center[0] + x_mm * SCALE)
            py = int(center[1] - y_mm * SCALE)
            cv2.circle(polar_img, (px, py), 2, color, -1)

        # Draw current COP marker
        x_mm = shared_state.get("cop_x", 0.0)
        y_mm = shared_state.get("cop_y", 0.0)
        px = int(center[0] + x_mm * SCALE)
        py = int(center[1] - y_mm * SCALE)

        cv2.circle(polar_img, (px, py), 7, DARK_GREY, -1)  # outer dark grey ring
        cv2.circle(polar_img, (px, py), 4, (255, 255, 255), -1)  # inner white dot

        ## === RIGHT PANEL: Time History Plot ===

        # --- Right panel plot configuration ---
        TIME_WINDOW = 10.0  # seconds

        # Y-axis ranges (adjust these as needed)
        POS_RANGE = (-180, 180)  # degrees
        VEL_RANGE = (-150, 150)  # deg/sec
        ACC_RANGE = (-300, 300)  # deg/sec²

        # Discontinuity thresholds
        POS_JUMP_THRESHOLD = 50  # degrees
        VEL_JUMP_THRESHOLD = 80  # deg/sec
        ACC_JUMP_THRESHOLD = 200  # deg/sec²
        PHASE_JUMP_THRESHOLD = 0.6  # large enough to skip across reset at reversal

        # === RIGHT PANEL: Time History ===

        # --- Right panel layout setup ---

        PLOT_FIELDS = [
            {
                "field": "angle",
                "label": "Angle (degrees)",
                "color": (130, 0, 0),
                "bg_color": (245, 230, 230),
                "vmin": POS_RANGE[0],
                "vmax": POS_RANGE[1],
                "threshold": POS_JUMP_THRESHOLD,
            },
            {
                "field": "velocity",
                "label": "Velocity (degrees/s)",
                "color": (0, 130, 0),
                "bg_color": (230, 245, 230),
                "vmin": VEL_RANGE[0],
                "vmax": VEL_RANGE[1],
                "threshold": VEL_JUMP_THRESHOLD,
            },
            {
                "field": "acceleration",
                "label": "Acceleration (degrees/s^2)",
                "color": (0, 0, 130),
                "bg_color": (230, 230, 245),
                "vmin": ACC_RANGE[0],
                "vmax": ACC_RANGE[1],
                "threshold": ACC_JUMP_THRESHOLD,
            },
            {
                "field": "phase",
                "label": "Phase",
                "vmin": 0.0,
                "vmax": 1.0,
                "bg_color": (240, 240, 240),
                "color": (0, 0, 255),
                "threshold": PHASE_JUMP_THRESHOLD,
            },
        ]
        # --- Right panel plot setup ---
        plot_margin = 0
        plot_w = timeplot_size[1] - 2 * plot_margin
        plot_h = timeplot_size[0] - 2 * plot_margin
        stripe_h = plot_h // len(PLOT_FIELDS)

        def t_to_x(t):
            return int((t - t0) / TIME_WINDOW * plot_w) + plot_margin

        def normalize(val, vmin, vmax):
            return (val - vmin) / (vmax - vmin + 1e-6)

        def draw_trace(history, field, y_offset, color, vmin, vmax, threshold):
            pts = [
                (s["timestamp"], s[field])
                for s in history
                if t0 <= s["timestamp"] <= t_now
            ]
            if len(pts) < 2:
                return
            for i in range(1, len(pts)):
                t0_, v0 = pts[i - 1]
                t1_, v1 = pts[i]
                if abs(v1 - v0) > threshold:
                    continue  # skip discontinuities
                x0 = t_to_x(t0_)
                x1 = t_to_x(t1_)
                y0 = int(stripe_h * (1 - normalize(v0, vmin, vmax))) + y_offset
                y1 = int(stripe_h * (1 - normalize(v1, vmin, vmax))) + y_offset
                cv2.line(time_img, (x0, y0), (x1, y1), color, 2)

        # === PASS 1: draw all backgrounds and labels first ===
        for i, cfg in enumerate(PLOT_FIELDS):
            y_offset = plot_margin + i * stripe_h
            top = y_offset
            bottom = y_offset + stripe_h

            # Fill lane background
            cv2.rectangle(
                time_img,
                (plot_margin, top),
                (plot_margin + plot_w, bottom),
                cfg["bg_color"],
                -1,
            )

            # Center axis (0 line)
            if cfg["vmin"] < 0 < cfg["vmax"]:
                y_center = (
                    int(stripe_h * (1 - normalize(0, cfg["vmin"], cfg["vmax"])))
                    + y_offset
                )
                cv2.line(
                    time_img,
                    (plot_margin, y_center),
                    (plot_margin + plot_w, y_center),
                    (0, 0, 0),
                    2,
                )

            # Label
            label = cfg["label"]
            (text_w, text_h), _ = cv2.getTextSize(label, FONT, 0.6, 1)
            label_pos = (5, y_offset + text_h + 5)
            cv2.rectangle(
                time_img,
                (label_pos[0] - 4, label_pos[1] - text_h - 4),
                (label_pos[0] + text_w + 4, label_pos[1] + 4),
                cfg["bg_color"],
                -1,
            )
            cv2.putText(
                time_img, label, label_pos, FONT, 0.6, TEXT_COLOR, 1, cv2.LINE_AA
            )

            # Draw vmin and vmax labels on right side
            vmin_text = f"{int(round(cfg['vmin']))}"
            vmax_text = f"{int(round(cfg['vmax']))}"

            # Calculate text size for both
            (vmin_w, vmin_h), _ = cv2.getTextSize(vmin_text, FONT, 0.5, 1)
            (vmax_w, vmax_h), _ = cv2.getTextSize(vmax_text, FONT, 0.5, 1)

            # Text positions
            x_text = plot_margin + plot_w - 30  # right of the plot
            y_vmax = top + vmax_h + 6
            y_vmin = bottom - 6

            # Draw text background rectangles
            for txt, x, y, w, h in [
                (vmax_text, x_text - vmax_w, y_vmax, vmax_w, vmax_h),
                (vmin_text, x_text - vmin_w, y_vmin, vmin_w, vmin_h),
            ]:
                cv2.rectangle(
                    time_img,
                    (x - 2, y - h - 2),
                    (x + w + 2, y + 2),
                    cfg["bg_color"],
                    -1,
                )
                cv2.putText(
                    time_img, txt, (x, y), FONT, 0.5, TEXT_COLOR, 1, cv2.LINE_AA
                )

        # === Draw vertical 1-second grid lines with labels ===
        start_sec = int(t0)
        end_sec = int(t_now)

        for sec in range(start_sec, end_sec + 1):
            x = t_to_x(sec)
            if plot_margin <= x <= plot_margin + plot_w:
                # Vertical grid line
                cv2.line(
                    time_img,
                    (x, plot_margin),
                    (x, plot_margin + plot_h),
                    (50, 50, 50),
                    1,
                )

                # Label: last 2 digits of seconds
                sec_label = f"{sec % 100:02d}"  # e.g., "47", "03"
                (tw, th), _ = cv2.getTextSize(sec_label, FONT, 0.4, 1)
                label_pos = (x - tw // 2, plot_margin + plot_h + th - 30)

                cv2.putText(
                    time_img, sec_label, label_pos, FONT, 0.4, (0, 0, 0), 1, cv2.LINE_AA
                )

        # Velocity Reversal markers
        for t_reversal, _ in pose_estimator.reversal_times:
            if t0 <= t_reversal <= t_now:
                x = t_to_x(t_reversal)
                cv2.line(
                    time_img, (x, plot_margin), (x, plot_margin + plot_h), (0, 0, 0), 2
                )

        # Draw reversal intervals in milliseconds
        for i in range(1, len(pose_estimator.reversal_times)):
            t1, _ = pose_estimator.reversal_times[i - 1]
            t2, _ = pose_estimator.reversal_times[i]

            if not (t0 <= t1 <= t_now and t0 <= t2 <= t_now):
                continue  # Skip if both not visible

            x1 = t_to_x(t1)
            x2 = t_to_x(t2)
            x_center = (x1 + x2) // 2

            interval_ms = int((t2 - t1) * 1000)
            text = f"{interval_ms:4d} ms"

            # Text box styling
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.6
            thickness = 2
            text_color = (50, 50, 70)
            text_size, _ = cv2.getTextSize(text, font, font_scale, thickness)
            text_w, text_h = text_size

            y_baseline = plot_margin + plot_h - 50  # Placement below the plot
            padding = 10

            outer_tl = (x_center - text_w // 2 - padding, y_baseline - text_h - padding)
            outer_br = (x_center + text_w // 2 + padding, y_baseline + padding)

            inner_tl = (outer_tl[0] + 2, outer_tl[1] + 2)
            inner_br = (outer_br[0] - 2, outer_br[1] - 2)

            # Draw outer red border
            cv2.rectangle(time_img, outer_tl, outer_br, text_color, 2)

            # Fill grey inner background
            cv2.rectangle(time_img, inner_tl, inner_br, (230, 230, 230), -1)

            # Draw text
            text_pos = (x_center - text_w // 2, y_baseline)
            cv2.putText(
                time_img, text, text_pos, font, font_scale, text_color, thickness
            )

        # === PASS 2: draw all data plots ===
        for i, cfg in enumerate(PLOT_FIELDS):
            y_offset = plot_margin + i * stripe_h
            draw_trace(
                history,
                cfg["field"],
                y_offset,
                cfg["color"],
                cfg["vmin"],
                cfg["vmax"],
                cfg["threshold"],
            )

        # Final baseline
        cv2.line(
            time_img,
            (plot_margin, plot_margin + plot_h),
            (plot_margin + plot_w, plot_margin + plot_h),
            (0, 0, 0),
            1,
        )

        # === Combine images for display ===

        combined = np.hstack([polar_img, time_img])
        cv2.imshow("Pose Visualization", combined)
        cv2.moveWindow("Pose Visualization", 50, 50)  # (x, y) screen coordinates

        if cv2.waitKey(1) == 27:
            break

        time.sleep(1 / 30.0)

    cv2.destroyAllWindows()
