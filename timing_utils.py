import time


class FrequencyEstimator:
    def __init__(self, alpha=0.9):
        self.last_time = None
        self.smoothed_freq = 0.0
        self.alpha = alpha

    def update(self):
        now = time.time()
        if self.last_time is not None:
            dt = now - self.last_time
            if dt > 0:
                instantaneous_freq = 1.0 / dt
                self.smoothed_freq = (
                    self.alpha * instantaneous_freq
                    + (1 - self.alpha) * self.smoothed_freq
                )
        self.last_time = now

    def get(self):
        return self.smoothed_freq
