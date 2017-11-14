# ==== this is the class of pd controller ====

class PD_controller:

    def __init__(sefl, kp = 0.0, kd = 0.0, start_time = 0):

        # set the init value of kp and kd
        self.kp_ = float(kp)
        self.kd_ = float(kd)

        # define the laster_error_ and set to 0.0
        self.laster_error_ = 0.0

        # store relevant data
        self.last_timestamp_ = 0.0
        self.set_point_ = 0.0
        self.start_time_ = start_time
        self.error_sum_ = 0.0

        # control effort history
        self.u_p = [0]
        self.u_d = [0]

    def setTarget(self, target):
        self.set_point_ = float(target)

    def setKP(self, kp):
        self.kp_ = float(kp)

    def setKD(self, kd):
        self.kd_ = float(kd)

    def update(self, measured_value, timestamp):
        delta_time = timestamp - self.last_timestamp_
        if delta_time == 0:
            # Delta time is zero
            return 0

        error = self.set_point_ - measured_value

        self.last_timestamp_ = timestamp
        self.error_sum_ += error * delta_time

        delta_error = error - self.last_error_

        self.last_error_ = error

        p = self.kp_ * error

        d = self.kd_ * (delta_error / delta_time)

        u = p + d

        self.u_p.append(p)
        self.u_d.append(d)

        return u
