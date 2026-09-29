"""HW290 serial protocol and device-clock reconstruction; no ROS dependency."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Sample:
    sequence: int
    micros: int
    accel: tuple
    gyro: tuple
    temperature: int
    accel_lsb_per_g: float = 16384.0
    gyro_lsb_per_dps: float = 131.0


def parse_sample(line: bytes) -> Sample:
    payload, checksum = line.strip().rsplit(b"*", 1)
    actual = 0
    for byte in payload:
        actual ^= byte
    if len(checksum) != 2 or actual != int(checksum, 16):
        raise ValueError("checksum mismatch")
    fields = payload.decode("ascii").split(",")
    if len(fields) != 10 or fields[0] not in ("IMU1", "IMU2"):
        raise ValueError("unsupported record")
    values = [int(x) for x in fields[1:]]
    if not all(0 <= v <= 0xffffffff for v in values[:2]):
        raise ValueError("invalid clock or sequence")
    if not all(-32768 <= v <= 32767 for v in values[2:]):
        raise ValueError("invalid raw measurement")
    return Sample(values[0], values[1], tuple(values[2:5]), tuple(values[5:8]), values[8],
                  8192.0 if fields[0] == "IMU2" else 16384.0,
                  65.5 if fields[0] == "IMU2" else 131.0)


class DeviceClock:
    """Preserve MCU intervals, correct drift slowly against serial receipt time.

    Receipt minus wire time bounds acquisition time. A startup minimum rejects
    USB scheduling delays. Phase corrections are bounded to 1% of each MCU
    interval. The remaining camera/IMU offset needs physical calibration.
    """
    def __init__(self, warmup=100):
        self.warmup = warmup
        self.count = 0
        self.last_us = self.sequence = None
        self.elapsed_ns = 0
        self.anchor_ns = self.stamp_ns = self.minimum_error = None
        self.dropped = 0

    def update(self, sample: Sample, receipt_ns: int):
        delta_us = 0
        if self.last_us is not None:
            delta_us = (sample.micros - self.last_us) & 0xffffffff
            delta_sequence = (sample.sequence - self.sequence) & 0xffffffff
            if delta_sequence == 0 or delta_sequence > 0x7fffffff:
                raise ValueError("duplicate/out-of-order sample or MCU reset; restart bridge")
            if delta_us == 0 or delta_us > 2_000_000:
                raise ValueError("MCU clock reset or sample gap >2 s; restart bridge")
            self.dropped += delta_sequence - 1
        self.last_us, self.sequence = sample.micros, sample.sequence
        self.elapsed_ns += delta_us * 1000
        candidate = receipt_ns - self.elapsed_ns
        self.count += 1
        if self.count <= self.warmup:
            self.anchor_ns = candidate if self.anchor_ns is None else min(self.anchor_ns, candidate)
            self.stamp_ns = self.anchor_ns + self.elapsed_ns
            return None
        predicted = self.stamp_ns + delta_us * 1000
        error = receipt_ns - predicted
        self.minimum_error = error if self.minimum_error is None else min(self.minimum_error, error)
        correction = max(-delta_us * 10, min(delta_us * 10, int(self.minimum_error * 0.02)))
        self.stamp_ns = predicted + correction
        if self.count % 100 == 0:
            self.minimum_error = None
        return self.stamp_ns
