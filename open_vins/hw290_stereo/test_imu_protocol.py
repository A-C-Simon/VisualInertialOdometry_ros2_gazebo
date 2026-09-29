import unittest
from imu_protocol import DeviceClock, Sample, parse_sample

class ProtocolTests(unittest.TestCase):
    def test_packet_version_preserves_physical_units(self):
        # Each packet represents 1 g and 2 degrees/s with its firmware range.
        for version, accel, gyro in [('IMU1', 16384, 262), ('IMU2', 8192, 131)]:
            data = f'{version},7,10000,0,0,{accel},{gyro},0,0,42'.encode()
            checksum = 0
            for byte in data: checksum ^= byte
            sample = parse_sample(data + f'*{checksum:02X}'.encode())
            self.assertEqual(sample.accel[2] / sample.accel_lsb_per_g, 1.0)
            self.assertEqual(sample.gyro[0] / sample.gyro_lsb_per_dps, 2.0)

    def test_checksum_and_ranges(self):
        data = b'IMU1,7,4294967290,0,-16384,32767,-32768,2,3,42'
        checksum = 0
        for x in data: checksum ^= x
        packet = data + f'*{checksum:02X}'.encode()
        sample = parse_sample(packet)
        self.assertEqual(sample.accel, (0, -16384, 32767))
        with self.assertRaises(ValueError): parse_sample(packet.replace(b',42', b',43'))

    def test_wrap_jitter_loss_and_reset(self):
        clock = DeviceClock(warmup=100)
        stamps = []
        for i in range(600):
            # Simulate micros wrap and USB batching/jitter, plus one lost packet.
            if i == 350: continue
            sample = Sample(i, (0xffffff00 + i*10000) & 0xffffffff, (0,0,16384), (0,0,0), 0)
            t = clock.update(sample, 10**12 + i*10000000 + (i%4)*2000000)
            if t is not None: stamps.append(t)
        delta = [b-a for a,b in zip(stamps, stamps[1:])]
        self.assertTrue(all(9_900_000 <= d <= 20_200_000 for d in delta))
        self.assertEqual(clock.dropped, 1)
        with self.assertRaises(ValueError): clock.update(sample, 10**13)
        with self.assertRaises(ValueError): clock.update(Sample(0,1000,(0,0,0),(0,0,0),0),10**13)

if __name__ == '__main__': unittest.main()
