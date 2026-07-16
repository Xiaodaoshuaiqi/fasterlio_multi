#!/usr/bin/env python3

import math

import numpy as np
import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def dms_to_degrees(degrees, minutes, seconds):
    sign = -1.0 if degrees < 0.0 else 1.0
    return degrees + sign * (
        abs(minutes) / 60.0 + abs(seconds) / 3600.0
    )


def add_enu_offset(latitude, longitude, altitude, east, north, up):
    latitude_rad = math.radians(latitude)
    sin_latitude = math.sin(latitude_rad)
    denominator = math.sqrt(
        1.0 - WGS84_E2 * sin_latitude * sin_latitude
    )
    prime_vertical = WGS84_A / denominator
    meridian = (
        WGS84_A
        * (1.0 - WGS84_E2)
        / (denominator * denominator * denominator)
    )
    noisy_latitude = latitude + math.degrees(
        north / (meridian + altitude)
    )
    noisy_longitude = longitude + math.degrees(
        east
        / (
            (prime_vertical + altitude)
            * max(abs(math.cos(latitude_rad)), 1e-9)
        )
    )
    return noisy_latitude, noisy_longitude, altitude + up


def load_samples(path, sigma_xy, sigma_z, seed):
    truth = []
    with open(path, encoding="ascii") as stream:
        for line in stream:
            fields = line.split()
            try:
                stamp = float(fields[0])
                latitude = dms_to_degrees(
                    float(fields[3]), float(fields[4]), float(fields[5])
                )
                longitude = dms_to_degrees(
                    float(fields[6]), float(fields[7]), float(fields[8])
                )
                altitude = float(fields[9])
            except (ValueError, IndexError):
                continue
            truth.append((stamp, latitude, longitude, altitude))
    if not truth:
        raise RuntimeError(f"no ground-truth samples in {path}")

    generator = np.random.default_rng(seed)
    noise = generator.normal(
        loc=0.0,
        scale=np.asarray([sigma_xy, sigma_xy, sigma_z]),
        size=(len(truth), 3),
    )
    samples = []
    for sample, enu_noise in zip(truth, noise):
        stamp, latitude, longitude, altitude = sample
        noisy_position = add_enu_offset(
            latitude,
            longitude,
            altitude,
            float(enu_noise[0]),
            float(enu_noise[1]),
            float(enu_noise[2]),
        )
        samples.append((stamp, *noisy_position))
    return samples


class SyntheticRtkReplay:
    def __init__(self):
        self.sigma_xy = float(rospy.get_param("~sigma_xy", 0.02))
        self.sigma_z = float(rospy.get_param("~sigma_z", 0.04))
        self.publish_delay = float(
            rospy.get_param("~publish_delay", 0.2)
        )
        self.maximum_publish_lag = float(
            rospy.get_param("~maximum_publish_lag", 2.0)
        )
        self.frame_id = rospy.get_param(
            "~frame_id", "synthetic_rtk_link"
        )
        self.samples = load_samples(
            rospy.get_param("~ground_truth_file"),
            self.sigma_xy,
            self.sigma_z,
            int(rospy.get_param("~seed", 20260716)),
        )
        self.publisher = rospy.Publisher(
            rospy.get_param("~fix_topic", "/gnss/fix"),
            NavSatFix,
            queue_size=20,
        )
        self.cursor = 0
        self.last_now = rospy.Time()
        self.timer = rospy.Timer(rospy.Duration(0.01), self.on_timer)
        rospy.loginfo(
            "Loaded %d synthetic RTK fixes: sigma_xy=%.3f m, "
            "sigma_z=%.3f m",
            len(self.samples),
            self.sigma_xy,
            self.sigma_z,
        )

    def reset(self, now):
        earliest = now.to_sec() - self.maximum_publish_lag
        self.cursor = int(
            np.searchsorted(
                [sample[0] for sample in self.samples],
                earliest,
                side="left",
            )
        )

    def on_timer(self, _event):
        now = rospy.Time.now()
        if now.is_zero():
            return
        if not self.last_now.is_zero() and now + rospy.Duration(
            0.1
        ) < self.last_now:
            self.reset(now)
        self.last_now = now

        publish_before = (
            now - rospy.Duration(self.publish_delay)
            + rospy.Duration(0.02)
        )
        while (
            self.cursor < len(self.samples)
            and self.samples[self.cursor][0] <= publish_before.to_sec()
        ):
            sample = self.samples[self.cursor]
            self.cursor += 1
            if now.to_sec() - sample[0] > self.maximum_publish_lag:
                continue
            self.publish(sample)

    def publish(self, sample):
        stamp, latitude, longitude, altitude = sample
        message = NavSatFix()
        message.header.stamp = rospy.Time.from_sec(stamp)
        message.header.frame_id = self.frame_id
        message.status.status = NavSatStatus.STATUS_GBAS_FIX
        message.status.service = (
            NavSatStatus.SERVICE_GPS
            | NavSatStatus.SERVICE_GLONASS
            | NavSatStatus.SERVICE_GALILEO
            | NavSatStatus.SERVICE_COMPASS
        )
        message.latitude = latitude
        message.longitude = longitude
        message.altitude = altitude
        message.position_covariance[0] = self.sigma_xy**2
        message.position_covariance[4] = self.sigma_xy**2
        message.position_covariance[8] = self.sigma_z**2
        message.position_covariance_type = (
            NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        )
        self.publisher.publish(message)


def main():
    rospy.init_node("synthetic_rtk_replay")
    SyntheticRtkReplay()
    rospy.spin()


if __name__ == "__main__":
    main()
