import numpy as np
import pytest

from s4.transform import (
    barycentric_coords,
    cartesian_to_rtheta,
    format_gcode_line,
    kabsch_2d,
    klipper_feed,
    tetrahedron_volume,
)


def test_barycentric_coords_centroid():
    tet = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ])
    centroid = np.array([0.25, 0.25, 0.25])
    b = barycentric_coords(centroid, tet)
    np.testing.assert_allclose(b, [0.25, 0.25, 0.25, 0.25], atol=1e-10)
    assert np.isclose(np.sum(b), 1.0)


def test_barycentric_coords_vertex():
    tet = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ])
    # vertex 0
    b0 = barycentric_coords(tet[0], tet)
    np.testing.assert_allclose(b0, [1.0, 0.0, 0.0, 0.0], atol=1e-10)

    # vertex 1
    b1 = barycentric_coords(tet[1], tet)
    np.testing.assert_allclose(b1, [0.0, 1.0, 0.0, 0.0], atol=1e-10)


def test_kabsch_2d_recovers_rotation():
    np.random.seed(42)
    orig = np.random.randn(4, 2)

    angle = np.deg2rad(30)
    R_mat = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle), np.cos(angle)],
    ])
    deformed = orig @ R_mat.T

    # S4's kabsch_2d uses a deliberate sign convention (-arccos + conditional
    # negate, matching the notebook), so compare magnitudes.
    recovered = kabsch_2d(orig, deformed)
    assert np.isclose(abs(recovered), abs(angle), atol=1e-6)


def test_cartesian_to_rtheta_basic():
    raw, theta_accum, r, z = cartesian_to_rtheta(1.0, 0.0, 5.0, 0.0, 0.0, 0.0, 42.0)
    assert theta_accum == 0.0
    assert r == 1.0
    assert z == 5.0

    raw2, theta_accum2, r2, z2 = cartesian_to_rtheta(0.0, 1.0, 5.0, 0.0, 0.0, 0.0, 42.0)
    assert np.isclose(theta_accum2, np.pi / 2)
    assert r2 == 1.0
    assert z2 == 5.0


def test_cartesian_to_rtheta_nozzle_offset():
    tilt = np.deg2rad(45)
    nozzle = 42.0
    raw, theta_accum, r, z = cartesian_to_rtheta(1.0, 0.0, 0.0, tilt, 0.0, 0.0, nozzle)

    expected_r = 1.0 - np.sin(tilt) * nozzle
    expected_z = 0.0 + (np.cos(tilt) - 1) * nozzle
    assert np.isclose(r, expected_r, atol=1e-6)
    assert np.isclose(z, expected_z, atol=1e-6)


def test_cartesian_to_rtheta_z_hop():
    tilt = 0.0
    nozzle = 42.0
    raw, theta_accum, r, z = cartesian_to_rtheta(1.0, 0.0, 0.0, tilt, 0.0, 0.0, nozzle, z_hop=1.0)

    expected_r = 1.0 - np.sin(tilt) * (nozzle + 1.0)
    expected_z = 0.0 + (np.cos(tilt) - 1) * (nozzle + 1.0) + 1.0
    assert np.isclose(r, expected_r, atol=1e-6)
    assert np.isclose(z, expected_z, atol=1e-6)


def test_cartesian_to_rtheta_multi_revolution():
    # Walk a point around the circle for 5 full turns; theta_accum must track the
    # true angle monotonically. Regression guard for the single-wrap bug that
    # drifted after ~1.5 revolutions.
    prev_raw = 0.0
    theta_accum = 0.0
    true_angle = 0.0
    step = np.deg2rad(30)
    for _ in range(60):  # 60 * 30deg = 1800deg = 5 revolutions
        true_angle += step
        x = np.cos(true_angle)
        y = np.sin(true_angle)
        prev_raw, theta_accum, r, z = cartesian_to_rtheta(
            x, y, 0.0, 0.0, prev_raw, theta_accum, 0.0
        )
    assert np.isclose(theta_accum, true_angle, atol=1e-6)


def test_format_gcode_line_rtheta():
    import re

    s = format_gcode_line(45.0, 10.0, 5.0, 15.0, 0.5, 300.0)
    assert re.match(r"^G01 C.* X.* Z.* B.* E.* F.*", s)


def test_format_gcode_line_cartesian_b():
    import re

    s = format_gcode_line(45.0, 10.0, 5.0, 15.0, None, None, kinematics="cartesian_b")
    assert re.match(r"G01 X.* Y.* Z.* B.*", s)


def test_format_gcode_line_no_extrusion_no_feed():
    s = format_gcode_line(0.0, 0.0, 0.0, 0.0, None, None, kinematics="rtheta")
    assert "E" not in s
    assert "F" not in s


def test_format_gcode_line_klipper_uses_rtheta_axes():
    # "klipper" mode keeps logical R-theta axis letters (D1 = Option B).
    s = format_gcode_line(45.0, 10.0, 5.0, 15.0, 0.5, 1234.0, kinematics="klipper")
    assert s.startswith("G01 C")
    assert " X" in s and " Z" in s and " B" in s
    assert " Y" not in s  # not cartesian


def test_klipper_feed_preserves_planar_time():
    # F = length * inv_time_feed, and Klipper time = length / F = 1/inv_time_feed
    # = planar_time, independent of how length splits across axes. Weight 1.0.
    inv_feed = 1.0 / 0.002  # planar_time = 0.002 min for this segment
    for dx, dz, dc, db in [(0.6, 0, 0, 0), (0, 0, 30, 0), (0.3, 0.1, 12, 4)]:
        f = klipper_feed(dx, dz, dc, db, inv_feed, 1.0, 1.0, 20000.0)
        length = np.sqrt(dx**2 + dz**2 + dc**2 + db**2)
        assert np.isclose(length / f, 0.002, atol=1e-9)


def test_klipper_feed_weight_slows_rotary():
    # Weight < 1 shrinks the rotary contribution -> smaller F -> longer Klipper
    # time (length metric unchanged firmware-side) -> slower rotary move.
    inv_feed = 500.0
    f_full = klipper_feed(0, 0, 30, 0, inv_feed, 1.0, 1.0, 20000.0)
    f_slow = klipper_feed(0, 0, 30, 0, inv_feed, 0.5, 1.0, 20000.0)
    assert f_slow < f_full


def test_klipper_feed_zero_length_falls_back_to_travel():
    assert klipper_feed(0, 0, 0, 0, 1000.0, 1.0, 1.0, 20000.0) == 20000.0
    # undefined time (inv_feed None) also falls back
    assert klipper_feed(1, 0, 0, 0, None, 1.0, 1.0, 20000.0) == 20000.0
