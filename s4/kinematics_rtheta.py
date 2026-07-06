"""Core-R-Theta coupled-pair kinematics (Phase 0 deliverable).

Ground truth: Joshua Bird's RRF `to4axis.g` line
    M669 K0 C0:0:0:0:1 X-1:0:0:1:0 Z0:0:1:0:0 B0.22222222:0:0:0.222222222:0
decoded with column order [X, Y, Z, B, C] (Y unused in 4-axis mode).

Only the X/B pair is coupled (drivers 1+2, 100 steps/mm each); C and Z are
identity. This module is the reference implementation + numerical validation
for the Klipper `polar_bc` C kinematic (D1 = Option B: logical axes X=radial,
B=tilt live in Klipper; Stage C keeps emitting logical R-theta).

Units: X in mm, B in degrees (RRF treats B as linear, 100 "steps/mm" = 100
steps/deg). Motor outputs m1, m2 are in the same linear unit (× 100 st/mm).
"""

# Forward matrix (logical axis -> motor), coupled pair only.
# m1 = -X + B ; m2 = K*(X + B)
K = 2.0 / 9.0  # 0.22222..., = 1/4.5 tilt reduction

def forward(x_mm, b_deg):
    """Logical (radial X, tilt B) -> coupled motor positions (m1, m2)."""
    m1 = -x_mm + b_deg
    m2 = K * (x_mm + b_deg)
    return m1, m2

def inverse(m1, m2):
    """Coupled motor positions -> logical (radial X, tilt B).

    From m1 = -X + B and m2 = K(X+B):  X+B = m2/K,  B-X = m1.
    """
    sum_xb = m2 / K          # X + B
    diff_bx = m1             # B - X
    x = (sum_xb - diff_bx) / 2.0
    b = (sum_xb + diff_bx) / 2.0
    return x, b


def _selfcheck():
    # 1. Identity round-trip on assorted poses.
    for x, b in [(0, 0), (50, -30), (-37.5, 90), (115.5, -180), (12.3, 4.5)]:
        m1, m2 = forward(x, b)
        rx, rb = inverse(m1, m2)
        assert abs(rx - x) < 1e-6 and abs(rb - b) < 1e-6, (x, b, rx, rb)

    # 2. Coupling sanity: BOTH motors move on a pure-radial or pure-tilt move.
    m = forward(10, 0)
    assert abs(m[0] - (-10)) < 1e-9 and abs(m[1] - 2.22222) < 1e-4, m
    m = forward(0, 10)
    assert abs(m[0] - 10) < 1e-9 and abs(m[1] - 2.22222) < 1e-4, m

    # 3. Fixtures from his RRF homing moves (exact matrix outputs).
    #    homex.g: G1 H1 X300  -> (-300, +66.667)
    m1, m2 = forward(300, 0)
    assert abs(m1 + 300) < 1e-6 and abs(m2 - 66.6667) < 1e-3, (m1, m2)
    #    homeb.g: G1 H1 B1000 -> (+1000, +222.22)
    m1, m2 = forward(0, 1000)
    assert abs(m1 - 1000) < 1e-6 and abs(m2 - 222.222) < 1e-3, (m1, m2)

    print("kinematics_rtheta self-check: PASS")


if __name__ == "__main__":
    _selfcheck()
