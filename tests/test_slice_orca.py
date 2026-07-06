from pathlib import Path

from s4.slice_orca import _reframe_gcode, _trim_to_toolpath, verify_planar_constraints


def test_reframe_gcode_shifts_only_xy(tmp_path):
    g = tmp_path / "p.gcode"
    g.write_text(
        "; comment X999 stays\n"
        "G1 X128.0 Y128.0 Z0.2 E0.5 F1200\n"
        "G0 X138 Y118\n"
        "M104 S200\n"
    )
    _reframe_gcode(str(g), 128.0, 128.0)
    out = g.read_text().splitlines()
    # X/Y shifted to the part frame, Z/E/F and non-move lines untouched.
    assert out[0] == "; comment X999 stays"
    assert out[1] == "G1 X0.00000 Y0.00000 Z0.2 E0.5 F1200"
    assert out[2] == "G0 X10.00000 Y-10.00000"
    assert out[3] == "M104 S200"


def test_trim_to_toolpath_keeps_only_model(tmp_path):
    g = tmp_path / "p.gcode"
    g.write_text(
        "; EXECUTABLE_BLOCK_START\n"
        "G1 X18 Y1 E15 ; nozzle purge (garbage)\n"
        "; CHANGE_LAYER\n"
        "G1 X1 Y1 E0.1 ; real model move\n"
        "; EXECUTABLE_BLOCK_END\n"
        "M104 S0 ; end gcode (dropped)\n"
    )
    _trim_to_toolpath(str(g))
    out = g.read_text().splitlines()
    assert out[0] == "; CHANGE_LAYER"
    assert "real model move" in out[1]
    assert not any("purge" in ln for ln in out)
    assert not any("end gcode" in ln for ln in out)


def test_trim_noop_without_markers(tmp_path):
    g = tmp_path / "p.gcode"
    original = "G1 X1 Y1 E0.1\nG1 X2 Y2 E0.1\n"
    g.write_text(original)
    _trim_to_toolpath(str(g))
    assert g.read_text() == original  # non-Orca gcode left untouched


def test_verify_planar_flags_zhop(tmp_path):
    g = tmp_path / "p.gcode"
    g.write_text("M83\nG1 X1 Y1 E0.1\nG1 Z5\n")  # Z with no X/Y, no extrusion
    warns = verify_planar_constraints(str(g))
    assert any("Z-hop" in w for w in warns)
