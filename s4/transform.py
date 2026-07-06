# S4_Slicer Stage C — planar gcode -> 4-axis gcode inverse-transform
#
# Lifted from Joshua Bird's S4_Slicer notebook (main.ipynb), cells ~15-18.
# All original algorithm logic preserved; only structure refactored into
# explicit function params/returns and lazy imports for heavy deps.

from dataclasses import dataclass

import numpy as np


@dataclass
class TransformParams:
    SEG_SIZE: float = 0.6
    MAX_ROTATION: float = 30
    MIN_ROTATION: float = -130
    NOZZLE_OFFSET: float = 42
    ROTATION_AVERAGING_ALPHA: float = 0.2
    ROTATION_MAX_DELTA: float = 1  # degrees, converted to rad in code
    RETRACTION_LENGTH: float = 1.0
    MAX_EXTRUSION_MULTIPLIER: float = 10
    kinematics: str = "rtheta"


# --- pure-math helpers (numpy/scipy only, unit-testable) ---------------------


def tetrahedron_volume(p1, p2, p3, p4):
    mat = np.vstack([p2 - p1, p3 - p1, p4 - p1])
    return np.abs(np.linalg.det(mat)) / 6


def barycentric_coords(point, tet_vertices):
    total_volume = tetrahedron_volume(
        tet_vertices[0], tet_vertices[1], tet_vertices[2], tet_vertices[3]
    )
    if total_volume == 0:
        raise ValueError("The points do not form a valid tetrahedron (zero volume).")
    vol_a = tetrahedron_volume(point, tet_vertices[1], tet_vertices[2], tet_vertices[3])
    vol_b = tetrahedron_volume(point, tet_vertices[0], tet_vertices[2], tet_vertices[3])
    vol_c = tetrahedron_volume(point, tet_vertices[0], tet_vertices[1], tet_vertices[3])
    vol_d = tetrahedron_volume(point, tet_vertices[0], tet_vertices[1], tet_vertices[2])
    lambda_a = vol_a / total_volume
    lambda_b = vol_b / total_volume
    lambda_c = vol_c / total_volume
    lambda_d = vol_d / total_volume
    return np.array([lambda_a, lambda_b, lambda_c, lambda_d])


def kabsch_2d(orig_pts, deformed_pts):
    covariance = deformed_pts.T @ orig_pts
    U, _, Vt = np.linalg.svd(covariance)
    R_mat = U @ Vt
    rotation = -np.arccos(np.clip(R_mat[0, 0], -1, 1))
    if R_mat[1, 0] < 0:
        rotation = -rotation
    return rotation


def cartesian_to_rtheta(x, y, z, tilt_rad, prev_raw_theta, theta_accum, nozzle_offset, z_hop=0):
    r = np.sqrt(x**2 + y**2)
    raw_theta = np.arctan2(y, x)

    r += -np.sin(tilt_rad) * (nozzle_offset + z_hop)
    z_adj = z + (np.cos(tilt_rad) - 1) * (nozzle_offset + z_hop) + z_hop

    # Accumulate against the previous RAW angle so a single wrap is always
    # sufficient -> continuous C axis across many revolutions.
    delta_theta = raw_theta - prev_raw_theta
    if delta_theta > np.pi:
        delta_theta -= 2 * np.pi
    elif delta_theta < -np.pi:
        delta_theta += 2 * np.pi

    return raw_theta, theta_accum + delta_theta, r, z_adj


def format_gcode_line(theta_deg, r, z, tilt_deg, e, inv_time_feed, kinematics="rtheta"):
    if kinematics == "cartesian_b":
        x = r * np.cos(np.deg2rad(theta_deg))
        y = r * np.sin(np.deg2rad(theta_deg))
        s = f"G01 X{x:.5f} Y{y:.5f} Z{z:.5f} B{tilt_deg:.5f}"
    else:
        s = f"G01 C{theta_deg:.5f} X{r:.5f} Z{z:.5f} B{tilt_deg:.5f}"

    if e is not None:
        s += f" E{e:.4f}"
    if inv_time_feed is not None:
        s += f" F{inv_time_feed:.4f}"
    return s


# --- heavy driver functions (lazy imports) -----------------------------------


def prepare_transform_fields(original_tet, deformed_tet, params=None):
    import pyvista as pv

    if params is None:
        params = TransformParams()

    vertex_transformations = deformed_tet.points - original_tet.points

    tangential_vectors = np.cross(
        np.array([0, 0, 1]), original_tet.cell_data["cell_center"][:, :2]
    )
    tangential_vectors /= np.linalg.norm(tangential_vectors, axis=1)[:, None]
    tangential_vectors[np.isnan(tangential_vectors).any(axis=1)] = [1, 0, 0]

    num_cells_per_vertex = np.zeros(original_tet.number_of_points)
    for cell in original_tet.field_data["cells"]:
        num_cells_per_vertex[cell] += 1

    vertex_rotations = np.zeros(deformed_tet.number_of_points)
    cell_rotations = np.zeros(deformed_tet.number_of_cells)

    for cell_index, cell in enumerate(deformed_tet.field_data["cells"]):
        new_vertices = deformed_tet.field_data["cell_vertices"][cell]
        new_cell_center = deformed_tet.cell_data["cell_center"][cell_index]
        old_vertices = original_tet.field_data["cell_vertices"][cell]
        old_cell_center = original_tet.cell_data["cell_center"][cell_index]

        new_vertices_centered = new_vertices - new_cell_center
        old_vertices_centered = old_vertices - old_cell_center

        # Guard the rotary axis (x=y=0): a zero radial vector would give NaN.
        radial_norm = np.linalg.norm(old_cell_center[:2])
        if radial_norm < 1e-9:
            plane_x_vector = np.array([1.0, 0.0, 0.0])
        else:
            plane_x_vector = np.array(
                [old_cell_center[0], old_cell_center[1], 0.0]
            ) / radial_norm
        plane_y_vector = np.array([0, 0, 1])

        def _project(pts, px, py):
            return np.array([np.sum(px * pts, axis=1), np.sum(py * pts, axis=1)]).T

        new_proj = _project(new_vertices_centered, plane_x_vector, plane_y_vector)
        old_proj = _project(old_vertices_centered, plane_x_vector, plane_y_vector)

        rotation = kabsch_2d(old_proj, new_proj)
        rotation = np.clip(
            rotation,
            np.deg2rad(params.MIN_ROTATION),
            np.deg2rad(params.MAX_ROTATION),
        )
        cell_rotations[cell_index] = rotation
        for vertex_index in cell:
            vertex_rotations[vertex_index] += rotation / num_cells_per_vertex[vertex_index]

    from s4.deform import _calculate_rotation_matrices

    tet_rotation_matrices = _calculate_rotation_matrices(original_tet, cell_rotations)

    z_squish_scales = np.full(deformed_tet.number_of_cells, np.nan)
    for cell_index, cell in enumerate(deformed_tet.field_data["cells"]):
        warped_vertices = deformed_tet.field_data["cell_vertices"][cell]
        unwarped_vertices = original_tet.field_data["cell_vertices"][cell]
        z_squish_scales[cell_index] = tetrahedron_volume(*unwarped_vertices) / tetrahedron_volume(
            *warped_vertices
        )

    return {
        "vertex_transformations": vertex_transformations,
        "vertex_rotations": vertex_rotations,
        "cell_rotations": cell_rotations,
        "z_squish_scales": z_squish_scales,
    }


def transform_gcode(planar_gcode_path, deformed_tet, fields, params=None):
    from pygcode import Line

    import pyvista as pv

    if params is None:
        params = TransformParams()

    vertex_transformations = fields["vertex_transformations"]
    vertex_rotations = fields["vertex_rotations"]
    z_squish_scales = fields["z_squish_scales"]

    pos = np.array([0.0, 0.0, 20.0])
    feed = 5000
    gcode_points = []

    with open(planar_gcode_path, "r") as fh:
        for line_text in fh.readlines():
            try:
                line = Line(line_text)
            except Exception:
                # Slicer-proprietary lines pygcode can't parse (e.g. Bambu's
                # "M1006 W" buzzer macro). Never toolpath moves — skip them.
                continue
            if not line.block.gcodes:
                continue
            for gcode in sorted(line.block.gcodes):
                if gcode.word not in ("G01", "G00"):
                    continue
                prev_pos = pos.copy()

                if gcode.X is not None:
                    pos[0] = gcode.X
                if gcode.Y is not None:
                    pos[1] = gcode.Y
                if gcode.Z is not None:
                    pos[2] = gcode.Z

                for word in line.block.words:
                    if word.letter == "F":
                        feed = word.value

                extrusion = None
                for param in line.block.modal_params:
                    if param.letter == "E":
                        extrusion = param.value

                delta_pos = pos - prev_pos
                distance = np.linalg.norm(delta_pos)
                seg_size = params.SEG_SIZE

                if distance > 0:
                    num_segments = int(np.ceil(distance / seg_size))
                    seg_distance = distance / num_segments

                    time_to_complete = (1 / feed) * seg_distance if feed else 0
                    inv_time_feed = 1 / time_to_complete if time_to_complete != 0 else None

                    for i in range(num_segments):
                        gcode_points.append({
                            "position": prev_pos + delta_pos * (i + 1) / num_segments,
                            "command": str(gcode.word),
                            "extrusion": extrusion / num_segments if extrusion is not None else None,
                            "inv_time_feed": inv_time_feed,
                            "move_length": seg_distance,
                            "feed": feed,
                        })
                else:
                    time_to_complete = (1 / feed) * distance if feed else 0
                    inv_time_feed = 1 / time_to_complete if time_to_complete != 0 else None
                    gcode_points.append({
                        "position": pos.copy(),
                        "command": str(gcode.word),
                        "extrusion": extrusion,
                        "inv_time_feed": inv_time_feed,
                        "move_length": distance,
                        "feed": feed,
                    })

    gcode_points_containing_cells = deformed_tet.find_containing_cell(
        [pt["position"] for pt in gcode_points]
    )
    gcode_points_closest_cells = deformed_tet.find_closest_cell(
        [pt["position"] for pt in gcode_points]
    )

    new_gcode_points = []
    prev_new_position = None
    travelling_over_air = False
    travelling = False
    prev_rotation = 0.0
    prev_travelling = False
    prev_command = "G00"
    alpha = params.ROTATION_AVERAGING_ALPHA
    max_rotation_delta_rad = np.deg2rad(params.ROTATION_MAX_DELTA)
    highest_printed_point = 0.0

    def _barycentric_interp(position, containing_cell_index, command, pt_idx):
        if command == "G00" and containing_cell_index == -1:
            return None, None
        if command == "G01" and containing_cell_index == -1:
            containing_cell_index = gcode_points_closest_cells[pt_idx]

        vert_indices = deformed_tet.field_data["cells"][containing_cell_index]
        cell_vertices = deformed_tet.field_data["cell_vertices"][vert_indices]
        bcoords = barycentric_coords(position, cell_vertices)
        if np.sum(bcoords) > 1.01:
            return None, None

        transformation = vertex_transformations[vert_indices] * bcoords[:, None]
        transformation = np.sum(transformation, axis=0)
        new_position = position - transformation
        rotation = np.sum(vertex_rotations[vert_indices] * bcoords)
        return new_position, rotation

    for pt_idx, (gcode_point, containing_cell_index) in enumerate(
        zip(gcode_points, gcode_points_containing_cells)
    ):
        position = gcode_point["position"]
        command = gcode_point["command"]
        inv_time_feed = gcode_point["inv_time_feed"]
        extrusion = gcode_point["extrusion"]

        dont_smooth_rotation = False
        new_position, rotation = _barycentric_interp(
            position, containing_cell_index, command, pt_idx
        )

        if new_position is None:
            if command == "G01":
                continue
            elif command == "G00" and not travelling_over_air and prev_new_position is not None:
                new_position = np.array(
                    [prev_new_position[0], prev_new_position[1], highest_printed_point]
                )
                rotation = np.clip(prev_rotation, np.deg2rad(-45), np.deg2rad(45))
                dont_smooth_rotation = True
                travelling_over_air = True
            elif travelling_over_air:
                continue
            else:
                continue
        else:
            if travelling_over_air:
                new_position[2] = highest_printed_point
                rotation = np.clip(rotation, np.deg2rad(-45), np.deg2rad(45))
                dont_smooth_rotation = True
            travelling_over_air = False

        extrusion_multiplier = 1.0
        if (
            extrusion is not None
            and extrusion != params.RETRACTION_LENGTH
            and extrusion != -params.RETRACTION_LENGTH
        ):
            if containing_cell_index != -1:
                extrusion_multiplier *= z_squish_scales[containing_cell_index]
            extrusion *= min(extrusion_multiplier, params.MAX_EXTRUSION_MULTIPLIER)
        elif extrusion == -params.RETRACTION_LENGTH:
            travelling = True
        elif extrusion == params.RETRACTION_LENGTH:
            travelling = False

        if prev_rotation is not None and not dont_smooth_rotation:
            rotation = alpha * rotation + (1 - alpha) * prev_rotation

        if (
            prev_rotation is not None
            and prev_new_position is not None
            and np.abs(rotation - prev_rotation) > max_rotation_delta_rad
        ):
            delta_rotation = rotation - prev_rotation
            num_interpolations = int(np.abs(delta_rotation) / max_rotation_delta_rad) + 1
            delta_pos_interp = new_position - prev_new_position
            for i in range(num_interpolations):
                new_gcode_points.append({
                    "position": prev_new_position
                    + delta_pos_interp * ((i + 1) / num_interpolations),
                    "rotation": prev_rotation
                    + delta_rotation * ((i + 1) / num_interpolations),
                    "command": prev_command,
                    "extrusion": extrusion / num_interpolations
                    if extrusion is not None
                    else None,
                    "inv_time_feed": inv_time_feed * num_interpolations
                    if inv_time_feed is not None
                    else None,
                    "extrusion_multiplier": extrusion_multiplier,
                    "feed": gcode_point.get("feed", 5000),
                    "travelling": prev_travelling,
                })
        else:
            new_gcode_points.append({
                "position": new_position,
                "rotation": rotation,
                "command": command,
                "extrusion": extrusion,
                "inv_time_feed": inv_time_feed,
                "extrusion_multiplier": extrusion_multiplier,
                "feed": gcode_point.get("feed", 5000),
                "travelling": travelling,
            })

        prev_rotation = rotation
        prev_new_position = new_position.copy()
        prev_travelling = travelling
        prev_command = command

        if (
            command == "G01"
            and extrusion is not None
            and extrusion > 0
            and (highest_printed_point != 0 or new_position[2] < 1)
        ):
            highest_printed_point = max(highest_printed_point, new_position[2])

    lines = []
    prev_r = 0.0
    prev_raw_theta = 0.0
    theta_accum = 0.0
    prev_z = 20.0
    noz = params.NOZZLE_OFFSET

    lines.append("G94 ; mm/min feed")
    lines.append("G28 ; home")
    lines.append("M83 ; relative extrusion")
    lines.append("G1 E10 ; prime extruder")
    lines.append("G94 ; mm/min feed")
    lines.append("G90 ; absolute positioning")
    lines.append(f"G0 C{theta_accum} X{prev_r} Z{prev_z} B0 ; go to start")
    lines.append("G93 ; inverse time feed")

    for point in new_gcode_points:
        position = point["position"]
        rotation = point["rotation"]

        if np.all(np.isnan(position)):
            continue
        if position[2] < 0:
            continue

        z_hop = 1.0 if point.get("travelling", False) else 0.0

        prev_raw_theta, theta_accum, r_val, z_val = cartesian_to_rtheta(
            position[0], position[1], position[2], rotation,
            prev_raw_theta, theta_accum, noz, z_hop
        )

        tilt_deg = np.rad2deg(rotation)
        theta_deg = np.rad2deg(theta_accum)
        e_val = point.get("extrusion")
        inv_feed = point.get("inv_time_feed")

        s = format_gcode_line(
            theta_deg, r_val, z_val, tilt_deg, e_val, inv_feed, params.kinematics
        )
        s = s.replace("G01", point["command"])

        no_feed_value = inv_feed is None
        if no_feed_value:
            lines.append("G94")
            if inv_feed is None:
                s += " F20000"
            lines.append(s)
            lines.append("G93")
        else:
            lines.append(s)

        prev_r = r_val
        prev_z = z_val

    return lines


def write_gcode(lines, out_path):
    with open(out_path, "w") as fh:
        fh.write("\n".join(lines) + "\n")
