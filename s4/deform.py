# S4_Slicer Stage A — mesh deformation
#
# Lifted from Joshua Bird's S4_Slicer notebook (main.ipynb), cells ~2-11.
# All original algorithm logic preserved; only structure refactored into
# explicit function params/returns and lazy imports for heavy deps.

import base64
import pickle
from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation as R

_up_vector = np.array([0, 0, 1])


@dataclass
class DeformParams:
    MAX_OVERHANG: float = 30
    ROTATION_MULTIPLIER: float = 2
    NEIGHBOUR_LOSS_WEIGHT: float = 20
    MAX_POS_ROTATION: float = 3600  # in degrees, converted to rad in code
    MAX_NEG_ROTATION: float = -3600
    STEEP_OVERHANG_COMPENSATION: bool = True
    SET_INITIAL_ROTATION_TO_ZERO: bool = False
    INITIAL_ROTATION_FIELD_SMOOTHING: int = 30
    rotation_iterations: int = 100
    deformation_iterations: int = 1000
    part_offset: tuple = (0, 0, 0)
    save_gif: bool = False


# --- internal helpers --------------------------------------------------------

def _encode_object(obj):
    return base64.b64encode(pickle.dumps(obj)).decode("utf-8")


def _decode_object(encoded_str):
    return pickle.loads(base64.b64decode(encoded_str))


def _plane_fit(points):
    points = np.reshape(points, (np.shape(points)[0], -1))
    assert points.shape[0] <= points.shape[1]
    ctr = points.mean(axis=1)
    x = points - ctr[:, np.newaxis]
    M = np.dot(x, x.T)
    return ctr, np.linalg.svd(M)[0][:, -1]


def _build_neighbour_dict(tet_grid):
    import pyvista as pv

    neighbour_types = ["point", "edge", "face"]
    cell_neighbour_dict = {
        nt: {face: [] for face in range(tet_grid.number_of_cells)}
        for nt in neighbour_types
    }
    for neighbour_type in neighbour_types:
        cell_neighbours = []
        for cell_index in range(tet_grid.number_of_cells):
            neighbours = tet_grid.cell_neighbors(cell_index, f"{neighbour_type}s")
            for neighbour in neighbours:
                if neighbour > cell_index:
                    cell_neighbours.append((cell_index, neighbour))
        for face_1, face_2 in np.array(cell_neighbours):
            cell_neighbour_dict[neighbour_type][face_1].append(face_2)
            cell_neighbour_dict[neighbour_type][face_2].append(face_1)
        tet_grid.field_data[f"cell_{neighbour_type}_neighbours"] = np.array(cell_neighbours)
    return cell_neighbour_dict


def _update_tet_attributes(tet_grid, cell_neighbour_graph):
    surface_mesh = tet_grid.extract_surface()
    cell_to_face = _decode_object(tet_grid.field_data["cell_to_face"])

    cells_arr = tet_grid.cells.reshape(-1, 5)[:, 1:]
    tet_grid.add_field_data(cells_arr, "cells")
    cell_vertices = tet_grid.points
    tet_grid.add_field_data(cell_vertices, "cell_vertices")
    faces_arr = surface_mesh.faces.reshape(-1, 4)[:, 1:]
    tet_grid.add_field_data(faces_arr, "faces")
    face_vertices = surface_mesh.points
    tet_grid.add_field_data(face_vertices, "face_vertices")

    tet_grid.cell_data["face_normal"] = np.full((tet_grid.number_of_cells, 3), np.nan)
    surface_mesh_face_normals = surface_mesh.face_normals
    for cell_index, face_indices in cell_to_face.items():
        face_normals = surface_mesh_face_normals[face_indices]
        most_down_normal_index = np.argmin(face_normals[:, 2])
        tet_grid.cell_data["face_normal"][cell_index] = face_normals[most_down_normal_index]
    tet_grid.cell_data["face_normal"] = (
        tet_grid.cell_data["face_normal"]
        / np.linalg.norm(tet_grid.cell_data["face_normal"], axis=1)[:, None]
    )

    tet_grid.cell_data["face_center"] = np.empty((tet_grid.number_of_cells, 3))
    tet_grid.cell_data["face_center"][:, :] = np.nan
    surface_mesh_cell_centers = surface_mesh.cell_centers().points
    for cell_index, face_indices in cell_to_face.items():
        face_centers = surface_mesh_cell_centers[face_indices]
        most_down_center_index = np.argmin(face_centers[:, 2])
        tet_grid.cell_data["face_center"][cell_index] = face_centers[most_down_center_index]

    tet_grid.cell_data["cell_center"] = tet_grid.cell_centers().points

    bottom_cell_threshold = np.nanmin(tet_grid.cell_data["face_center"][:, 2]) + 0.3
    bottom_cells_mask = tet_grid.cell_data["face_center"][:, 2] < bottom_cell_threshold
    tet_grid.cell_data["is_bottom"] = bottom_cells_mask
    bottom_cells = np.where(bottom_cells_mask)[0]

    face_normals = tet_grid.cell_data["face_normal"].copy()
    face_normals[bottom_cells_mask] = np.nan
    overhang_angle = np.arccos(np.dot(face_normals, _up_vector))
    tet_grid.cell_data["overhang_angle"] = overhang_angle

    overhang_direction = face_normals[:, :2].copy()
    overhang_direction /= np.linalg.norm(overhang_direction, axis=1)[:, None]
    tet_grid.cell_data["overhang_direction"] = overhang_direction

    import networkx as nx

    IN_AIR_THRESHOLD = 1
    tet_grid.cell_data["in_air"] = np.full(tet_grid.number_of_cells, False)
    _, paths_to_bottom = nx.multi_source_dijkstra(cell_neighbour_graph, set(bottom_cells))
    tet_grid.cell_data["path_to_bottom"] = np.full(
        (tet_grid.number_of_cells, max(len(x) for x in paths_to_bottom.values())), -1
    )
    for cell_index, path_to_bottom in paths_to_bottom.items():
        tet_grid.cell_data["path_to_bottom"][cell_index, : len(path_to_bottom)] = path_to_bottom

    for cell_index in range(tet_grid.number_of_cells):
        path_to_bottom = paths_to_bottom[cell_index]
        if len(path_to_bottom) > 1:
            cell_heights = tet_grid.cell_data["cell_center"][path_to_bottom, 2]
            if np.any(cell_heights > tet_grid.cell_data["cell_center"][cell_index, 2] + IN_AIR_THRESHOLD):
                tet_grid.cell_data["in_air"][cell_index] = True

    tet_grid.cell_data["overhang_angle"][bottom_cells] = np.nan

    return bottom_cells_mask, bottom_cells


def _calculate_rotation_matrices(tet_grid, rotation_field):
    tangential_vectors = np.cross(np.array([0, 0, 1]), tet_grid.cell_data["cell_center"][:, :2])
    tangential_vectors /= np.linalg.norm(tangential_vectors, axis=1)[:, None]
    tangential_vectors[np.isnan(tangential_vectors).any(axis=1)] = [1, 0, 0]
    rotation_matrices = R.from_rotvec(rotation_field[:, None] * tangential_vectors).as_matrix()
    return rotation_matrices


def _calculate_unique_vertices_rotated(tet_grid, rotation_field):
    rotation_matrices = _calculate_rotation_matrices(tet_grid, rotation_field)
    unique_vertices = np.zeros((tet_grid.number_of_cells, 4, 3))
    for cell_index, cell in enumerate(tet_grid.field_data["cells"]):
        unique_vertices[cell_index] = tet_grid.field_data["cell_vertices"][cell]
    cell_centers = tet_grid.cell_data["cell_center"]
    unique_vertices_rotated = (
        cell_centers.reshape(-1, 1, 3, 1)
        + rotation_matrices.reshape(-1, 1, 3, 3)
        @ (unique_vertices.reshape(-1, 4, 3, 1) - cell_centers.reshape(-1, 1, 3, 1))
    )
    return unique_vertices_rotated


def _apply_rotation_field_unique_vertices(tet_grid, rotation_field):
    import pyvista as pv

    unique_vertices_rotated = _calculate_unique_vertices_rotated(tet_grid, rotation_field)
    unique_cells = np.zeros((tet_grid.number_of_cells, 5), dtype=int)
    unique_cells[:, 0] = 4
    unique_cells[:, 1:] = np.arange(tet_grid.number_of_cells * 4).reshape(-1, 4)
    new_tet = pv.UnstructuredGrid(
        unique_cells.flatten(),
        np.full(tet_grid.number_of_cells, pv.CellType.TETRA),
        unique_vertices_rotated.reshape(-1, 3),
    )
    return new_tet


def _calculate_path_length_to_base_gradient(
    tet_grid, cell_neighbour_dict, cell_neighbour_graph, bottom_cells, params
):
    import networkx as nx

    path_length_to_base_gradient = np.zeros(tet_grid.number_of_cells)

    cell_distance_to_bottom = np.empty(tet_grid.number_of_cells)
    cell_distance_to_bottom[:] = np.nan
    distances_to_bottom, paths_to_bottom = nx.multi_source_dijkstra(
        cell_neighbour_graph, set(bottom_cells)
    )
    closest_bottom_cell_indices = np.zeros(tet_grid.number_of_cells, dtype=int)
    for cell_index in range(tet_grid.number_of_cells):
        face_normal = tet_grid.cell_data["face_normal"][cell_index]
        cell_is_overhang = np.arccos(np.dot(face_normal, [0, 0, 1])) > np.deg2rad(
            90 + params.MAX_OVERHANG
        )
        if cell_is_overhang and cell_index not in bottom_cells:
            closest_bottom_cell_indices[cell_index] = paths_to_bottom[cell_index][0]
            cell_distance_to_bottom[cell_index] = distances_to_bottom[cell_index]

    tet_grid.cell_data["cell_distance_to_bottom"] = cell_distance_to_bottom

    for cell_index in range(tet_grid.number_of_cells):
        if not np.isnan(cell_distance_to_bottom[cell_index]):
            local_cells = cell_neighbour_dict["edge"][cell_index]
            local_cells = np.hstack((local_cells, cell_index))
            local_cell_path_lengths = np.array(
                [cell_distance_to_bottom[lc] for lc in local_cells]
            )
            local_cells_arr = np.array(local_cells)
            mask = ~np.isnan(local_cell_path_lengths)
            local_cells_arr = local_cells_arr[mask]
            local_cell_path_lengths = local_cell_path_lengths[mask]

            if len(local_cell_path_lengths) < 3:
                location_to_roll_to = tet_grid.cell_data["cell_center"][
                    closest_bottom_cell_indices[cell_index], :2
                ]
                direction_to_bottom = (
                    location_to_roll_to - tet_grid.cell_data["cell_center"][cell_index, :2]
                )
                direction_to_bottom /= np.linalg.norm(direction_to_bottom)
                cell_center = tet_grid.cell_data["cell_center"][cell_index, :2].copy()
                cell_center /= np.linalg.norm(cell_center)
                optimal_rotation_direction = np.dot(cell_center, direction_to_bottom) / np.abs(
                    np.dot(cell_center, direction_to_bottom)
                )
                if np.isnan(optimal_rotation_direction):
                    optimal_rotation_direction = 0
                path_length_to_base_gradient[cell_index] = optimal_rotation_direction
            else:
                points = np.hstack(
                    (
                        tet_grid.cell_data["cell_center"][local_cells_arr, :2],
                        local_cell_path_lengths[:, None],
                    )
                )
                _, plane_normal = _plane_fit(points.T)
                cell_center_direction_normalized = (
                    tet_grid.cell_data["cell_center"][cell_index, :2]
                    / np.linalg.norm(tet_grid.cell_data["cell_center"][cell_index, :2])
                )
                gradient_in_radial_direction = np.dot(
                    cell_center_direction_normalized, plane_normal[:2]
                )
                if np.isnan(gradient_in_radial_direction):
                    gradient_in_radial_direction = np.mean(
                        path_length_to_base_gradient[local_cells_arr][
                            ~np.isnan(path_length_to_base_gradient[local_cells_arr])
                        ]
                    )
                    if np.isnan(gradient_in_radial_direction):
                        gradient_in_radial_direction = 0
                path_length_to_base_gradient[cell_index] = gradient_in_radial_direction

    if params.INITIAL_ROTATION_FIELD_SMOOTHING != 0:
        for _ in range(params.INITIAL_ROTATION_FIELD_SMOOTHING):
            smoothed = np.zeros(tet_grid.number_of_cells)
            for cell_index in range(tet_grid.number_of_cells):
                if path_length_to_base_gradient[cell_index] != 0:
                    neighbours = cell_neighbour_dict["point"][cell_index]
                    local_cells = neighbours.copy()
                    for neighbour in neighbours:
                        local_cells.extend(cell_neighbour_dict["point"][neighbour])
                    local_cells = np.array(list(set(local_cells)))
                    local_cells = local_cells[path_length_to_base_gradient[local_cells] != 0]
                    smoothed[cell_index] = np.mean(path_length_to_base_gradient[local_cells])
        # Commit once, OUTSIDE the loop: every pass recomputes `smoothed` from the
        # unchanged source, so this is a single neighbourhood-average pass (matches
        # the notebook). Committing inside the loop compounded it into N diffusion
        # passes and changed the optimizer's fidelity target.
        path_length_to_base_gradient = smoothed

    if not params.SET_INITIAL_ROTATION_TO_ZERO:
        path_length_to_base_gradient[path_length_to_base_gradient == 0] = np.nan
    tet_grid.cell_data["path_length_to_base_gradient"] = path_length_to_base_gradient

    return path_length_to_base_gradient


def _calculate_initial_rotation_field(tet_grid, path_length_to_base_gradient, params):
    initial_rotation_field = np.full(tet_grid.number_of_cells, np.nan)
    initial_rotation_field = np.abs(
        np.deg2rad(90 + params.MAX_OVERHANG) - tet_grid.cell_data["overhang_angle"]
    )

    if params.STEEP_OVERHANG_COMPENSATION:
        initial_rotation_field[tet_grid.cell_data["in_air"]] += 2 * (
            np.deg2rad(180) - tet_grid.cell_data["overhang_angle"][tet_grid.cell_data["in_air"]]
        )

    initial_rotation_field *= path_length_to_base_gradient
    initial_rotation_field = np.clip(
        initial_rotation_field * params.ROTATION_MULTIPLIER,
        -np.deg2rad(360),
        np.deg2rad(360),
    )
    # DeformParams stores the caps in degrees (notebook passes np.deg2rad(...)).
    max_pos = np.deg2rad(params.MAX_POS_ROTATION)
    max_neg = np.deg2rad(params.MAX_NEG_ROTATION)
    initial_rotation_field = np.clip(initial_rotation_field, max_neg, max_pos)
    tet_grid.cell_data["initial_rotation_field"] = initial_rotation_field
    return initial_rotation_field


def _optimize_rotations(tet_grid, initial_rotation_field, params):
    import pyvista as pv

    num_cells_with_initial_rotation = np.sum(~np.isnan(initial_rotation_field))

    def objective_function(rotation_field):
        cell_face_neighbours = tet_grid.field_data["cell_face_neighbours"]
        neighbour_differences = (
            rotation_field[cell_face_neighbours[:, 0]]
            - rotation_field[cell_face_neighbours[:, 1]]
        )
        neighbour_losses = params.NEIGHBOUR_LOSS_WEIGHT * neighbour_differences**2

        valid_cell_indices = np.where(~np.isnan(initial_rotation_field))[0]
        initial_rotation_losses = (
            rotation_field[valid_cell_indices] - initial_rotation_field[valid_cell_indices]
        ) ** 2
        return np.concatenate((neighbour_losses, initial_rotation_losses))

    def objective_jacobian(rotation_field):
        cell_face_neighbours = tet_grid.field_data["cell_face_neighbours"]
        jac = lil_matrix(
            (len(cell_face_neighbours) + num_cells_with_initial_rotation,
             tet_grid.number_of_cells),
            dtype=np.float32,
        )
        cell_1 = cell_face_neighbours[:, 0]
        cell_2 = cell_face_neighbours[:, 1]
        differences = rotation_field[cell_1] - rotation_field[cell_2]
        jac[range(len(cell_face_neighbours)), cell_1] = 2 * params.NEIGHBOUR_LOSS_WEIGHT * differences
        jac[range(len(cell_face_neighbours)), cell_2] = -2 * params.NEIGHBOUR_LOSS_WEIGHT * differences

        valid_cell_indices = np.where(~np.isnan(initial_rotation_field))[0]
        jac[len(cell_face_neighbours) + np.arange(len(valid_cell_indices)), valid_cell_indices] = (
            2 * (rotation_field[valid_cell_indices] - initial_rotation_field[valid_cell_indices])
        )
        return jac.tocsr()

    def jac_sparsity():
        cell_face_neighbours = tet_grid.field_data["cell_face_neighbours"]
        sparsity = lil_matrix(
            (len(cell_face_neighbours) + num_cells_with_initial_rotation,
             tet_grid.number_of_cells),
            dtype=np.int8,
        )
        for i, (cell_1, cell_2) in enumerate(cell_face_neighbours):
            sparsity[i, cell_1] = 1
            sparsity[i, cell_2] = 1
        valid_cell_indices = np.where(~np.isnan(initial_rotation_field))[0]
        i = 0
        for cell_index, _ in enumerate(initial_rotation_field):
            if cell_index in valid_cell_indices:
                sparsity[len(cell_face_neighbours) + i, cell_index] = 1
                i += 1
        return sparsity.tocsr()

    smoothed_rotation_field = np.zeros(tet_grid.number_of_cells)
    result = least_squares(
        objective_function,
        smoothed_rotation_field,
        jac=objective_jacobian,
        max_nfev=params.rotation_iterations,
        jac_sparsity=jac_sparsity(),
        verbose=0,
        method="trf",
        ftol=1e-6,
    )
    return result.x


# --- public API --------------------------------------------------------------

def tetrahedralize_stl(stl_path, part_offset=(0, 0, 0)):
    import open3d as o3d
    import tetgen
    import pyvista as pv

    mesh = o3d.io.read_triangle_mesh(stl_path)
    input_tet = tetgen.TetGen(np.asarray(mesh.vertices), np.asarray(mesh.triangles))
    input_tet.tetrahedralize()
    tet_grid = input_tet.grid
    part_offset = np.array(part_offset)
    x_min, x_max, y_min, y_max, z_min, z_max = tet_grid.bounds
    tet_grid.points -= (
        np.array([(x_min + x_max) / 2, (y_min + y_max) / 2, z_min]) + part_offset
    )
    return tet_grid


def build_cell_graph(tet_grid):
    import networkx as nx

    cell_neighbour_dict = _build_neighbour_dict(tet_grid)
    cell_centers = tet_grid.cell_centers().points
    cell_neighbour_graph = nx.Graph()
    for edge in tet_grid.field_data["cell_point_neighbours"]:
        distance = np.linalg.norm(cell_centers[edge[0]] - cell_centers[edge[1]])
        cell_neighbour_graph.add_weighted_edges_from([(edge[0], edge[1], distance)])
    return cell_neighbour_graph, cell_neighbour_dict


def compute_tet_attributes(tet_grid, cell_neighbour_graph, cell_neighbour_dict):
    import pyvista as pv

    surface_mesh = tet_grid.extract_surface()

    cells_arr = tet_grid.cells.reshape(-1, 5)[:, 1:]
    tet_grid.add_field_data(cells_arr, "cells")
    cell_vertices = tet_grid.points
    tet_grid.add_field_data(cell_vertices, "cell_vertices")
    faces_arr = surface_mesh.faces.reshape(-1, 4)[:, 1:]
    tet_grid.add_field_data(faces_arr, "faces")
    face_vertices_arr = surface_mesh.points
    tet_grid.add_field_data(face_vertices_arr, "face_vertices")

    shared_vertices = []
    for cell_1, cell_2 in tet_grid.field_data["cell_point_neighbours"]:
        shared_vertices_these_faces = np.intersect1d(cells_arr[cell_1], cells_arr[cell_2])
        for vertex in shared_vertices_these_faces:
            shared_vertices.append({
                "cell_1_index": cell_1,
                "cell_2_index": cell_2,
                "cell_1_vertex_index": np.where(cells_arr[cell_1] == vertex)[0][0],
                "cell_2_vertex_index": np.where(cells_arr[cell_2] == vertex)[0][0],
            })

    cell_to_face = {}
    face_to_cell = {face_index: [] for face_index in range(len(faces_arr))}
    cell_to_face_vertices = {}
    face_to_cell_vertices = {}
    for cell_vertex_index, cell_vertex in enumerate(
        tet_grid.field_data["cell_vertices"].reshape(-1, 3)
    ):
        face_vertex_index = np.where((face_vertices_arr == cell_vertex).all(axis=1))[0]
        if len(face_vertex_index) == 1:
            cell_to_face_vertices[cell_vertex_index] = face_vertex_index[0]
            face_to_cell_vertices[face_vertex_index[0]] = cell_vertex_index

    for cell_index, cell in enumerate(tet_grid.field_data["cells"]):
        face_vertex_indices = [
            cell_to_face_vertices[cvi]
            for cvi in cell
            if cvi in cell_to_face_vertices
        ]
        if len(face_vertex_indices) >= 3:
            extracted = surface_mesh.extract_points(
                face_vertex_indices, adjacent_cells=False
            )
            if extracted.number_of_cells >= 1:
                cell_to_face[cell_index] = list(extracted.cell_data["vtkOriginalCellIds"])
                for face_index in extracted.cell_data["vtkOriginalCellIds"]:
                    face_to_cell[face_index].append(cell_index)

    tet_grid.add_field_data(_encode_object(cell_to_face), "cell_to_face")
    tet_grid.add_field_data(_encode_object(face_to_cell), "face_to_cell")

    tet_grid.cell_data["has_face"] = np.zeros(tet_grid.number_of_cells)
    for cell_index in cell_to_face:
        tet_grid.cell_data["has_face"][cell_index] = 1

    bottom_cells_mask, bottom_cells = _update_tet_attributes(
        tet_grid, cell_neighbour_graph
    )

    import networkx as nx

    bottom_cell_graph = nx.Graph()
    for cell_index in bottom_cells:
        bottom_cell_graph.add_node(cell_index)
    for cell_index in bottom_cells:
        for neighbour in cell_neighbour_dict["point"][cell_index]:
            if neighbour in bottom_cells:
                bottom_cell_graph.add_edge(cell_index, neighbour)

    bottom_cell_groups = [
        list(x) for x in list(nx.connected_components(bottom_cell_graph))
    ]
    tet_grid.add_field_data(np.array(bottom_cell_groups, dtype=object), "bottom_cell_groups")

    attrs = {
        "bottom_cells": bottom_cells,
        "bottom_cells_mask": bottom_cells_mask,
        "bottom_cell_groups": bottom_cell_groups,
    }
    return attrs


def compute_rotation_field(tet_grid, cell_neighbour_graph, cell_neighbour_dict, attrs, params):
    path_length_to_base_gradient = _calculate_path_length_to_base_gradient(
        tet_grid, cell_neighbour_dict, cell_neighbour_graph,
        attrs["bottom_cells"], params,
    )

    initial_rotation_field = _calculate_initial_rotation_field(
        tet_grid, path_length_to_base_gradient, params
    )

    rotation_field = _optimize_rotations(tet_grid, initial_rotation_field, params)
    tet_grid.cell_data["rotation_field"] = rotation_field
    return rotation_field


def deform_mesh(tet_grid, rotation_field, params):
    import pyvista as pv

    N_mat = np.eye(4) - 1 / 4 * np.ones((4, 4))

    new_vertices = tet_grid.points.copy()
    params_flat = new_vertices.flatten()

    rotation_matrices = _calculate_rotation_matrices(tet_grid, rotation_field)
    old_vertices = tet_grid.field_data["cell_vertices"][tet_grid.field_data["cells"]]
    old_vertices_transformed = np.einsum(
        "ijk,ikl->ijl",
        rotation_matrices,
        (N_mat @ old_vertices).transpose(0, 2, 1),
    )

    def objective_function(p):
        new_v = p[: tet_grid.number_of_points * 3].reshape(-1, 3)
        new_v_transformed = (N_mat @ new_v[tet_grid.field_data["cells"]]).transpose(0, 2, 1)
        position_losses = (
            np.linalg.norm(new_v_transformed - old_vertices_transformed, axis=(1, 2)) ** 2
        )
        return position_losses

    def objective_jacobian(p):
        J = lil_matrix((tet_grid.number_of_cells, len(p)), dtype=np.float32)
        new_v = p[: tet_grid.number_of_points * 3].reshape(-1, 3)
        new_v_transformed = (N_mat @ new_v[tet_grid.field_data["cells"]]).transpose(0, 2, 1)
        diff = new_v_transformed - old_vertices_transformed
        diff = diff.transpose(0, 2, 1)
        cell_indices = np.repeat(
            np.arange(tet_grid.number_of_cells), len(tet_grid.field_data["cells"][0])
        )
        vertex_indices = np.ravel(tet_grid.field_data["cells"])
        for dim in range(3):
            J[cell_indices, vertex_indices * 3 + dim] = 2 * diff[:, :, dim].ravel()
        return J.tocsr()

    def jac_sparsity():
        sparsity = lil_matrix(
            (tet_grid.number_of_cells, len(params_flat)), dtype=np.int8
        )
        cell_indices = np.repeat(
            np.arange(tet_grid.number_of_cells), len(tet_grid.field_data["cells"][0])
        )
        vertex_indices = np.ravel(tet_grid.field_data["cells"])
        for dim in range(3):
            sparsity[cell_indices, vertex_indices * 3 + dim] = 1
        return sparsity.tocsr()

    result = least_squares(
        objective_function,
        params_flat,
        max_nfev=params.deformation_iterations,
        verbose=0,
        jac=objective_jacobian,
        jac_sparsity=jac_sparsity(),
        method="trf",
        x_scale="jac",
    )

    new_vertices_solved = result.x[: tet_grid.number_of_points * 3].reshape(-1, 3)
    deformed_tet = pv.UnstructuredGrid(
        tet_grid.cells,
        np.full(tet_grid.number_of_cells, pv.CellType.TETRA),
        new_vertices_solved,
    )
    for key in tet_grid.field_data.keys():
        deformed_tet.field_data[key] = tet_grid.field_data[key]
    for key in tet_grid.cell_data.keys():
        deformed_tet.cell_data[key] = tet_grid.cell_data[key]

    return deformed_tet


def export_surface_stl(deformed_tet_grid, out_stl_path):
    x_min, x_max, y_min, y_max, z_min, z_max = deformed_tet_grid.bounds
    offsets_applied = np.array([(x_min + x_max) / 2, (y_min + y_max) / 2, z_min])
    deformed_tet_grid.points -= offsets_applied
    deformed_tet_grid.extract_surface().save(out_stl_path)


def run_deform(stl_path, params=None):
    if params is None:
        params = DeformParams()

    tet_grid = tetrahedralize_stl(stl_path, params.part_offset)
    cell_neighbour_graph, cell_neighbour_dict = build_cell_graph(tet_grid)

    attrs = compute_tet_attributes(tet_grid, cell_neighbour_graph, cell_neighbour_dict)
    # Snapshot the fully-attributed undeformed grid AFTER attributes are computed
    # and BEFORE deformation: Stage C reads cell_center / cells / cell_vertices /
    # points off this reference. Copying earlier returned a bare grid -> KeyError.
    original_tet_grid = tet_grid.copy()
    rotation_field = compute_rotation_field(
        tet_grid, cell_neighbour_graph, cell_neighbour_dict, attrs, params
    )
    deformed_tet_grid = deform_mesh(tet_grid, rotation_field, params)
    _update_tet_attributes(deformed_tet_grid, cell_neighbour_graph)

    return deformed_tet_grid, original_tet_grid
