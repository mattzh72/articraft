from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


CUBE_SIZE = 0.055
CUBE_CLEARANCE = 0.002
CUBE_PITCH = CUBE_SIZE + CUBE_CLEARANCE
HINGE_RADIUS = 0.0035
HINGE_LEAF_THICKNESS = 0.002
HINGE_LENGTH = 0.040
HINGE_KNUCKLE_GAP = 0.002
HINGE_LEAF_WIDTH = 0.020
HINGE_BRIDGE_WIDTH = 0.011
PANEL_THICKNESS = 0.0012
PANEL_INSET = 0.008
EMBED = 0.00045

AXES = ("x", "y", "z")


def _axis_rotation(axis_index: int) -> tuple[float, float, float]:
    """Rotate a local +Z cylinder onto the requested world/local axis."""
    if axis_index == 0:
        return (0.0, math.pi / 2.0, 0.0)
    if axis_index == 1:
        return (-math.pi / 2.0, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def _rounded_cube_mesh() -> object:
    return cq.Workplane("XY").box(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE).edges().fillet(0.004)


def _vec_sub(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _box_for_axes(
    axis_dim: float,
    normal_dim: float,
    edge_dim: float,
    *,
    axis_index: int,
    normal_index: int,
    edge_index: int,
) -> Box:
    dims = [0.0, 0.0, 0.0]
    dims[axis_index] = axis_dim
    dims[normal_index] = normal_dim
    dims[edge_index] = edge_dim
    return Box(tuple(dims))


def _add_face_panel(
    part,
    *,
    center_world: tuple[float, float, float],
    frame_world: tuple[float, float, float],
    axis_index: int,
    sign: int,
    material: Material,
    name: str,
) -> None:
    dims = [CUBE_SIZE - 2.0 * PANEL_INSET] * 3
    dims[axis_index] = PANEL_THICKNESS
    panel_center = list(center_world)
    panel_center[axis_index] += sign * (CUBE_SIZE / 2.0 + PANEL_THICKNESS / 2.0 - EMBED)
    part.visual(
        Box(tuple(dims)),
        origin=Origin(xyz=_vec_sub(tuple(panel_center), frame_world)),
        material=material,
        name=name,
    )


def _add_hinge_segment(
    part,
    *,
    frame_world: tuple[float, float, float],
    pin_center_world: tuple[float, float, float],
    body_center_world: tuple[float, float, float],
    axis_index: int,
    normal_index: int,
    normal_sign_to_child: int,
    edge_index: int,
    edge_sign: int,
    segment_center_offset: float,
    segment_length: float,
    side_sign: int,
    material: Material,
    name: str,
) -> None:
    """Add one barrel knuckle plus its small wrapped leaf, all mounted to a cube."""
    pin_center = list(pin_center_world)
    pin_center[axis_index] += segment_center_offset

    part.visual(
        Cylinder(radius=HINGE_RADIUS, length=segment_length),
        origin=Origin(xyz=_vec_sub(tuple(pin_center), frame_world), rpy=_axis_rotation(axis_index)),
        material=material,
        name=f"{name}_barrel",
    )

    seam = (pin_center_world[normal_index])
    bridge_center = list(pin_center)
    bridge_center[normal_index] = seam + side_sign * normal_sign_to_child * (HINGE_BRIDGE_WIDTH / 2.0)
    body_face = body_center_world[edge_index] + edge_sign * (CUBE_SIZE / 2.0 - EMBED)
    bridge_center[edge_index] = (body_face + pin_center_world[edge_index]) / 2.0
    bridge_edge_dim = abs(pin_center_world[edge_index] - body_face) + HINGE_RADIUS
    part.visual(
        _box_for_axes(
            segment_length * 0.95,
            HINGE_BRIDGE_WIDTH,
            bridge_edge_dim,
            axis_index=axis_index,
            normal_index=normal_index,
            edge_index=edge_index,
        ),
        origin=Origin(xyz=_vec_sub(tuple(bridge_center), frame_world)),
        material=material,
        name=f"{name}_wrap",
    )

    leaf_center = list(pin_center)
    leaf_center[normal_index] = seam + side_sign * normal_sign_to_child * (HINGE_LEAF_WIDTH / 2.0)
    leaf_center[edge_index] = body_center_world[edge_index] + edge_sign * (
        CUBE_SIZE / 2.0 + HINGE_LEAF_THICKNESS / 2.0 - EMBED
    )
    part.visual(
        _box_for_axes(
            segment_length * 0.95,
            HINGE_LEAF_WIDTH,
            HINGE_LEAF_THICKNESS,
            axis_index=axis_index,
            normal_index=normal_index,
            edge_index=edge_index,
        ),
        origin=Origin(xyz=_vec_sub(tuple(leaf_center), frame_world)),
        material=material,
        name=f"{name}_leaf",
    )


def _add_hinge_hardware(
    parent,
    child,
    *,
    parent_frame: tuple[float, float, float],
    child_frame: tuple[float, float, float],
    parent_center: tuple[float, float, float],
    child_center: tuple[float, float, float],
    pin_center: tuple[float, float, float],
    axis_index: int,
    normal_index: int,
    normal_sign_to_child: int,
    edge_index: int,
    edge_sign: int,
    material: Material,
    index: int,
) -> None:
    child_length = HINGE_LENGTH * 0.44
    parent_length = (HINGE_LENGTH - child_length - 2.0 * HINGE_KNUCKLE_GAP) / 2.0
    parent_offset = child_length / 2.0 + HINGE_KNUCKLE_GAP + parent_length / 2.0

    _add_hinge_segment(
        child,
        frame_world=child_frame,
        pin_center_world=pin_center,
        body_center_world=child_center,
        axis_index=axis_index,
        normal_index=normal_index,
        normal_sign_to_child=normal_sign_to_child,
        edge_index=edge_index,
        edge_sign=edge_sign,
        segment_center_offset=0.0,
        segment_length=child_length,
        side_sign=1,
        material=material,
        name=f"in_hinge_{index}",
    )
    for n, offset in enumerate((-parent_offset, parent_offset)):
        _add_hinge_segment(
            parent,
            frame_world=parent_frame,
            pin_center_world=pin_center,
            body_center_world=parent_center,
            axis_index=axis_index,
            normal_index=normal_index,
            normal_sign_to_child=normal_sign_to_child,
            edge_index=edge_index,
            edge_sign=edge_sign,
            segment_center_offset=offset,
            segment_length=parent_length,
            side_sign=-1,
            material=material,
            name=f"out_hinge_{index}_{n}",
        )

    parent.visual(
        Cylinder(radius=HINGE_RADIUS * 0.46, length=HINGE_LENGTH + 0.004),
        origin=Origin(xyz=_vec_sub(pin_center, parent_frame), rpy=_axis_rotation(axis_index)),
        material=material,
        name=f"out_hinge_{index}_pin",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="edge_hinged_infinity_cube")

    blue_plastic = model.material("blue_plastic", rgba=(0.02, 0.12, 0.45, 1.0))
    teal_plastic = model.material("teal_plastic", rgba=(0.00, 0.42, 0.50, 1.0))
    violet_plastic = model.material("violet_plastic", rgba=(0.26, 0.09, 0.48, 1.0))
    warm_panel = model.material("warm_panel", rgba=(0.95, 0.44, 0.16, 1.0))
    yellow_panel = model.material("yellow_panel", rgba=(1.00, 0.78, 0.16, 1.0))
    white_panel = model.material("white_panel", rgba=(0.92, 0.93, 0.88, 1.0))
    hinge_metal = model.material("brushed_steel", rgba=(0.74, 0.75, 0.70, 1.0))

    body_mesh = mesh_from_cadquery(_rounded_cube_mesh(), "rounded_cube_body", tolerance=0.0007)

    cube_centers = [
        (0.0, 0.0, 0.0),
        (CUBE_PITCH, 0.0, 0.0),
        (CUBE_PITCH, CUBE_PITCH, 0.0),
        (0.0, CUBE_PITCH, 0.0),
        (0.0, CUBE_PITCH, CUBE_PITCH),
        (CUBE_PITCH, CUBE_PITCH, CUBE_PITCH),
        (CUBE_PITCH, 0.0, CUBE_PITCH),
        (0.0, 0.0, CUBE_PITCH),
    ]

    hinge_specs = [
        # parent, child, hinge axis, positive fold sign, exposed outside-edge axis and sign
        (0, 1, 2, -1, 1, -1),  # front vertical edge
        (1, 2, 0, -1, 2, -1),  # lower horizontal edge
        (2, 3, 2, -1, 1, 1),   # rear vertical edge
        (3, 4, 1, -1, 0, -1),  # left vertical-stack edge
        (4, 5, 2, 1, 1, 1),    # rear upper vertical edge
        (5, 6, 0, -1, 2, 1),   # top horizontal edge
        (6, 7, 2, 1, 1, -1),   # front upper vertical edge
    ]

    frame_positions: list[tuple[float, float, float]] = [cube_centers[0]]
    pin_centers: list[tuple[float, float, float]] = []
    normal_indices: list[int] = []
    normal_signs: list[int] = []

    for parent_i, child_i, axis_i, axis_sign, edge_i, edge_sign in hinge_specs:
        parent_center = cube_centers[parent_i]
        child_center = cube_centers[child_i]
        delta = [child_center[k] - parent_center[k] for k in range(3)]
        normal_i = max(range(3), key=lambda k: abs(delta[k]))
        normal_sign = 1 if delta[normal_i] > 0 else -1
        pin = [parent_center[k] for k in range(3)]
        pin[normal_i] = (parent_center[normal_i] + child_center[normal_i]) / 2.0
        pin[edge_i] = parent_center[edge_i] + edge_sign * (
            CUBE_SIZE / 2.0 + HINGE_LEAF_THICKNESS + HINGE_RADIUS
        )
        pin_centers.append(tuple(pin))
        normal_indices.append(normal_i)
        normal_signs.append(normal_sign)
        frame_positions.append(tuple(pin))

    parts = []
    body_materials = [blue_plastic, teal_plastic, violet_plastic, blue_plastic, teal_plastic, violet_plastic, blue_plastic, teal_plastic]
    panel_materials = [warm_panel, yellow_panel, white_panel]
    for i, center in enumerate(cube_centers):
        part = model.part(f"cube_{i}")
        part.visual(
            body_mesh,
            origin=Origin(xyz=_vec_sub(center, frame_positions[i])),
            material=body_materials[i],
            name="body",
        )
        panel_count = 0
        for axis_i in range(3):
            if abs(center[axis_i] - 0.0) < 1e-9:
                _add_face_panel(
                    part,
                    center_world=center,
                    frame_world=frame_positions[i],
                    axis_index=axis_i,
                    sign=-1,
                    material=panel_materials[(i + axis_i) % len(panel_materials)],
                    name=f"face_panel_{panel_count}",
                )
                panel_count += 1
            if abs(center[axis_i] - CUBE_PITCH) < 1e-9:
                _add_face_panel(
                    part,
                    center_world=center,
                    frame_world=frame_positions[i],
                    axis_index=axis_i,
                    sign=1,
                    material=panel_materials[(i + axis_i + 1) % len(panel_materials)],
                    name=f"face_panel_{panel_count}",
                )
                panel_count += 1
        parts.append(part)

    for i, (parent_i, child_i, axis_i, axis_sign, edge_i, edge_sign) in enumerate(hinge_specs):
        parent_center = cube_centers[parent_i]
        child_center = cube_centers[child_i]
        normal_i = normal_indices[i]
        _add_hinge_hardware(
            parts[parent_i],
            parts[child_i],
            parent_frame=frame_positions[parent_i],
            child_frame=frame_positions[child_i],
            parent_center=parent_center,
            child_center=child_center,
            pin_center=pin_centers[i],
            axis_index=axis_i,
            normal_index=normal_i,
            normal_sign_to_child=normal_signs[i],
            edge_index=edge_i,
            edge_sign=edge_sign,
            material=hinge_metal,
            index=i,
        )

        model.articulation(
            f"hinge_{i}",
            ArticulationType.REVOLUTE,
            parent=parts[parent_i],
            child=parts[child_i],
            origin=Origin(xyz=_vec_sub(pin_centers[i], frame_positions[parent_i])),
            axis=tuple(float(axis_sign) if k == axis_i else 0.0 for k in range(3)),
            motion_limits=MotionLimits(effort=1.8, velocity=4.0, lower=0.0, upper=math.pi),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.check("eight cube links", len(object_model.parts) == 8, details=f"parts={len(object_model.parts)}")
    revolute_joints = [joint for joint in object_model.articulations if joint.articulation_type == ArticulationType.REVOLUTE]
    ctx.check(
        "seven edge hinges",
        len(revolute_joints) == 7,
        details=f"revolute_joints={len(revolute_joints)}",
    )

    cube_centers = [
        (0.0, 0.0, 0.0),
        (CUBE_PITCH, 0.0, 0.0),
        (CUBE_PITCH, CUBE_PITCH, 0.0),
        (0.0, CUBE_PITCH, 0.0),
        (0.0, CUBE_PITCH, CUBE_PITCH),
        (CUBE_PITCH, CUBE_PITCH, CUBE_PITCH),
        (CUBE_PITCH, 0.0, CUBE_PITCH),
        (0.0, 0.0, CUBE_PITCH),
    ]
    chain_pairs = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 6), (6, 7)]
    for i, (a, b) in enumerate(chain_pairs):
        cube_a = object_model.get_part(f"cube_{a}")
        cube_b = object_model.get_part(f"cube_{b}")
        ctx.allow_overlap(
            cube_a,
            cube_b,
            elem_a=f"out_hinge_{i}_pin",
            elem_b=f"in_hinge_{i}_barrel",
            reason="The hinge pin is intentionally captured inside the moving cube's barrel knuckle.",
        )
        ctx.allow_overlap(
            cube_a,
            cube_b,
            elem_a=f"out_hinge_{i}_pin",
            elem_b=f"in_hinge_{i}_wrap",
            reason="The hinge pin locally passes through the wrapped hinge leaf at the knuckle root.",
        )
        delta = [cube_centers[b][k] - cube_centers[a][k] for k in range(3)]
        normal_i = max(range(3), key=lambda k: abs(delta[k]))
        axis = AXES[normal_i]
        if delta[normal_i] > 0:
            positive, negative = cube_b, cube_a
        else:
            positive, negative = cube_a, cube_b
        ctx.expect_gap(
            positive,
            negative,
            axis=axis,
            max_gap=CUBE_CLEARANCE + 0.002,
            max_penetration=0.0,
            positive_elem="body",
            negative_elem="body",
            name=f"cube_pair_{i}_body_clearance",
        )
        overlap_axes = "".join(AXES[k] for k in range(3) if k != normal_i)
        ctx.expect_overlap(
            cube_a,
            cube_b,
            axes=overlap_axes,
            min_overlap=CUBE_SIZE * 0.65,
            elem_a="body",
            elem_b="body",
            name=f"cube_pair_{i}_shared_face_overlap",
        )
        hinge_axis = object_model.get_articulation(f"hinge_{i}").axis
        hinge_axis_i = max(range(3), key=lambda k: abs(hinge_axis[k]))
        radial_axes = "".join(AXES[k] for k in range(3) if k != hinge_axis_i)
        ctx.expect_within(
            cube_a,
            cube_b,
            axes=radial_axes,
            inner_elem=f"out_hinge_{i}_pin",
            outer_elem=f"in_hinge_{i}_barrel",
            margin=0.001,
            name=f"hinge_{i}_pin_centered",
        )
        ctx.expect_overlap(
            cube_a,
            cube_b,
            axes=AXES[hinge_axis_i],
            min_overlap=HINGE_LENGTH * 0.35,
            elem_a=f"out_hinge_{i}_pin",
            elem_b=f"in_hinge_{i}_barrel",
            name=f"hinge_{i}_pin_engaged",
        )

    def _elem_center(part_name: str) -> tuple[float, float, float] | None:
        box = ctx.part_element_world_aabb(object_model.get_part(part_name), elem="body")
        if box is None:
            return None
        lo, hi = box
        return ((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0)

    root_pos = _elem_center("cube_7")
    with ctx.pose({"hinge_6": math.pi / 2.0}):
        folded_pos = _elem_center("cube_7")
    ctx.check(
        "last hinge folds the chain",
        root_pos is not None
        and folded_pos is not None
        and sum((folded_pos[k] - root_pos[k]) ** 2 for k in range(3)) > 0.001,
        details=f"rest={root_pos}, folded={folded_pos}",
    )

    return ctx.report()


object_model = build_object_model()
