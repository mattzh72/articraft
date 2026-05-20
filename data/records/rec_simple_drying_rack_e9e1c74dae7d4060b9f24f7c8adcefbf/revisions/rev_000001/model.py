from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


METAL = Material("warm_white_powder_coated_steel", rgba=(0.92, 0.93, 0.88, 1.0))
HINGE = Material("matte_grey_hinge_plastic", rgba=(0.34, 0.35, 0.34, 1.0))
RUBBER = Material("dark_rubber_feet", rgba=(0.03, 0.03, 0.025, 1.0))


def _rod_x(part, x: float, y: float, z: float, length: float, radius: float, name: str, material=METAL) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _rod_y(part, x: float, y: float, z: float, length: float, radius: float, name: str, material=METAL) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _wire_rod(part, points, radius: float, name: str, material=METAL, corner_radius: float = 0.0) -> None:
    geom = wire_from_points(
        points,
        radius=radius,
        radial_segments=16,
        cap_ends=True,
        corner_mode="fillet" if corner_radius > 0.0 else "miter",
        corner_radius=corner_radius,
        corner_segments=8,
    )
    part.visual(
        mesh_from_geometry(geom, f"{part.name}_{name}"),
        origin=Origin(),
        material=material,
        name=name,
    )


def _add_central_rack(central) -> None:
    top_z = 0.90
    half_w = 0.36
    half_l = 0.675
    frame_r = 0.012
    rail_r = 0.006

    # End tubes and a set of hanging rails span the fixed center rack.
    _rod_x(central, 0.0, -half_l, top_z, 0.76, frame_r, "rear_frame_rail")
    _rod_x(central, 0.0, half_l, top_z, 0.76, frame_r, "front_frame_rail")
    # Inboard longitudinal stiffeners keep the fixed rack visibly and
    # geometrically continuous even where the outer side rails are interrupted
    # for hinge barrels.
    _rod_y(central, -0.30, 0.0, top_z, 1.35, 0.007, "center_stringer_0")
    _rod_y(central, 0.30, 0.0, top_z, 1.35, 0.007, "center_stringer_1")
    for i, y in enumerate((-0.56, -0.44, -0.12, 0.0, 0.12, 0.44, 0.56)):
        _rod_x(central, 0.0, y, top_z, 0.74, rail_r, f"center_hanging_rail_{i}")

    # Long side rails are intentionally broken into hinge-knuckle segments so
    # each wing can occupy alternating barrels on the same hinge line.
    side_segments = [(-0.675, -0.43), (-0.19, 0.19), (0.43, 0.675)]
    for side_index, x in enumerate((-half_w, half_w)):
        for seg_index, (y0, y1) in enumerate(side_segments):
            _rod_y(
                central,
                x,
                (y0 + y1) / 2.0,
                top_z,
                y1 - y0,
                frame_r,
                f"side_hinge_rail_{side_index}_{seg_index}",
                HINGE if seg_index == 1 else METAL,
            )

    # Lower support hinge hardware below the rear edge.
    hinge_y = -0.58
    hinge_z = 0.82
    _rod_x(central, 0.0, hinge_y, hinge_z, 0.36, 0.011, "lower_parent_hinge", HINGE)
    for i, x in enumerate((-0.12, 0.12)):
        _wire_rod(
            central,
            [(x, -0.675, top_z), (x, -0.625, 0.86), (x, hinge_y, hinge_z)],
            0.007,
            f"lower_hinge_bracket_{i}",
            HINGE,
            corner_radius=0.008,
        )


def _add_wing(wing, sign: float) -> None:
    frame_r = 0.010
    rail_r = 0.0055
    half_l = 0.675
    inner_x = sign * 0.045
    outer_x = sign * 0.58

    # Alternating hinge barrels on the wing side of the hinge line.
    for i, y in enumerate((-0.31, 0.31)):
        _rod_y(wing, 0.0, y, 0.0, 0.24, 0.011, f"wing_hinge_barrel_{i}", HINGE)
        _rod_x(wing, sign * 0.022, y, 0.0, 0.055, 0.007, f"hinge_bridge_{i}", HINGE)

    # Rectangular tubular wing frame with many slender clothes rails.
    _rod_y(wing, inner_x, 0.0, 0.0, 1.35, frame_r, "inner_wing_rail")
    _rod_y(wing, outer_x, 0.0, 0.0, 1.35, frame_r, "outer_wing_rail")
    _rod_x(wing, sign * 0.3125, -half_l, 0.0, 0.555, frame_r, "rear_wing_rail")
    _rod_x(wing, sign * 0.3125, half_l, 0.0, 0.555, frame_r, "front_wing_rail")
    for i, y in enumerate((-0.52, -0.36, -0.18, 0.0, 0.18, 0.36, 0.52)):
        _rod_x(wing, sign * 0.3125, y, 0.0, 0.54, rail_r, f"wing_hanging_rail_{i}")


def _add_lower_support(lower) -> None:
    # Local frame: hinge line is the X axis at the part origin.  In the open
    # pose the support slopes down and forward to rubber feet near the floor.
    left_hinge = (-0.28, 0.0, 0.0)
    right_hinge = (0.28, 0.0, 0.0)
    left_foot = (-0.34, 0.70, -0.75)
    right_foot = (0.34, 0.70, -0.75)

    for i, x in enumerate((-0.28, 0.28)):
        _rod_x(lower, x, 0.0, 0.0, 0.20, 0.011, f"lower_hinge_barrel_{i}", HINGE)

    _wire_rod(
        lower,
        [left_hinge, left_foot, right_foot, right_hinge],
        0.010,
        "lower_u_frame",
        METAL,
        corner_radius=0.025,
    )

    for i, t in enumerate((0.22, 0.40, 0.58, 0.76)):
        lx = left_hinge[0] + (left_foot[0] - left_hinge[0]) * t
        ly = left_hinge[1] + (left_foot[1] - left_hinge[1]) * t
        lz = left_hinge[2] + (left_foot[2] - left_hinge[2]) * t
        rx = right_hinge[0] + (right_foot[0] - right_hinge[0]) * t
        ry = right_hinge[1] + (right_foot[1] - right_hinge[1]) * t
        rz = right_hinge[2] + (right_foot[2] - right_hinge[2]) * t
        _wire_rod(lower, [(lx, ly, lz), (rx, ry, rz)], 0.0055, f"lower_hanging_rail_{i}")

    _wire_rod(lower, [(-0.40, 0.70, -0.75), (-0.34, 0.70, -0.75)], 0.014, "foot_pad_0", RUBBER)
    _wire_rod(lower, [(0.34, 0.70, -0.75), (0.40, 0.70, -0.75)], 0.014, "foot_pad_1", RUBBER)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_drying_rack")

    central = model.part("central_rack")
    _add_central_rack(central)

    wing_0 = model.part("wing_0")
    _add_wing(wing_0, sign=1.0)

    wing_1 = model.part("wing_1")
    _add_wing(wing_1, sign=-1.0)

    lower_support = model.part("lower_support")
    _add_lower_support(lower_support)

    model.articulation(
        "central_to_wing_0",
        ArticulationType.REVOLUTE,
        parent=central,
        child=wing_0,
        origin=Origin(xyz=(0.36, 0.0, 0.90)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.55),
    )
    model.articulation(
        "central_to_wing_1",
        ArticulationType.REVOLUTE,
        parent=central,
        child=wing_1,
        origin=Origin(xyz=(-0.36, 0.0, 0.90)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.55),
    )
    model.articulation(
        "central_to_lower_support",
        ArticulationType.REVOLUTE,
        parent=central,
        child=lower_support,
        origin=Origin(xyz=(0.0, -0.58, 0.82)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.6, lower=0.0, upper=0.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    central = object_model.get_part("central_rack")
    wing_0 = object_model.get_part("wing_0")
    wing_1 = object_model.get_part("wing_1")
    lower = object_model.get_part("lower_support")
    wing_joint_0 = object_model.get_articulation("central_to_wing_0")
    wing_joint_1 = object_model.get_articulation("central_to_wing_1")
    lower_joint = object_model.get_articulation("central_to_lower_support")

    ctx.expect_overlap(central, wing_0, axes="y", min_overlap=1.20, name="right wing shares rack length")
    ctx.expect_overlap(central, wing_1, axes="y", min_overlap=1.20, name="left wing shares rack length")
    ctx.expect_overlap(central, lower, axes="x", min_overlap=0.55, name="lower support centered under rack")

    a0 = ctx.part_world_aabb(wing_0)
    a1 = ctx.part_world_aabb(wing_1)
    if a0 is not None and a1 is not None:
        c0 = (a0[0][0] + a0[1][0]) / 2.0
        c1 = (a1[0][0] + a1[1][0]) / 2.0
        w0 = a0[1][0] - a0[0][0]
        w1 = a1[1][0] - a1[0][0]
        ctx.check(
            "open wings are symmetric about center plane",
            abs(c0 + c1) < 0.015 and abs(w0 - w1) < 0.015,
            details=f"wing centers {c0:.3f}, {c1:.3f}; widths {w0:.3f}, {w1:.3f}",
        )
    else:
        ctx.fail("open wings are symmetric about center plane", "missing wing AABB")

    before_0 = ctx.part_element_world_aabb(wing_0, elem="outer_wing_rail")
    before_1 = ctx.part_element_world_aabb(wing_1, elem="outer_wing_rail")
    with ctx.pose({wing_joint_0: 1.0, wing_joint_1: 1.0}):
        after_0 = ctx.part_element_world_aabb(wing_0, elem="outer_wing_rail")
        after_1 = ctx.part_element_world_aabb(wing_1, elem="outer_wing_rail")
    if before_0 and before_1 and after_0 and after_1:
        ctx.check(
            "both wing frames fold upward",
            after_0[0][2] > before_0[0][2] + 0.30 and after_1[0][2] > before_1[0][2] + 0.30,
            details=f"wing z before {before_0[0][2]:.3f}, {before_1[0][2]:.3f}; after {after_0[0][2]:.3f}, {after_1[0][2]:.3f}",
        )

    lower_before = ctx.part_element_world_aabb(lower, elem="lower_u_frame")
    with ctx.pose({lower_joint: 0.75}):
        lower_after = ctx.part_element_world_aabb(lower, elem="lower_u_frame")
    if lower_before and lower_after:
        ctx.check(
            "lower support frame folds on its hinge",
            lower_after[0][2] > lower_before[0][2] + 0.18,
            details=f"lower support min z before {lower_before[0][2]:.3f}, after {lower_after[0][2]:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
