from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _arc_pane_origin(radius: float, angle_deg: float, z_center: float) -> Origin:
    angle = math.radians(angle_deg)
    return Origin(
        xyz=(radius * math.cos(angle), radius * math.sin(angle), z_center),
        rpy=(0.0, 0.0, angle),
    )


def _add_fold_leaf(
    part,
    *,
    width: float,
    height: float,
    direction: float,
    metal,
    glass,
) -> None:
    stile = 0.04
    rail = 0.045
    glass_x = width * 0.5 * direction
    part.visual(
        Box((width - 2.0 * stile, 0.012, height - 2.0 * rail)),
        origin=Origin(xyz=(glass_x, 0.0, height * 0.5)),
        material=glass,
        name="glass_panel",
    )
    part.visual(
        Box((stile, 0.045, height)),
        origin=Origin(xyz=(direction * (stile * 0.5), 0.0, height * 0.5)),
        material=metal,
        name="hinge_stile",
    )
    part.visual(
        Box((stile, 0.045, height)),
        origin=Origin(
            xyz=(direction * (width - stile * 0.5), 0.0, height * 0.5),
        ),
        material=metal,
        name="knuckle_stile",
    )
    part.visual(
        Box((width, 0.045, rail)),
        origin=Origin(xyz=(direction * (width * 0.5), 0.0, rail * 0.5)),
        material=metal,
        name="bottom_rail",
    )
    part.visual(
        Box((width, 0.045, rail)),
        origin=Origin(
            xyz=(direction * (width * 0.5), 0.0, height - rail * 0.5),
        ),
        material=metal,
        name="top_rail",
    )
    part.visual(
        Cylinder(radius=0.017, length=height - 0.08),
        origin=Origin(
            xyz=(direction * 0.017, 0.0, height * 0.5),
        ),
        material=metal,
        name="hinge_barrel",
    )


def _span(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None, axis: str) -> float | None:
    if aabb is None:
        return None
    axis_index = {"x": 0, "y": 1, "z": 2}[axis]
    return aabb[1][axis_index] - aabb[0][axis_index]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_wing_revolving_door_with_bypass_pairs")

    aluminum = model.material("aluminum", rgba=(0.58, 0.60, 0.63, 1.0))
    dark_aluminum = model.material("dark_aluminum", rgba=(0.27, 0.29, 0.32, 1.0))
    threshold_gray = model.material("threshold_gray", rgba=(0.42, 0.43, 0.45, 1.0))
    clear_glass = model.material("clear_glass", rgba=(0.72, 0.86, 0.94, 0.28))
    smoke_glass = model.material("smoke_glass", rgba=(0.66, 0.80, 0.88, 0.22))
    seal_black = model.material("seal_black", rgba=(0.08, 0.08, 0.09, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((4.70, 2.50, 2.60)),
        mass=480.0,
        origin=Origin(xyz=(0.0, 0.0, 1.30)),
    )

    frame.visual(
        Box((4.70, 0.24, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=threshold_gray,
        name="base_sill",
    )
    frame.visual(
        Box((4.70, 0.32, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 2.39)),
        material=aluminum,
        name="header_beam",
    )
    frame.visual(
        Cylinder(radius=1.12, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_aluminum,
        name="drum_threshold",
    )
    frame.visual(
        Cylinder(radius=1.18, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 2.42)),
        material=dark_aluminum,
        name="drum_canopy",
    )

    post_height = 2.22
    post_center_z = 0.08 + post_height * 0.5
    for x_pos, name in (
        (-2.15, "left_outer_jamb"),
        (-1.25, "left_inner_post"),
        (1.25, "right_inner_post"),
        (2.15, "right_outer_jamb"),
    ):
        frame.visual(
            Box((0.10, 0.18, post_height)),
            origin=Origin(xyz=(x_pos, 0.0, post_center_z)),
            material=aluminum,
            name=name,
        )

    boundary_radius = 1.05
    for angle_deg, name in (
        (45.0, "front_right_boundary_post"),
        (135.0, "front_left_boundary_post"),
        (225.0, "rear_left_boundary_post"),
        (315.0, "rear_right_boundary_post"),
    ):
        angle = math.radians(angle_deg)
        frame.visual(
            Cylinder(radius=0.04, length=post_height),
            origin=Origin(
                xyz=(
                    boundary_radius * math.cos(angle),
                    boundary_radius * math.sin(angle),
                    post_center_z,
                )
            ),
            material=aluminum,
            name=name,
        )

    pane_angles = (-45.0, -15.0, 15.0, 45.0, 135.0, 165.0, 195.0, 225.0)
    for index, angle_deg in enumerate(pane_angles):
        frame.visual(
            Box((0.024, 0.54, post_height)),
            origin=_arc_pane_origin(boundary_radius, angle_deg, post_center_z),
            material=smoke_glass,
            name=f"drum_side_glass_{index:02d}",
        )

    for x_pos, y_pos, name in (
        (0.56, 0.92, "front_right_entry_post"),
        (-0.56, 0.92, "front_left_entry_post"),
        (0.56, -0.92, "rear_right_entry_post"),
        (-0.56, -0.92, "rear_left_entry_post"),
    ):
        frame.visual(
            Box((0.08, 0.08, post_height)),
            origin=Origin(xyz=(x_pos, y_pos, post_center_z)),
            material=aluminum,
            name=name,
        )

    rotor = model.part("rotor")
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.16, length=2.28),
        mass=120.0,
        origin=Origin(xyz=(0.0, 0.0, 1.18)),
    )
    rotor.visual(
        Cylinder(radius=0.07, length=2.16),
        origin=Origin(xyz=(0.0, 0.0, 1.18)),
        material=dark_aluminum,
        name="central_shaft",
    )
    rotor.visual(
        Cylinder(radius=0.13, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=dark_aluminum,
        name="lower_hub",
    )
    rotor.visual(
        Cylinder(radius=0.13, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 2.23)),
        material=dark_aluminum,
        name="upper_hub",
    )
    for prefix, sign in (("front", 1.0), ("rear", -1.0)):
        rotor.visual(
            Box((0.014, 0.92, 1.94)),
            origin=Origin(xyz=(0.0, sign * 0.46, 1.15)),
            material=clear_glass,
            name=f"{prefix}_wing_glass",
        )
        rotor.visual(
            Box((0.046, 0.92, 0.04)),
            origin=Origin(xyz=(0.0, sign * 0.46, 0.12)),
            material=aluminum,
            name=f"{prefix}_bottom_rail",
        )
        rotor.visual(
            Box((0.046, 0.92, 0.04)),
            origin=Origin(xyz=(0.0, sign * 0.46, 2.18)),
            material=aluminum,
            name=f"{prefix}_top_rail",
        )
        rotor.visual(
            Box((0.046, 0.05, 2.10)),
            origin=Origin(xyz=(0.0, sign * 0.90, 1.15)),
            material=aluminum,
            name=f"{prefix}_outer_stile",
        )
        rotor.visual(
            Box((0.026, 0.92, 0.02)),
            origin=Origin(xyz=(0.0, sign * 0.46, 0.09)),
            material=seal_black,
            name=f"{prefix}_bottom_sweep",
        )

    left_outer_leaf = model.part("left_outer_leaf")
    left_outer_leaf.inertial = Inertial.from_geometry(
        Box((0.40, 0.05, 2.20)),
        mass=38.0,
        origin=Origin(xyz=(0.20, 0.0, 1.10)),
    )
    _add_fold_leaf(
        left_outer_leaf,
        width=0.40,
        height=2.20,
        direction=1.0,
        metal=aluminum,
        glass=clear_glass,
    )

    left_inner_leaf = model.part("left_inner_leaf")
    left_inner_leaf.inertial = Inertial.from_geometry(
        Box((0.40, 0.05, 2.20)),
        mass=34.0,
        origin=Origin(xyz=(0.20, 0.0, 1.10)),
    )
    _add_fold_leaf(
        left_inner_leaf,
        width=0.40,
        height=2.20,
        direction=1.0,
        metal=aluminum,
        glass=clear_glass,
    )

    right_outer_leaf = model.part("right_outer_leaf")
    right_outer_leaf.inertial = Inertial.from_geometry(
        Box((0.40, 0.05, 2.20)),
        mass=38.0,
        origin=Origin(xyz=(-0.20, 0.0, 1.10)),
    )
    _add_fold_leaf(
        right_outer_leaf,
        width=0.40,
        height=2.20,
        direction=-1.0,
        metal=aluminum,
        glass=clear_glass,
    )

    right_inner_leaf = model.part("right_inner_leaf")
    right_inner_leaf.inertial = Inertial.from_geometry(
        Box((0.40, 0.05, 2.20)),
        mass=34.0,
        origin=Origin(xyz=(-0.20, 0.0, 1.10)),
    )
    _add_fold_leaf(
        right_inner_leaf,
        width=0.40,
        height=2.20,
        direction=-1.0,
        metal=aluminum,
        glass=clear_glass,
    )

    model.articulation(
        "frame_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.8),
    )
    model.articulation(
        "frame_to_left_outer_leaf",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_outer_leaf,
        origin=Origin(xyz=(-2.10, 0.0, 0.08)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.2,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "left_outer_to_inner_leaf",
        ArticulationType.REVOLUTE,
        parent=left_outer_leaf,
        child=left_inner_leaf,
        origin=Origin(xyz=(0.40, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.6,
            lower=0.0,
            upper=2.90,
        ),
    )
    model.articulation(
        "frame_to_right_outer_leaf",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_outer_leaf,
        origin=Origin(xyz=(2.10, 0.0, 0.08)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.2,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "right_outer_to_inner_leaf",
        ArticulationType.REVOLUTE,
        parent=right_outer_leaf,
        child=right_inner_leaf,
        origin=Origin(xyz=(-0.40, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.6,
            lower=0.0,
            upper=2.90,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    left_outer = object_model.get_part("left_outer_leaf")
    left_inner = object_model.get_part("left_inner_leaf")
    right_outer = object_model.get_part("right_outer_leaf")
    right_inner = object_model.get_part("right_inner_leaf")

    drum_joint = object_model.get_articulation("frame_to_rotor")
    left_outer_joint = object_model.get_articulation("frame_to_left_outer_leaf")
    left_inner_joint = object_model.get_articulation("left_outer_to_inner_leaf")
    right_outer_joint = object_model.get_articulation("frame_to_right_outer_leaf")
    right_inner_joint = object_model.get_articulation("right_outer_to_inner_leaf")

    ctx.check(
        "continuous drum joint is vertical",
        drum_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(drum_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={drum_joint.articulation_type}, axis={drum_joint.axis}",
    )
    ctx.check(
        "mirrored bypass outer hinge axes",
        tuple(left_outer_joint.axis) == (0.0, 0.0, 1.0)
        and tuple(right_outer_joint.axis) == (0.0, 0.0, -1.0),
        details=f"left={left_outer_joint.axis}, right={right_outer_joint.axis}",
    )
    ctx.check(
        "mirrored bypass inner hinge axes",
        tuple(left_inner_joint.axis) == (0.0, 0.0, -1.0)
        and tuple(right_inner_joint.axis) == (0.0, 0.0, 1.0),
        details=f"left={left_inner_joint.axis}, right={right_inner_joint.axis}",
    )

    ctx.expect_contact(
        left_outer,
        frame,
        elem_a="hinge_stile",
        elem_b="left_outer_jamb",
        name="left outer leaf seats at its jamb",
    )
    ctx.expect_contact(
        right_outer,
        frame,
        elem_a="hinge_stile",
        elem_b="right_outer_jamb",
        name="right outer leaf seats at its jamb",
    )
    ctx.expect_contact(
        left_outer,
        left_inner,
        elem_a="knuckle_stile",
        elem_b="hinge_stile",
        name="left bypass leaves meet at their knuckle",
    )
    ctx.expect_contact(
        right_outer,
        right_inner,
        elem_a="knuckle_stile",
        elem_b="hinge_stile",
        name="right bypass leaves meet at their knuckle",
    )

    rest_wing = ctx.part_element_world_aabb(rotor, elem="front_wing_glass")
    with ctx.pose({drum_joint: math.pi * 0.5}):
        quarter_turn_wing = ctx.part_element_world_aabb(rotor, elem="front_wing_glass")
    rest_x = _span(rest_wing, "x")
    rest_y = _span(rest_wing, "y")
    turn_x = _span(quarter_turn_wing, "x")
    turn_y = _span(quarter_turn_wing, "y")
    ctx.check(
        "drum quarter turn swings the wing across the plan",
        rest_x is not None
        and rest_y is not None
        and turn_x is not None
        and turn_y is not None
        and rest_y > 0.85
        and rest_x < 0.08
        and turn_x > 0.85
        and turn_y < 0.08,
        details=f"rest=({rest_x}, {rest_y}), quarter=({turn_x}, {turn_y})",
    )

    with ctx.pose(
        {
            left_outer_joint: 1.20,
            left_inner_joint: 2.30,
            right_outer_joint: 1.20,
            right_inner_joint: 2.30,
        }
    ):
        left_outer_aabb = ctx.part_world_aabb(left_outer)
        left_inner_aabb = ctx.part_world_aabb(left_inner)
        right_outer_aabb = ctx.part_world_aabb(right_outer)
        right_inner_aabb = ctx.part_world_aabb(right_inner)

    opened_y = []
    for aabb in (left_outer_aabb, left_inner_aabb, right_outer_aabb, right_inner_aabb):
        if aabb is not None:
            opened_y.append(aabb[1][1])
    ctx.check(
        "all bypass leaves swing out of the closed wall plane",
        len(opened_y) == 4 and min(opened_y) > 0.18,
        details=f"opened_max_y={opened_y}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
