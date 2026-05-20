from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


CABINET_WIDTH = 0.62
CABINET_HEIGHT = 0.82
CABINET_DEPTH = 0.22
SHELL_WALL = 0.008
DOOR_WIDTH = 0.602
DOOR_HEIGHT = 0.798
DOOR_SKIN_THICKNESS = 0.004
DOOR_RETURN_DEPTH = 0.020
HINGE_AXIS_X = -0.301
HINGE_AXIS_Y = 0.222
HINGE_AXIS_Z = CABINET_HEIGHT * 0.5
DOOR_OPEN_ANGLE = 2.05
LATCH_TURN_ANGLE = pi * 0.5


def _rounded_panel_mesh(name: str, width: float, height: float, thickness: float, radius: float):
    panel = ExtrudeGeometry.centered(
        rounded_rect_profile(width, height, radius, corner_segments=8),
        thickness,
        cap=True,
        closed=True,
    ).rotate_x(pi / 2.0)
    return mesh_from_geometry(panel, name)


def _aabb_extent(aabb, axis_index: int) -> float | None:
    if aabb is None:
        return None
    return aabb[1][axis_index] - aabb[0][axis_index]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_grp_electrical_cabinet")

    grp_body = model.material("grp_body", rgba=(0.86, 0.87, 0.84, 1.0))
    grp_door = model.material("grp_door", rgba=(0.90, 0.90, 0.87, 1.0))
    galvanized = model.material("galvanized", rgba=(0.62, 0.65, 0.68, 1.0))
    mounting_panel = model.material("mounting_panel", rgba=(0.79, 0.80, 0.82, 1.0))
    gasket = model.material("gasket", rgba=(0.12, 0.12, 0.12, 1.0))
    latch_black = model.material("latch_black", rgba=(0.10, 0.10, 0.11, 1.0))

    bracket = model.part("wall_bracket")
    bracket.inertial = Inertial.from_geometry(
        Box((0.46, 0.07, 0.70)),
        mass=9.0,
        origin=Origin(xyz=(0.0, -0.035, 0.41)),
    )
    bracket.visual(
        Box((0.46, 0.008, 0.70)),
        origin=Origin(xyz=(0.0, -0.066, 0.41)),
        material=galvanized,
        name="wall_plate",
    )
    for side_x in (-0.17, 0.17):
        bracket.visual(
            Box((0.05, 0.016, 0.64)),
            origin=Origin(xyz=(side_x, -0.008, 0.41)),
            material=galvanized,
            name=f"mount_rail_{'left' if side_x < 0.0 else 'right'}",
        )
        for z_pos, suffix in ((0.20, "lower"), (0.62, "upper")):
            bracket.visual(
                Box((0.05, 0.054, 0.04)),
                origin=Origin(xyz=(side_x, -0.035, z_pos)),
                material=galvanized,
                name=f"standoff_{'left' if side_x < 0.0 else 'right'}_{suffix}",
            )
            bracket.visual(
                Box((0.018, 0.044, 0.075)),
                origin=Origin(
                    xyz=(side_x, -0.043, z_pos - 0.002),
                    rpy=((pi / 7.5) * (-1.0 if z_pos > 0.4 else 1.0), 0.0, 0.0),
                ),
                material=galvanized,
                name=f"gusset_{'left' if side_x < 0.0 else 'right'}_{suffix}",
            )

    cabinet = model.part("cabinet_body")
    cabinet.inertial = Inertial.from_geometry(
        Box((CABINET_WIDTH, CABINET_DEPTH, CABINET_HEIGHT)),
        mass=28.0,
        origin=Origin(xyz=(0.0, CABINET_DEPTH * 0.5, CABINET_HEIGHT * 0.5)),
    )
    cabinet.visual(
        Box((0.612, 0.008, 0.816)),
        origin=Origin(xyz=(0.0, 0.004, 0.408)),
        material=grp_body,
        name="back_panel",
    )
    cabinet.visual(
        Box((SHELL_WALL, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(xyz=(-0.306, 0.11, 0.41)),
        material=grp_body,
        name="side_left",
    )
    cabinet.visual(
        Box((SHELL_WALL, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(xyz=(0.306, 0.11, 0.41)),
        material=grp_body,
        name="side_right",
    )
    cabinet.visual(
        Box((CABINET_WIDTH, CABINET_DEPTH, SHELL_WALL)),
        origin=Origin(xyz=(0.0, 0.11, 0.816)),
        material=grp_body,
        name="top_shell",
    )
    cabinet.visual(
        Box((CABINET_WIDTH, CABINET_DEPTH, SHELL_WALL)),
        origin=Origin(xyz=(0.0, 0.11, 0.004)),
        material=grp_body,
        name="bottom_shell",
    )
    cabinet.visual(
        Box((0.030, 0.012, 0.756)),
        origin=Origin(xyz=(-0.295, 0.214, 0.41)),
        material=gasket,
        name="seal_left",
    )
    cabinet.visual(
        Box((0.030, 0.012, 0.756)),
        origin=Origin(xyz=(0.295, 0.214, 0.41)),
        material=gasket,
        name="seal_right",
    )
    cabinet.visual(
        Box((0.560, 0.012, 0.032)),
        origin=Origin(xyz=(0.0, 0.214, 0.804)),
        material=gasket,
        name="seal_top",
    )
    cabinet.visual(
        Box((0.560, 0.012, 0.032)),
        origin=Origin(xyz=(0.0, 0.214, 0.016)),
        material=gasket,
        name="seal_bottom",
    )
    cabinet.visual(
        Box((0.66, 0.030, 0.010)),
        origin=Origin(xyz=(0.0, 0.226, 0.821)),
        material=grp_body,
        name="rain_hood",
    )
    cabinet.visual(
        Box((0.54, 0.006, 0.72)),
        origin=Origin(xyz=(0.0, 0.047, 0.41)),
        material=mounting_panel,
        name="inner_mounting_panel",
    )
    for x_pos, z_pos, name in (
        (-0.24, 0.72, "panel_standoff_tl"),
        (0.24, 0.72, "panel_standoff_tr"),
        (-0.24, 0.10, "panel_standoff_bl"),
        (0.24, 0.10, "panel_standoff_br"),
    ):
        cabinet.visual(
            Box((0.028, 0.044, 0.028)),
            origin=Origin(xyz=(x_pos, 0.026, z_pos)),
            material=mounting_panel,
            name=name,
        )
    for z_pos, suffix in ((0.14, "lower"), (0.68, "upper")):
        cabinet.visual(
            Cylinder(radius=0.011, length=0.10),
            origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, z_pos)),
            material=galvanized,
            name=f"hinge_barrel_{suffix}",
        )
        cabinet.visual(
            Box((0.018, 0.040, 0.082)),
            origin=Origin(xyz=(-0.296, 0.201, z_pos)),
            material=galvanized,
            name=f"hinge_leaf_{suffix}",
        )

    door = model.part("door")
    door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, 0.030, DOOR_HEIGHT)),
        mass=7.5,
        origin=Origin(xyz=(DOOR_WIDTH * 0.5, 0.015, 0.0)),
    )
    door.visual(
        _rounded_panel_mesh("door_skin", DOOR_WIDTH, DOOR_HEIGHT, DOOR_SKIN_THICKNESS, 0.020),
        origin=Origin(xyz=(DOOR_WIDTH * 0.5, 0.022, 0.0)),
        material=grp_door,
        name="door_skin",
    )
    door.visual(
        _rounded_panel_mesh("door_raised_panel", 0.480, 0.650, 0.002, 0.015),
        origin=Origin(xyz=(DOOR_WIDTH * 0.5, 0.025, 0.0)),
        material=grp_body,
        name="door_raised_panel",
    )
    door.visual(
        Box((0.566, DOOR_RETURN_DEPTH, 0.018)),
        origin=Origin(xyz=(DOOR_WIDTH * 0.5, 0.010, 0.390)),
        material=grp_door,
        name="door_frame_top",
    )
    door.visual(
        Box((0.566, DOOR_RETURN_DEPTH, 0.018)),
        origin=Origin(xyz=(DOOR_WIDTH * 0.5, 0.010, -0.390)),
        material=grp_door,
        name="door_frame_bottom",
    )
    door.visual(
        Box((0.018, DOOR_RETURN_DEPTH, 0.762)),
        origin=Origin(xyz=(0.019, 0.010, 0.0)),
        material=grp_door,
        name="door_frame_left",
    )
    door.visual(
        Box((0.018, DOOR_RETURN_DEPTH, 0.762)),
        origin=Origin(xyz=(0.593, 0.010, 0.0)),
        material=grp_door,
        name="door_frame_right",
    )
    door.visual(
        Box((0.060, 0.008, 0.180)),
        origin=Origin(xyz=(0.552, 0.016, 0.0)),
        material=grp_body,
        name="latch_pad",
    )
    for z_pos, suffix in ((0.27, "lower"), (-0.27, "upper")):
        door.visual(
            Box((0.016, 0.020, 0.10)),
            origin=Origin(xyz=(0.019, 0.010, z_pos)),
            material=galvanized,
            name=f"door_hinge_leaf_{suffix}",
        )

    latch = model.part("cam_latch")
    latch.inertial = Inertial.from_geometry(
        Box((0.10, 0.04, 0.10)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.012, 0.0)),
    )
    latch.visual(
        Cylinder(radius=0.021, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=latch_black,
        name="latch_rosette",
    )
    latch.visual(
        Cylinder(radius=0.007, length=0.020),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="latch_spindle",
    )
    latch.visual(
        Box((0.024, 0.014, 0.092)),
        origin=Origin(xyz=(0.0, 0.014, 0.0)),
        material=latch_black,
        name="latch_handle",
    )
    latch.visual(
        Box((0.052, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.024, -0.028)),
        material=latch_black,
        name="finger_grip",
    )

    model.articulation(
        "bracket_to_cabinet",
        ArticulationType.FIXED,
        parent=bracket,
        child=cabinet,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=0.0,
            upper=DOOR_OPEN_ANGLE,
        ),
    )
    model.articulation(
        "door_to_cam_latch",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(0.556, 0.024, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.0,
            lower=0.0,
            upper=LATCH_TURN_ANGLE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("wall_bracket")
    cabinet = object_model.get_part("cabinet_body")
    door = object_model.get_part("door")
    latch = object_model.get_part("cam_latch")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    latch_joint = object_model.get_articulation("door_to_cam_latch")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        cabinet,
        bracket,
        elem_a="back_panel",
        elem_b="mount_rail_left",
        name="cabinet back panel is mounted to the wall bracket rail",
    )
    ctx.expect_contact(
        latch,
        door,
        elem_a="latch_rosette",
        elem_b="door_skin",
        name="cam latch is mounted on the door skin",
    )

    ctx.check(
        "door hinge is a vertical left-edge revolute joint",
        tuple(door_hinge.axis) == (0.0, 0.0, 1.0)
        and door_hinge.motion_limits is not None
        and abs((door_hinge.motion_limits.lower or 0.0) - 0.0) < 1e-9
        and door_hinge.motion_limits.upper is not None
        and door_hinge.motion_limits.upper >= 1.9,
        details=f"axis={door_hinge.axis}, limits={door_hinge.motion_limits}",
    )
    ctx.check(
        "cam latch has quarter-turn rotation about door normal",
        tuple(latch_joint.axis) == (0.0, 1.0, 0.0)
        and latch_joint.motion_limits is not None
        and abs((latch_joint.motion_limits.lower or 0.0) - 0.0) < 1e-9
        and latch_joint.motion_limits.upper is not None
        and abs(latch_joint.motion_limits.upper - (pi * 0.5)) < 1e-9,
        details=f"axis={latch_joint.axis}, limits={latch_joint.motion_limits}",
    )

    with ctx.pose({door_hinge: 0.0, latch_joint: 0.0}):
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            positive_elem="door_frame_right",
            negative_elem="seal_right",
            min_gap=0.0005,
            max_gap=0.004,
            name="closed door sits just proud of the cabinet seal at the latch edge",
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="xz",
            min_overlap=0.55,
            elem_a="door_skin",
            elem_b="back_panel",
            name="closed door covers the cabinet opening footprint",
        )
        closed_door_skin = ctx.part_element_world_aabb(door, elem="door_skin")
        closed_latch_handle = ctx.part_element_world_aabb(latch, elem="latch_handle")

    with ctx.pose({door_hinge: DOOR_OPEN_ANGLE, latch_joint: 0.0}):
        open_door_skin = ctx.part_element_world_aabb(door, elem="door_skin")

    with ctx.pose({door_hinge: 0.0, latch_joint: LATCH_TURN_ANGLE}):
        turned_latch_handle = ctx.part_element_world_aabb(latch, elem="latch_handle")

    ctx.check(
        "door swings outward on the hinge axis",
        closed_door_skin is not None
        and open_door_skin is not None
        and open_door_skin[1][1] > closed_door_skin[1][1] + 0.30,
        details=f"closed={closed_door_skin}, open={open_door_skin}",
    )

    closed_latch_dx = _aabb_extent(closed_latch_handle, 0)
    closed_latch_dz = _aabb_extent(closed_latch_handle, 2)
    turned_latch_dx = _aabb_extent(turned_latch_handle, 0)
    turned_latch_dz = _aabb_extent(turned_latch_handle, 2)
    ctx.check(
        "quarter-turn latch rotates from vertical to transverse",
        closed_latch_dx is not None
        and closed_latch_dz is not None
        and turned_latch_dx is not None
        and turned_latch_dz is not None
        and closed_latch_dz > closed_latch_dx
        and turned_latch_dx > turned_latch_dz
        and turned_latch_dx > closed_latch_dx + 0.04,
        details=(
            f"closed_dx={closed_latch_dx}, closed_dz={closed_latch_dz}, "
            f"turned_dx={turned_latch_dx}, turned_dz={turned_latch_dz}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
