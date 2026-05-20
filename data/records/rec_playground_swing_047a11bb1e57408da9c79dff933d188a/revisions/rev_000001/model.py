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


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="strap_swing")

    beam_wood = model.material("beam_wood", rgba=(0.55, 0.41, 0.26, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.24, 0.26, 1.0))
    zinc = model.material("zinc", rgba=(0.72, 0.74, 0.77, 1.0))
    seat_black = model.material("seat_black", rgba=(0.10, 0.10, 0.11, 1.0))

    seat_profile = rounded_rect_profile(0.46, 0.19, radius=0.022, corner_segments=8)
    seat_panel_mesh = _mesh(
        "swing_seat_panel",
        ExtrudeGeometry.centered(seat_profile, 0.028, cap=True),
    )

    beam_mount = model.part("beam_mount")
    beam_mount.visual(
        Box((0.75, 0.09, 0.09)),
        material=beam_wood,
        name="beam_stub",
    )
    beam_mount.visual(
        Box((0.16, 0.08, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.054)),
        material=dark_steel,
        name="hanger_plate",
    )
    beam_mount.visual(
        Cylinder(radius=0.024, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, -0.081)),
        material=zinc,
        name="bearing_housing",
    )
    beam_mount.inertial = Inertial.from_geometry(
        Box((0.75, 0.10, 0.13)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
    )

    swivel_block = model.part("swivel_block")
    swivel_block.visual(
        Cylinder(radius=0.026, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=zinc,
        name="swivel_cap",
    )
    swivel_block.visual(
        Box((0.08, 0.045, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.033)),
        material=dark_steel,
        name="swivel_body",
    )
    swivel_block.visual(
        Box((0.06, 0.010, 0.05)),
        origin=Origin(xyz=(0.0, 0.0225, -0.070)),
        material=dark_steel,
        name="front_cheek",
    )
    swivel_block.visual(
        Box((0.06, 0.010, 0.05)),
        origin=Origin(xyz=(0.0, -0.0225, -0.070)),
        material=dark_steel,
        name="rear_cheek",
    )
    swivel_block.inertial = Inertial.from_geometry(
        Box((0.09, 0.06, 0.10)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
    )

    hanger_frame = model.part("hanger_frame")
    hanger_frame.visual(
        Cylinder(radius=0.0175, length=0.32),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=zinc,
        name="top_crosshead",
    )
    hanger_frame.visual(
        Box((0.028, 0.012, 0.30)),
        origin=Origin(xyz=(0.17, 0.0, -0.1675)),
        material=zinc,
        name="left_side_link",
    )
    hanger_frame.visual(
        Box((0.028, 0.012, 0.30)),
        origin=Origin(xyz=(-0.17, 0.0, -0.1675)),
        material=zinc,
        name="right_side_link",
    )
    hanger_frame.visual(
        Box((0.04, 0.024, 0.04)),
        origin=Origin(xyz=(0.17, 0.0, -0.3375)),
        material=dark_steel,
        name="left_lower_block",
    )
    hanger_frame.visual(
        Box((0.04, 0.024, 0.04)),
        origin=Origin(xyz=(-0.17, 0.0, -0.3375)),
        material=dark_steel,
        name="right_lower_block",
    )
    hanger_frame.inertial = Inertial.from_geometry(
        Box((0.38, 0.05, 0.38)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, -0.19)),
    )

    seat = model.part("seat")
    seat.visual(
        seat_panel_mesh,
        material=seat_black,
        name="seat_panel",
    )
    seat.visual(
        Box((0.020, 0.024, 0.090)),
        origin=Origin(xyz=(0.17, 0.0, 0.059)),
        material=dark_steel,
        name="left_seat_bracket",
    )
    seat.visual(
        Box((0.020, 0.024, 0.090)),
        origin=Origin(xyz=(-0.17, 0.0, 0.059)),
        material=dark_steel,
        name="right_seat_bracket",
    )
    seat.inertial = Inertial.from_geometry(
        Box((0.46, 0.19, 0.12)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    model.articulation(
        "beam_to_swivel",
        ArticulationType.CONTINUOUS,
        parent=beam_mount,
        child=swivel_block,
        origin=Origin(xyz=(0.0, 0.0, -0.099)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0),
    )
    model.articulation(
        "swivel_to_hanger",
        ArticulationType.REVOLUTE,
        parent=swivel_block,
        child=hanger_frame,
        origin=Origin(xyz=(0.0, 0.0, -0.077)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.0, lower=-0.55, upper=0.85),
    )
    model.articulation(
        "hanger_to_seat",
        ArticulationType.FIXED,
        parent=hanger_frame,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, -0.4615)),
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

    beam_mount = object_model.get_part("beam_mount")
    swivel_block = object_model.get_part("swivel_block")
    hanger_frame = object_model.get_part("hanger_frame")
    seat = object_model.get_part("seat")

    swivel_joint = object_model.get_articulation("beam_to_swivel")
    swing_joint = object_model.get_articulation("swivel_to_hanger")

    ctx.check(
        "swivel uses vertical axis",
        tuple(swivel_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={swivel_joint.axis}",
    )
    ctx.check(
        "hanger swing uses lateral axis",
        tuple(swing_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={swing_joint.axis}",
    )

    with ctx.pose({swivel_joint: 0.0, swing_joint: 0.0}):
        ctx.expect_contact(
            seat,
            hanger_frame,
            elem_a="left_seat_bracket",
            elem_b="left_lower_block",
            name="left bracket is mounted to left hanger block",
        )
        ctx.expect_contact(
            seat,
            hanger_frame,
            elem_a="right_seat_bracket",
            elem_b="right_lower_block",
            name="right bracket is mounted to right hanger block",
        )
        ctx.expect_origin_gap(
            swivel_block,
            seat,
            axis="z",
            min_gap=0.45,
            name="seat hangs well below the swivel block",
        )

        rest_pos = ctx.part_world_position(seat)

    with ctx.pose({swivel_joint: 0.0, swing_joint: 0.55}):
        swung_pos = ctx.part_world_position(seat)

    ctx.check(
        "positive hanger swing moves seat forward and upward",
        rest_pos is not None
        and swung_pos is not None
        and swung_pos[1] > rest_pos[1] + 0.10
        and swung_pos[2] > rest_pos[2] + 0.05,
        details=f"rest={rest_pos}, swung={swung_pos}",
    )

    with ctx.pose({swivel_joint: 0.0, swing_joint: 0.0}):
        rest_aabb = ctx.part_world_aabb(seat)
    with ctx.pose({swivel_joint: pi / 2.0, swing_joint: 0.0}):
        yawed_aabb = ctx.part_world_aabb(seat)

    rest_dx = None if rest_aabb is None else rest_aabb[1][0] - rest_aabb[0][0]
    rest_dy = None if rest_aabb is None else rest_aabb[1][1] - rest_aabb[0][1]
    yawed_dx = None if yawed_aabb is None else yawed_aabb[1][0] - yawed_aabb[0][0]
    yawed_dy = None if yawed_aabb is None else yawed_aabb[1][1] - yawed_aabb[0][1]

    ctx.check(
        "swivel yaw rotates the seat footprint",
        rest_dx is not None
        and rest_dy is not None
        and yawed_dx is not None
        and yawed_dy is not None
        and yawed_dy > rest_dy + 0.15
        and yawed_dx < rest_dx - 0.15,
        details=(
            f"rest_dx={rest_dx}, rest_dy={rest_dy}, "
            f"yawed_dx={yawed_dx}, yawed_dy={yawed_dy}"
        ),
    )

    ctx.expect_contact(
        swivel_block,
        beam_mount,
        elem_a="swivel_cap",
        elem_b="bearing_housing",
        name="swivel block seats against the bearing housing",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
