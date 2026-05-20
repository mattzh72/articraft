from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BODY_W = 0.86
BODY_D = 0.50
BODY_H = 0.72
CONSOLE_H = 0.12
WALL_T = 0.024
FLOOR_T = 0.020
TOP_T = 0.030

WASH_CENTER_X = -0.19
SPIN_CENTER_X = 0.19

WASH_OPEN_R = 0.14
SPIN_OPEN_R = 0.12

WASH_DRUM_OUTER_R = 0.118
WASH_DRUM_INNER_R = 0.108
WASH_DRUM_HEIGHT = 0.53
WASH_HUB_R = 0.038
WASH_HUB_H = 0.050
WASH_PEDESTAL_R = 0.045
WASH_PEDESTAL_H = 0.100

SPIN_DRUM_OUTER_R = 0.100
SPIN_DRUM_INNER_R = 0.090
SPIN_DRUM_HEIGHT = 0.49
SPIN_HUB_R = 0.034
SPIN_HUB_H = 0.045
SPIN_PEDESTAL_R = 0.040
SPIN_PEDESTAL_H = 0.095

LID_FRAME_T = 0.016
HINGE_BARREL_R = 0.016

WASH_LID_OUTER_R = 0.155
WASH_LID_WINDOW_R = 0.118
WASH_BARREL_LEN = 0.180
WASH_HINGE_X = WASH_CENTER_X - WASH_LID_OUTER_R

SPIN_LID_OUTER_R = 0.135
SPIN_LID_WINDOW_R = 0.098
SPIN_BARREL_LEN = 0.160
SPIN_HINGE_X = SPIN_CENTER_X - SPIN_LID_OUTER_R


def _circle_profile(radius: float, segments: int = 56) -> list[tuple[float, float]]:
    return [
        (radius * cos((2.0 * pi * i) / segments), radius * sin((2.0 * pi * i) / segments))
        for i in range(segments)
    ]


def _offset_profile(
    profile: list[tuple[float, float]], dx: float, dy: float
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _drum_mesh(name: str, outer_r: float, inner_r: float, height: float):
    wall_start = 0.028
    bottom_t = 0.012
    lip_t = 0.010

    shell = ExtrudeWithHolesGeometry(
        _circle_profile(outer_r),
        [_circle_profile(inner_r)],
        height - wall_start,
        center=False,
    )
    shell.translate(0.0, 0.0, wall_start)

    bottom = ExtrudeGeometry.from_z0(_circle_profile(inner_r), bottom_t)
    bottom.translate(0.0, 0.0, wall_start)
    shell.merge(bottom)

    lip = ExtrudeWithHolesGeometry(
        _circle_profile(outer_r + 0.006),
        [_circle_profile(inner_r)],
        lip_t,
        center=False,
    )
    lip.translate(0.0, 0.0, height - lip_t)
    shell.merge(lip)

    return mesh_from_geometry(shell, name)


def _lid_frame_mesh(name: str, outer_r: float, window_r: float):
    frame = ExtrudeWithHolesGeometry(
        _circle_profile(outer_r),
        [_circle_profile(window_r)],
        LID_FRAME_T,
        center=False,
    )
    frame.translate(outer_r, 0.0, -LID_FRAME_T)
    return mesh_from_geometry(frame, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_tub_washing_machine")

    housing = model.material("housing_white", rgba=(0.93, 0.94, 0.96, 1.0))
    trim = model.material("trim_gray", rgba=(0.63, 0.67, 0.72, 1.0))
    console = model.material("console_gray", rgba=(0.44, 0.48, 0.53, 1.0))
    drum_metal = model.material("drum_metal", rgba=(0.77, 0.80, 0.83, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.22, 0.24, 0.27, 1.0))
    lid_tint = model.material("lid_tint", rgba=(0.72, 0.84, 0.95, 0.45))

    body = model.part("body")

    wall_height = BODY_H - TOP_T
    body.visual(
        Box((BODY_W - 2.0 * WALL_T, BODY_D - 2.0 * WALL_T, FLOOR_T)),
        origin=Origin(xyz=(0.0, 0.0, FLOOR_T / 2.0)),
        material=trim,
        name="cabinet_floor",
    )
    body.visual(
        Box((BODY_W, WALL_T, wall_height)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - WALL_T / 2.0, wall_height / 2.0)),
        material=housing,
        name="rear_wall",
    )
    body.visual(
        Box((BODY_W, WALL_T, wall_height)),
        origin=Origin(xyz=(0.0, -BODY_D / 2.0 + WALL_T / 2.0, wall_height / 2.0)),
        material=housing,
        name="front_wall",
    )
    body.visual(
        Box((WALL_T, BODY_D, wall_height)),
        origin=Origin(xyz=(-BODY_W / 2.0 + WALL_T / 2.0, 0.0, wall_height / 2.0)),
        material=housing,
        name="left_wall",
    )
    body.visual(
        Box((WALL_T, BODY_D, wall_height)),
        origin=Origin(xyz=(BODY_W / 2.0 - WALL_T / 2.0, 0.0, wall_height / 2.0)),
        material=housing,
        name="right_wall",
    )
    body.visual(
        Box((0.036, BODY_D - 2.0 * WALL_T, wall_height - FLOOR_T)),
        origin=Origin(xyz=(0.0, 0.0, FLOOR_T + (wall_height - FLOOR_T) / 2.0)),
        material=trim,
        name="center_divider",
    )

    top_deck = ExtrudeWithHolesGeometry(
        rounded_rect_profile(BODY_W, BODY_D, 0.038, corner_segments=10),
        [
            _offset_profile(_circle_profile(WASH_OPEN_R), WASH_CENTER_X, 0.0),
            _offset_profile(_circle_profile(SPIN_OPEN_R), SPIN_CENTER_X, 0.0),
        ],
        TOP_T,
        center=False,
    )
    top_deck.translate(0.0, 0.0, BODY_H - TOP_T)
    body.visual(
        mesh_from_geometry(top_deck, "top_deck"),
        material=housing,
        name="top_deck",
    )
    body.visual(
        Box((BODY_W * 0.92, 0.06, CONSOLE_H)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - 0.03, BODY_H + CONSOLE_H / 2.0)),
        material=console,
        name="control_console",
    )

    body.visual(
        Cylinder(radius=WASH_PEDESTAL_R, length=WASH_PEDESTAL_H),
        origin=Origin(xyz=(WASH_CENTER_X, 0.0, FLOOR_T + WASH_PEDESTAL_H / 2.0)),
        material=dark_trim,
        name="wash_pedestal",
    )
    body.visual(
        Cylinder(radius=SPIN_PEDESTAL_R, length=SPIN_PEDESTAL_H),
        origin=Origin(xyz=(SPIN_CENTER_X, 0.0, FLOOR_T + SPIN_PEDESTAL_H / 2.0)),
        material=dark_trim,
        name="spin_pedestal",
    )

    hinge_block_size = (0.024, 0.008, 0.040)
    for prefix, hinge_x, barrel_len in (
        ("wash", WASH_HINGE_X, WASH_BARREL_LEN),
        ("spin", SPIN_HINGE_X, SPIN_BARREL_LEN),
    ):
        for suffix, y_sign in (("front", -1.0), ("rear", 1.0)):
            body.visual(
                Box(hinge_block_size),
                origin=Origin(
                    xyz=(
                        hinge_x,
                        y_sign * (barrel_len / 2.0 + hinge_block_size[1] / 2.0),
                        BODY_H + HINGE_BARREL_R,
                    )
                ),
                material=trim,
                name=f"{prefix}_hinge_{suffix}_block",
            )

    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H + CONSOLE_H)),
        mass=29.0,
        origin=Origin(xyz=(0.0, 0.0, (BODY_H + CONSOLE_H) / 2.0)),
    )

    wash_drum = model.part("wash_drum")
    wash_drum.visual(
        _drum_mesh("wash_basket", WASH_DRUM_OUTER_R, WASH_DRUM_INNER_R, WASH_DRUM_HEIGHT),
        material=drum_metal,
        name="wash_basket",
    )
    wash_drum.visual(
        Cylinder(radius=WASH_HUB_R, length=WASH_HUB_H),
        origin=Origin(xyz=(0.0, 0.0, WASH_HUB_H / 2.0)),
        material=dark_trim,
        name="wash_hub",
    )
    wash_drum.inertial = Inertial.from_geometry(
        Cylinder(radius=WASH_DRUM_OUTER_R, length=WASH_DRUM_HEIGHT),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, WASH_DRUM_HEIGHT / 2.0)),
    )

    spin_drum = model.part("spin_drum")
    spin_drum.visual(
        _drum_mesh("spin_basket", SPIN_DRUM_OUTER_R, SPIN_DRUM_INNER_R, SPIN_DRUM_HEIGHT),
        material=drum_metal,
        name="spin_basket",
    )
    spin_drum.visual(
        Cylinder(radius=SPIN_HUB_R, length=SPIN_HUB_H),
        origin=Origin(xyz=(0.0, 0.0, SPIN_HUB_H / 2.0)),
        material=dark_trim,
        name="spin_hub",
    )
    spin_drum.inertial = Inertial.from_geometry(
        Cylinder(radius=SPIN_DRUM_OUTER_R, length=SPIN_DRUM_HEIGHT),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, SPIN_DRUM_HEIGHT / 2.0)),
    )

    wash_door = model.part("wash_door")
    wash_door.visual(
        _lid_frame_mesh("wash_door_frame", WASH_LID_OUTER_R, WASH_LID_WINDOW_R),
        material=trim,
        name="wash_door_frame",
    )
    wash_door.visual(
        Cylinder(radius=WASH_LID_WINDOW_R, length=LID_FRAME_T),
        origin=Origin(xyz=(WASH_LID_OUTER_R, 0.0, -LID_FRAME_T / 2.0)),
        material=lid_tint,
        name="wash_door_window",
    )
    wash_door.visual(
        Cylinder(radius=HINGE_BARREL_R, length=WASH_BARREL_LEN),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="wash_hinge_barrel",
    )
    wash_door.inertial = Inertial.from_geometry(
        Cylinder(radius=WASH_LID_OUTER_R, length=LID_FRAME_T),
        mass=0.7,
        origin=Origin(xyz=(WASH_LID_OUTER_R, 0.0, -LID_FRAME_T / 2.0)),
    )

    spin_door = model.part("spin_door")
    spin_door.visual(
        _lid_frame_mesh("spin_door_frame", SPIN_LID_OUTER_R, SPIN_LID_WINDOW_R),
        material=trim,
        name="spin_door_frame",
    )
    spin_door.visual(
        Cylinder(radius=SPIN_LID_WINDOW_R, length=LID_FRAME_T),
        origin=Origin(xyz=(SPIN_LID_OUTER_R, 0.0, -LID_FRAME_T / 2.0)),
        material=lid_tint,
        name="spin_door_window",
    )
    spin_door.visual(
        Cylinder(radius=HINGE_BARREL_R, length=SPIN_BARREL_LEN),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="spin_hinge_barrel",
    )
    spin_door.inertial = Inertial.from_geometry(
        Cylinder(radius=SPIN_LID_OUTER_R, length=LID_FRAME_T),
        mass=0.55,
        origin=Origin(xyz=(SPIN_LID_OUTER_R, 0.0, -LID_FRAME_T / 2.0)),
    )

    model.articulation(
        "body_to_wash_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wash_drum,
        origin=Origin(xyz=(WASH_CENTER_X, 0.0, FLOOR_T + WASH_PEDESTAL_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=6.0),
    )
    model.articulation(
        "body_to_spin_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=spin_drum,
        origin=Origin(xyz=(SPIN_CENTER_X, 0.0, FLOOR_T + SPIN_PEDESTAL_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=9.0),
    )
    model.articulation(
        "body_to_wash_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wash_door,
        origin=Origin(xyz=(WASH_HINGE_X, 0.0, BODY_H + HINGE_BARREL_R)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.45),
    )
    model.articulation(
        "body_to_spin_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=spin_door,
        origin=Origin(xyz=(SPIN_HINGE_X, 0.0, BODY_H + HINGE_BARREL_R)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    wash_drum = object_model.get_part("wash_drum")
    spin_drum = object_model.get_part("spin_drum")
    wash_door = object_model.get_part("wash_door")
    spin_door = object_model.get_part("spin_door")

    wash_axle = object_model.get_articulation("body_to_wash_drum")
    spin_axle = object_model.get_articulation("body_to_spin_drum")
    wash_hinge = object_model.get_articulation("body_to_wash_door")
    spin_hinge = object_model.get_articulation("body_to_spin_door")

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

    ctx.check(
        "wash drum uses a vertical continuous axle",
        wash_axle.articulation_type == ArticulationType.CONTINUOUS and wash_axle.axis == (0.0, 0.0, 1.0),
        details=f"type={wash_axle.articulation_type}, axis={wash_axle.axis}",
    )
    ctx.check(
        "spin drum uses a vertical continuous axle",
        spin_axle.articulation_type == ArticulationType.CONTINUOUS and spin_axle.axis == (0.0, 0.0, 1.0),
        details=f"type={spin_axle.articulation_type}, axis={spin_axle.axis}",
    )
    ctx.check(
        "wash door hinges on its left edge",
        wash_hinge.articulation_type == ArticulationType.REVOLUTE
        and wash_hinge.axis == (0.0, -1.0, 0.0)
        and wash_hinge.motion_limits is not None
        and wash_hinge.motion_limits.lower == 0.0
        and wash_hinge.motion_limits.upper is not None
        and wash_hinge.motion_limits.upper >= 1.3,
        details=f"type={wash_hinge.articulation_type}, axis={wash_hinge.axis}, limits={wash_hinge.motion_limits}",
    )
    ctx.check(
        "spin door hinges on its left edge",
        spin_hinge.articulation_type == ArticulationType.REVOLUTE
        and spin_hinge.axis == (0.0, -1.0, 0.0)
        and spin_hinge.motion_limits is not None
        and spin_hinge.motion_limits.lower == 0.0
        and spin_hinge.motion_limits.upper is not None
        and spin_hinge.motion_limits.upper >= 1.3,
        details=f"type={spin_hinge.articulation_type}, axis={spin_hinge.axis}, limits={spin_hinge.motion_limits}",
    )

    ctx.expect_contact(
        wash_drum,
        body,
        elem_a="wash_hub",
        elem_b="wash_pedestal",
        name="wash drum hub bears on the left pedestal",
    )
    ctx.expect_contact(
        spin_drum,
        body,
        elem_a="spin_hub",
        elem_b="spin_pedestal",
        name="spin drum hub bears on the right pedestal",
    )
    ctx.expect_contact(
        wash_door,
        body,
        elem_a="wash_hinge_barrel",
        elem_b="wash_hinge_front_block",
        name="wash door hinge barrel meets the front hinge block",
    )
    ctx.expect_contact(
        wash_door,
        body,
        elem_a="wash_hinge_barrel",
        elem_b="wash_hinge_rear_block",
        name="wash door hinge barrel meets the rear hinge block",
    )
    ctx.expect_contact(
        spin_door,
        body,
        elem_a="spin_hinge_barrel",
        elem_b="spin_hinge_front_block",
        name="spin door hinge barrel meets the front hinge block",
    )
    ctx.expect_contact(
        spin_door,
        body,
        elem_a="spin_hinge_barrel",
        elem_b="spin_hinge_rear_block",
        name="spin door hinge barrel meets the rear hinge block",
    )

    with ctx.pose({wash_hinge: 0.0, spin_hinge: 0.0}):
        ctx.expect_gap(
            wash_door,
            body,
            axis="z",
            positive_elem="wash_door_frame",
            negative_elem="top_deck",
            max_gap=0.001,
            max_penetration=0.0,
            name="wash lid sits flush on the top deck when closed",
        )
        ctx.expect_gap(
            spin_door,
            body,
            axis="z",
            positive_elem="spin_door_frame",
            negative_elem="top_deck",
            max_gap=0.001,
            max_penetration=0.0,
            name="spin lid sits flush on the top deck when closed",
        )

        wash_closed = ctx.part_element_world_aabb(wash_door, elem="wash_door_frame")
        spin_closed = ctx.part_element_world_aabb(spin_door, elem="spin_door_frame")

    with ctx.pose({wash_hinge: 1.30}):
        wash_open = ctx.part_element_world_aabb(wash_door, elem="wash_door_frame")

    with ctx.pose({spin_hinge: 1.30}):
        spin_open = ctx.part_element_world_aabb(spin_door, elem="spin_door_frame")

    ctx.check(
        "wash door swings upward above the deck",
        wash_closed is not None
        and wash_open is not None
        and wash_open[0][2] >= BODY_H - 0.001
        and wash_open[1][2] > wash_closed[1][2] + 0.12,
        details=f"closed={wash_closed}, open={wash_open}",
    )
    ctx.check(
        "spin door swings upward above the deck",
        spin_closed is not None
        and spin_open is not None
        and spin_open[0][2] >= BODY_H - 0.001
        and spin_open[1][2] > spin_closed[1][2] + 0.10,
        details=f"closed={spin_closed}, open={spin_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
