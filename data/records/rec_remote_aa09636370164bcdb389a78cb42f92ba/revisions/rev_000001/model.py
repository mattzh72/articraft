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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BODY_WIDTH = 0.060
BODY_LENGTH = 0.162
BODY_THICKNESS = 0.016

TRAY_WIDTH = 0.042
TRAY_LENGTH = 0.120
TRAY_TRAVEL = 0.078


def _rounded_plate(width: float, length: float, thickness: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(width, length, radius),
            thickness,
            cap=True,
            center=True,
            closed=True,
        ),
        name,
    )


def _add_key_row(
    tray_part,
    *,
    row_index: int,
    count: int,
    center_y: float,
    x_offset: float,
    key_width: float,
    key_length: float,
    key_height: float,
    key_z: float,
    material,
) -> None:
    pitch = key_width + 0.0006
    start_x = -0.5 * pitch * (count - 1) + x_offset
    for column in range(count):
        tray_part.visual(
            Box((key_width, key_length, key_height)),
            origin=Origin(xyz=(start_x + column * pitch, center_y, key_z)),
            material=material,
            name=f"key_{row_index}_{column}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="conference_room_presenter")

    shell_dark = model.material("shell_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    shell_black = model.material("shell_black", rgba=(0.10, 0.11, 0.12, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.28, 0.30, 0.33, 1.0))
    key_dark = model.material("key_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    legend_grey = model.material("legend_grey", rgba=(0.78, 0.79, 0.80, 1.0))
    laser_red = model.material("laser_red", rgba=(0.78, 0.11, 0.11, 1.0))
    glass = model.material("glass", rgba=(0.16, 0.28, 0.34, 0.55))
    foot_rubber = model.material("foot_rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    body = model.part("body")
    body.visual(
        _rounded_plate(
            BODY_WIDTH - 0.002,
            BODY_LENGTH - 0.002,
            0.0016,
            0.011,
            "presenter_back_panel",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0008)),
        material=shell_black,
        name="back_panel",
    )
    body.visual(
        _rounded_plate(
            BODY_WIDTH,
            BODY_LENGTH,
            0.0050,
            0.012,
            "presenter_top_shell",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0135)),
        material=shell_dark,
        name="top_shell",
    )
    body.visual(
        Box((0.0040, BODY_LENGTH - 0.014, 0.0108)),
        origin=Origin(xyz=(-0.0245, 0.0, 0.00695)),
        material=trim_grey,
        name="left_side_wall",
    )
    body.visual(
        Box((0.0040, BODY_LENGTH - 0.014, 0.0108)),
        origin=Origin(xyz=(0.0245, 0.0, 0.00695)),
        material=trim_grey,
        name="right_side_wall",
    )
    body.visual(
        Box((0.044, 0.006, 0.0108)),
        origin=Origin(xyz=(0.0, 0.075, 0.00695)),
        material=trim_grey,
        name="top_end_wall",
    )
    body.visual(
        Box((0.006, 0.124, 0.0006)),
        origin=Origin(xyz=(-0.018, -0.006, 0.0019)),
        material=trim_grey,
        name="left_guide_rail",
    )
    body.visual(
        Box((0.006, 0.124, 0.0006)),
        origin=Origin(xyz=(0.018, -0.006, 0.0019)),
        material=trim_grey,
        name="right_guide_rail",
    )
    body.visual(
        Box((0.034, 0.024, 0.0012)),
        origin=Origin(xyz=(0.0, 0.050, 0.01655)),
        material=glass,
        name="screen_window",
    )
    body.visual(
        Cylinder(radius=0.0065, length=0.0018),
        origin=Origin(xyz=(0.0, 0.018, 0.01685)),
        material=legend_grey,
        name="nav_ring",
    )
    body.visual(
        Cylinder(radius=0.0038, length=0.0022),
        origin=Origin(xyz=(0.0, 0.018, 0.01705)),
        material=shell_black,
        name="select_pad",
    )
    body.visual(
        Box((0.016, 0.009, 0.0018)),
        origin=Origin(xyz=(0.0, -0.004, 0.01685)),
        material=legend_grey,
        name="page_forward",
    )
    body.visual(
        Box((0.016, 0.009, 0.0018)),
        origin=Origin(xyz=(0.0, -0.018, 0.01685)),
        material=legend_grey,
        name="page_back",
    )
    body.visual(
        Cylinder(radius=0.0045, length=0.0020),
        origin=Origin(xyz=(0.0, 0.071, 0.01695)),
        material=laser_red,
        name="laser_button",
    )
    body.visual(
        Cylinder(radius=0.0020, length=0.006),
        origin=Origin(
            xyz=(0.0, 0.079, 0.0130),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=shell_black,
        name="laser_window",
    )
    body.visual(
        Cylinder(radius=0.0028, length=0.008),
        origin=Origin(
            xyz=(-0.0125, -0.077, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_grey,
        name="left_hinge_lug",
    )
    body.visual(
        Cylinder(radius=0.0028, length=0.008),
        origin=Origin(
            xyz=(0.0125, -0.077, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_grey,
        name="right_hinge_lug",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_LENGTH, BODY_THICKNESS)),
        mass=0.26,
        origin=Origin(xyz=(0.0, 0.0, BODY_THICKNESS / 2.0)),
    )

    tray = model.part("keyboard_tray")
    tray.visual(
        Box((TRAY_WIDTH, TRAY_LENGTH, 0.0024)),
        origin=Origin(xyz=(0.0, TRAY_LENGTH / 2.0, 0.0043)),
        material=shell_black,
        name="tray_deck",
    )
    tray.visual(
        Box((0.0030, TRAY_LENGTH - 0.012, 0.0010)),
        origin=Origin(xyz=(-0.0185, (TRAY_LENGTH - 0.012) / 2.0 + 0.004, 0.0027)),
        material=trim_grey,
        name="left_runner",
    )
    tray.visual(
        Box((0.0030, TRAY_LENGTH - 0.012, 0.0010)),
        origin=Origin(xyz=(0.0185, (TRAY_LENGTH - 0.012) / 2.0 + 0.004, 0.0027)),
        material=trim_grey,
        name="right_runner",
    )
    tray.visual(
        Box((0.030, 0.006, 0.0032)),
        origin=Origin(xyz=(0.0, -0.001, 0.0047)),
        material=trim_grey,
        name="tray_pull",
    )
    _add_key_row(
        tray,
        row_index=0,
        count=10,
        center_y=0.073,
        x_offset=0.0,
        key_width=0.0035,
        key_length=0.0060,
        key_height=0.0020,
        key_z=0.00645,
        material=key_dark,
    )
    _add_key_row(
        tray,
        row_index=1,
        count=9,
        center_y=0.064,
        x_offset=-0.0021,
        key_width=0.0036,
        key_length=0.0060,
        key_height=0.0020,
        key_z=0.00645,
        material=key_dark,
    )
    _add_key_row(
        tray,
        row_index=2,
        count=9,
        center_y=0.055,
        x_offset=-0.0008,
        key_width=0.0036,
        key_length=0.0060,
        key_height=0.0020,
        key_z=0.00645,
        material=key_dark,
    )
    tray.visual(
        Box((0.0060, 0.0060, 0.0020)),
        origin=Origin(xyz=(-0.015, 0.044, 0.00645)),
        material=key_dark,
        name="modifier_left",
    )
    tray.visual(
        Box((0.0060, 0.0060, 0.0020)),
        origin=Origin(xyz=(0.015, 0.044, 0.00645)),
        material=key_dark,
        name="modifier_right",
    )
    tray.visual(
        Box((0.020, 0.0060, 0.0020)),
        origin=Origin(xyz=(0.0, 0.044, 0.00645)),
        material=key_dark,
        name="space_bar",
    )
    tray.inertial = Inertial.from_geometry(
        Box((TRAY_WIDTH, TRAY_LENGTH, 0.008)),
        mass=0.08,
        origin=Origin(xyz=(0.0, TRAY_LENGTH / 2.0, 0.004)),
    )

    stand = model.part("tilt_stand")
    stand.visual(
        Box((0.034, 0.090, 0.0028)),
        origin=Origin(xyz=(0.0, 0.045, -0.0014)),
        material=trim_grey,
        name="leg_panel",
    )
    stand.visual(
        Box((0.012, 0.040, 0.0040)),
        origin=Origin(xyz=(0.0, 0.025, -0.0020)),
        material=trim_grey,
        name="leg_rib",
    )
    stand.visual(
        Box((0.026, 0.010, 0.0035)),
        origin=Origin(xyz=(0.0, 0.086, -0.00175)),
        material=foot_rubber,
        name="stand_foot",
    )
    stand.visual(
        Box((0.014, 0.006, 0.0040)),
        origin=Origin(xyz=(0.0, 0.003, -0.0020)),
        material=trim_grey,
        name="hinge_bridge",
    )
    stand.visual(
        Cylinder(radius=0.0026, length=0.014),
        origin=Origin(
            xyz=(0.0, 0.003, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_grey,
        name="hinge_barrel",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.034, 0.090, 0.008)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.045, -0.002)),
    )

    model.articulation(
        "body_to_keyboard_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, -0.081, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.18,
            lower=0.0,
            upper=TRAY_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_tilt_stand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=stand,
        origin=Origin(xyz=(0.0, -0.077, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=1.08,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    tray = object_model.get_part("keyboard_tray")
    stand = object_model.get_part("tilt_stand")
    tray_joint = object_model.get_articulation("body_to_keyboard_tray")
    stand_joint = object_model.get_articulation("body_to_tilt_stand")

    tray_upper = tray_joint.motion_limits.upper if tray_joint.motion_limits is not None else 0.0
    stand_upper = stand_joint.motion_limits.upper if stand_joint.motion_limits is not None else 0.0

    with ctx.pose({tray_joint: 0.0}):
        ctx.expect_within(
            tray,
            body,
            axes=("x", "z"),
            margin=0.001,
            name="closed tray stays within body width and thickness",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="y",
            min_overlap=0.108,
            name="closed tray remains mostly inserted",
        )

    tray_rest_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_joint: tray_upper}):
        ctx.expect_within(
            tray,
            body,
            axes=("x", "z"),
            margin=0.001,
            name="extended tray stays aligned on the guide rails",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="y",
            min_overlap=0.038,
            name="extended tray retains insertion in the body",
        )
        ctx.expect_gap(
            tray,
            body,
            axis="x",
            positive_elem="tray_deck",
            negative_elem="left_side_wall",
            min_gap=0.0004,
            name="extended tray clears the left inner wall",
        )
        ctx.expect_gap(
            body,
            tray,
            axis="x",
            positive_elem="right_side_wall",
            negative_elem="tray_deck",
            min_gap=0.0004,
            name="extended tray clears the right inner wall",
        )
        ctx.expect_gap(
            tray,
            body,
            axis="z",
            positive_elem="tray_deck",
            negative_elem="back_panel",
            min_gap=0.0012,
            name="extended tray deck stays above the back panel",
        )
        ctx.expect_gap(
            tray,
            body,
            axis="z",
            positive_elem="left_runner",
            negative_elem="left_guide_rail",
            max_gap=0.0001,
            max_penetration=0.0,
            name="left runner rides on the left guide rail",
        )
        ctx.expect_gap(
            tray,
            body,
            axis="z",
            positive_elem="right_runner",
            negative_elem="right_guide_rail",
            max_gap=0.0001,
            max_penetration=0.0,
            name="right runner rides on the right guide rail",
        )
        tray_open_pos = ctx.part_world_position(tray)

    ctx.check(
        "keyboard tray extends out from the bottom edge",
        tray_rest_pos is not None
        and tray_open_pos is not None
        and tray_open_pos[1] < tray_rest_pos[1] - 0.05,
        details=f"rest={tray_rest_pos}, open={tray_open_pos}",
    )

    with ctx.pose({stand_joint: 0.0}):
        ctx.expect_contact(
            stand,
            body,
            elem_a="leg_panel",
            elem_b="back_panel",
            name="tilt stand folds flush to the back panel",
        )
        stand_closed_aabb = ctx.part_element_world_aabb(stand, elem="stand_foot")

    with ctx.pose({stand_joint: stand_upper}):
        stand_open_aabb = ctx.part_element_world_aabb(stand, elem="stand_foot")

    ctx.check(
        "tilt stand swings down away from the back",
        stand_closed_aabb is not None
        and stand_open_aabb is not None
        and stand_open_aabb[0][2] < stand_closed_aabb[0][2] - 0.04,
        details=f"closed={stand_closed_aabb}, open={stand_open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
