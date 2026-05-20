from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BODY_W = 0.240
BODY_L = 0.380
BODY_H = 0.082
BODY_WALL_T = 0.004
BODY_FLOOR_T = 0.005
BODY_FRONT_LIP_H = 0.015
BODY_SIDE_RAIL_H = 0.024

MID_W = 0.224
MID_L = 0.294
MID_H = 0.056
MID_WALL_T = 0.003
MID_FLOOR_T = 0.0035
MID_FRONT_LIP_H = 0.018

INNER_W = 0.164
INNER_L = 0.146
INNER_H = 0.041
INNER_WALL_T = 0.0028
INNER_FLOOR_T = 0.003
INNER_FRONT_LIP_H = 0.026

MIDDLE_CLOSED_Y = BODY_L / 2.0 - BODY_WALL_T - MID_L / 2.0
INNER_CLOSED_Y = MID_L / 2.0 - MID_WALL_T - INNER_L / 2.0 - 0.010

MIDDLE_EXTENSION = 0.140
INNER_EXTENSION = 0.090


def _add_box_visual(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material,
) -> None:
    _, _, size_z = size
    pos_x, pos_y, base_z = xyz
    part.visual(
        Box(size),
        origin=Origin(xyz=(pos_x, pos_y, base_z + size_z / 2.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_sample_tray_carrier")

    body_color = model.material("body_graphite", rgba=(0.19, 0.21, 0.24, 1.0))
    middle_color = model.material("middle_tray_beige", rgba=(0.79, 0.78, 0.73, 1.0))
    inner_color = model.material("inner_tray_blue", rgba=(0.27, 0.45, 0.66, 1.0))

    outer_body = model.part("outer_body")
    _add_box_visual(
        outer_body,
        name="outer_floor",
        size=(BODY_W, BODY_L, BODY_FLOOR_T),
        xyz=(0.0, 0.0, 0.0),
        material=body_color,
    )
    _add_box_visual(
        outer_body,
        name="outer_left_rail",
        size=(BODY_WALL_T, BODY_L, BODY_SIDE_RAIL_H),
        xyz=(-(BODY_W / 2.0) + BODY_WALL_T / 2.0, 0.0, 0.0),
        material=body_color,
    )
    _add_box_visual(
        outer_body,
        name="outer_right_rail",
        size=(BODY_WALL_T, BODY_L, BODY_SIDE_RAIL_H),
        xyz=((BODY_W / 2.0) - BODY_WALL_T / 2.0, 0.0, 0.0),
        material=body_color,
    )
    _add_box_visual(
        outer_body,
        name="outer_back_wall",
        size=(BODY_W, BODY_WALL_T, BODY_H),
        xyz=(0.0, -(BODY_L / 2.0) + BODY_WALL_T / 2.0, 0.0),
        material=body_color,
    )
    _add_box_visual(
        outer_body,
        name="outer_front_lip",
        size=(BODY_W, BODY_WALL_T, BODY_FRONT_LIP_H),
        xyz=(0.0, (BODY_L / 2.0) - BODY_WALL_T / 2.0, 0.0),
        material=body_color,
    )
    outer_body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_L, BODY_H)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    middle_tray = model.part("middle_tray")
    _add_box_visual(
        middle_tray,
        name="middle_floor",
        size=(MID_W, MID_L, MID_FLOOR_T),
        xyz=(0.0, 0.0, 0.0),
        material=middle_color,
    )
    _add_box_visual(
        middle_tray,
        name="middle_left_wall",
        size=(MID_WALL_T, MID_L, MID_H),
        xyz=(-(MID_W / 2.0) + MID_WALL_T / 2.0, 0.0, 0.0),
        material=middle_color,
    )
    _add_box_visual(
        middle_tray,
        name="middle_right_wall",
        size=(MID_WALL_T, MID_L, MID_H),
        xyz=((MID_W / 2.0) - MID_WALL_T / 2.0, 0.0, 0.0),
        material=middle_color,
    )
    _add_box_visual(
        middle_tray,
        name="middle_back_wall",
        size=(MID_W, MID_WALL_T, MID_H),
        xyz=(0.0, -(MID_L / 2.0) + MID_WALL_T / 2.0, 0.0),
        material=middle_color,
    )
    _add_box_visual(
        middle_tray,
        name="middle_front_lip",
        size=(MID_W, MID_WALL_T, MID_FRONT_LIP_H),
        xyz=(0.0, (MID_L / 2.0) - MID_WALL_T / 2.0, 0.0),
        material=middle_color,
    )
    middle_tray.inertial = Inertial.from_geometry(
        Box((MID_W, MID_L, MID_H)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, MID_H / 2.0)),
    )

    inner_tray = model.part("inner_tray")
    _add_box_visual(
        inner_tray,
        name="inner_floor",
        size=(INNER_W, INNER_L, INNER_FLOOR_T),
        xyz=(0.0, 0.0, 0.0),
        material=inner_color,
    )
    _add_box_visual(
        inner_tray,
        name="inner_left_wall",
        size=(INNER_WALL_T, INNER_L, INNER_H),
        xyz=(-(INNER_W / 2.0) + INNER_WALL_T / 2.0, 0.0, 0.0),
        material=inner_color,
    )
    _add_box_visual(
        inner_tray,
        name="inner_right_wall",
        size=(INNER_WALL_T, INNER_L, INNER_H),
        xyz=((INNER_W / 2.0) - INNER_WALL_T / 2.0, 0.0, 0.0),
        material=inner_color,
    )
    _add_box_visual(
        inner_tray,
        name="inner_back_wall",
        size=(INNER_W, INNER_WALL_T, INNER_H),
        xyz=(0.0, -(INNER_L / 2.0) + INNER_WALL_T / 2.0, 0.0),
        material=inner_color,
    )
    _add_box_visual(
        inner_tray,
        name="inner_front_lip",
        size=(INNER_W, INNER_WALL_T, INNER_FRONT_LIP_H),
        xyz=(0.0, (INNER_L / 2.0) - INNER_WALL_T / 2.0, 0.0),
        material=inner_color,
    )
    inner_tray.inertial = Inertial.from_geometry(
        Box((INNER_W, INNER_L, INNER_H)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, INNER_H / 2.0)),
    )

    model.articulation(
        "outer_to_middle_slide",
        ArticulationType.PRISMATIC,
        parent=outer_body,
        child=middle_tray,
        origin=Origin(xyz=(0.0, MIDDLE_CLOSED_Y, BODY_FLOOR_T)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.35,
            lower=0.0,
            upper=MIDDLE_EXTENSION,
        ),
    )
    model.articulation(
        "middle_to_inner_slide",
        ArticulationType.PRISMATIC,
        parent=middle_tray,
        child=inner_tray,
        origin=Origin(xyz=(0.0, INNER_CLOSED_Y, MID_FLOOR_T)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.30,
            lower=0.0,
            upper=INNER_EXTENSION,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_body = object_model.get_part("outer_body")
    middle_tray = object_model.get_part("middle_tray")
    inner_tray = object_model.get_part("inner_tray")
    outer_to_middle = object_model.get_articulation("outer_to_middle_slide")
    middle_to_inner = object_model.get_articulation("middle_to_inner_slide")

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
        "serial_tray_slides_are_prismatic",
        outer_to_middle.articulation_type == ArticulationType.PRISMATIC
        and middle_to_inner.articulation_type == ArticulationType.PRISMATIC,
        "Both extension stages should be modeled as prismatic slides.",
    )
    ctx.check(
        "serial_tray_slides_share_axis",
        tuple(outer_to_middle.axis) == (0.0, 1.0, 0.0)
        and tuple(middle_to_inner.axis) == (0.0, 1.0, 0.0),
        "Middle and inner trays should translate forward along the same +Y axis.",
    )

    ctx.expect_contact(
        middle_tray,
        outer_body,
        name="middle_tray_supported_by_outer_body",
    )
    ctx.expect_contact(
        inner_tray,
        middle_tray,
        name="inner_tray_supported_by_middle_tray",
    )
    ctx.expect_within(
        middle_tray,
        outer_body,
        axes="xy",
        margin=0.0,
        name="middle_tray_nests_inside_outer_body",
    )
    ctx.expect_within(
        inner_tray,
        middle_tray,
        axes="xy",
        margin=0.0,
        name="inner_tray_nests_inside_middle_tray",
    )
    ctx.expect_origin_gap(
        inner_tray,
        middle_tray,
        axis="y",
        min_gap=0.055,
        max_gap=0.070,
        name="inner_tray_is_mounted_at_front_of_middle_tray",
    )

    with ctx.pose(
        {
            outer_to_middle: 0.105,
            middle_to_inner: 0.060,
        }
    ):
        ctx.expect_contact(
            middle_tray,
            outer_body,
            name="middle_tray_remains_supported_when_extended",
        )
        ctx.expect_contact(
            inner_tray,
            middle_tray,
            name="inner_tray_remains_supported_when_extended",
        )
        ctx.expect_origin_gap(
            middle_tray,
            outer_body,
            axis="y",
            min_gap=0.140,
            name="middle_tray_extends_forward_from_outer_body",
        )
        ctx.expect_origin_gap(
            inner_tray,
            middle_tray,
            axis="y",
            min_gap=0.115,
            name="inner_tray_extends_forward_relative_to_middle_tray",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
