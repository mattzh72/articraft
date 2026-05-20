from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SUPPORT_PLATE_T = 0.008
SUPPORT_W = 0.110
SUPPORT_H = 0.150
SUPPORT_Z0 = -0.035
SUPPORT_PAD_T = 0.014
SUPPORT_PAD_W = 0.052
SUPPORT_PAD_H = 0.060
SUPPORT_PAD_Z0 = -0.004
SUPPORT_HOLE_R = 0.0042

OUTER_LENGTH = 0.340
OUTER_W = 0.030
OUTER_H = 0.032
OUTER_FLOOR_T = 0.004
OUTER_WALL_T = 0.003
OUTER_LIP_W = 0.005
OUTER_LIP_T = 0.002
OUTER_FLANGE_T = 0.006
OUTER_FLANGE_W = 0.044
OUTER_FLANGE_H = 0.050

MIDDLE_LENGTH = 0.280
MIDDLE_W = 0.020
MIDDLE_H = 0.021
MIDDLE_FLOOR_T = 0.003
MIDDLE_WALL_T = 0.0025
MIDDLE_LIP_W = 0.004
MIDDLE_LIP_T = 0.002

INNER_LENGTH = 0.240
INNER_W = 0.013
INNER_H = 0.014
INNER_FLOOR_T = 0.003
INNER_WALL_T = 0.0025
INNER_PAD_T = 0.010
INNER_PAD_H = 0.014

OUTER_TO_MIDDLE_INSERT = 0.026
MIDDLE_TO_INNER_INSERT = 0.050
OUTER_TO_MIDDLE_TRAVEL = 0.180
MIDDLE_TO_INNER_TRAVEL = 0.160


def _box_solid(
    x_size: float,
    y_size: float,
    z_size: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(x_size, y_size, z_size, centered=(False, True, False))
        .translate((x, y, z))
    )


def _u_channel(
    length: float,
    width: float,
    height: float,
    floor_t: float,
    wall_t: float,
    *,
    lip_w: float = 0.0,
    lip_t: float = 0.0,
) -> cq.Workplane:
    shape = _box_solid(length, width, floor_t)
    wall_y = (width - wall_t) / 2.0
    shape = shape.union(_box_solid(length, wall_t, height, y=wall_y))
    shape = shape.union(_box_solid(length, wall_t, height, y=-wall_y))
    if lip_w > 0.0 and lip_t > 0.0:
        lip_y = (width - lip_w) / 2.0
        lip_z = height - lip_t
        shape = shape.union(_box_solid(length, lip_w, lip_t, y=lip_y, z=lip_z))
        shape = shape.union(_box_solid(length, lip_w, lip_t, y=-lip_y, z=lip_z))
    return shape


def _support_shape() -> cq.Workplane:
    plate = _box_solid(
        SUPPORT_PLATE_T,
        SUPPORT_W,
        SUPPORT_H,
        x=-SUPPORT_PLATE_T,
        z=SUPPORT_Z0,
    )
    pad = _box_solid(
        SUPPORT_PAD_T,
        SUPPORT_PAD_W,
        SUPPORT_PAD_H,
        x=-SUPPORT_PAD_T,
        z=SUPPORT_PAD_Z0,
    )
    shape = plate.union(pad)

    hole_positions = (
        (-0.033, -0.006),
        (0.033, -0.006),
        (-0.033, 0.076),
        (0.033, 0.076),
    )
    for y_pos, z_pos in hole_positions:
        cutter = (
            cq.Workplane("YZ")
            .circle(SUPPORT_HOLE_R)
            .extrude(SUPPORT_PLATE_T + SUPPORT_PAD_T + 0.006)
            .translate((-SUPPORT_PAD_T - 0.003, y_pos, z_pos))
        )
        shape = shape.cut(cutter)
    return shape


def _outer_channel_shape() -> cq.Workplane:
    return _u_channel(
        OUTER_LENGTH,
        OUTER_W,
        OUTER_H,
        OUTER_FLOOR_T,
        OUTER_WALL_T,
        lip_w=OUTER_LIP_W,
        lip_t=OUTER_LIP_T,
    )


def _outer_flange_shape() -> cq.Workplane:
    return _box_solid(OUTER_FLANGE_T, OUTER_FLANGE_W, OUTER_FLANGE_H)


def _middle_shape() -> cq.Workplane:
    return _u_channel(
        MIDDLE_LENGTH,
        MIDDLE_W,
        MIDDLE_H,
        MIDDLE_FLOOR_T,
        MIDDLE_WALL_T,
        lip_w=MIDDLE_LIP_W,
        lip_t=MIDDLE_LIP_T,
    )


def _inner_channel_shape() -> cq.Workplane:
    return _u_channel(
        INNER_LENGTH,
        INNER_W,
        INNER_H,
        INNER_FLOOR_T,
        INNER_WALL_T,
    )


def _inner_pad_shape() -> cq.Workplane:
    return _box_solid(
        INNER_PAD_T,
        INNER_W,
        INNER_PAD_H,
        x=INNER_LENGTH - INNER_PAD_T,
    )


def _set_box_inertial(
    part,
    size: tuple[float, float, float],
    center_xyz: tuple[float, float, float],
    mass: float,
) -> None:
    part.inertial = Inertial.from_geometry(
        Box(size),
        mass=mass,
        origin=Origin(xyz=center_xyz),
    )


def _add_box_visual(
    part,
    size: tuple[float, float, float],
    center_xyz: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center_xyz),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_three_stage_extension_slide")

    model.material("support_gray", rgba=(0.66, 0.68, 0.71, 1.0))
    model.material("outer_steel", rgba=(0.27, 0.29, 0.32, 1.0))
    model.material("middle_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    model.material("inner_steel", rgba=(0.78, 0.79, 0.82, 1.0))
    model.material("terminal_face", rgba=(0.83, 0.84, 0.86, 1.0))

    back_support = model.part("back_support")
    back_support.visual(
        mesh_from_cadquery(_support_shape(), "back_support"),
        material="support_gray",
        name="support_body",
    )
    _set_box_inertial(
        back_support,
        (SUPPORT_PAD_T, SUPPORT_W, SUPPORT_H),
        (-SUPPORT_PAD_T / 2.0, 0.0, SUPPORT_Z0 + SUPPORT_H / 2.0),
        0.90,
    )

    outer_sleeve = model.part("outer_sleeve")
    _add_box_visual(
        outer_sleeve,
        (OUTER_LENGTH, OUTER_W, OUTER_FLOOR_T),
        (OUTER_LENGTH / 2.0, 0.0, OUTER_FLOOR_T / 2.0),
        "outer_steel",
        "outer_floor",
    )
    _add_box_visual(
        outer_sleeve,
        (OUTER_LENGTH, OUTER_WALL_T, OUTER_H),
        (OUTER_LENGTH / 2.0, (OUTER_W - OUTER_WALL_T) / 2.0, OUTER_H / 2.0),
        "outer_steel",
        "outer_right_wall",
    )
    _add_box_visual(
        outer_sleeve,
        (OUTER_LENGTH, OUTER_WALL_T, OUTER_H),
        (OUTER_LENGTH / 2.0, -(OUTER_W - OUTER_WALL_T) / 2.0, OUTER_H / 2.0),
        "outer_steel",
        "outer_left_wall",
    )
    _add_box_visual(
        outer_sleeve,
        (OUTER_LENGTH, OUTER_LIP_W, OUTER_LIP_T),
        (
            OUTER_LENGTH / 2.0,
            (OUTER_W - OUTER_LIP_W) / 2.0,
            OUTER_H - OUTER_LIP_T / 2.0,
        ),
        "outer_steel",
        "outer_right_lip",
    )
    _add_box_visual(
        outer_sleeve,
        (OUTER_LENGTH, OUTER_LIP_W, OUTER_LIP_T),
        (
            OUTER_LENGTH / 2.0,
            -(OUTER_W - OUTER_LIP_W) / 2.0,
            OUTER_H - OUTER_LIP_T / 2.0,
        ),
        "outer_steel",
        "outer_left_lip",
    )
    _add_box_visual(
        outer_sleeve,
        (OUTER_FLANGE_T, OUTER_FLANGE_W, OUTER_FLANGE_H),
        (OUTER_FLANGE_T / 2.0, 0.0, OUTER_FLANGE_H / 2.0),
        "outer_steel",
        "rear_flange",
    )
    _set_box_inertial(
        outer_sleeve,
        (OUTER_LENGTH, OUTER_FLANGE_W, OUTER_FLANGE_H),
        (OUTER_LENGTH / 2.0, 0.0, OUTER_FLANGE_H / 2.0),
        0.55,
    )

    middle_section = model.part("middle_section")
    _add_box_visual(
        middle_section,
        (MIDDLE_LENGTH, MIDDLE_W, MIDDLE_FLOOR_T),
        (MIDDLE_LENGTH / 2.0, 0.0, MIDDLE_FLOOR_T / 2.0),
        "middle_steel",
        "middle_floor",
    )
    _add_box_visual(
        middle_section,
        (MIDDLE_LENGTH, MIDDLE_WALL_T, MIDDLE_H),
        (MIDDLE_LENGTH / 2.0, (MIDDLE_W - MIDDLE_WALL_T) / 2.0, MIDDLE_H / 2.0),
        "middle_steel",
        "middle_right_wall",
    )
    _add_box_visual(
        middle_section,
        (MIDDLE_LENGTH, MIDDLE_WALL_T, MIDDLE_H),
        (MIDDLE_LENGTH / 2.0, -(MIDDLE_W - MIDDLE_WALL_T) / 2.0, MIDDLE_H / 2.0),
        "middle_steel",
        "middle_left_wall",
    )
    _add_box_visual(
        middle_section,
        (MIDDLE_LENGTH, MIDDLE_LIP_W, MIDDLE_LIP_T),
        (
            MIDDLE_LENGTH / 2.0,
            (MIDDLE_W - MIDDLE_LIP_W) / 2.0,
            MIDDLE_H - MIDDLE_LIP_T / 2.0,
        ),
        "middle_steel",
        "middle_right_lip",
    )
    _add_box_visual(
        middle_section,
        (MIDDLE_LENGTH, MIDDLE_LIP_W, MIDDLE_LIP_T),
        (
            MIDDLE_LENGTH / 2.0,
            -(MIDDLE_W - MIDDLE_LIP_W) / 2.0,
            MIDDLE_H - MIDDLE_LIP_T / 2.0,
        ),
        "middle_steel",
        "middle_left_lip",
    )
    _set_box_inertial(
        middle_section,
        (MIDDLE_LENGTH, MIDDLE_W, MIDDLE_H),
        (MIDDLE_LENGTH / 2.0, 0.0, MIDDLE_H / 2.0),
        0.36,
    )

    inner_section = model.part("inner_section")
    _add_box_visual(
        inner_section,
        (INNER_LENGTH, INNER_W, INNER_FLOOR_T),
        (INNER_LENGTH / 2.0, 0.0, INNER_FLOOR_T / 2.0),
        "inner_steel",
        "inner_floor",
    )
    _add_box_visual(
        inner_section,
        (INNER_LENGTH, INNER_WALL_T, INNER_H),
        (INNER_LENGTH / 2.0, (INNER_W - INNER_WALL_T) / 2.0, INNER_H / 2.0),
        "inner_steel",
        "inner_right_wall",
    )
    _add_box_visual(
        inner_section,
        (INNER_LENGTH, INNER_WALL_T, INNER_H),
        (INNER_LENGTH / 2.0, -(INNER_W - INNER_WALL_T) / 2.0, INNER_H / 2.0),
        "inner_steel",
        "inner_left_wall",
    )
    _add_box_visual(
        inner_section,
        (INNER_PAD_T, INNER_W, INNER_PAD_H),
        (INNER_LENGTH - INNER_PAD_T / 2.0, 0.0, INNER_PAD_H / 2.0),
        "terminal_face",
        "terminal_pad",
    )
    _set_box_inertial(
        inner_section,
        (INNER_LENGTH, INNER_W, INNER_PAD_H),
        (INNER_LENGTH / 2.0, 0.0, INNER_PAD_H / 2.0),
        0.22,
    )

    model.articulation(
        "support_to_outer",
        ArticulationType.FIXED,
        parent=back_support,
        child=outer_sleeve,
        origin=Origin(),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_sleeve,
        child=middle_section,
        origin=Origin(xyz=(OUTER_TO_MIDDLE_INSERT, 0.0, OUTER_FLOOR_T)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.45,
            lower=0.0,
            upper=OUTER_TO_MIDDLE_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_section,
        child=inner_section,
        origin=Origin(xyz=(MIDDLE_TO_INNER_INSERT, 0.0, MIDDLE_FLOOR_T)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.45,
            lower=0.0,
            upper=MIDDLE_TO_INNER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    back_support = object_model.get_part("back_support")
    outer_sleeve = object_model.get_part("outer_sleeve")
    middle_section = object_model.get_part("middle_section")
    inner_section = object_model.get_part("inner_section")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

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
        "serial_prismatic_structure",
        outer_to_middle.articulation_type == ArticulationType.PRISMATIC
        and middle_to_inner.articulation_type == ArticulationType.PRISMATIC
        and outer_to_middle.parent == outer_sleeve.name
        and outer_to_middle.child == middle_section.name
        and middle_to_inner.parent == middle_section.name
        and middle_to_inner.child == inner_section.name,
        "expected outer->middle and middle->inner serial prismatic joints",
    )
    ctx.check(
        "common_extension_axis",
        tuple(outer_to_middle.axis) == (1.0, 0.0, 0.0)
        and tuple(middle_to_inner.axis) == (1.0, 0.0, 0.0),
        "both moving stages should slide on the shared +X axis",
    )

    with ctx.pose({outer_to_middle: 0.0, middle_to_inner: 0.0}):
        ctx.expect_contact(
            back_support,
            outer_sleeve,
            elem_a="support_body",
            elem_b="rear_flange",
            name="back_support_carries_outer_sleeve",
        )
        ctx.expect_overlap(
            back_support,
            outer_sleeve,
            axes="yz",
            elem_a="support_body",
            elem_b="rear_flange",
            min_overlap=0.040,
            name="outer_flange_has_real_mount_face",
        )
        ctx.expect_contact(
            outer_sleeve,
            middle_section,
            name="middle_bears_on_outer_sleeve",
        )
        ctx.expect_contact(
            middle_section,
            inner_section,
            name="inner_bears_on_middle_section",
        )
        ctx.expect_within(
            middle_section,
            outer_sleeve,
            axes="yz",
            margin=0.0015,
            name="middle_stays_nested_in_outer_profile",
        )
        ctx.expect_within(
            inner_section,
            middle_section,
            axes="yz",
            margin=0.0015,
            name="inner_stays_nested_in_middle_profile",
        )
        ctx.expect_overlap(
            outer_sleeve,
            middle_section,
            axes="x",
            min_overlap=0.240,
            name="middle_has_long_closed_overlap_in_outer",
        )
        ctx.expect_overlap(
            middle_section,
            inner_section,
            axes="x",
            min_overlap=0.180,
            name="inner_has_long_closed_overlap_in_middle",
        )

    with ctx.pose(
        {
            outer_to_middle: OUTER_TO_MIDDLE_TRAVEL,
            middle_to_inner: MIDDLE_TO_INNER_TRAVEL,
        }
    ):
        ctx.expect_contact(
            outer_sleeve,
            middle_section,
            name="middle_remains_guided_when_extended",
        )
        ctx.expect_contact(
            middle_section,
            inner_section,
            name="inner_remains_guided_when_extended",
        )
        ctx.expect_within(
            middle_section,
            outer_sleeve,
            axes="yz",
            margin=0.0015,
            name="middle_stays_on_outer_axis_when_extended",
        )
        ctx.expect_within(
            inner_section,
            middle_section,
            axes="yz",
            margin=0.0015,
            name="inner_stays_on_middle_axis_when_extended",
        )
        ctx.expect_origin_gap(
            middle_section,
            outer_sleeve,
            axis="x",
            min_gap=0.200,
            name="middle_projects_forward_when_extended",
        )
        ctx.expect_origin_gap(
            inner_section,
            middle_section,
            axis="x",
            min_gap=0.200,
            name="inner_projects_forward_from_middle_when_extended",
        )
        ctx.expect_origin_gap(
            inner_section,
            outer_sleeve,
            axis="x",
            min_gap=0.400,
            name="inner_projects_furthest_from_wall",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
