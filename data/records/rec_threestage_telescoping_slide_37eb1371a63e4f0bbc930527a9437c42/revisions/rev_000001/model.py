from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


OUTER_LENGTH = 0.50
OUTER_WIDTH = 0.022
OUTER_HEIGHT = 0.046
OUTER_WALL = 0.0015
OUTER_SLOT_WIDTH = 0.012

MIDDLE_LENGTH = 0.36
MIDDLE_WIDTH = 0.016
MIDDLE_HEIGHT = 0.040
MIDDLE_WALL = 0.0013
MIDDLE_SLOT_WIDTH = 0.009

INNER_LENGTH = 0.24
INNER_WIDTH = 0.0115
INNER_HEIGHT = 0.035
INNER_WALL = 0.0011
INNER_SLOT_WIDTH = 0.0065

OUTER_TO_MIDDLE_INSERT = 0.055
OUTER_TO_MIDDLE_TRAVEL = 0.18
MIDDLE_TO_INNER_INSERT = 0.045
MIDDLE_TO_INNER_TRAVEL = 0.14

OUTER_OPENING_HEIGHT = 0.028
MIDDLE_OPENING_HEIGHT = 0.024
INNER_OPENING_HEIGHT = 0.021

MIDDLE_GLIDE_LENGTH = MIDDLE_LENGTH - 0.020
MIDDLE_GLIDE_WIDTH = MIDDLE_WIDTH * 0.72
MIDDLE_GLIDE_THICKNESS = (-MIDDLE_HEIGHT / 2.0) - (-OUTER_HEIGHT / 2.0 + OUTER_WALL)

INNER_GLIDE_LENGTH = INNER_LENGTH - 0.020
INNER_GLIDE_WIDTH = INNER_WIDTH * 0.74
INNER_GLIDE_THICKNESS = (-INNER_HEIGHT / 2.0) - (-MIDDLE_HEIGHT / 2.0 + MIDDLE_WALL)

MOUNT_WEB_START = 0.120
MOUNT_WEB_LENGTH = 0.305
MOUNT_WEB_DEPTH = 0.019
MOUNT_WEB_CENTER_Y = 0.00475
MOUNT_WEB_HEIGHT = 0.016

MOUNT_PLATE_START = 0.405
MOUNT_PLATE_LENGTH = 0.125
MOUNT_PLATE_HEIGHT = 0.060
MOUNT_PLATE_THICKNESS = 0.004
MOUNT_PLATE_CENTER_Y = 0.016
MOUNT_PLATE_CENTER_Z = 0.0


def _add_box_visual(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _add_channel_member(
    part,
    *,
    length: float,
    width: float,
    height: float,
    wall: float,
    opening_height: float,
    pocket_spans: tuple[tuple[float, float], ...],
    stop_xs: tuple[float, ...],
    material: str,
) -> None:
    half_w = width / 2.0
    half_h = height / 2.0
    lip_height = ((height - (2.0 * wall)) - opening_height) / 2.0
    flange_center_y = -half_w + ((width - wall) / 2.0)
    back_center_y = -half_w + (wall / 2.0)
    lip_center_y = half_w - (wall / 2.0)
    top_lip_center_z = (opening_height / 2.0) + (lip_height / 2.0)
    bottom_lip_center_z = -top_lip_center_z

    x_cursor = 0.0
    segment_index = 0
    for pocket_start, pocket_end in pocket_spans:
        segment_length = pocket_start - x_cursor
        if segment_length > 0.003:
            _add_box_visual(
                part,
                name=f"back_web_{segment_index}",
                size=(segment_length, wall, height),
                center=(x_cursor + (segment_length / 2.0), back_center_y, 0.0),
                material=material,
            )
            segment_index += 1
        x_cursor = pocket_end
    final_length = length - x_cursor
    if final_length > 0.003:
        _add_box_visual(
            part,
            name=f"back_web_{segment_index}",
            size=(final_length, wall, height),
            center=(x_cursor + (final_length / 2.0), back_center_y, 0.0),
            material=material,
        )

    _add_box_visual(
        part,
        name="top_flange",
        size=(length, width - wall, wall),
        center=(length / 2.0, flange_center_y, half_h - (wall / 2.0)),
        material=material,
    )
    _add_box_visual(
        part,
        name="bottom_flange",
        size=(length, width - wall, wall),
        center=(length / 2.0, flange_center_y, -half_h + (wall / 2.0)),
        material=material,
    )
    _add_box_visual(
        part,
        name="top_lip",
        size=(length, wall, lip_height),
        center=(length / 2.0, lip_center_y, top_lip_center_z),
        material=material,
    )
    _add_box_visual(
        part,
        name="bottom_lip",
        size=(length, wall, lip_height),
        center=(length / 2.0, lip_center_y, bottom_lip_center_z),
        material=material,
    )

    stop_depth = wall * 1.7
    stop_height = lip_height * 0.58
    stop_center_y = half_w - wall - (stop_depth / 2.0)
    for index, stop_x in enumerate(stop_xs):
        _add_box_visual(
            part,
            name=f"top_stop_{index}",
            size=(0.010, stop_depth, stop_height),
            center=(
                stop_x + 0.005,
                stop_center_y,
                (opening_height / 2.0) + (stop_height / 2.0),
            ),
            material=material,
        )
        _add_box_visual(
            part,
            name=f"bottom_stop_{index}",
            size=(0.010, stop_depth, stop_height),
            center=(
                stop_x + 0.005,
                stop_center_y,
                -(opening_height / 2.0) - (stop_height / 2.0),
            ),
            material=material,
        )


def _mount_plate() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(
            MOUNT_PLATE_LENGTH,
            MOUNT_PLATE_THICKNESS,
            MOUNT_PLATE_HEIGHT,
            centered=(False, True, True),
        )
        .translate((MOUNT_PLATE_START, MOUNT_PLATE_CENTER_Y, 0.0))
    )

    for slot_x in (MOUNT_PLATE_START + 0.050, MOUNT_PLATE_START + 0.122):
        cutter = (
            cq.Workplane("XZ")
            .center(slot_x, 0.0)
            .slot2D(0.026, 0.008)
            .extrude(MOUNT_PLATE_THICKNESS * 3.0, both=True)
            .translate((0.0, MOUNT_PLATE_CENTER_Y, 0.0))
        )
        plate = plate.cut(cutter)

    return plate


def _mount_brace() -> cq.Workplane:
    back_wall_outer_y = -(INNER_WIDTH / 2.0) + INNER_WALL
    plate_inner_y = MOUNT_PLATE_CENTER_Y - (MOUNT_PLATE_THICKNESS / 2.0)
    web_depth = plate_inner_y - back_wall_outer_y
    web_center_y = back_wall_outer_y + (web_depth / 2.0)

    bracket = (
        cq.Workplane("XY")
        .box(
            MOUNT_WEB_LENGTH,
            web_depth,
            MOUNT_WEB_HEIGHT,
            centered=(False, True, True),
        )
        .translate((MOUNT_WEB_START, web_center_y, 0.0))
    )

    gusset_top = (
        cq.Workplane("YZ")
        .polyline(
            [
                (plate_inner_y - 0.007, 0.0),
                (plate_inner_y, (MOUNT_PLATE_HEIGHT / 2.0) - 0.004),
                (plate_inner_y, 0.0),
            ]
        )
        .close()
        .extrude(0.022)
    )
    gusset_bottom = (
        cq.Workplane("YZ")
        .polyline(
            [
                (plate_inner_y - 0.007, 0.0),
                (plate_inner_y, -(MOUNT_PLATE_HEIGHT / 2.0) + 0.004),
                (plate_inner_y, 0.0),
            ]
        )
        .close()
        .extrude(0.022)
    )

    for rib_x in (0.412, 0.478):
        bracket = bracket.union(gusset_top.translate((rib_x, 0.0, 0.0)))
        bracket = bracket.union(gusset_bottom.translate((rib_x, 0.0, 0.0)))

    return bracket


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="equipment_drawer_slide_module")

    model.material("outer_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    model.material("middle_steel", rgba=(0.53, 0.55, 0.58, 1.0))
    model.material("inner_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("glide_polymer", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("mount_plate_steel", rgba=(0.80, 0.82, 0.85, 1.0))

    outer = model.part("outer_member")
    _add_channel_member(
        outer,
        length=OUTER_LENGTH,
        width=OUTER_WIDTH,
        height=OUTER_HEIGHT,
        wall=OUTER_WALL,
        opening_height=OUTER_OPENING_HEIGHT,
        pocket_spans=((0.096, 0.144), (0.360, 0.408)),
        stop_xs=(0.028, 0.452),
        material="outer_steel",
    )

    middle = model.part("middle_member")
    _add_channel_member(
        middle,
        length=MIDDLE_LENGTH,
        width=MIDDLE_WIDTH,
        height=MIDDLE_HEIGHT,
        wall=MIDDLE_WALL,
        opening_height=MIDDLE_OPENING_HEIGHT,
        pocket_spans=((0.074, 0.116), (0.250, 0.292)),
        stop_xs=(0.022, 0.318),
        material="middle_steel",
    )
    middle.visual(
        Box((MIDDLE_GLIDE_LENGTH, MIDDLE_GLIDE_WIDTH, MIDDLE_GLIDE_THICKNESS)),
        origin=Origin(
            xyz=(
                MIDDLE_LENGTH / 2.0,
                0.0,
                (-MIDDLE_HEIGHT / 2.0) - (MIDDLE_GLIDE_THICKNESS / 2.0),
            )
        ),
        material="glide_polymer",
        name="bottom_glide",
    )

    inner = model.part("inner_member")
    _add_channel_member(
        inner,
        length=INNER_LENGTH,
        width=INNER_WIDTH,
        height=INNER_HEIGHT,
        wall=INNER_WALL,
        opening_height=INNER_OPENING_HEIGHT,
        pocket_spans=((0.060, 0.092), (0.160, 0.192)),
        stop_xs=(0.018, 0.210),
        material="inner_steel",
    )
    inner.visual(
        Box((INNER_GLIDE_LENGTH, INNER_GLIDE_WIDTH, INNER_GLIDE_THICKNESS)),
        origin=Origin(
            xyz=(
                INNER_LENGTH / 2.0,
                0.0,
                (-INNER_HEIGHT / 2.0) - (INNER_GLIDE_THICKNESS / 2.0),
            )
        ),
        material="glide_polymer",
        name="bottom_glide",
    )
    inner.visual(
        mesh_from_cadquery(_mount_brace(), "inner_mount_brace"),
        material="inner_steel",
        name="mount_web",
    )
    inner.visual(
        mesh_from_cadquery(_mount_plate(), "inner_mount_plate"),
        material="mount_plate_steel",
        name="mount_plate",
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(OUTER_TO_MIDDLE_INSERT, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=280.0,
            velocity=0.70,
            lower=0.0,
            upper=OUTER_TO_MIDDLE_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(MIDDLE_TO_INNER_INSERT, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.70,
            lower=0.0,
            upper=MIDDLE_TO_INNER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_member")
    middle = object_model.get_part("middle_member")
    inner = object_model.get_part("inner_member")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

    outer_bottom_flange = outer.get_visual("bottom_flange")
    outer_top_flange = outer.get_visual("top_flange")
    middle_bottom_flange = middle.get_visual("bottom_flange")
    middle_glide = middle.get_visual("bottom_glide")
    inner_glide = inner.get_visual("bottom_glide")
    mount_plate = inner.get_visual("mount_plate")

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
        "stage_lengths_descend",
        OUTER_LENGTH > MIDDLE_LENGTH > INNER_LENGTH,
        details=(
            f"expected outer > middle > inner, got "
            f"{OUTER_LENGTH:.3f}, {MIDDLE_LENGTH:.3f}, {INNER_LENGTH:.3f}"
        ),
    )
    ctx.check(
        "prismatic_axes_are_x_aligned",
        outer_to_middle.axis == (1.0, 0.0, 0.0)
        and middle_to_inner.axis == (1.0, 0.0, 0.0),
        details=(
            f"axes were {outer_to_middle.axis} and {middle_to_inner.axis}, "
            "expected serial x-axis extension"
        ),
    )

    with ctx.pose({outer_to_middle: 0.0, middle_to_inner: 0.0}):
        ctx.expect_contact(
            middle,
            outer,
            elem_a=middle_glide,
            elem_b=outer_bottom_flange,
            name="middle_glide_carries_outer_stage",
        )
        ctx.expect_contact(
            inner,
            middle,
            elem_a=inner_glide,
            elem_b=middle_bottom_flange,
            name="inner_glide_carries_inner_stage",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.30,
            name="middle_stage_starts_nested_in_outer",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.20,
            name="inner_stage_starts_nested_in_middle",
        )
        ctx.expect_gap(
            inner,
            outer,
            axis="x",
            positive_elem=mount_plate,
            negative_elem=outer_top_flange,
            min_gap=0.003,
            name="mount_plate_starts_ahead_of_outer_rail",
        )

    with ctx.pose(
        {
            outer_to_middle: OUTER_TO_MIDDLE_TRAVEL,
            middle_to_inner: MIDDLE_TO_INNER_TRAVEL,
        }
    ):
        ctx.expect_contact(
            middle,
            outer,
            elem_a=middle_glide,
            elem_b=outer_bottom_flange,
            name="middle_glide_keeps_contact_at_full_extension",
        )
        ctx.expect_contact(
            inner,
            middle,
            elem_a=inner_glide,
            elem_b=middle_bottom_flange,
            name="inner_glide_keeps_contact_at_full_extension",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.12,
            name="middle_stage_remains_engaged_when_extended",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.08,
            name="inner_stage_remains_engaged_when_extended",
        )
        ctx.expect_gap(
            inner,
            outer,
            axis="x",
            positive_elem=mount_plate,
            negative_elem=outer_top_flange,
            min_gap=0.003,
            name="mount_plate_stays_ahead_when_extended",
        )
        ctx.check(
            "serial_extension_order_visible",
            ctx.part_world_position(inner)[0] > ctx.part_world_position(middle)[0] > 0.0,
            details=(
                f"expected inner origin ahead of middle origin in extension pose; got "
                f"{ctx.part_world_position(inner)} vs {ctx.part_world_position(middle)}"
            ),
        )

    ctx.fail_if_articulation_overlaps(max_pose_samples=12)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
