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


OUTER_LENGTH = 0.220
OUTER_HEIGHT = 0.032
OUTER_DEPTH = 0.015
OUTER_THICKNESS = 0.0016
OUTER_LIP_HEIGHT = 0.0012

MIDDLE_LENGTH = 0.185
MIDDLE_HEIGHT = 0.024
MIDDLE_DEPTH = 0.0102
MIDDLE_THICKNESS = 0.0014
MIDDLE_LIP_HEIGHT = 0.0009

CARRIAGE_LENGTH = 0.135
CARRIAGE_HEIGHT = 0.0184
CARRIAGE_WEB_THICKNESS = 0.0016
CARRIAGE_RIB_DEPTH = 0.0044
CARRIAGE_RIB_HEIGHT = 0.0036
CARRIAGE_FRONT_DECK_LENGTH = 0.024
CARRIAGE_FRONT_DECK_DEPTH = 0.0058
CARRIAGE_FRONT_DECK_HEIGHT = 0.010

WEAR_PAD_DEPTH = 0.0006
OUTER_WEAR_LENGTH = 0.170
OUTER_WEAR_HEIGHT = 0.0072
OUTER_WEAR_START = 0.018
OUTER_WEAR_Z = 0.0086

MIDDLE_WEAR_LENGTH = 0.148
MIDDLE_WEAR_HEIGHT = 0.0056
MIDDLE_WEAR_START = 0.014
MIDDLE_WEAR_Z = 0.0066

CHANNEL_TAB_LENGTH = 0.008
CHANNEL_TAB_PROJECTION = 0.0024
CHANNEL_TAB_OFFSET = 0.020

MIDDLE_HOME_X = 0.012
CARRIAGE_HOME_X = 0.018
MIDDLE_TRAVEL = 0.120
CARRIAGE_TRAVEL = 0.090


def _cq_box(
    length: float,
    depth: float,
    height: float,
    *,
    x0: float = 0.0,
    y0: float = 0.0,
    z_center: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XY").box(
        length,
        depth,
        height,
        centered=(False, False, True),
    ).translate((x0, y0, z_center))


def _channel_body(
    *,
    length: float,
    height: float,
    depth: float,
    thickness: float,
    lip_height: float,
    fillet_radius: float,
) -> cq.Workplane:
    body = _cq_box(length, thickness, height)
    body = body.union(
        _cq_box(
            length,
            depth,
            thickness,
            z_center=height / 2.0 - thickness / 2.0,
        )
    )
    body = body.union(
        _cq_box(
            length,
            depth,
            thickness,
            z_center=-height / 2.0 + thickness / 2.0,
        )
    )
    body = body.union(
        _cq_box(
            length,
            thickness,
            lip_height,
            y0=depth - thickness,
            z_center=height / 2.0 - thickness - lip_height / 2.0,
        )
    )
    body = body.union(
        _cq_box(
            length,
            thickness,
            lip_height,
            y0=depth - thickness,
            z_center=-height / 2.0 + thickness + lip_height / 2.0,
        )
    )
    return body


def _channel_stop_tabs(
    *,
    length: float,
    height: float,
    depth: float,
    thickness: float,
    lip_height: float,
    tab_length: float = CHANNEL_TAB_LENGTH,
    tab_projection: float = CHANNEL_TAB_PROJECTION,
    tab_offset: float = CHANNEL_TAB_OFFSET,
) -> cq.Workplane:
    top_z = height / 2.0 - thickness - lip_height / 2.0
    bottom_z = -height / 2.0 + thickness + lip_height / 2.0
    x_positions = (tab_offset, length - tab_offset - tab_length)

    tabs = _cq_box(
        tab_length,
        tab_projection,
        lip_height,
        x0=x_positions[0],
        y0=depth,
        z_center=top_z,
    )
    tabs = tabs.union(
        _cq_box(
            tab_length,
            tab_projection,
            lip_height,
            x0=x_positions[0],
            y0=depth,
            z_center=bottom_z,
        )
    )
    tabs = tabs.union(
        _cq_box(
            tab_length,
            tab_projection,
            lip_height,
            x0=x_positions[1],
            y0=depth,
            z_center=top_z,
        )
    )
    tabs = tabs.union(
        _cq_box(
            tab_length,
            tab_projection,
            lip_height,
            x0=x_positions[1],
            y0=depth,
            z_center=bottom_z,
        )
    )
    return tabs


def _carriage_body() -> cq.Workplane:
    body = _cq_box(
        CARRIAGE_LENGTH,
        CARRIAGE_WEB_THICKNESS,
        CARRIAGE_HEIGHT,
    )
    body = body.union(
        _cq_box(
            CARRIAGE_LENGTH,
            CARRIAGE_RIB_DEPTH,
            CARRIAGE_RIB_HEIGHT,
            z_center=CARRIAGE_HEIGHT / 2.0 - CARRIAGE_RIB_HEIGHT / 2.0,
        )
    )
    body = body.union(
        _cq_box(
            CARRIAGE_LENGTH,
            CARRIAGE_RIB_DEPTH,
            CARRIAGE_RIB_HEIGHT,
            z_center=-CARRIAGE_HEIGHT / 2.0 + CARRIAGE_RIB_HEIGHT / 2.0,
        )
    )
    body = body.union(
        _cq_box(
            CARRIAGE_FRONT_DECK_LENGTH,
            CARRIAGE_FRONT_DECK_DEPTH,
            CARRIAGE_FRONT_DECK_HEIGHT,
            x0=CARRIAGE_LENGTH - CARRIAGE_FRONT_DECK_LENGTH,
            z_center=0.0,
        )
    )
    body = body.union(
        _cq_box(
            0.012,
            CARRIAGE_RIB_DEPTH,
            0.006,
            x0=0.006,
            z_center=0.0,
        )
    )
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_extension_slide")

    model.material("outer_steel", rgba=(0.26, 0.28, 0.31, 1.0))
    model.material("middle_steel", rgba=(0.56, 0.58, 0.61, 1.0))
    model.material("carriage_steel", rgba=(0.76, 0.78, 0.81, 1.0))
    model.material("wear_polymer", rgba=(0.11, 0.12, 0.13, 1.0))
    model.material("stop_oxide", rgba=(0.17, 0.18, 0.20, 1.0))

    outer = model.part("outer_rail")
    outer.visual(
        mesh_from_cadquery(
            _channel_body(
                length=OUTER_LENGTH,
                height=OUTER_HEIGHT,
                depth=OUTER_DEPTH,
                thickness=OUTER_THICKNESS,
                lip_height=OUTER_LIP_HEIGHT,
                fillet_radius=0.00045,
            ),
            "outer_rail_body",
        ),
        material="outer_steel",
        name="body",
    )
    outer.visual(
        mesh_from_cadquery(
            _channel_stop_tabs(
                length=OUTER_LENGTH,
                height=OUTER_HEIGHT,
                depth=OUTER_DEPTH,
                thickness=OUTER_THICKNESS,
                lip_height=OUTER_LIP_HEIGHT,
            ),
            "outer_rail_stop_tabs",
        ),
        material="stop_oxide",
        name="stop_tabs",
    )
    outer.visual(
        Box((OUTER_WEAR_LENGTH, WEAR_PAD_DEPTH, OUTER_WEAR_HEIGHT)),
        origin=Origin(
            xyz=(
                OUTER_WEAR_START + OUTER_WEAR_LENGTH / 2.0,
                OUTER_THICKNESS + WEAR_PAD_DEPTH / 2.0,
                OUTER_WEAR_Z,
            )
        ),
        material="wear_polymer",
        name="wear_upper",
    )
    outer.visual(
        Box((OUTER_WEAR_LENGTH, WEAR_PAD_DEPTH, OUTER_WEAR_HEIGHT)),
        origin=Origin(
            xyz=(
                OUTER_WEAR_START + OUTER_WEAR_LENGTH / 2.0,
                OUTER_THICKNESS + WEAR_PAD_DEPTH / 2.0,
                -OUTER_WEAR_Z,
            )
        ),
        material="wear_polymer",
        name="wear_lower",
    )

    middle = model.part("middle_rail")
    middle.visual(
        mesh_from_cadquery(
            _channel_body(
                length=MIDDLE_LENGTH,
                height=MIDDLE_HEIGHT,
                depth=MIDDLE_DEPTH,
                thickness=MIDDLE_THICKNESS,
                lip_height=MIDDLE_LIP_HEIGHT,
                fillet_radius=0.00038,
            ),
            "middle_rail_body",
        ),
        material="middle_steel",
        name="body",
    )
    middle.visual(
        mesh_from_cadquery(
            _channel_stop_tabs(
                length=MIDDLE_LENGTH,
                height=MIDDLE_HEIGHT,
                depth=MIDDLE_DEPTH,
                thickness=MIDDLE_THICKNESS,
                lip_height=MIDDLE_LIP_HEIGHT,
            ),
            "middle_rail_stop_tabs",
        ),
        material="stop_oxide",
        name="stop_tabs",
    )
    middle.visual(
        Box((MIDDLE_WEAR_LENGTH, WEAR_PAD_DEPTH, MIDDLE_WEAR_HEIGHT)),
        origin=Origin(
            xyz=(
                MIDDLE_WEAR_START + MIDDLE_WEAR_LENGTH / 2.0,
                MIDDLE_THICKNESS + WEAR_PAD_DEPTH / 2.0,
                MIDDLE_WEAR_Z,
            )
        ),
        material="wear_polymer",
        name="wear_upper",
    )
    middle.visual(
        Box((MIDDLE_WEAR_LENGTH, WEAR_PAD_DEPTH, MIDDLE_WEAR_HEIGHT)),
        origin=Origin(
            xyz=(
                MIDDLE_WEAR_START + MIDDLE_WEAR_LENGTH / 2.0,
                MIDDLE_THICKNESS + WEAR_PAD_DEPTH / 2.0,
                -MIDDLE_WEAR_Z,
            )
        ),
        material="wear_polymer",
        name="wear_lower",
    )

    carriage = model.part("terminal_carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_body(), "terminal_carriage_body"),
        material="carriage_steel",
        name="body",
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(MIDDLE_HOME_X, OUTER_THICKNESS + WEAR_PAD_DEPTH, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.45,
            lower=0.0,
            upper=MIDDLE_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_carriage",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=carriage,
        origin=Origin(xyz=(CARRIAGE_HOME_X, MIDDLE_THICKNESS + WEAR_PAD_DEPTH, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=55.0,
            velocity=0.45,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_rail")
    middle = object_model.get_part("middle_rail")
    carriage = object_model.get_part("terminal_carriage")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_carriage = object_model.get_articulation("middle_to_carriage")

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
    ctx.fail_if_articulation_overlaps(
        max_pose_samples=20,
        name="no_overlap_through_prismatic_travel",
    )

    joint_limits_ok = (
        outer_to_middle.motion_limits is not None
        and middle_to_carriage.motion_limits is not None
        and tuple(round(v, 6) for v in outer_to_middle.axis) == (1.0, 0.0, 0.0)
        and tuple(round(v, 6) for v in middle_to_carriage.axis) == (1.0, 0.0, 0.0)
        and outer_to_middle.motion_limits.lower == 0.0
        and middle_to_carriage.motion_limits.lower == 0.0
        and outer_to_middle.motion_limits.upper == MIDDLE_TRAVEL
        and middle_to_carriage.motion_limits.upper == CARRIAGE_TRAVEL
    )
    ctx.check(
        "serial_prismatic_stage_setup",
        joint_limits_ok,
        "Both moving members should be serial +X prismatic stages with the authored travel limits.",
    )

    ctx.expect_contact(
        outer,
        middle,
        elem_a="wear_upper",
        elem_b="body",
        name="outer_upper_wear_supports_middle_stowed",
    )
    ctx.expect_contact(
        outer,
        middle,
        elem_a="wear_lower",
        elem_b="body",
        name="outer_lower_wear_supports_middle_stowed",
    )
    ctx.expect_contact(
        middle,
        carriage,
        elem_a="wear_upper",
        elem_b="body",
        name="middle_upper_wear_supports_carriage_stowed",
    )
    ctx.expect_contact(
        middle,
        carriage,
        elem_a="wear_lower",
        elem_b="body",
        name="middle_lower_wear_supports_carriage_stowed",
    )
    ctx.expect_overlap(
        outer,
        middle,
        axes="x",
        min_overlap=0.18,
        elem_a="body",
        elem_b="body",
        name="outer_middle_stage_overlap_when_stowed",
    )
    ctx.expect_overlap(
        middle,
        carriage,
        axes="x",
        min_overlap=0.13,
        elem_a="body",
        elem_b="body",
        name="middle_carriage_stage_overlap_when_stowed",
    )
    ctx.expect_within(
        middle,
        outer,
        axes="yz",
        inner_elem="body",
        outer_elem="body",
        name="middle_body_nested_inside_outer_body",
    )
    ctx.expect_within(
        carriage,
        middle,
        axes="yz",
        inner_elem="body",
        outer_elem="body",
        name="carriage_body_nested_inside_middle_body",
    )

    with ctx.pose({outer_to_middle: MIDDLE_TRAVEL, middle_to_carriage: CARRIAGE_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_part_overlap_fully_extended")
        ctx.expect_contact(
            outer,
            middle,
            elem_a="wear_upper",
            elem_b="body",
            name="outer_upper_wear_supports_middle_extended",
        )
        ctx.expect_contact(
            outer,
            middle,
            elem_a="wear_lower",
            elem_b="body",
            name="outer_lower_wear_supports_middle_extended",
        )
        ctx.expect_contact(
            middle,
            carriage,
            elem_a="wear_upper",
            elem_b="body",
            name="middle_upper_wear_supports_carriage_extended",
        )
        ctx.expect_contact(
            middle,
            carriage,
            elem_a="wear_lower",
            elem_b="body",
            name="middle_lower_wear_supports_carriage_extended",
        )
        ctx.expect_overlap(
            outer,
            middle,
            axes="x",
            min_overlap=0.085,
            elem_a="body",
            elem_b="body",
            name="outer_middle_retained_overlap_extended",
        )
        ctx.expect_overlap(
            middle,
            carriage,
            axes="x",
            min_overlap=0.075,
            elem_a="body",
            elem_b="body",
            name="middle_carriage_retained_overlap_extended",
        )
        ctx.expect_within(
            middle,
            outer,
            axes="yz",
            inner_elem="body",
            outer_elem="body",
            name="middle_body_stays_captured_when_extended",
        )
        ctx.expect_within(
            carriage,
            middle,
            axes="yz",
            inner_elem="body",
            outer_elem="body",
            name="carriage_body_stays_captured_when_extended",
        )
        ctx.expect_origin_gap(
            carriage,
            outer,
            axis="x",
            min_gap=0.20,
            name="carriage_origin_advances_forward_when_extended",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
