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


BASE_WIDTH = 0.58
BASE_DEPTH = 0.58
BASE_THICKNESS = 0.04
BASE_DOUBLER_SIZE = 0.20
BASE_DOUBLER_THICKNESS = 0.012

OUTER_SHOE_SIZE = 0.14
OUTER_SHOE_THICKNESS = 0.012
OUTER_SECTION_SIZE = 0.078
OUTER_SECTION_WALL = 0.004
OUTER_SECTION_HEIGHT = 0.92
OUTER_CLAMP_SIZE = 0.100
CLAMP_HEIGHT = 0.065

MIDDLE_SECTION_SIZE = 0.060
MIDDLE_SECTION_WALL = 0.0035
MIDDLE_SECTION_TOTAL = 0.98
MIDDLE_SECTION_INSERT = 0.56
MIDDLE_SECTION_EXPOSED = MIDDLE_SECTION_TOTAL - MIDDLE_SECTION_INSERT
MIDDLE_SECTION_TRAVEL = 0.34
MIDDLE_COLLAR_SIZE = 0.094

INNER_SECTION_SIZE = 0.044
INNER_SECTION_WALL = 0.003
INNER_SECTION_TOTAL = 0.88
INNER_SECTION_INSERT = 0.50
INNER_SECTION_EXPOSED = INNER_SECTION_TOTAL - INNER_SECTION_INSERT
INNER_SECTION_TRAVEL = 0.28
INNER_COLLAR_SIZE = 0.078
INNER_TOP_CAP_SIZE = 0.060

COLLAR_THICKNESS = 0.010
VISUAL_FUSE_EPS = 0.0004

HEAD_PLATE_LENGTH = 0.17
HEAD_PLATE_WIDTH = 0.125
HEAD_PLATE_THICKNESS = 0.010
HEAD_BEARING_RADIUS = 0.022
HEAD_BEARING_HEIGHT = 0.016


def _combine(*solids: cq.Workplane) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _square_tube(outer_size: float, wall: float, height: float) -> cq.Workplane:
    outer = cq.Workplane("XY").box(
        outer_size,
        outer_size,
        height,
        centered=(True, True, False),
    )
    inner = (
        cq.Workplane("XY")
        .box(
            outer_size - 2.0 * wall,
            outer_size - 2.0 * wall,
            height + 0.004,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, -0.002))
    )
    return outer.cut(inner)


def _square_ring(
    outer_size: float,
    inner_size: float,
    height: float,
) -> cq.Workplane:
    outer = cq.Workplane("XY").box(
        outer_size,
        outer_size,
        height,
        centered=(True, True, False),
    )
    inner = (
        cq.Workplane("XY")
        .box(
            inner_size,
            inner_size,
            height + 0.004,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, -0.002))
    )
    return outer.cut(inner)


def _base_body() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(
            BASE_WIDTH,
            BASE_DEPTH,
            BASE_THICKNESS,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(0.03)
    )
    doubler = cq.Workplane("XY").box(
        BASE_DOUBLER_SIZE,
        BASE_DOUBLER_SIZE,
        BASE_DOUBLER_THICKNESS,
        centered=(True, True, False),
    ).translate((0.0, 0.0, BASE_THICKNESS))
    body = plate.union(doubler)

    handle_cuts = (
        cq.Workplane("XY")
        .pushPoints([(0.0, 0.19), (0.0, -0.19)])
        .slot2D(0.16, 0.04, 0.0)
        .extrude(BASE_THICKNESS + 0.03)
    )
    anchor_cuts = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.22, -0.22),
                (-0.22, 0.22),
                (0.22, -0.22),
                (0.22, 0.22),
            ]
        )
        .circle(0.010)
        .extrude(BASE_THICKNESS + 0.02)
    )
    return body.cut(handle_cuts).cut(anchor_cuts)


def _outer_body() -> cq.Workplane:
    shoe = cq.Workplane("XY").box(
        OUTER_SHOE_SIZE,
        OUTER_SHOE_SIZE,
        OUTER_SHOE_THICKNESS,
        centered=(True, True, False),
    )
    tube = _square_tube(
        OUTER_SECTION_SIZE,
        OUTER_SECTION_WALL,
        OUTER_SECTION_HEIGHT,
    ).translate((0.0, 0.0, OUTER_SHOE_THICKNESS))
    clamp_band = _square_ring(
        OUTER_CLAMP_SIZE,
        OUTER_SECTION_SIZE - VISUAL_FUSE_EPS,
        CLAMP_HEIGHT,
    ).translate((0.0, 0.0, OUTER_SHOE_THICKNESS + OUTER_SECTION_HEIGHT - CLAMP_HEIGHT))
    clamp_block = cq.Workplane("XY").box(0.026, 0.046, 0.042).translate(
        (
            OUTER_CLAMP_SIZE * 0.5 + 0.013,
            0.0,
            OUTER_SHOE_THICKNESS + OUTER_SECTION_HEIGHT - 0.030,
        )
    )
    clamp_knob = (
        cq.Workplane("YZ")
        .circle(0.009)
        .extrude(0.022)
        .translate(
            (
                OUTER_CLAMP_SIZE * 0.5 + 0.026,
                0.0,
                OUTER_SHOE_THICKNESS + OUTER_SECTION_HEIGHT - 0.030,
            )
        )
    )
    return _combine(shoe, tube, clamp_band, clamp_block, clamp_knob)


def _middle_tube() -> cq.Workplane:
    return _square_tube(
        MIDDLE_SECTION_SIZE,
        MIDDLE_SECTION_WALL,
        MIDDLE_SECTION_TOTAL,
    ).translate((0.0, 0.0, -MIDDLE_SECTION_INSERT))


def _middle_lower_collar() -> cq.Workplane:
    return _square_ring(
        MIDDLE_COLLAR_SIZE,
        MIDDLE_SECTION_SIZE - VISUAL_FUSE_EPS,
        COLLAR_THICKNESS,
    )


def _middle_upper_clamp() -> cq.Workplane:
    clamp_band = _square_ring(
        0.078,
        MIDDLE_SECTION_SIZE - VISUAL_FUSE_EPS,
        CLAMP_HEIGHT,
    ).translate((0.0, 0.0, MIDDLE_SECTION_EXPOSED - CLAMP_HEIGHT))
    clamp_block = cq.Workplane("XY").box(0.024, 0.040, 0.036).translate(
        (-0.051, 0.0, MIDDLE_SECTION_EXPOSED - 0.028)
    )
    clamp_knob = (
        cq.Workplane("YZ")
        .circle(0.008)
        .extrude(0.020)
        .translate((-0.071, 0.0, MIDDLE_SECTION_EXPOSED - 0.028))
    )
    return _combine(clamp_band, clamp_block, clamp_knob)


def _inner_tube() -> cq.Workplane:
    tube = _square_tube(
        INNER_SECTION_SIZE,
        INNER_SECTION_WALL,
        INNER_SECTION_TOTAL,
    ).translate((0.0, 0.0, -INNER_SECTION_INSERT))
    top_cap = cq.Workplane("XY").box(
        INNER_TOP_CAP_SIZE,
        INNER_TOP_CAP_SIZE,
        0.008,
        centered=(True, True, False),
    ).translate((0.0, 0.0, INNER_SECTION_EXPOSED - 0.008))
    return tube.union(top_cap)


def _inner_lower_collar() -> cq.Workplane:
    return _square_ring(
        INNER_COLLAR_SIZE,
        INNER_SECTION_SIZE - VISUAL_FUSE_EPS,
        COLLAR_THICKNESS,
    )


def _head_plate() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(
            HEAD_PLATE_LENGTH,
            HEAD_PLATE_WIDTH,
            HEAD_PLATE_THICKNESS,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(0.008)
    )
    slot_cuts = (
        cq.Workplane("XY")
        .pushPoints([(-0.045, 0.0), (0.045, 0.0), (0.0, -0.030), (0.0, 0.030)])
        .slot2D(0.024, 0.008, 90.0)
        .extrude(HEAD_PLATE_THICKNESS + 0.01)
    )
    return plate.cut(slot_cuts).translate((0.0, 0.0, HEAD_BEARING_HEIGHT - VISUAL_FUSE_EPS))


def _head_bearing() -> cq.Workplane:
    return cq.Workplane("XY").circle(HEAD_BEARING_RADIUS).extrude(HEAD_BEARING_HEIGHT)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="survey_mast")

    base_color = model.material("base_charcoal", rgba=(0.17, 0.18, 0.19, 1.0))
    mast_color = model.material("mast_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    accent_color = model.material("survey_yellow", rgba=(0.82, 0.75, 0.22, 1.0))
    dark_hardware = model.material("hardware_black", rgba=(0.16, 0.16, 0.16, 1.0))

    base = model.part(
        "base",
        inertial=Inertial.from_geometry(
            Box((BASE_WIDTH, BASE_DEPTH, BASE_THICKNESS + BASE_DOUBLER_THICKNESS)),
            mass=22.0,
            origin=Origin(
                xyz=(0.0, 0.0, 0.5 * (BASE_THICKNESS + BASE_DOUBLER_THICKNESS))
            ),
        ),
    )
    base.visual(
        mesh_from_cadquery(_base_body(), "base_body"),
        material=base_color,
        name="base_body",
    )

    outer = model.part(
        "outer_mast",
        inertial=Inertial.from_geometry(
            Box((OUTER_CLAMP_SIZE + 0.05, OUTER_CLAMP_SIZE, OUTER_SECTION_HEIGHT + OUTER_SHOE_THICKNESS)),
            mass=8.0,
            origin=Origin(
                xyz=(0.0, 0.0, 0.5 * (OUTER_SECTION_HEIGHT + OUTER_SHOE_THICKNESS))
            ),
        ),
    )
    outer.visual(
        mesh_from_cadquery(_outer_body(), "outer_body"),
        material=mast_color,
        name="outer_body",
    )

    middle = model.part(
        "middle_mast",
        inertial=Inertial.from_geometry(
            Box((MIDDLE_COLLAR_SIZE, MIDDLE_COLLAR_SIZE, MIDDLE_SECTION_TOTAL)),
            mass=4.6,
            origin=Origin(
                xyz=(0.0, 0.0, 0.5 * (MIDDLE_SECTION_EXPOSED - MIDDLE_SECTION_INSERT))
            ),
        ),
    )
    middle.visual(
        mesh_from_cadquery(_middle_tube(), "middle_tube"),
        material=mast_color,
        name="middle_tube",
    )
    middle.visual(
        mesh_from_cadquery(_middle_lower_collar(), "middle_lower_collar"),
        material=dark_hardware,
        name="middle_lower_collar",
    )
    middle.visual(
        mesh_from_cadquery(_middle_upper_clamp(), "middle_upper_clamp"),
        material=dark_hardware,
        name="middle_upper_clamp",
    )

    inner = model.part(
        "inner_mast",
        inertial=Inertial.from_geometry(
            Box((INNER_COLLAR_SIZE, INNER_COLLAR_SIZE, INNER_SECTION_TOTAL)),
            mass=3.2,
            origin=Origin(
                xyz=(0.0, 0.0, 0.5 * (INNER_SECTION_EXPOSED - INNER_SECTION_INSERT))
            ),
        ),
    )
    inner.visual(
        mesh_from_cadquery(_inner_tube(), "inner_tube"),
        material=mast_color,
        name="inner_tube",
    )
    inner.visual(
        mesh_from_cadquery(_inner_lower_collar(), "inner_lower_collar"),
        material=dark_hardware,
        name="inner_lower_collar",
    )

    head = model.part(
        "head_plate",
        inertial=Inertial.from_geometry(
            Box((HEAD_PLATE_LENGTH, HEAD_PLATE_WIDTH, HEAD_PLATE_THICKNESS + HEAD_BEARING_HEIGHT)),
            mass=1.4,
            origin=Origin(
                xyz=(0.0, 0.0, 0.5 * (HEAD_PLATE_THICKNESS + HEAD_BEARING_HEIGHT))
            ),
        ),
    )
    head.visual(
        mesh_from_cadquery(_head_plate(), "head_plate"),
        material=accent_color,
        name="head_plate",
    )
    head.visual(
        mesh_from_cadquery(_head_bearing(), "head_bearing"),
        material=dark_hardware,
        name="head_bearing",
    )

    model.articulation(
        "base_to_outer",
        ArticulationType.FIXED,
        parent=base,
        child=outer,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + BASE_DOUBLER_THICKNESS)),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(0.0, 0.0, OUTER_SHOE_THICKNESS + OUTER_SECTION_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.35,
            lower=0.0,
            upper=MIDDLE_SECTION_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_SECTION_EXPOSED)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.35,
            lower=0.0,
            upper=INNER_SECTION_TRAVEL,
        ),
    )
    model.articulation(
        "inner_to_head",
        ArticulationType.CONTINUOUS,
        parent=inner,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, INNER_SECTION_EXPOSED)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    outer = object_model.get_part("outer_mast")
    middle = object_model.get_part("middle_mast")
    inner = object_model.get_part("inner_mast")
    head = object_model.get_part("head_plate")

    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")
    inner_to_head = object_model.get_articulation("inner_to_head")

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
        "outer_to_middle_prismatic_vertical",
        outer_to_middle.articulation_type == ArticulationType.PRISMATIC
        and tuple(outer_to_middle.axis) == (0.0, 0.0, 1.0)
        and outer_to_middle.motion_limits is not None
        and outer_to_middle.motion_limits.lower == 0.0
        and outer_to_middle.motion_limits.upper == MIDDLE_SECTION_TRAVEL,
        "Outer-to-middle mast stage must telescope vertically on a bounded prismatic joint.",
    )
    ctx.check(
        "middle_to_inner_prismatic_vertical",
        middle_to_inner.articulation_type == ArticulationType.PRISMATIC
        and tuple(middle_to_inner.axis) == (0.0, 0.0, 1.0)
        and middle_to_inner.motion_limits is not None
        and middle_to_inner.motion_limits.lower == 0.0
        and middle_to_inner.motion_limits.upper == INNER_SECTION_TRAVEL,
        "Middle-to-inner mast stage must telescope vertically on a bounded prismatic joint.",
    )
    ctx.check(
        "head_pan_is_vertical_continuous",
        inner_to_head.articulation_type == ArticulationType.CONTINUOUS
        and tuple(inner_to_head.axis) == (0.0, 0.0, 1.0)
        and inner_to_head.motion_limits is not None
        and inner_to_head.motion_limits.lower is None
        and inner_to_head.motion_limits.upper is None,
        "Head plate should pan continuously around the mast's vertical axis.",
    )

    with ctx.pose(
        {
            outer_to_middle: 0.0,
            middle_to_inner: 0.0,
            inner_to_head: 0.0,
        }
    ):
        ctx.expect_contact(
            outer,
            base,
            elem_a="outer_body",
            elem_b="base_body",
            name="outer_shoe_is_seated_on_base",
        )
        ctx.expect_contact(
            middle,
            outer,
            elem_a="middle_lower_collar",
            elem_b="outer_body",
            name="middle_stage_collar_seats_on_outer_stage",
        )
        ctx.expect_contact(
            inner,
            middle,
            elem_a="inner_lower_collar",
            elem_b="middle_tube",
            name="inner_stage_collar_seats_on_middle_stage",
        )
        ctx.expect_contact(
            head,
            inner,
            elem_a="head_bearing",
            elem_b="inner_tube",
            name="head_bearing_is_seated_on_inner_stage",
        )
        ctx.expect_within(
            middle,
            outer,
            axes="xy",
            margin=0.010,
            inner_elem="middle_tube",
            outer_elem="outer_body",
            name="middle_tube_nests_inside_outer_tube",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="xy",
            margin=0.010,
            inner_elem="inner_tube",
            outer_elem="middle_tube",
            name="inner_tube_nests_inside_middle_tube",
        )

    with ctx.pose(
        {
            outer_to_middle: 0.30,
            middle_to_inner: 0.24,
            inner_to_head: 1.1,
        }
    ):
        ctx.expect_origin_gap(
            head,
            base,
            axis="z",
            min_gap=2.20,
            max_gap=2.55,
            name="extended_head_height_is_field_mast_scale",
        )
        ctx.expect_within(
            middle,
            outer,
            axes="xy",
            margin=0.010,
            inner_elem="middle_tube",
            outer_elem="outer_body",
            name="middle_stage_stays_coaxial_when_extended",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="xy",
            margin=0.010,
            inner_elem="inner_tube",
            outer_elem="middle_tube",
            name="inner_stage_stays_coaxial_when_extended",
        )
        ctx.expect_contact(
            head,
            inner,
            elem_a="head_bearing",
            elem_b="inner_tube",
            name="pan_head_stays_seated_while_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
