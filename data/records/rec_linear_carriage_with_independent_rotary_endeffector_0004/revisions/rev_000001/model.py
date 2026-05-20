from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

RAIL_LENGTH = 0.280
RAIL_WIDTH = 0.060
RAIL_BASE_HEIGHT = 0.010
WAY_LENGTH = 0.240
WAY_WIDTH = 0.012
WAY_HEIGHT = 0.010
WAY_Y_OFFSET = 0.015
WAY_TOP_Z = RAIL_BASE_HEIGHT + WAY_HEIGHT

CARRIAGE_LENGTH = 0.075
CARRIAGE_WIDTH = 0.072
CARRIAGE_BODY_HEIGHT = 0.026
CARRIAGE_BODY_BOTTOM_Z = 0.008
CARRIAGE_BODY_TOP_Z = CARRIAGE_BODY_BOTTOM_Z + CARRIAGE_BODY_HEIGHT
RUNNER_LENGTH = 0.064
RUNNER_WIDTH = 0.014
RUNNER_HEIGHT = 0.008
SLIDE_TRAVEL = 0.200

HOUSING_RADIUS = 0.018
HOUSING_LENGTH = 0.030
HOUSING_CENTER_Y = 0.041
SPINDLE_AXIS_Z = 0.022
HOUSING_BORE_RADIUS = 0.0095

SPINDLE_JOURNAL_RADIUS = 0.0088
SPINDLE_JOURNAL_LENGTH = 0.030
RETAINER_RADIUS = 0.0125
RETAINER_LENGTH = 0.0025
SPINDLE_NOSE_RADIUS = 0.006
SPINDLE_NOSE_LENGTH = 0.022
SPINDLE_COLLET_RADIUS = 0.010
SPINDLE_COLLET_LENGTH = 0.010
TOOL_FLAG_SIZE = (0.006, 0.010, 0.004)
TOOL_FLAG_CENTER = (0.0, 0.044, 0.010)


def _carriage_body_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_BODY_HEIGHT)
        .translate((0.0, 0.0, CARRIAGE_BODY_BOTTOM_Z + CARRIAGE_BODY_HEIGHT / 2.0))
    )

    housing = (
        cq.Workplane("XZ")
        .circle(HOUSING_RADIUS)
        .extrude(HOUSING_LENGTH, both=True)
        .translate((0.0, HOUSING_CENTER_Y, SPINDLE_AXIS_Z))
    )

    boss = (
        cq.Workplane("XY")
        .box(0.040, 0.018, 0.018)
        .translate((0.0, 0.028, 0.018))
    )

    clearance_bore = (
        cq.Workplane("XZ")
        .circle(HOUSING_BORE_RADIUS)
        .extrude(HOUSING_LENGTH + 0.010, both=True)
        .translate((0.0, HOUSING_CENTER_Y, SPINDLE_AXIS_Z))
    )

    return body.union(housing).union(boss).cut(clearance_bore)


def _tool_flag_shape() -> cq.Workplane:
    return cq.Workplane("XY").box(*TOOL_FLAG_SIZE).translate(TOOL_FLAG_CENTER)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_rotary_carriage", assets=ASSETS)

    rail_gray = model.material("rail_gray", rgba=(0.63, 0.66, 0.70, 1.0))
    carriage_gray = model.material("carriage_gray", rgba=(0.32, 0.35, 0.38, 1.0))
    spindle_steel = model.material("spindle_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    accent_orange = model.material("accent_orange", rgba=(0.86, 0.42, 0.14, 1.0))

    rail = model.part("rail")
    rail.visual(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, RAIL_BASE_HEIGHT / 2.0)),
        material=rail_gray,
        name="rail_base",
    )
    rail.visual(
        Box((WAY_LENGTH, WAY_WIDTH, WAY_HEIGHT)),
        origin=Origin(xyz=(0.0, -WAY_Y_OFFSET, RAIL_BASE_HEIGHT + WAY_HEIGHT / 2.0)),
        material=rail_gray,
        name="left_way",
    )
    rail.visual(
        Box((WAY_LENGTH, WAY_WIDTH, WAY_HEIGHT)),
        origin=Origin(xyz=(0.0, WAY_Y_OFFSET, RAIL_BASE_HEIGHT + WAY_HEIGHT / 2.0)),
        material=rail_gray,
        name="right_way",
    )
    rail.inertial = Inertial.from_geometry(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_BASE_HEIGHT + WAY_HEIGHT)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, (RAIL_BASE_HEIGHT + WAY_HEIGHT) / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_body_shape(), "carriage_body.obj", assets=ASSETS),
        material=carriage_gray,
        name="carriage_body",
    )
    carriage.visual(
        Box((RUNNER_LENGTH, RUNNER_WIDTH, RUNNER_HEIGHT)),
        origin=Origin(xyz=(0.0, -WAY_Y_OFFSET, RUNNER_HEIGHT / 2.0)),
        material=carriage_gray,
        name="left_runner",
    )
    carriage.visual(
        Box((RUNNER_LENGTH, RUNNER_WIDTH, RUNNER_HEIGHT)),
        origin=Origin(xyz=(0.0, WAY_Y_OFFSET, RUNNER_HEIGHT / 2.0)),
        material=carriage_gray,
        name="right_runner",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_BODY_TOP_Z)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_BODY_TOP_Z / 2.0)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=SPINDLE_JOURNAL_RADIUS, length=SPINDLE_JOURNAL_LENGTH),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=spindle_steel,
        name="spindle_journal",
    )
    spindle.visual(
        Cylinder(radius=RETAINER_RADIUS, length=RETAINER_LENGTH),
        origin=Origin(
            xyz=(0.0, -(SPINDLE_JOURNAL_LENGTH + RETAINER_LENGTH) / 2.0, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=spindle_steel,
        name="rear_retainer",
    )
    spindle.visual(
        Cylinder(radius=RETAINER_RADIUS, length=RETAINER_LENGTH),
        origin=Origin(
            xyz=(0.0, (SPINDLE_JOURNAL_LENGTH + RETAINER_LENGTH) / 2.0, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=spindle_steel,
        name="front_retainer",
    )
    spindle.visual(
        Cylinder(radius=SPINDLE_NOSE_RADIUS, length=SPINDLE_NOSE_LENGTH),
        origin=Origin(
            xyz=(
                0.0,
                SPINDLE_JOURNAL_LENGTH / 2.0
                + RETAINER_LENGTH
                + SPINDLE_NOSE_LENGTH / 2.0,
                0.0,
            ),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=spindle_steel,
        name="spindle_nose",
    )
    spindle.visual(
        Cylinder(radius=SPINDLE_COLLET_RADIUS, length=SPINDLE_COLLET_LENGTH),
        origin=Origin(
            xyz=(
                0.0,
                SPINDLE_JOURNAL_LENGTH / 2.0
                + RETAINER_LENGTH
                + SPINDLE_NOSE_LENGTH
                + SPINDLE_COLLET_LENGTH / 2.0,
                0.0,
            ),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=spindle_steel,
        name="collet_nut",
    )
    spindle.visual(
        mesh_from_cadquery(_tool_flag_shape(), "tool_flag.obj", assets=ASSETS),
        material=accent_orange,
        name="tool_flag",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(
            radius=SPINDLE_COLLET_RADIUS,
            length=SPINDLE_JOURNAL_LENGTH + SPINDLE_NOSE_LENGTH + SPINDLE_COLLET_LENGTH,
        ),
        mass=0.24,
        origin=Origin(
            xyz=(
                0.0,
                (
                    SPINDLE_JOURNAL_LENGTH
                    + SPINDLE_NOSE_LENGTH
                    + SPINDLE_COLLET_LENGTH
                )
                / 2.0
                - SPINDLE_JOURNAL_LENGTH / 2.0,
                0.0,
            ),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, WAY_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.45,
            lower=-SLIDE_TRAVEL / 2.0,
            upper=SLIDE_TRAVEL / 2.0,
        ),
    )
    model.articulation(
        "carriage_to_spindle",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=spindle,
        origin=Origin(xyz=(0.0, HOUSING_CENTER_Y, SPINDLE_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=4.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    rail = object_model.get_part("rail")
    carriage = object_model.get_part("carriage")
    spindle = object_model.get_part("spindle")
    slide = object_model.get_articulation("rail_to_carriage")
    rotate = object_model.get_articulation("carriage_to_spindle")

    left_way = rail.get_visual("left_way")
    right_way = rail.get_visual("right_way")
    left_runner = carriage.get_visual("left_runner")
    right_runner = carriage.get_visual("right_runner")
    carriage_body = carriage.get_visual("carriage_body")
    spindle_journal = spindle.get_visual("spindle_journal")
    front_retainer = spindle.get_visual("front_retainer")
    rear_retainer = spindle.get_visual("rear_retainer")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.expect_gap(
        carriage,
        rail,
        axis="z",
        max_gap=1e-6,
        max_penetration=0.0,
        positive_elem=left_runner,
        negative_elem=left_way,
        name="left_runner_seats_on_left_way",
    )
    ctx.expect_gap(
        carriage,
        rail,
        axis="z",
        max_gap=1e-6,
        max_penetration=0.0,
        positive_elem=right_runner,
        negative_elem=right_way,
        name="right_runner_seats_on_right_way",
    )
    ctx.expect_overlap(
        carriage,
        rail,
        axes="xy",
        min_overlap=0.010,
        elem_a=left_runner,
        elem_b=left_way,
        name="left_runner_overlaps_guide_way",
    )
    ctx.expect_overlap(
        carriage,
        rail,
        axes="xy",
        min_overlap=0.010,
        elem_a=right_runner,
        elem_b=right_way,
        name="right_runner_overlaps_guide_way",
    )
    ctx.expect_contact(
        spindle,
        carriage,
        elem_a=front_retainer,
        elem_b=carriage_body,
        name="front_retainer_contacts_housing_face",
    )
    ctx.expect_contact(
        spindle,
        carriage,
        elem_a=rear_retainer,
        elem_b=carriage_body,
        name="rear_retainer_contacts_housing_face",
    )
    ctx.expect_overlap(
        spindle,
        carriage,
        axes="xz",
        min_overlap=0.012,
        elem_a=spindle_journal,
        elem_b=carriage_body,
        name="spindle_axis_stays_coaxial_with_housing_bore",
    )

    with ctx.pose({slide: SLIDE_TRAVEL / 2.0}):
        ctx.expect_origin_gap(
            carriage,
            rail,
            axis="x",
            min_gap=SLIDE_TRAVEL / 2.0 - 1e-6,
            max_gap=SLIDE_TRAVEL / 2.0 + 1e-6,
            name="carriage_reaches_positive_slide_limit",
        )
        ctx.expect_gap(
            carriage,
            rail,
            axis="z",
            max_gap=1e-6,
            max_penetration=0.0,
            positive_elem=left_runner,
            negative_elem=left_way,
            name="left_runner_stays_seated_at_positive_limit",
        )
        ctx.expect_gap(
            carriage,
            rail,
            axis="z",
            max_gap=1e-6,
            max_penetration=0.0,
            positive_elem=right_runner,
            negative_elem=right_way,
            name="right_runner_stays_seated_at_positive_limit",
        )

    with ctx.pose({slide: -SLIDE_TRAVEL / 2.0}):
        ctx.expect_origin_gap(
            rail,
            carriage,
            axis="x",
            min_gap=SLIDE_TRAVEL / 2.0 - 1e-6,
            max_gap=SLIDE_TRAVEL / 2.0 + 1e-6,
            name="carriage_reaches_negative_slide_limit",
        )
        ctx.expect_gap(
            carriage,
            rail,
            axis="z",
            max_gap=1e-6,
            max_penetration=0.0,
            positive_elem=left_runner,
            negative_elem=left_way,
            name="left_runner_stays_seated_at_negative_limit",
        )
        ctx.expect_gap(
            carriage,
            rail,
            axis="z",
            max_gap=1e-6,
            max_penetration=0.0,
            positive_elem=right_runner,
            negative_elem=right_way,
            name="right_runner_stays_seated_at_negative_limit",
        )

    def _visual_center_z(part, visual_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=visual_name)
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    tool_flag_rest_z = _visual_center_z(spindle, "tool_flag")
    with ctx.pose({rotate: math.pi}):
        tool_flag_inverted_z = _visual_center_z(spindle, "tool_flag")
        ctx.expect_overlap(
            spindle,
            carriage,
            axes="xz",
            min_overlap=0.012,
            elem_a=spindle_journal,
            elem_b=carriage_body,
            name="spindle_journal_remains_in_housing_at_180_deg",
        )
        spindle_axis_z = ctx.part_world_position(spindle)[2]
    ctx.check(
        "tool_flag_flips_across_spindle_axis",
        tool_flag_rest_z is not None
        and tool_flag_inverted_z is not None
        and spindle_axis_z is not None
        and tool_flag_rest_z > spindle_axis_z
        and tool_flag_inverted_z < spindle_axis_z,
        details=(
            f"rest_z={tool_flag_rest_z}, inverted_z={tool_flag_inverted_z}, "
            f"axis_z={spindle_axis_z}"
        ),
    )

    ctx.check(
        "slide_motion_limits_cover_200mm",
        math.isclose(slide.motion_limits.lower, -0.100, abs_tol=1e-9)
        and math.isclose(slide.motion_limits.upper, 0.100, abs_tol=1e-9),
        details=(
            f"lower={slide.motion_limits.lower}, upper={slide.motion_limits.upper}"
        ),
    )
    ctx.check(
        "spindle_motion_limits_cover_180deg_each_way",
        math.isclose(rotate.motion_limits.lower, -math.pi, abs_tol=1e-9)
        and math.isclose(rotate.motion_limits.upper, math.pi, abs_tol=1e-9),
        details=(
            f"lower={rotate.motion_limits.lower}, upper={rotate.motion_limits.upper}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
