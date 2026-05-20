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


ROOT_PIN_RADIUS = 0.0040
ROOT_PIN_LENGTH = 0.028

ARM_OUTER_LENGTH = 0.270
ARM_OUTER_WIDTH = 0.052
ARM_OUTER_HEIGHT = 0.024
ARM_TUBE_X0 = 0.060
ARM_TUBE_X1 = ARM_TUBE_X0 + ARM_OUTER_LENGTH
ARM_RAIL_WIDTH = 0.008
ARM_RAIL_HEIGHT = 0.024
ARM_RAIL_Y = 0.018
ARM_GUIDE_WIDTH = 0.022
ARM_GUIDE_HEIGHT = 0.004
ARM_GUIDE_Z = -0.009

SLIDER_TRAVEL = 0.180
SLIDER_JOINT_X = 0.090
SLIDER_BAR_START_X = -0.030
SLIDER_BAR_LENGTH = 0.235
SLIDER_BAR_WIDTH = 0.020
SLIDER_BAR_HEIGHT = 0.014

HEAD_BORE_RADIUS = 0.0027
HEAD_PIN_RADIUS = 0.0024
HEAD_PIN_LENGTH = 0.022
HEAD_FORK_LENGTH = 0.018
HEAD_FORK_WIDTH = 0.024
HEAD_FORK_HEIGHT = 0.024
HEAD_JOINT_X = 0.245


def _make_mounting_bracket() -> cq.Workplane:
    plate = cq.Workplane("XY").box(0.008, 0.080, 0.160).translate((-0.044, 0.0, 0.0))
    plate = (
        plate.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(0.0, 0.045), (0.0, -0.045)])
        .hole(0.012, depth=0.016)
    )

    boss = cq.Workplane("XY").box(0.020, 0.018, 0.018).translate((-0.020, 0.0, 0.0))
    pivot_pin = cq.Workplane("XY").circle(ROOT_PIN_RADIUS).extrude(ROOT_PIN_LENGTH / 2.0, both=True)

    return plate.union(boss).union(pivot_pin)


def _make_primary_arm() -> cq.Workplane:
    root_lug = cq.Workplane("XY").box(0.022, 0.020, 0.018).translate((0.011, 0.0, 0.0))
    root_bore = cq.Workplane("XY").circle(ROOT_PIN_RADIUS).extrude(0.024, both=True)
    root_lug = root_lug.cut(root_bore)

    bridge = cq.Workplane("XY").box(0.036, 0.032, 0.012).translate((0.038, 0.0, 0.0))
    rail_center_x = (ARM_TUBE_X0 + ARM_TUBE_X1) / 2.0
    left_rail = cq.Workplane("XY").box(ARM_OUTER_LENGTH, ARM_RAIL_WIDTH, ARM_RAIL_HEIGHT).translate((rail_center_x, ARM_RAIL_Y, 0.0))
    right_rail = cq.Workplane("XY").box(ARM_OUTER_LENGTH, ARM_RAIL_WIDTH, ARM_RAIL_HEIGHT).translate((rail_center_x, -ARM_RAIL_Y, 0.0))
    guide = cq.Workplane("XY").box(ARM_OUTER_LENGTH, ARM_GUIDE_WIDTH, ARM_GUIDE_HEIGHT).translate((rail_center_x, 0.0, ARM_GUIDE_Z))

    return root_lug.union(bridge).union(left_rail).union(right_rail).union(guide)


def _make_center_extension() -> cq.Workplane:
    bar = (
        cq.Workplane("XY")
        .box(SLIDER_BAR_LENGTH, SLIDER_BAR_WIDTH, SLIDER_BAR_HEIGHT)
        .translate((SLIDER_BAR_START_X + SLIDER_BAR_LENGTH / 2.0, 0.0, 0.0))
    )

    clevis_base = cq.Workplane("XY").box(0.022, 0.018, 0.018).translate((0.216, 0.0, 0.0))
    ear = cq.Workplane("XY").box(HEAD_FORK_LENGTH, 0.004, HEAD_FORK_HEIGHT)
    left_ear = ear.translate((0.236, 0.010, 0.0))
    right_ear = ear.translate((0.236, -0.010, 0.0))
    cross_pin = (
        cq.Workplane("XZ")
        .circle(HEAD_PIN_RADIUS)
        .extrude(HEAD_PIN_LENGTH / 2.0, both=True)
        .translate((HEAD_JOINT_X, 0.0, 0.0))
    )

    return bar.union(clevis_base).union(left_ear).union(right_ear).union(cross_pin)


def _make_head_body() -> cq.Workplane:
    lug = cq.Workplane("XY").box(0.012, 0.016, 0.018).translate((0.006, 0.0, 0.0))
    lug_bore = cq.Workplane("XZ").circle(HEAD_BORE_RADIUS).extrude(0.014, both=True)
    lug = lug.cut(lug_bore)

    neck = cq.Workplane("XY").box(0.050, 0.012, 0.012).translate((0.030, 0.0, 0.0))
    plate = cq.Workplane("XY").box(0.006, 0.072, 0.092).translate((0.068, 0.0, 0.0))
    plate = plate.edges("|X").fillet(0.006)
    return lug.union(neck).union(plate)


def _axis_matches(axis: tuple[float, float, float], target: tuple[float, float, float], tol: float = 1e-8) -> bool:
    return all(abs(a - b) <= tol for a, b in zip(axis, target))


def _extent(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None, axis: int) -> float | None:
    if aabb is None:
        return None
    return aabb[1][axis] - aabb[0][axis]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_boom")

    bracket_finish = model.material("bracket_finish", rgba=(0.20, 0.21, 0.23, 1.0))
    arm_finish = model.material("arm_finish", rgba=(0.70, 0.72, 0.74, 1.0))
    slider_finish = model.material("slider_finish", rgba=(0.56, 0.59, 0.62, 1.0))
    head_finish = model.material("head_finish", rgba=(0.16, 0.17, 0.19, 1.0))
    plate_finish = model.material("plate_finish", rgba=(0.84, 0.85, 0.87, 1.0))

    bracket = model.part("mounting_bracket")
    bracket.visual(
        mesh_from_cadquery(_make_mounting_bracket(), "inspection_boom_mounting_bracket"),
        material=bracket_finish,
        name="bracket_body",
    )
    bracket.inertial = Inertial.from_geometry(
        Box((0.050, 0.080, 0.180)),
        mass=1.8,
        origin=Origin(xyz=(-0.020, 0.0, 0.0)),
    )

    arm = model.part("primary_arm")
    arm.visual(
        mesh_from_cadquery(_make_primary_arm(), "inspection_boom_primary_arm"),
        material=arm_finish,
        name="arm_body",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.330, ARM_OUTER_WIDTH, 0.040)),
        mass=1.2,
        origin=Origin(xyz=(0.165, 0.0, 0.0)),
    )

    extension = model.part("center_extension")
    extension.visual(
        mesh_from_cadquery(_make_center_extension(), "inspection_boom_center_extension"),
        material=slider_finish,
        name="extension_body",
    )
    extension.inertial = Inertial.from_geometry(
        Box((0.250, HEAD_FORK_WIDTH, HEAD_FORK_HEIGHT)),
        mass=0.75,
        origin=Origin(xyz=(0.105, 0.0, 0.0)),
    )

    head = model.part("head_plate")
    head.visual(
        mesh_from_cadquery(_make_head_body(), "inspection_boom_head_body"),
        material=plate_finish,
        name="head_body",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.060, 0.072, 0.092)),
        mass=0.28,
        origin=Origin(xyz=(0.028, 0.0, 0.0)),
    )

    model.articulation(
        "root_pivot",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=arm,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.1, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "center_slide",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=extension,
        origin=Origin(xyz=(SLIDER_JOINT_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.20, lower=0.0, upper=SLIDER_TRAVEL),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=extension,
        child=head,
        origin=Origin(xyz=(HEAD_JOINT_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.70, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("mounting_bracket")
    arm = object_model.get_part("primary_arm")
    extension = object_model.get_part("center_extension")
    head = object_model.get_part("head_plate")

    root_pivot = object_model.get_articulation("root_pivot")
    center_slide = object_model.get_articulation("center_slide")
    head_tilt = object_model.get_articulation("head_tilt")

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
    ctx.allow_overlap(
        arm,
        extension,
        reason=(
            "The center stage is modeled as a captured telescoping slider that shares "
            "the primary arm's guide envelope in the fully retracted pose."
        ),
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(bracket, arm, name="root_pivot_is_seated_in_bracket")
    ctx.expect_contact(arm, extension, name="center_stage_is_supported_when_retracted")
    ctx.expect_contact(extension, head, name="head_cross_pin_is_seated")
    ctx.expect_within(extension, arm, axes="yz", margin=0.0, name="extension_stays_within_arm_section")

    ctx.check(
        "joint_axes_match_mechanism",
        _axis_matches(root_pivot.axis, (0.0, 0.0, 1.0))
        and _axis_matches(center_slide.axis, (1.0, 0.0, 0.0))
        and _axis_matches(head_tilt.axis, (0.0, 1.0, 0.0)),
        details=(
            f"root={root_pivot.axis}, slide={center_slide.axis}, head={head_tilt.axis}"
        ),
    )
    ctx.check(
        "joint_limits_match_expected_ranges",
        root_pivot.motion_limits is not None
        and center_slide.motion_limits is not None
        and head_tilt.motion_limits is not None
        and root_pivot.motion_limits.lower is not None
        and root_pivot.motion_limits.upper is not None
        and center_slide.motion_limits.lower == 0.0
        and center_slide.motion_limits.upper == SLIDER_TRAVEL
        and head_tilt.motion_limits.lower is not None
        and head_tilt.motion_limits.upper is not None
        and root_pivot.motion_limits.lower < 0.0 < root_pivot.motion_limits.upper
        and head_tilt.motion_limits.lower < 0.0 < head_tilt.motion_limits.upper,
        details="unexpected articulation limits for the boom mechanism",
    )

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({center_slide: 0.160}):
        extended_head_pos = ctx.part_world_position(head)
    ctx.check(
        "center_stage_gains_reach_through_prismatic_travel",
        rest_head_pos is not None
        and extended_head_pos is not None
        and extended_head_pos[0] - rest_head_pos[0] > 0.140,
        details=f"rest={rest_head_pos}, extended={extended_head_pos}",
    )

    with ctx.pose({root_pivot: 0.75}):
        swung_head_pos = ctx.part_world_position(head)
    ctx.check(
        "root_pivot_swings_boom_sideways",
        rest_head_pos is not None
        and swung_head_pos is not None
        and abs(swung_head_pos[1] - rest_head_pos[1]) > 0.200,
        details=f"rest={rest_head_pos}, swung={swung_head_pos}",
    )

    rest_plate_aabb = ctx.part_element_world_aabb(head, elem="head_body")
    with ctx.pose({head_tilt: 0.60}):
        tilted_plate_aabb = ctx.part_element_world_aabb(head, elem="head_body")
    rest_plate_x = _extent(rest_plate_aabb, 0)
    tilted_plate_x = _extent(tilted_plate_aabb, 0)
    ctx.check(
        "head_plate_tilts_about_cross_pin",
        rest_plate_x is not None
        and tilted_plate_x is not None
        and tilted_plate_x > rest_plate_x + 0.030,
        details=f"rest_x_extent={rest_plate_x}, tilted_x_extent={tilted_plate_x}",
    )

    with ctx.pose({root_pivot: 0.30, center_slide: 0.120, head_tilt: 0.18}):
        ctx.expect_contact(extension, head, name="head_stays_seated_in_articulated_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
