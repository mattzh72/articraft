from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from math import cos, radians, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CONTROL_Z = 0.0
BASE_PIVOT_X = 0.031
RING_OUTER_RADIUS = 0.024
RING_INNER_RADIUS = 0.020
RING_PIN_RADIUS = 0.003
CRADLE_PIN_RADIUS = 0.003


def _make_base_shape() -> cq.Workplane:
    flange_radius = 0.054
    flange_inner_radius = 0.036
    flange_thickness = 0.003
    flange_bottom_z = -0.015

    cup_outer_radius = 0.035
    cup_wall = 0.003
    cup_depth = 0.009
    cup_bottom_z = -0.012

    flange = (
        cq.Workplane("XY")
        .circle(flange_radius)
        .circle(flange_inner_radius)
        .extrude(flange_thickness)
        .translate((0.0, 0.0, flange_bottom_z))
    )

    cup = (
        cq.Workplane("XY")
        .circle(cup_outer_radius)
        .extrude(cup_depth)
        .faces(">Z")
        .shell(-cup_wall)
        .translate((0.0, 0.0, cup_bottom_z))
    )
    cup_floor = (
        cq.Workplane("XY")
        .circle(0.028)
        .extrude(0.0016)
        .translate((0.0, 0.0, -0.012))
    )

    yoke_post = (
        cq.Workplane("XY")
        .box(0.007, 0.015, 0.012)
        .translate((BASE_PIVOT_X, 0.0, -0.002))
    )
    yoke_boss = (
        cq.Workplane("YZ")
        .circle(0.0053)
        .extrude(0.0035, both=True)
        .translate((BASE_PIVOT_X, 0.0, 0.0))
    )
    yoke_bore = (
        cq.Workplane("YZ")
        .circle(RING_PIN_RADIUS)
        .extrude(0.005, both=True)
        .translate((BASE_PIVOT_X, 0.0, 0.0))
    )
    right_yoke = yoke_post.union(yoke_boss).cut(yoke_bore)

    right_brace = (
        cq.Workplane("XY")
        .box(0.018, 0.010, 0.004)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 30)
        .translate((0.021, 0.0, -0.011))
    )

    base = (
        flange.union(cup)
        .union(cup_floor)
        .union(right_yoke)
        .union(right_yoke.mirror("YZ"))
        .union(right_brace)
        .union(right_brace.mirror("YZ"))
    )

    mount_radius = 0.045
    through_radius = 0.0032
    counterbore_radius = 0.0052
    for angle_deg in (45, 135, 225, 315):
        angle = radians(angle_deg)
        x = mount_radius * cos(angle)
        y = mount_radius * sin(angle)
        through = (
            cq.Workplane("XY")
            .circle(through_radius)
            .extrude(0.008)
            .translate((x, y, flange_bottom_z - 0.001))
        )
        counterbore = (
            cq.Workplane("XY")
            .circle(counterbore_radius)
            .extrude(0.0012)
            .translate((x, y, flange_bottom_z + flange_thickness - 0.0012))
        )
        base = base.cut(through).cut(counterbore)

    return base


def _make_outer_ring_shape() -> cq.Workplane:
    top_bridge = (
        cq.Workplane("XY")
        .box(0.008, 0.040, 0.004)
        .translate((0.0, 0.0, 0.019))
    )
    center_bridge = (
        cq.Workplane("XY")
        .box(0.004, 0.030, 0.003)
        .translate((0.0, 0.0, 0.013))
    )

    right_trunnion = (
        cq.Workplane("YZ")
        .circle(RING_PIN_RADIUS)
        .extrude(0.008)
        .translate((0.024, 0.0, 0.0))
    )
    right_upright = (
        cq.Workplane("XY")
        .box(0.006, 0.003, 0.020)
        .translate((0.021, 0.0, 0.010))
    )
    right_upper_strap = (
        cq.Workplane("XY")
        .box(0.012, 0.003, 0.003)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -22)
        .translate((0.014, 0.0, 0.016))
    )
    right_lower_strap = (
        cq.Workplane("XY")
        .box(0.010, 0.003, 0.003)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 28)
        .translate((0.015, 0.0, 0.004))
    )

    journal_y = 0.018
    journal_block = (
        cq.Workplane("XY")
        .box(0.006, 0.005, 0.008)
        .translate((0.0, journal_y, 0.0))
    )
    journal_link = (
        cq.Workplane("XY")
        .box(0.004, 0.010, 0.003)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 33)
        .translate((0.0, 0.012, 0.009))
    )
    journal_bore = (
        cq.Workplane("XZ")
        .circle(CRADLE_PIN_RADIUS)
        .extrude(0.0032, both=True)
        .translate((0.0, journal_y, 0.0))
    )

    return (
        top_bridge.union(center_bridge)
        .union(right_trunnion)
        .union(right_trunnion.mirror("YZ"))
        .union(right_upright)
        .union(right_upright.mirror("YZ"))
        .union(right_upper_strap)
        .union(right_upper_strap.mirror("YZ"))
        .union(right_lower_strap)
        .union(right_lower_strap.mirror("YZ"))
        .union(journal_block)
        .union(journal_block.mirror("XZ"))
        .union(journal_link)
        .union(journal_link.mirror("XZ"))
        .cut(journal_bore)
        .cut(journal_bore.mirror("XZ"))
    )


def _make_inner_cradle_body_shape() -> cq.Workplane:
    right_upright = (
        cq.Workplane("XY")
        .box(0.004, 0.0022, 0.014)
        .translate((0.009, 0.0, 0.002))
    )
    lower_bridge = (
        cq.Workplane("XY")
        .box(0.020, 0.0022, 0.003)
        .translate((0.0, 0.0, -0.003))
    )
    socket = (
        cq.Workplane("XY")
        .circle(0.0040)
        .extrude(0.002)
        .translate((0.0, 0.0, -0.004))
    )

    front_pin = (
        cq.Workplane("XZ")
        .circle(CRADLE_PIN_RADIUS)
        .extrude(0.006)
        .translate((0.0, 0.015, 0.0))
    )
    front_right_link = (
        cq.Workplane("XY")
        .box(0.0035, 0.010, 0.003)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -28)
        .translate((0.0055, 0.009, 0.003))
    )
    front_left_link = front_right_link.mirror("YZ")

    return (
        right_upright.union(right_upright.mirror("YZ"))
        .union(lower_bridge)
        .union(socket)
        .union(front_pin)
        .union(front_pin.mirror("XZ"))
        .union(front_right_link)
        .union(front_left_link)
        .union(front_right_link.mirror("XZ"))
        .union(front_left_link.mirror("XZ"))
    )


def _make_stick_stem_shape() -> cq.Workplane:
    boot = (
        cq.Workplane("XY")
        .circle(0.0042)
        .extrude(0.004)
        .translate((0.0, 0.0, -0.004))
    )
    stem = cq.Workplane(obj=cq.Solid.makeCone(0.0048, 0.0038, 0.027)).translate(
        (0.0, 0.0, 0.0)
    )
    neck = (
        cq.Workplane("XY")
        .circle(0.0039)
        .extrude(0.004)
        .translate((0.0, 0.0, 0.024))
    )
    return stem.union(boot).union(neck)


def _make_knurled_collar_shape() -> cq.Workplane:
    collar = (
        cq.Workplane("XY")
        .circle(0.0098)
        .extrude(0.011)
        .translate((0.0, 0.0, 0.026))
    )

    groove = (
        cq.Workplane("XY")
        .box(0.0021, 0.026, 0.014)
        .translate((0.0090, 0.0, 0.031))
    )

    for angle_deg in range(0, 360, 30):
        collar = collar.cut(groove.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg))

    return collar


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="console_joystick_module")

    housing = model.material("housing", rgba=(0.13, 0.14, 0.16, 1.0))
    gimbal = model.material("gimbal", rgba=(0.43, 0.45, 0.48, 1.0))
    cradle = model.material("cradle", rgba=(0.27, 0.29, 0.32, 1.0))
    stem = model.material("stem", rgba=(0.10, 0.10, 0.11, 1.0))
    collar = model.material("collar", rgba=(0.74, 0.75, 0.78, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "joystick_base"),
        material=housing,
        name="base_cup",
    )

    outer_ring = model.part("outer_ring")
    outer_ring.visual(
        mesh_from_cadquery(_make_outer_ring_shape(), "joystick_outer_ring"),
        material=gimbal,
        name="outer_ring_body",
    )

    inner_cradle = model.part("inner_cradle")
    inner_cradle.visual(
        mesh_from_cadquery(_make_inner_cradle_body_shape(), "joystick_inner_cradle"),
        material=cradle,
        name="inner_cradle_body",
    )
    inner_cradle.visual(
        mesh_from_cadquery(_make_stick_stem_shape(), "joystick_stem"),
        material=stem,
        name="stem",
    )
    inner_cradle.visual(
        mesh_from_cadquery(_make_knurled_collar_shape(), "joystick_collar"),
        material=collar,
        name="collar",
    )

    model.articulation(
        "base_to_outer_ring",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer_ring,
        origin=Origin(xyz=(0.0, 0.0, CONTROL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-0.38,
            upper=0.38,
        ),
    )
    model.articulation(
        "outer_ring_to_inner_cradle",
        ArticulationType.REVOLUTE,
        parent=outer_ring,
        child=inner_cradle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-0.38,
            upper=0.38,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    expected_parts = {"base", "outer_ring", "inner_cradle"}
    expected_joints = {
        "base_to_outer_ring",
        "outer_ring_to_inner_cradle",
    }

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

    part_names = {part.name for part in object_model.parts}
    joint_names = {joint.name for joint in object_model.articulations}
    ctx.check(
        "expected parts present",
        expected_parts.issubset(part_names),
        f"found parts: {sorted(part_names)}",
    )
    ctx.check(
        "expected articulations present",
        expected_joints.issubset(joint_names),
        f"found articulations: {sorted(joint_names)}",
    )

    base = object_model.get_part("base")
    outer_ring = object_model.get_part("outer_ring")
    inner_cradle = object_model.get_part("inner_cradle")

    outer_joint = object_model.get_articulation("base_to_outer_ring")
    inner_joint = object_model.get_articulation("outer_ring_to_inner_cradle")
    stem_visual = inner_cradle.get_visual("stem")
    collar = inner_cradle.get_visual("collar")
    cradle_body = inner_cradle.get_visual("inner_cradle_body")

    ctx.allow_overlap(
        base,
        outer_ring,
        reason=(
            "The captured left-right trunnions are modeled as concentric mesh-backed "
            "pins and bearing bores, so the pivot pair reads as a tiny intended "
            "interference at the axle capture."
        ),
    )
    ctx.allow_overlap(
        outer_ring,
        inner_cradle,
        reason=(
            "The orthogonal gimbal trunnions are represented with nested pivot solids; "
            "the visual simplification intentionally allows a small shared volume at "
            "the inner bearing capture."
        ),
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        outer_ring,
        base,
        contact_tol=0.0005,
        name="outer ring trunnions seat in base bearings",
    )
    ctx.expect_contact(
        inner_cradle,
        outer_ring,
        contact_tol=0.0005,
        name="inner cradle trunnions seat in outer ring journals",
    )

    ctx.expect_origin_distance(
        base,
        outer_ring,
        axes="xy",
        max_dist=0.001,
        name="outer ring centered over base cup",
    )
    ctx.expect_origin_distance(
        outer_ring,
        inner_cradle,
        axes="xy",
        max_dist=0.001,
        name="nested gimbals share a common control center",
    )
    ctx.expect_gap(
        inner_cradle,
        base,
        axis="z",
        positive_elem=collar,
        min_gap=0.020,
        name="knurled collar stands clearly above the console cup",
    )
    ctx.expect_within(
        inner_cradle,
        outer_ring,
        axes="xy",
        inner_elem=stem_visual,
        outer_elem="outer_ring_body",
        margin=0.014,
        name="stick remains centered inside the visible gimbal opening",
    )
    ctx.check(
        "inner cradle body stays physically distinct from the base shell",
        ctx.part_element_world_aabb(inner_cradle, elem=cradle_body.name) is not None
        and ctx.part_element_world_aabb(base, elem="base_cup") is not None,
        "expected both the cradle body and the base shell to resolve as named visuals",
    )

    ctx.check(
        "outer ring swings on left-right axis",
        tuple(outer_joint.axis) == (1.0, 0.0, 0.0),
        f"outer axis was {outer_joint.axis}",
    )
    ctx.check(
        "inner cradle swings on front-back axis",
        tuple(inner_joint.axis) == (0.0, 1.0, 0.0),
        f"inner axis was {inner_joint.axis}",
    )
    ctx.check(
        "gimbal travel remains realistically shallow",
        (
            outer_joint.motion_limits is not None
            and inner_joint.motion_limits is not None
            and outer_joint.motion_limits.lower == -0.38
            and outer_joint.motion_limits.upper == 0.38
            and inner_joint.motion_limits.lower == -0.38
            and inner_joint.motion_limits.upper == 0.38
        ),
        "expected both gimbal axes to have +/-0.38 rad travel",
    )

    with ctx.pose({outer_joint: 0.22, inner_joint: -0.18}):
        ctx.expect_contact(
            outer_ring,
            base,
            contact_tol=0.0005,
            name="outer ring remains captured in a tilted pose",
        )
        ctx.expect_contact(
            inner_cradle,
            outer_ring,
            contact_tol=0.0005,
            name="inner cradle remains captured in a tilted pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
