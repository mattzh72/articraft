from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SUPPORT_LENGTH = 1.10
SUPPORT_WIDTH = 0.16
SUPPORT_HEIGHT = 0.08
RAIL_LENGTH = 0.94
RAIL_WIDTH = 0.08
RAIL_HEIGHT = 0.05
RAIL_CENTER_Z = -(SUPPORT_HEIGHT + RAIL_HEIGHT) / 2.0
RAIL_BOTTOM_Z = RAIL_CENTER_Z - RAIL_HEIGHT / 2.0

CARRIAGE_LENGTH = 0.18
CARRIAGE_WIDTH = 0.12
CARRIAGE_BODY_HEIGHT = 0.05
CARRIAGE_BODY_CENTER_Z = -0.02
CARRIAGE_CHEEK_THICKNESS = 0.02
CARRIAGE_CHEEK_CENTER_Y = 0.05
CARRIAGE_CHEEK_HEIGHT = 0.06
CARRIAGE_CHEEK_CENTER_Z = -0.03
SHOULDER_Z = -0.13

UPPER_ARM_LENGTH = 0.32
FOREARM_LENGTH = 0.28


def _box(
    size: tuple[float, float, float],
    center: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def make_top_support() -> cq.Workplane:
    beam = cq.Workplane("XY").box(SUPPORT_LENGTH, SUPPORT_WIDTH, SUPPORT_HEIGHT)
    beam = beam.edges("|X").fillet(0.006)

    rail = cq.Workplane("XY").box(RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT).translate(
        (0.0, 0.0, RAIL_CENTER_Z)
    )
    rail = rail.edges("|X").fillet(0.004)
    return beam.union(rail)


def make_carriage() -> cq.Workplane:
    slider_block = _box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, 0.04), (0.0, 0.0, -0.02))
    left_hanger = _box((0.05, 0.016, 0.12), (0.0, 0.035, -0.07))
    right_hanger = _box((0.05, 0.016, 0.12), (0.0, -0.035, -0.07))
    return slider_block.union(left_hanger).union(right_hanger)


def make_upper_arm() -> cq.Workplane:
    shoulder_crosshead = _box((0.055, 0.07, 0.03), (0.0, 0.0, -0.015))
    main_beam = _box((0.045, 0.024, 0.275), (0.0, 0.0, -0.1625))
    elbow_pad = _box((0.055, 0.045, 0.04), (0.0, 0.0, -0.30))

    return shoulder_crosshead.union(main_beam).union(elbow_pad)


def make_forearm() -> cq.Workplane:
    elbow_head = _box((0.05, 0.04, 0.04), (0.0, 0.0, -0.02))
    beam = _box((0.04, 0.022, 0.225), (0.0, 0.0, -0.1475))
    end_pad = _box((0.08, 0.035, 0.05), (0.0, 0.0, -0.275))

    return elbow_head.union(beam).union(end_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_slide_arm_unit")

    support_color = model.material("support_gray", rgba=(0.38, 0.40, 0.43, 1.0))
    carriage_color = model.material("carriage_graphite", rgba=(0.14, 0.15, 0.17, 1.0))
    arm_color = model.material("industrial_orange", rgba=(0.92, 0.48, 0.11, 1.0))
    arm_dark = model.material("arm_dark", rgba=(0.72, 0.38, 0.08, 1.0))

    top_support = model.part("top_support")
    top_support.visual(
        mesh_from_cadquery(make_top_support(), "top_support"),
        material=support_color,
        name="support_shell",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(make_carriage(), "carriage"),
        material=carriage_color,
        name="carriage_shell",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(make_upper_arm(), "upper_arm"),
        material=arm_color,
        name="upper_arm_shell",
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(make_forearm(), "forearm"),
        material=arm_dark,
        name="forearm_shell",
    )

    model.articulation(
        "support_to_carriage",
        ArticulationType.PRISMATIC,
        parent=top_support,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, RAIL_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=500.0,
            velocity=0.6,
            lower=-0.30,
            upper=0.30,
        ),
    )

    model.articulation(
        "carriage_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.8,
            lower=-1.10,
            upper=1.15,
        ),
    )

    model.articulation(
        "upper_arm_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.0, 0.0, -UPPER_ARM_LENGTH)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=2.0,
            lower=0.0,
            upper=2.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_support = object_model.get_part("top_support")
    carriage = object_model.get_part("carriage")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")

    slide = object_model.get_articulation("support_to_carriage")
    shoulder = object_model.get_articulation("carriage_to_upper_arm")
    elbow = object_model.get_articulation("upper_arm_to_forearm")

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

    ctx.expect_contact(
        carriage,
        top_support,
        contact_tol=0.001,
        name="carriage is borne by the support rail",
    )
    ctx.expect_contact(
        upper_arm,
        carriage,
        contact_tol=0.001,
        name="upper arm sits in the shoulder clevis",
    )
    ctx.expect_contact(
        forearm,
        upper_arm,
        contact_tol=0.001,
        name="forearm sits in the elbow clevis",
    )

    ctx.expect_origin_gap(
        top_support,
        carriage,
        axis="z",
        min_gap=0.08,
        max_gap=0.11,
        name="carriage hangs below the top support",
    )
    ctx.expect_origin_gap(
        carriage,
        upper_arm,
        axis="z",
        min_gap=0.12,
        max_gap=0.14,
        name="shoulder joint is below the carriage slide",
    )
    ctx.expect_origin_gap(
        upper_arm,
        forearm,
        axis="z",
        min_gap=0.28,
        max_gap=0.36,
        name="elbow is below the shoulder",
    )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.24}):
        moved_carriage = ctx.part_world_position(carriage)
        ctx.check(
            "slide joint translates the hanging carriage along x",
            rest_carriage is not None
            and moved_carriage is not None
            and moved_carriage[0] > rest_carriage[0] + 0.20
            and abs(moved_carriage[1] - rest_carriage[1]) < 0.002
            and abs(moved_carriage[2] - rest_carriage[2]) < 0.002,
            details=f"rest={rest_carriage}, moved={moved_carriage}",
        )

    rest_forearm_origin = ctx.part_world_position(forearm)
    with ctx.pose({shoulder: 0.85}):
        swung_forearm_origin = ctx.part_world_position(forearm)
        ctx.check(
            "shoulder revolute lifts the elbow forward",
            rest_forearm_origin is not None
            and swung_forearm_origin is not None
            and swung_forearm_origin[0] > rest_forearm_origin[0] + 0.18
            and swung_forearm_origin[2] > rest_forearm_origin[2] + 0.10,
            details=f"rest={rest_forearm_origin}, swung={swung_forearm_origin}",
        )

    def aabb_center(
        aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None,
    ) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3))

    with ctx.pose({shoulder: 0.85, elbow: 0.0}):
        straight_forearm_center = aabb_center(ctx.part_world_aabb(forearm))
    with ctx.pose({shoulder: 0.85, elbow: 1.10}):
        folded_forearm_center = aabb_center(ctx.part_world_aabb(forearm))
        ctx.check(
            "elbow revolute folds the forearm upward",
            straight_forearm_center is not None
            and folded_forearm_center is not None
            and folded_forearm_center[0] > straight_forearm_center[0] + 0.015
            and folded_forearm_center[2] > straight_forearm_center[2] + 0.07,
            details=f"straight={straight_forearm_center}, folded={folded_forearm_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
