from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_RADIUS = 0.075
BASE_LOWER_HEIGHT = 0.022
BASE_UPPER_RADIUS = 0.050
BASE_UPPER_HEIGHT = 0.008
BASE_HEIGHT = BASE_LOWER_HEIGHT + BASE_UPPER_HEIGHT

POST_RADIUS = 0.015
POST_HEIGHT = 0.190

SUPPORT_OFFSET = 0.034
SUPPORT_COLLAR_RADIUS = 0.026
SUPPORT_BOSS_RADIUS = 0.009
SUPPORT_THICKNESS = 0.014
SUPPORT_WEB_WIDTH = 0.014
SUPPORT_SADDLE_DEPTH = 0.012
SUPPORT_SADDLE_WIDTH = 0.030

TAB_BARREL_OUTER_RADIUS = 0.013
TAB_ARM_LENGTH = 0.055
TAB_ARM_WIDTH = 0.012
TAB_ARM_HEIGHT = 0.010
OUTPUT_FACE_THICKNESS = 0.008
OUTPUT_FACE_WIDTH = 0.022
OUTPUT_FACE_HEIGHT = 0.020

TAB_LEVELS = (0.055, 0.105, 0.155)
TAB_ANGLES = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
TAB_LIMITS = (-0.90, 0.90)


def make_base_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(BASE_RADIUS)
        .extrude(BASE_LOWER_HEIGHT)
        .faces(">Z")
        .workplane()
        .circle(BASE_UPPER_RADIUS)
        .extrude(BASE_UPPER_HEIGHT)
    )


def make_support_shape() -> cq.Workplane:
    raw = (
        cq.Workplane("XY")
        .center(POST_RADIUS + SUPPORT_SADDLE_DEPTH / 2.0, 0.0)
        .rect(SUPPORT_SADDLE_DEPTH, SUPPORT_SADDLE_WIDTH)
        .extrude(SUPPORT_THICKNESS)
        .union(
            cq.Workplane("XY")
            .center(SUPPORT_OFFSET, 0.0)
            .circle(SUPPORT_BOSS_RADIUS)
            .extrude(SUPPORT_THICKNESS)
        )
        .union(
            cq.Workplane("XY")
            .center((POST_RADIUS + SUPPORT_OFFSET) / 2.0, 0.0)
            .rect(SUPPORT_OFFSET - POST_RADIUS, SUPPORT_WEB_WIDTH)
            .extrude(SUPPORT_THICKNESS)
        )
    )
    cleared = raw.cut(
        cq.Workplane("XY")
        .circle(POST_RADIUS)
        .extrude(SUPPORT_THICKNESS)
    )
    return cleared.translate((0.0, 0.0, -SUPPORT_THICKNESS / 2.0))


def make_tab_core_shape() -> cq.Workplane:
    arm_overlap = 0.003
    core = (
        cq.Workplane("XY")
        .circle(TAB_BARREL_OUTER_RADIUS)
        .extrude(TAB_ARM_HEIGHT)
        .union(
            cq.Workplane("XY")
            .center(TAB_BARREL_OUTER_RADIUS + TAB_ARM_LENGTH / 2.0 - arm_overlap / 2.0, 0.0)
            .rect(TAB_ARM_LENGTH + arm_overlap, TAB_ARM_WIDTH)
            .extrude(TAB_ARM_HEIGHT)
        )
    )
    return core


def polar_xyz(radius: float, angle: float, z: float) -> tuple[float, float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle), z)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_three_branch_fixture")

    base_mat = model.material("base_powder_coat", rgba=(0.18, 0.19, 0.20, 1.0))
    post_mat = model.material("post_satin_aluminum", rgba=(0.70, 0.71, 0.73, 1.0))
    support_mat = model.material("support_dark_alloy", rgba=(0.32, 0.33, 0.36, 1.0))
    tab_mat = model.material("tab_orange_enamel", rgba=(0.82, 0.38, 0.12, 1.0))
    face_mat = model.material("output_face_machined", rgba=(0.83, 0.84, 0.86, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_base_shape(), "fixture_base"),
        material=base_mat,
        name="base_shell",
    )

    post = model.part("post")
    post.visual(
        Cylinder(radius=POST_RADIUS, length=POST_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, POST_HEIGHT / 2.0)),
        material=post_mat,
        name="post_shaft",
    )

    model.articulation(
        "base_to_post",
        ArticulationType.FIXED,
        parent=base,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
    )

    for index, (level, angle) in enumerate(zip(TAB_LEVELS, TAB_ANGLES), start=1):
        support = model.part(f"support_{index}")
        support.visual(
            mesh_from_cadquery(make_support_shape(), f"support_{index}_body"),
            material=support_mat,
            name="support_body",
        )

        model.articulation(
            f"post_to_support_{index}",
            ArticulationType.FIXED,
            parent=post,
            child=support,
            origin=Origin(
                xyz=polar_xyz(0.0, angle, level),
                rpy=(0.0, 0.0, angle),
            ),
        )

        tab = model.part(f"tab_{index}")
        tab.visual(
            mesh_from_cadquery(make_tab_core_shape(), f"tab_{index}_core"),
            material=tab_mat,
            name="tab_core",
        )
        tab.visual(
            Box((OUTPUT_FACE_THICKNESS, OUTPUT_FACE_WIDTH, OUTPUT_FACE_HEIGHT)),
            origin=Origin(
                xyz=(
                    TAB_BARREL_OUTER_RADIUS
                    + TAB_ARM_LENGTH
                    + OUTPUT_FACE_THICKNESS / 2.0,
                    0.0,
                    OUTPUT_FACE_HEIGHT / 2.0,
                )
            ),
            material=face_mat,
            name="output_face",
        )

        model.articulation(
            f"support_{index}_to_tab_{index}",
            ArticulationType.REVOLUTE,
            parent=support,
            child=tab,
            origin=Origin(xyz=(SUPPORT_OFFSET, 0.0, SUPPORT_THICKNESS / 2.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=2.0,
                lower=TAB_LIMITS[0],
                upper=TAB_LIMITS[1],
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    base = object_model.get_part("base")
    post = object_model.get_part("post")

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
        mins, maxs = aabb
        return tuple((lo + hi) / 2.0 for lo, hi in zip(mins, maxs))

    ctx.expect_contact(post, base, name="post_seats_on_base")

    for index in range(1, 4):
        support = object_model.get_part(f"support_{index}")
        tab = object_model.get_part(f"tab_{index}")
        joint = object_model.get_articulation(f"support_{index}_to_tab_{index}")

        ctx.expect_contact(support, post, name=f"support_{index}_mounted_to_post")
        ctx.expect_contact(tab, support, name=f"tab_{index}_carried_by_support_{index}")
        ctx.expect_overlap(
            tab,
            support,
            axes="xy",
            min_overlap=0.010,
            name=f"tab_{index}_pivot_footprint_overlaps_support",
        )

        limits = joint.motion_limits
        ctx.check(
            f"tab_{index}_joint_axis_is_vertical",
            tuple(joint.axis) == (0.0, 0.0, 1.0),
            details=f"axis={joint.axis}",
        )
        ctx.check(
            f"tab_{index}_joint_limits_are_compact",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and abs(limits.lower - TAB_LIMITS[0]) < 1e-9
            and abs(limits.upper - TAB_LIMITS[1]) < 1e-9,
            details=f"limits={limits}",
        )

        with ctx.pose({joint: TAB_LIMITS[0]}):
            lower_face_aabb = ctx.part_element_world_aabb(tab, elem="output_face")
            lower_contact_ok = ctx.expect_contact(
                tab,
                support,
                name=f"tab_{index}_remains_pinned_at_lower_limit",
            )

        with ctx.pose({joint: TAB_LIMITS[1]}):
            upper_face_aabb = ctx.part_element_world_aabb(tab, elem="output_face")
            upper_contact_ok = ctx.expect_contact(
                tab,
                support,
                name=f"tab_{index}_remains_pinned_at_upper_limit",
            )

        if lower_face_aabb is None or upper_face_aabb is None:
            ctx.fail(
                f"tab_{index}_output_face_pose_trace_available",
                "missing output_face world AABB in sampled poses",
            )
        else:
            lower_center = aabb_center(lower_face_aabb)
            upper_center = aabb_center(upper_face_aabb)
            lateral_travel = math.hypot(
                upper_center[0] - lower_center[0],
                upper_center[1] - lower_center[1],
            )
            vertical_drift = abs(upper_center[2] - lower_center[2])
            ctx.check(
                f"tab_{index}_output_face_sweeps_horizontally",
                lower_contact_ok
                and upper_contact_ok
                and lateral_travel > 0.040
                and vertical_drift < 0.003,
                details=(
                    f"lateral_travel={lateral_travel:.4f}, "
                    f"vertical_drift={vertical_drift:.4f}"
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
