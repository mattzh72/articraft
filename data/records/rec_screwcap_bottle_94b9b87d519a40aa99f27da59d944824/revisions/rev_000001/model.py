from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _make_bottle_body() -> LatheGeometry:
    """Tall hollow bottle shell with a tapered shoulder, banded threads, and lip."""
    outer_profile = [
        (0.029, 0.000),
        (0.034, 0.006),
        (0.036, 0.018),
        (0.036, 0.168),
        (0.034, 0.188),
        (0.028, 0.204),
        (0.020, 0.218),
        (0.013, 0.220),
        (0.0148, 0.222),
        (0.0148, 0.224),
        (0.0130, 0.226),
        (0.013, 0.228),
        (0.012, 0.230),
        (0.0142, 0.2315),
        (0.0142, 0.2335),
        (0.0120, 0.2350),
        (0.0120, 0.2370),
        (0.0142, 0.2385),
        (0.0142, 0.2405),
        (0.0120, 0.2420),
        (0.0120, 0.2440),
        (0.0140, 0.2455),
        (0.0140, 0.2475),
        (0.0120, 0.2490),
        (0.0120, 0.258),
        (0.0140, 0.262),
        (0.0140, 0.267),
    ]
    inner_profile = [
        (0.026, 0.005),
        (0.032, 0.014),
        (0.032, 0.167),
        (0.031, 0.184),
        (0.026, 0.199),
        (0.017, 0.216),
        (0.009, 0.234),
        (0.009, 0.267),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=96,
        start_cap="round",
        end_cap="flat",
        lip_samples=8,
    )


def _make_cap() -> cq.Workplane:
    """Open-bottom screw closure with a hollow skirt and vertical grip ribs."""
    outer_radius = 0.0175
    inner_radius = 0.0151
    height = 0.047
    top_thickness = 0.006

    cap = cq.Workplane("XY").circle(outer_radius).extrude(height)
    rib_count = 28
    rib_depth = 0.0024
    rib_width = 0.0024
    rib_height = 0.035
    rib_z = 0.006
    for index in range(rib_count):
        angle_deg = 360.0 * index / rib_count
        rib = (
            cq.Workplane("XY")
            .box(rib_depth, rib_width, rib_height, centered=(True, True, False))
            .translate((outer_radius + rib_depth * 0.40, 0.0, rib_z))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        cap = cap.union(rib)

    cavity = cq.Workplane("XY").circle(inner_radius).extrude(height - top_thickness)
    cap = cap.cut(cavity)

    top_bead = (
        cq.Workplane("XY")
        .circle(outer_radius * 0.82)
        .circle(inner_radius * 0.70)
        .extrude(0.0015)
        .translate((0.0, 0.0, height))
    )
    return cap.union(top_bead)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_cap_bottle")

    bottle_plastic = model.material("clear_bottle_plastic", rgba=(0.72, 0.90, 1.0, 0.42))
    cap_plastic = model.material("blue_cap_plastic", rgba=(0.05, 0.20, 0.78, 1.0))

    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_geometry(_make_bottle_body(), "hollow_bottle_body"),
        material=bottle_plastic,
        name="body_shell",
    )

    cap = model.part("cap")
    cap.visual(
        mesh_from_cadquery(_make_cap(), "ribbed_screw_cap", tolerance=0.0007),
        material=cap_plastic,
        name="cap_shell",
    )

    model.articulation(
        "neck_to_cap",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.226)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    bottle = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    spin = object_model.get_articulation("neck_to_cap")

    ctx.allow_overlap(
        bottle,
        cap,
        elem_a="body_shell",
        elem_b="cap_shell",
        reason=(
            "The hollow screw cap intentionally encloses the threaded neck; "
            "the mesh collision proxy reports the captured threaded fit as an overlap."
        ),
    )
    ctx.check(
        "cap joint is continuous",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={spin.articulation_type}",
    )
    ctx.expect_origin_distance(
        cap,
        bottle,
        axes="xy",
        max_dist=0.0001,
        name="cap axis is coaxial with bottle neck",
    )
    ctx.expect_origin_gap(
        cap,
        bottle,
        axis="z",
        min_gap=0.220,
        max_gap=0.240,
        name="cap sits at the top end of the tall bottle",
    )
    ctx.expect_overlap(
        cap,
        bottle,
        axes="xy",
        min_overlap=0.025,
        elem_a="cap_shell",
        elem_b="body_shell",
        name="cap skirt surrounds the neck footprint",
    )

    bottle_aabb = ctx.part_world_aabb(bottle)
    cap_aabb = ctx.part_world_aabb(cap)
    if bottle_aabb is not None and cap_aabb is not None:
        (bmin, bmax), (cmin, cmax) = bottle_aabb, cap_aabb
        bottle_height = bmax[2] - bmin[2]
        bottle_diameter = max(bmax[0] - bmin[0], bmax[1] - bmin[1])
        cap_height = cmax[2] - cmin[2]
        cap_diameter = max(cmax[0] - cmin[0], cmax[1] - cmin[1])
        ctx.check(
            "bottle is tall and cap is small",
            bottle_height > 3.4 * bottle_diameter and cap_height < 0.25 * bottle_height
            and cap_diameter < 0.65 * bottle_diameter,
            details=(
                f"bottle_height={bottle_height:.3f}, bottle_diameter={bottle_diameter:.3f}, "
                f"cap_height={cap_height:.3f}, cap_diameter={cap_diameter:.3f}"
            ),
        )

    rest_position = ctx.part_world_position(cap)
    with ctx.pose({spin: 2.0 * math.pi}):
        spun_position = ctx.part_world_position(cap)
    ctx.check(
        "cap spins in place about the neck",
        rest_position is not None
        and spun_position is not None
        and max(abs(rest_position[i] - spun_position[i]) for i in range(3)) < 1e-6,
        details=f"rest={rest_position}, spun={spun_position}",
    )

    return ctx.report()


object_model = build_object_model()
