from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _louver_blade_mesh(length: float, depth: float, thickness: float):
    """Rounded plantation-shutter blade, modeled along local +X."""
    profile = rounded_rect_profile(thickness, depth, min(thickness, depth) * 0.48, corner_segments=8)
    return ExtrudeGeometry(profile, length, center=True).rotate_y(math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plantation_shutter_panel")

    painted_wood = Material("painted_warm_white", color=(0.88, 0.84, 0.74, 1.0))
    edge_shadow = Material("soft_shadow_in_gaps", color=(0.36, 0.32, 0.25, 1.0))
    hinge_metal = Material("brushed_hinge_metal", color=(0.63, 0.60, 0.54, 1.0))
    pin_metal = Material("dark_pivot_pins", color=(0.16, 0.15, 0.14, 1.0))

    outer_w = 0.80
    outer_h = 1.48
    frame_w = 0.070
    frame_d = 0.075
    panel_w = 0.640
    panel_h = 1.320
    panel_d = 0.045
    panel_y = -0.040
    hinge_x = -0.330
    hinge_y = 0.052

    outer_frame = model.part("outer_frame")
    outer_frame.visual(
        Box((frame_w, frame_d, outer_h)),
        origin=Origin(xyz=(-outer_w / 2.0 + frame_w / 2.0, 0.0, 0.0)),
        material=painted_wood,
        name="hinge_jamb",
    )
    outer_frame.visual(
        Box((frame_w, frame_d, outer_h)),
        origin=Origin(xyz=(outer_w / 2.0 - frame_w / 2.0, 0.0, 0.0)),
        material=painted_wood,
        name="strike_jamb",
    )
    outer_frame.visual(
        Box((outer_w, frame_d, frame_w)),
        origin=Origin(xyz=(0.0, 0.0, outer_h / 2.0 - frame_w / 2.0)),
        material=painted_wood,
        name="head_rail",
    )
    outer_frame.visual(
        Box((outer_w, frame_d, frame_w)),
        origin=Origin(xyz=(0.0, 0.0, -outer_h / 2.0 + frame_w / 2.0)),
        material=painted_wood,
        name="sill_rail",
    )

    # A shallow dark reveal inside the fixed frame makes the open clearance read
    # as a real rabbet rather than a solid rectangle.
    outer_frame.visual(
        Box((outer_w - 2 * frame_w, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, 0.040, outer_h / 2.0 - frame_w - 0.009)),
        material=edge_shadow,
        name="top_reveal",
    )
    outer_frame.visual(
        Box((outer_w - 2 * frame_w, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, 0.040, -outer_h / 2.0 + frame_w + 0.009)),
        material=edge_shadow,
        name="bottom_reveal",
    )

    hinge_zs = (-0.46, 0.0, 0.46)
    for i, z in enumerate(hinge_zs):
        outer_frame.visual(
            Box((0.043, 0.010, 0.104)),
            origin=Origin(xyz=(hinge_x - 0.021, 0.040, z)),
            material=hinge_metal,
            name=f"frame_hinge_leaf_{i}",
        )
        for suffix, dz in (("lower", -0.034), ("upper", 0.034)):
            outer_frame.visual(
                Cylinder(radius=0.008, length=0.030),
                origin=Origin(xyz=(hinge_x, hinge_y, z + dz)),
                material=hinge_metal,
                name=f"frame_hinge_{suffix}_{i}",
            )

    panel = model.part("panel")
    stile_w = 0.052
    rail_h = 0.060
    panel.visual(
        Box((stile_w, panel_d, panel_h)),
        origin=Origin(xyz=(stile_w / 2.0, panel_y, 0.0)),
        material=painted_wood,
        name="hinge_stile",
    )
    panel.visual(
        Box((stile_w, panel_d, panel_h)),
        origin=Origin(xyz=(panel_w - stile_w / 2.0, panel_y, 0.0)),
        material=painted_wood,
        name="strike_stile",
    )
    panel.visual(
        Box((panel_w, panel_d, rail_h)),
        origin=Origin(xyz=(panel_w / 2.0, panel_y, panel_h / 2.0 - rail_h / 2.0)),
        material=painted_wood,
        name="top_panel_rail",
    )
    panel.visual(
        Box((panel_w, panel_d, rail_h)),
        origin=Origin(xyz=(panel_w / 2.0, panel_y, -panel_h / 2.0 + rail_h / 2.0)),
        material=painted_wood,
        name="bottom_panel_rail",
    )
    panel.visual(
        Box((0.010, 0.010, panel_h - 2.0 * rail_h)),
        origin=Origin(xyz=(stile_w + 0.005, panel_y + panel_d / 2.0 + 0.004, 0.0)),
        material=edge_shadow,
        name="inner_left_shadow",
    )
    panel.visual(
        Box((0.010, 0.010, panel_h - 2.0 * rail_h)),
        origin=Origin(xyz=(panel_w - stile_w - 0.005, panel_y + panel_d / 2.0 + 0.004, 0.0)),
        material=edge_shadow,
        name="inner_right_shadow",
    )
    panel.visual(
        Box((0.030, 0.048, 0.100)),
        origin=Origin(xyz=(panel_w / 2.0, 0.004, panel_h / 2.0 - rail_h - 0.050)),
        material=hinge_metal,
        name="top_rod_clip",
    )
    panel.visual(
        Box((0.030, 0.048, 0.100)),
        origin=Origin(xyz=(panel_w / 2.0, 0.004, -panel_h / 2.0 + rail_h + 0.050)),
        material=hinge_metal,
        name="bottom_rod_clip",
    )
    for i, z in enumerate(hinge_zs):
        panel.visual(
            Box((0.046, 0.012, 0.104)),
            origin=Origin(xyz=(0.024, -0.012, z)),
            material=hinge_metal,
            name=f"panel_hinge_leaf_{i}",
        )
        panel.visual(
            Cylinder(radius=0.008, length=0.032),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=hinge_metal,
            name=f"panel_hinge_knuckle_{i}",
        )

    model.articulation(
        "frame_to_panel",
        ArticulationType.REVOLUTE,
        parent=outer_frame,
        child=panel,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.65),
    )

    louver_count = 9
    louver_span = panel_w - 2.0 * stile_w
    blade_len = louver_span - 0.026
    pivot_len = louver_span
    louver_depth = 0.096
    louver_thick = 0.014
    louver_pitch = -0.26
    louver_zs = [(-0.44 + i * 0.11) for i in range(louver_count)]
    blade_mesh = mesh_from_geometry(
        _louver_blade_mesh(blade_len, louver_depth, louver_thick),
        "rounded_louver_blade",
    )

    tilt_rod = model.part("tilt_rod")
    rod_len = louver_zs[-1] - louver_zs[0] + 0.150
    tilt_rod.visual(
        Box((0.020, 0.012, rod_len)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=painted_wood,
        name="vertical_rod",
    )
    for i, z in enumerate(louver_zs):
        tilt_rod.visual(
            Box((0.036, 0.002, 0.010)),
            origin=Origin(xyz=(0.0, -0.0065, z - 0.010)),
            material=pin_metal,
            name=f"rod_pin_{i}",
        )

    model.articulation(
        "panel_to_tilt_rod",
        ArticulationType.REVOLUTE,
        parent=panel,
        child=tilt_rod,
        origin=Origin(xyz=(panel_w / 2.0, 0.034, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.0, lower=-0.65, upper=0.65),
    )

    for i, z in enumerate(louver_zs):
        louver = model.part(f"louver_{i}")
        louver.visual(
            blade_mesh,
            origin=Origin(rpy=(louver_pitch, 0.0, 0.0)),
            material=painted_wood,
            name="blade",
        )
        louver.visual(
            Cylinder(radius=0.0045, length=pivot_len),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=pin_metal,
            name="pivot_pin",
        )
        louver.visual(
            Box((0.034, 0.028, 0.008)),
            origin=Origin(xyz=(0.0, 0.052, -0.012)),
            material=pin_metal,
            name="tilt_tab",
        )
        model.articulation(
            f"panel_to_louver_{i}",
            ArticulationType.REVOLUTE,
            parent=panel,
            child=louver,
            origin=Origin(xyz=(panel_w / 2.0, panel_y, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-0.85, upper=0.85),
            mimic=Mimic("panel_to_tilt_rod", multiplier=1.15, offset=0.0),
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

    outer_frame = object_model.get_part("outer_frame")
    panel = object_model.get_part("panel")
    tilt_rod = object_model.get_part("tilt_rod")
    mid_louver = object_model.get_part("louver_4")
    panel_hinge = object_model.get_articulation("frame_to_panel")
    tilt_joint = object_model.get_articulation("panel_to_tilt_rod")

    ctx.expect_within(
        panel,
        outer_frame,
        axes="xz",
        margin=0.002,
        name="closed panel sits inside the fixed frame outline",
    )
    ctx.expect_overlap(
        tilt_rod,
        panel,
        axes="z",
        min_overlap=0.80,
        name="tilt rod spans the louver bank",
    )
    ctx.expect_contact(
        mid_louver,
        panel,
        elem_a="pivot_pin",
        name="middle louver pivots reach the side stiles",
        contact_tol=0.001,
    )
    ctx.expect_contact(
        tilt_rod,
        panel,
        elem_a="vertical_rod",
        elem_b="top_rod_clip",
        name="tilt rod is retained by a rail-mounted guide clip",
        contact_tol=0.001,
    )

    closed_latch = ctx.part_element_world_aabb(panel, elem="strike_stile")
    with ctx.pose({panel_hinge: 1.0}):
        opened_latch = ctx.part_element_world_aabb(panel, elem="strike_stile")
    closed_y = (closed_latch[0][1] + closed_latch[1][1]) / 2.0 if closed_latch else None
    opened_y = (opened_latch[0][1] + opened_latch[1][1]) / 2.0 if opened_latch else None
    ctx.check(
        "panel swings outward on vertical side hinges",
        closed_y is not None and opened_y is not None and opened_y > closed_y + 0.25,
        details=f"closed_y={closed_y}, opened_y={opened_y}",
    )

    rest_blade = ctx.part_element_world_aabb(mid_louver, elem="blade")
    with ctx.pose({tilt_joint: 0.560}):
        tilted_blade = ctx.part_element_world_aabb(mid_louver, elem="blade")
    rest_z = (rest_blade[1][2] - rest_blade[0][2]) if rest_blade else None
    tilted_z = (tilted_blade[1][2] - tilted_blade[0][2]) if tilted_blade else None
    ctx.check(
        "tilt rod motion rotates the louver blade",
        rest_z is not None and tilted_z is not None and tilted_z > rest_z + 0.015,
        details=f"rest_z_extent={rest_z}, tilted_z_extent={tilted_z}",
    )

    return ctx.report()


object_model = build_object_model()
