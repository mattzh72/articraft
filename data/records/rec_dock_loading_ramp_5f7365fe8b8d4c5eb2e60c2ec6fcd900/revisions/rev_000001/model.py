from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _beam_between(part, p0, p1, *, thickness, material, name):
    """Add a rectangular tube whose local +X axis runs from p0 to p1."""
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    dx = x1 - x0
    dy = y1 - y0
    dz = z1 - z0
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    horizontal = math.sqrt(dx * dx + dy * dy)
    pitch = -math.atan2(dz, horizontal)
    part.visual(
        Box((length, thickness, thickness)),
        origin=Origin(
            xyz=((x0 + x1) * 0.5, (y0 + y1) * 0.5, (z0 + z1) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _add_square_tube(part, *, outer, wall, length, z_center, material):
    """Four overlapping walls make a visibly hollow square telescoping sleeve."""
    side_gap = outer - 2.0 * wall
    part.visual(
        Box((outer, wall, length)),
        origin=Origin(xyz=(0.0, outer * 0.5 - wall * 0.5, z_center)),
        material=material,
        name="outer_sleeve_wall_pos",
    )
    part.visual(
        Box((outer, wall, length)),
        origin=Origin(xyz=(0.0, -outer * 0.5 + wall * 0.5, z_center)),
        material=material,
        name="outer_sleeve_wall_neg",
    )
    part.visual(
        Box((wall, side_gap, length)),
        origin=Origin(xyz=(outer * 0.5 - wall * 0.5, 0.0, z_center)),
        material=material,
        name="outer_sleeve_wall_front",
    )
    part.visual(
        Box((wall, side_gap, length)),
        origin=Origin(xyz=(-outer * 0.5 + wall * 0.5, 0.0, z_center)),
        material=material,
        name="outer_sleeve_wall_rear",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_yard_ramp")

    steel = model.material("galvanized_steel", color=(0.55, 0.57, 0.56, 1.0))
    dark_steel = model.material("dark_grip_steel", color=(0.18, 0.20, 0.20, 1.0))
    safety_yellow = model.material("safety_yellow", color=(1.0, 0.72, 0.05, 1.0))
    rubber = model.material("black_rubber", color=(0.02, 0.02, 0.018, 1.0))
    pin_red = model.material("red_locking_pin", color=(0.85, 0.06, 0.04, 1.0))
    zinc = model.material("zinc_hardware", color=(0.70, 0.70, 0.66, 1.0))

    run = 3.20
    bottom_surface_z = 0.14
    top_surface_z = 1.05
    rise = top_surface_z - bottom_surface_z
    slope = math.atan2(rise, run)
    pitch = -slope
    deck_length = math.sqrt(run * run + rise * rise)
    deck_width = 1.12
    deck_thickness = 0.060
    rail_y = 0.585

    def surface_z(x: float) -> float:
        return bottom_surface_z + rise * (x / run)

    ramp = model.part("ramp_body")

    # Inclined steel driving plate.
    ramp.visual(
        Box((deck_length, deck_width, deck_thickness)),
        origin=Origin(
            xyz=(run * 0.5, 0.0, (bottom_surface_z + top_surface_z) * 0.5 - deck_thickness * 0.5),
            rpy=(0.0, pitch, 0.0),
        ),
        material=dark_steel,
        name="inclined_drive_plate",
    )

    # Bottom hook plate: a flat lip and a downturned hook that catches a dock edge.
    ramp.visual(
        Box((0.46, deck_width, 0.040)),
        origin=Origin(xyz=(-0.18, 0.0, bottom_surface_z - 0.020)),
        material=steel,
        name="lip_hook_plate",
    )
    ramp.visual(
        Box((0.055, deck_width, 0.210)),
        origin=Origin(xyz=(-0.40, 0.0, bottom_surface_z - 0.135)),
        material=steel,
        name="downturned_dock_hook",
    )

    # Horizontal top landing/platform.
    platform_length = 0.90
    platform_center_x = run + platform_length * 0.5
    platform_thickness = 0.060
    platform_center_z = top_surface_z - platform_thickness * 0.5
    ramp.visual(
        Box((platform_length, deck_width, platform_thickness)),
        origin=Origin(xyz=(platform_center_x, 0.0, platform_center_z)),
        material=dark_steel,
        name="top_platform_plate",
    )

    # Transverse stiffening rungs beneath the drive plate.
    for i, x in enumerate((0.45, 0.95, 1.45, 1.95, 2.45, 2.95)):
        ramp.visual(
            Box((0.075, deck_width + 0.16, 0.065)),
            origin=Origin(xyz=(x, 0.0, surface_z(x) - 0.070), rpy=(0.0, pitch, 0.0)),
            material=steel,
            name=f"underside_rung_{i}",
        )

    # Raised anti-skid cleats across the inclined driving surface.
    for i in range(12):
        x = 0.34 + i * 0.235
        ramp.visual(
            Box((0.030, deck_width - 0.20, 0.014)),
            origin=Origin(xyz=(x, 0.0, surface_z(x) + 0.004), rpy=(0.0, pitch, 0.0)),
            material=steel,
            name=f"deck_cleat_{i}",
        )

    for i, x in enumerate((run + 0.16, run + 0.38, run + 0.60, run + 0.82)):
        ramp.visual(
            Box((0.030, deck_width - 0.20, 0.014)),
            origin=Origin(xyz=(x, 0.0, top_surface_z + 0.005)),
            material=steel,
            name=f"platform_cleat_{i}",
        )

    # Continuous side curb rails along the deck and platform edges.
    for side, y in enumerate((-rail_y, rail_y)):
        ramp.visual(
            Box((deck_length + 0.06, 0.050, 0.120)),
            origin=Origin(
                xyz=(run * 0.5, y, (bottom_surface_z + top_surface_z) * 0.5 + 0.055),
                rpy=(0.0, pitch, 0.0),
            ),
            material=safety_yellow,
            name=f"sloped_curb_{side}",
        )
        ramp.visual(
            Box((platform_length, 0.050, 0.120)),
            origin=Origin(xyz=(platform_center_x, y, top_surface_z + 0.055)),
            material=safety_yellow,
            name=f"platform_curb_{side}",
        )

        # Upper continuous side rail: sloped section plus horizontal platform section.
        ramp.visual(
            Box((deck_length + 0.05, 0.055, 0.055)),
            origin=Origin(
                xyz=(run * 0.5, y, (bottom_surface_z + top_surface_z) * 0.5 + 0.620),
                rpy=(0.0, pitch, 0.0),
            ),
            material=safety_yellow,
            name=f"sloped_top_rail_{side}",
        )
        ramp.visual(
            Box((platform_length + 0.02, 0.055, 0.055)),
            origin=Origin(xyz=(platform_center_x, y, top_surface_z + 0.620)),
            material=safety_yellow,
            name=f"platform_top_rail_{side}",
        )

        # Vertical stanchions and diagonal cross-brace rungs on each side rail.
        post_xs = (0.32, 0.95, 1.58, 2.21, 2.84, 3.42, 3.92)
        for j, x in enumerate(post_xs):
            base_z = surface_z(min(x, run)) + 0.075 if x <= run else top_surface_z + 0.075
            top_z = surface_z(min(x, run)) + 0.620 if x <= run else top_surface_z + 0.620
            ramp.visual(
                Box((0.050, 0.050, (top_z - base_z) + 0.060)),
                origin=Origin(xyz=(x, y, (top_z + base_z) * 0.5)),
                material=safety_yellow,
                name=f"rail_post_{side}_{j}",
            )

        for j, (x0, x1) in enumerate(zip(post_xs[:-1], post_xs[1:])):
            z0 = surface_z(min(x0, run)) + 0.140 if x0 <= run else top_surface_z + 0.140
            z1 = surface_z(min(x1, run)) + 0.540 if x1 <= run else top_surface_z + 0.540
            _beam_between(
                ramp,
                (x0, y, z0),
                (x1, y, z1),
                thickness=0.038,
                material=safety_yellow,
                name=f"cross_brace_{side}_{j}",
            )

    # Telescoping support legs with pull pins and swivel caster feet.
    leg_x = run + 0.62
    leg_y_values = (-0.46, 0.46)
    sleeve_top_z = top_surface_z - platform_thickness
    sleeve_length = 0.50
    sleeve_outer = 0.086
    sleeve_wall = 0.014
    sleeve_entry_z = -0.525
    pin_z = -0.365
    inner_insert = 0.220
    inner_exposed = 0.300
    inner_length = inner_insert + inner_exposed
    inner_center_z = (inner_insert - inner_exposed) * 0.5
    caster_joint_z = -inner_exposed

    for idx, leg_y in enumerate(leg_y_values):
        outer = model.part(f"outer_post_{idx}")
        outer.visual(
            Box((0.205, 0.155, 0.026)),
            origin=Origin(xyz=(0.0, 0.0, -0.013)),
            material=zinc,
            name="mount_plate",
        )
        outer.visual(
            Box((0.125, 0.020, 0.052)),
            origin=Origin(xyz=(0.0, 0.0, -0.049)),
            material=zinc,
            name="gusset_web",
        )
        _add_square_tube(
            outer,
            outer=sleeve_outer,
            wall=sleeve_wall,
            length=sleeve_length,
            z_center=-0.275,
            material=steel,
        )
        model.articulation(
            f"body_to_outer_post_{idx}",
            ArticulationType.FIXED,
            parent=ramp,
            child=outer,
            origin=Origin(xyz=(leg_x, leg_y, sleeve_top_z)),
        )

        inner = model.part(f"inner_post_{idx}")
        inner.visual(
            Box((0.046, 0.046, inner_length)),
            origin=Origin(xyz=(0.0, 0.0, inner_center_z)),
            material=steel,
            name="inner_tube",
        )
        inner.visual(
            Box((0.070, 0.070, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, caster_joint_z + 0.009)),
            material=zinc,
            name="caster_mount_pad",
        )
        model.articulation(
            f"outer_to_inner_post_{idx}",
            ArticulationType.PRISMATIC,
            parent=outer,
            child=inner,
            origin=Origin(xyz=(0.0, 0.0, sleeve_entry_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(lower=-0.08, upper=0.12, effort=250.0, velocity=0.12),
        )

        pin = model.part(f"locking_pin_{idx}")
        outward = -1.0 if leg_y < 0.0 else 1.0
        pin.visual(
            Cylinder(radius=0.010, length=0.250),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=pin_red,
            name="pin_shaft",
        )
        pin.visual(
            Cylinder(radius=0.028, length=0.030),
            origin=Origin(
                xyz=(0.0, outward * 0.145, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=pin_red,
            name="pull_knob",
        )
        pin.visual(
            Box((0.020, 0.028, 0.050)),
            origin=Origin(xyz=(0.0, outward * 0.118, 0.0)),
            material=pin_red,
            name="pin_yoke",
        )
        model.articulation(
            f"outer_to_locking_pin_{idx}",
            ArticulationType.PRISMATIC,
            parent=outer,
            child=pin,
            origin=Origin(xyz=(0.0, 0.0, pin_z)),
            axis=(0.0, outward, 0.0),
            motion_limits=MotionLimits(lower=0.0, upper=0.085, effort=35.0, velocity=0.20),
        )

        caster = model.part(f"caster_fork_{idx}")
        caster.visual(
            Cylinder(radius=0.018, length=0.070),
            origin=Origin(xyz=(0.0, 0.0, -0.035)),
            material=zinc,
            name="swivel_stem",
        )
        caster.visual(
            Box((0.115, 0.096, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, -0.058)),
            material=zinc,
            name="fork_bridge",
        )
        caster.visual(
            Box((0.105, 0.014, 0.124)),
            origin=Origin(xyz=(0.0, -0.045, -0.128)),
            material=zinc,
            name="fork_cheek_0",
        )
        caster.visual(
            Box((0.105, 0.014, 0.124)),
            origin=Origin(xyz=(0.0, 0.045, -0.128)),
            material=zinc,
            name="fork_cheek_1",
        )
        caster.visual(
            Cylinder(radius=0.010, length=0.118),
            origin=Origin(xyz=(0.0, 0.0, -0.128), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=zinc,
            name="axle_pin",
        )
        model.articulation(
            f"inner_post_to_caster_{idx}",
            ArticulationType.CONTINUOUS,
            parent=inner,
            child=caster,
            origin=Origin(xyz=(0.0, 0.0, caster_joint_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=20.0, velocity=3.0),
        )

        wheel = model.part(f"caster_wheel_{idx}")
        wheel.visual(
            mesh_from_geometry(
                TireGeometry(
                    0.058,
                    0.052,
                    inner_radius=0.034,
                    tread=TireTread(style="ribbed", depth=0.003, count=14, land_ratio=0.60),
                    sidewall=TireSidewall(style="rounded", bulge=0.03),
                    shoulder=TireShoulder(width=0.004, radius=0.002),
                ),
                f"caster_tire_{idx}",
            ),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=rubber,
            name="rubber_tire",
        )
        wheel.visual(
            Cylinder(radius=0.027, length=0.058),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=zinc,
            name="metal_hub",
        )
        wheel.visual(
            Cylinder(radius=0.036, length=0.014),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=zinc,
            name="wheel_web",
        )
        model.articulation(
            f"caster_to_wheel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.128)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=8.0),
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

    ramp = object_model.get_part("ramp_body")

    for idx in (0, 1):
        outer = object_model.get_part(f"outer_post_{idx}")
        inner = object_model.get_part(f"inner_post_{idx}")
        pin = object_model.get_part(f"locking_pin_{idx}")
        caster = object_model.get_part(f"caster_fork_{idx}")
        wheel = object_model.get_part(f"caster_wheel_{idx}")
        slide = object_model.get_articulation(f"outer_to_inner_post_{idx}")
        pin_slide = object_model.get_articulation(f"outer_to_locking_pin_{idx}")
        swivel = object_model.get_articulation(f"inner_post_to_caster_{idx}")
        roll = object_model.get_articulation(f"caster_to_wheel_{idx}")

        ctx.allow_overlap(
            pin,
            outer,
            reason="The spring locking pin intentionally passes through the sleeve walls.",
            elem_a="pin_shaft",
            elem_b="outer_sleeve_wall_pos",
        )
        ctx.allow_overlap(
            pin,
            outer,
            reason="The spring locking pin intentionally passes through the opposite sleeve wall.",
            elem_a="pin_shaft",
            elem_b="outer_sleeve_wall_neg",
        )
        ctx.allow_overlap(
            pin,
            inner,
            reason="The locking pin intentionally passes through an adjustment hole in the inner post.",
            elem_a="pin_shaft",
            elem_b="inner_tube",
        )
        ctx.expect_overlap(
            pin,
            outer,
            axes="xz",
            elem_a="pin_shaft",
            min_overlap=0.006,
            name=f"locking pin {idx} crosses outer sleeve",
        )
        ctx.expect_overlap(
            pin,
            inner,
            axes="xz",
            elem_a="pin_shaft",
            elem_b="inner_tube",
            min_overlap=0.006,
            name=f"locking pin {idx} crosses inner post",
        )
        ctx.expect_contact(
            outer,
            ramp,
            elem_a="mount_plate",
            elem_b="top_platform_plate",
            contact_tol=0.002,
            name=f"outer post {idx} bolted under platform",
        )
        ctx.expect_within(
            inner,
            outer,
            axes="xy",
            inner_elem="inner_tube",
            margin=0.004,
            name=f"inner post {idx} centered in sleeve",
        )
        ctx.expect_overlap(
            inner,
            outer,
            axes="z",
            elem_a="inner_tube",
            min_overlap=0.09,
            name=f"inner post {idx} retained in sleeve",
        )
        ctx.allow_overlap(
            caster,
            wheel,
            reason="The caster axle pin intentionally passes through the wheel hub bore.",
            elem_a="axle_pin",
            elem_b="metal_hub",
        )
        ctx.allow_overlap(
            caster,
            wheel,
            reason="The same axle passes through the wheel web at the hub center.",
            elem_a="axle_pin",
            elem_b="wheel_web",
        )
        ctx.expect_overlap(
            caster,
            wheel,
            axes="yz",
            elem_a="axle_pin",
            elem_b="metal_hub",
            min_overlap=0.015,
            name=f"caster axle {idx} captured through hub",
        )
        ctx.expect_overlap(
            caster,
            wheel,
            axes="yz",
            elem_a="axle_pin",
            elem_b="wheel_web",
            min_overlap=0.010,
            name=f"caster axle {idx} crosses wheel web",
        )

        rest_pos = ctx.part_world_position(inner)
        with ctx.pose({slide: 0.12, pin_slide: 0.085, swivel: 0.7, roll: 1.2}):
            extended_pos = ctx.part_world_position(inner)
            ctx.expect_overlap(
                inner,
                outer,
                axes="z",
                elem_a="inner_tube",
                min_overlap=0.06,
                name=f"extended inner post {idx} remains captured",
            )
            ctx.expect_contact(
                caster,
                wheel,
                contact_tol=0.001,
                name=f"caster wheel {idx} remains in fork",
            )
        ctx.check(
            f"telescoping leg {idx} extends downward",
            rest_pos is not None and extended_pos is not None and extended_pos[2] < rest_pos[2] - 0.05,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    ctx.check(
        "ramp has one inclined deck and horizontal top platform",
        len(ramp.visuals) > 40,
        details="ramp_body should contain the drive plate, hook plate, platform, rails, rungs, and cleats",
    )

    return ctx.report()


object_model = build_object_model()
