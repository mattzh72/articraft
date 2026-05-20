from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_mouth_soup_blender")

    satin_black = model.material("satin_black", rgba=(0.02, 0.022, 0.024, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.04, 0.045, 0.05, 1.0))
    smoky_glass = model.material("smoky_clear_jug", rgba=(0.65, 0.90, 1.0, 0.34))
    stainless = model.material("brushed_stainless", rgba=(0.78, 0.78, 0.74, 1.0))
    gasket = model.material("soft_gasket", rgba=(0.01, 0.012, 0.013, 1.0))
    accent = model.material("warm_indicator", rgba=(0.95, 0.55, 0.16, 1.0))

    base_top_z = 0.270

    base = model.part("base")
    base_body = LatheGeometry(
        [
            (0.000, 0.000),
            (0.170, 0.000),
            (0.215, 0.016),
            (0.226, 0.060),
            (0.210, 0.120),
            (0.178, 0.175),
            (0.150, 0.225),
            (0.138, base_top_z),
            (0.000, base_top_z),
        ],
        segments=80,
    )
    base.visual(
        mesh_from_geometry(base_body, "rounded_round_base"),
        material=satin_black,
        name="rounded_base",
    )
    base.visual(
        Cylinder(radius=0.116, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, base_top_z - 0.006)),
        material=dark_plastic,
        name="twist_socket_plate",
    )
    # Low bayonet-stop blocks sit inside the receiver and make the twist-lock
    # purpose visible without intersecting the jug collar.
    for i, x in enumerate((-0.070, 0.070)):
        base.visual(
            Box((0.052, 0.018, 0.020)),
            origin=Origin(xyz=(x, -0.055, base_top_z - 0.022)),
            material=accent,
            name=f"bayonet_stop_{i}",
        )

    jug = model.part("jug")
    jug_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.112, 0.035),
            (0.145, 0.035),
            (0.156, 0.060),
            (0.160, 0.260),
            (0.164, 0.610),
            (0.175, 0.730),
            (0.181, 0.765),
            (0.176, 0.785),
        ],
        inner_profile=[
            (0.078, 0.068),
            (0.128, 0.072),
            (0.142, 0.260),
            (0.148, 0.610),
            (0.158, 0.730),
            (0.160, 0.765),
        ],
        segments=96,
        lip_samples=8,
    )
    jug.visual(
        mesh_from_geometry(jug_shell, "wide_mouth_clear_jug_shell"),
        material=smoky_glass,
        name="jug_shell",
    )
    locking_collar = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.145, 0.000), (0.145, 0.080)],
        inner_profile=[(0.055, 0.000), (0.055, 0.080)],
        segments=72,
    )
    jug.visual(
        mesh_from_geometry(locking_collar, "annular_locking_collar"),
        material=dark_plastic,
        name="locking_collar",
    )
    for i, angle in enumerate((0.0, math.pi)):
        jug.visual(
            Box((0.080, 0.026, 0.020)),
            origin=Origin(
                xyz=(0.0, 0.152, 0.020),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_plastic,
            name=f"bayonet_lug_{i}",
        )
    jug.visual(
        mesh_from_geometry(
            TorusGeometry(0.040, 0.006, radial_segments=36, tubular_segments=12),
            "blade_bearing_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=gasket,
        name="blade_bearing_ring",
    )
    jug.visual(
        Cylinder(radius=0.0175, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.081)),
        material=gasket,
        name="drive_bushing",
    )
    for i, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        jug.visual(
            Box((0.040, 0.008, 0.012)),
            origin=Origin(
                xyz=(0.0365 * math.cos(angle), 0.0365 * math.sin(angle), 0.081),
                rpy=(0.0, 0.0, angle),
            ),
            material=gasket,
            name=f"bushing_spoke_{i}",
        )
    # A practical rear handle: two molded pads sink into the transparent wall
    # and a vertical round grip bridges them, so the assembly reads supported.
    jug.visual(
        Cylinder(radius=0.019, length=0.410),
        origin=Origin(xyz=(0.0, -0.230, 0.405)),
        material=dark_plastic,
        name="rear_handle_grip",
    )
    for name, z in (("upper_handle_pad", 0.595), ("lower_handle_pad", 0.255)):
        jug.visual(
            Box((0.100, 0.105, 0.040)),
            origin=Origin(xyz=(0.0, -0.185, z)),
            material=dark_plastic,
            name=name,
        )
    # Fixed hinge knuckles on the rear rim. The moving lid owns the middle
    # knuckle, leaving small axial gaps between the barrels.
    for i, x in enumerate((-0.062, 0.062)):
        jug.visual(
            Cylinder(radius=0.011, length=0.045),
            origin=Origin(
                xyz=(x, -0.170, 0.800),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_plastic,
            name=f"fixed_hinge_knuckle_{i}",
        )
        jug.visual(
            Box((0.038, 0.026, 0.052)),
            origin=Origin(xyz=(x, -0.188, 0.778)),
            material=dark_plastic,
            name=f"hinge_rim_pad_{i}",
        )

    blade = model.part("blade")
    blade.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.108,
                0.025,
                4,
                thickness=0.014,
                blade_pitch_deg=18.0,
                blade_sweep_deg=35.0,
                blade=FanRotorBlade(shape="broad", tip_pitch_deg=10.0, camber=0.08),
                hub=FanRotorHub(style="capped", bore_diameter=0.010),
            ),
            "stainless_cross_blade",
        ),
        material=stainless,
        name="cutting_blade",
    )
    blade.visual(
        Cylinder(radius=0.011, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=stainless,
        name="blade_drive_stem",
    )

    lid = model.part("lid")
    lid.visual(
        Cylinder(radius=0.166, length=0.025),
        origin=Origin(xyz=(0.0, 0.185, 0.000)),
        material=dark_plastic,
        name="lid_disk",
    )
    lid.visual(
        Cylinder(radius=0.110, length=0.012),
        origin=Origin(xyz=(0.0, 0.185, 0.018)),
        material=gasket,
        name="removable_center_cap",
    )
    lid.visual(
        Box((0.112, 0.072, 0.025)),
        origin=Origin(xyz=(0.0, 0.343, 0.004)),
        material=dark_plastic,
        name="pour_spout_cover",
    )
    lid.visual(
        Box((0.070, 0.064, 0.014)),
        origin=Origin(xyz=(0.0, 0.032, -0.004)),
        material=dark_plastic,
        name="hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.004, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="hinge_pin",
    )
    lid.visual(
        Cylinder(radius=0.011, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="moving_hinge_knuckle",
    )

    model.articulation(
        "twist_mount",
        ArticulationType.REVOLUTE,
        parent=base,
        child=jug,
        origin=Origin(xyz=(0.0, 0.0, base_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=-0.52, upper=0.52),
    )
    model.articulation(
        "blade_spin",
        ArticulationType.CONTINUOUS,
        parent=jug,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=120.0),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=jug,
        child=lid,
        origin=Origin(xyz=(0.0, -0.170, 0.800)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.75),
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

    base = object_model.get_part("base")
    jug = object_model.get_part("jug")
    lid = object_model.get_part("lid")
    blade = object_model.get_part("blade")
    twist = object_model.get_articulation("twist_mount")
    hinge = object_model.get_articulation("lid_hinge")
    spin = object_model.get_articulation("blade_spin")

    ctx.allow_overlap(
        jug,
        blade,
        elem_a="drive_bushing",
        elem_b="blade_drive_stem",
        reason="The blade drive stem is intentionally captured inside the molded bushing proxy.",
    )
    for knuckle in ("fixed_hinge_knuckle_0", "fixed_hinge_knuckle_1"):
        ctx.allow_overlap(
            jug,
            lid,
            elem_a=knuckle,
            elem_b="hinge_pin",
            reason="The metal hinge pin is intentionally captured through the fixed hinge knuckle.",
        )

    ctx.check(
        "jug uses a limited revolute twist mount",
        twist.articulation_type == ArticulationType.REVOLUTE
        and twist.motion_limits is not None
        and twist.motion_limits.lower < 0.0
        and twist.motion_limits.upper > 0.0,
        details=f"type={twist.articulation_type}, limits={twist.motion_limits}",
    )
    ctx.check(
        "blade is a continuous vertical rotor",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_gap(
        jug,
        base,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="locking_collar",
        negative_elem="rounded_base",
        name="jug collar sits on the round base",
    )
    ctx.expect_overlap(
        jug,
        base,
        axes="xy",
        min_overlap=0.20,
        elem_a="locking_collar",
        elem_b="rounded_base",
        name="twist mount is centered on the base",
    )
    ctx.expect_within(
        blade,
        jug,
        axes="xy",
        margin=0.0,
        inner_elem="cutting_blade",
        outer_elem="jug_shell",
        name="blade stays within the wide jug wall",
    )
    ctx.expect_within(
        blade,
        jug,
        axes="xy",
        margin=0.0,
        inner_elem="blade_drive_stem",
        outer_elem="drive_bushing",
        name="drive stem is centered in the bushing",
    )
    ctx.expect_overlap(
        blade,
        jug,
        axes="z",
        min_overlap=0.012,
        elem_a="blade_drive_stem",
        elem_b="drive_bushing",
        name="drive stem remains inserted in the bushing",
    )
    ctx.expect_gap(
        lid,
        jug,
        axis="z",
        min_gap=0.0,
        max_gap=0.006,
        positive_elem="lid_disk",
        negative_elem="jug_shell",
        name="closed lid rests just above the jug rim",
    )
    for knuckle in ("fixed_hinge_knuckle_0", "fixed_hinge_knuckle_1"):
        ctx.expect_within(
            lid,
            jug,
            axes="yz",
            margin=0.001,
            inner_elem="hinge_pin",
            outer_elem=knuckle,
            name=f"hinge pin is concentric in {knuckle}",
        )
        ctx.expect_overlap(
            lid,
            jug,
            axes="x",
            min_overlap=0.035,
            elem_a="hinge_pin",
            elem_b=knuckle,
            name=f"hinge pin passes through {knuckle}",
        )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({hinge: 1.25}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid hinge opens the pour spout upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.10,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    rest_blade = ctx.part_world_position(blade)
    with ctx.pose({spin: 2.0}):
        spun_blade = ctx.part_world_position(blade)
    ctx.check(
        "blade spin keeps the hub on the jug axis",
        rest_blade is not None
        and spun_blade is not None
        and abs(rest_blade[0] - spun_blade[0]) < 1e-6
        and abs(rest_blade[1] - spun_blade[1]) < 1e-6,
        details=f"rest={rest_blade}, spun={spun_blade}",
    )

    return ctx.report()


object_model = build_object_model()
