from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _tube_mesh(
    outer_profile: list[tuple[float, float]],
    inner_radius: float,
    name: str,
    *,
    segments: int = 72,
):
    """Build a lathed, open-bore stepped tube along local Z."""
    z0 = outer_profile[0][1]
    z1 = outer_profile[-1][1]
    inner_profile = [(inner_radius, z0), (inner_radius, z1)]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def _x_cylinder_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def _y_cylinder_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(-math.pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="push_pull_plunger_chain")

    parkerized = model.material("parkerized_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    machined = model.material("machined_steel", rgba=(0.55, 0.57, 0.60, 1.0))
    polished = model.material("polished_bearing_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    bronze = model.material("bronze_bushing", rgba=(0.70, 0.47, 0.20, 1.0))
    nitrile = model.material("black_nitrile_seals", rgba=(0.02, 0.02, 0.018, 1.0))
    primer = model.material("fabricated_gray_primer", rgba=(0.33, 0.35, 0.37, 1.0))
    fastener = model.material("black_oxide_fasteners", rgba=(0.03, 0.032, 0.035, 1.0))

    base = model.part("base_frame")
    base.visual(
        Box((1.58, 0.36, 0.045)),
        origin=Origin(xyz=(0.245, 0.0, 0.0225)),
        material=primer,
        name="skid_plate",
    )
    base.visual(
        Box((1.50, 0.045, 0.070)),
        origin=Origin(xyz=(0.25, 0.182, 0.080)),
        material=parkerized,
        name="side_rail_pos",
    )
    base.visual(
        Box((1.50, 0.045, 0.070)),
        origin=Origin(xyz=(0.25, -0.182, 0.080)),
        material=parkerized,
        name="side_rail_neg",
    )

    for x in (-0.39, 0.08, 0.53, 0.88):
        base.visual(
            Box((0.080, 0.360, 0.120)),
            origin=Origin(xyz=(x, 0.0, 0.105)),
            material=primer,
            name=f"crossmember_{int((x + 0.5) * 100):02d}",
        )

    # Stationary guide sleeve and visible gland/flange stack.
    outer_sleeve_mesh = _tube_mesh(
        [
            (0.082, -0.335),
            (0.082, -0.295),
            (0.058, -0.295),
            (0.058, -0.095),
            (0.062, -0.095),
            (0.062, -0.055),
            (0.058, -0.055),
            (0.058, 0.220),
            (0.086, 0.220),
            (0.086, 0.270),
        ],
        0.047,
        "outer_stepped_sleeve",
        segments=88,
    )
    base.visual(
        outer_sleeve_mesh,
        origin=Origin(xyz=(-0.145, 0.0, 0.225), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="outer_sleeve",
    )
    for x in (-0.405, 0.075):
        base.visual(
            Box((0.055, 0.145, 0.125)),
            origin=Origin(xyz=(x, 0.0, 0.1075)),
            material=primer,
            name=f"sleeve_saddle_{'rear' if x < 0 else 'front'}",
        )
        base.visual(
            Box((0.065, 0.020, 0.178)),
            origin=Origin(xyz=(x, 0.082, 0.158)),
            material=parkerized,
            name=f"saddle_cheek_pos_{'rear' if x < 0 else 'front'}",
        )
        base.visual(
            Box((0.065, 0.020, 0.178)),
            origin=Origin(xyz=(x, -0.082, 0.158)),
            material=parkerized,
            name=f"saddle_cheek_neg_{'rear' if x < 0 else 'front'}",
        )

    # Bronze guide bushings and black seal glands read as replaceable wear parts.
    for x in (-0.437, 0.125):
        base.visual(
            _tube_mesh([(0.054, -0.016), (0.054, 0.016)], 0.047, f"bronze_bushing_{x:.2f}", segments=64),
            origin=Origin(xyz=(x, 0.0, 0.225), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bronze,
            name=f"bushing_{'rear' if x < 0 else 'front'}",
        )
        base.visual(
            _tube_mesh([(0.061, -0.008), (0.061, 0.008)], 0.048, f"seal_gland_{x:.2f}", segments=64),
            origin=Origin(xyz=(x + (0.026 if x < 0 else -0.026), 0.0, 0.225), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=nitrile,
            name=f"seal_gland_{'rear' if x < 0 else 'front'}",
        )

    # Parallel reference guide rods and adjustable return-stop hardware.
    for y in (-0.132, 0.132):
        base.visual(
            Cylinder(radius=0.010, length=1.30),
            origin=_x_cylinder_origin(0.245, y, 0.150),
            material=polished,
            name=f"guide_rod_{'neg' if y < 0 else 'pos'}",
        )
        for x in (-0.43, 0.92):
            base.visual(
                Box((0.045, 0.042, 0.094)),
                origin=Origin(xyz=(x, y, 0.111)),
                material=parkerized,
                name=f"guide_rod_block_{'neg' if y < 0 else 'pos'}_{'rear' if x < 0 else 'front'}",
            )
        base.visual(
            Cylinder(radius=0.0065, length=0.78),
            origin=_x_cylinder_origin(0.37, y * 0.78, 0.305),
            material=polished,
            name=f"limit_rod_{'neg' if y < 0 else 'pos'}",
        )
        for x in (0.08, 0.66):
            base.visual(
                Box((0.020, 0.020, 0.260)),
                origin=Origin(xyz=(x, y * 0.78, 0.175)),
                material=parkerized,
                name=f"limit_post_{'neg' if y < 0 else 'pos'}_{'rear' if x < 0.2 else 'front'}",
            )
            base.visual(
                Cylinder(radius=0.017, length=0.014),
                origin=_x_cylinder_origin(x, y * 0.78, 0.305),
                material=fastener,
                name=f"limit_nut_{'neg' if y < 0 else 'pos'}_{'rear' if x < 0.2 else 'front'}",
            )

    # Removable inspection covers with proud cap screws, kept as bolted hard-surface plates.
    for y, suffix in ((0.112, "pos"), (-0.112, "neg")):
        base.visual(
            Box((0.300, 0.018, 0.072)),
            origin=Origin(xyz=(-0.160, y, 0.305)),
            material=primer,
            name=f"access_cover_{suffix}",
        )
        for x in (-0.285, -0.045):
            base.visual(
                Box((0.024, 0.018, 0.224)),
                origin=Origin(xyz=(x, y, 0.157)),
                material=parkerized,
                name=f"cover_standoff_{suffix}_{int((x + 0.3) * 1000):03d}",
            )
        for x in (-0.285, -0.175, -0.045):
            base.visual(
                Cylinder(radius=0.007, length=0.006),
                origin=_y_cylinder_origin(x, y + (0.012 if y > 0 else -0.012), 0.327),
                material=fastener,
                name=f"cover_screw_{suffix}_{int((x + 0.3) * 1000):03d}",
            )

    # Four slotted flange lugs around the stationary sleeve.
    for x in (-0.440, 0.122):
        for y in (-0.105, 0.105):
            base.visual(
                Box((0.070, 0.045, 0.018)),
                origin=Origin(xyz=(x, y, 0.225)),
                material=machined,
                name=f"flange_ear_{'rear' if x < 0 else 'front'}_{'neg' if y < 0 else 'pos'}",
            )
            base.visual(
                Cylinder(radius=0.008, length=0.012),
                origin=Origin(xyz=(x, y, 0.238)),
                material=fastener,
                name=f"flange_bolt_{'rear' if x < 0 else 'front'}_{'neg' if y < 0 else 'pos'}",
            )

    first = model.part("first_plunger")
    first.visual(
        _tube_mesh(
            [
                (0.043, -0.480),
                (0.043, -0.430),
                (0.034, -0.430),
                (0.034, 0.120),
                (0.041, 0.120),
                (0.041, 0.175),
                (0.055, 0.175),
                (0.055, 0.245),
            ],
            0.024,
            "first_plunger_tube",
            segments=80,
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="stage_body",
    )
    first.visual(
        Box((0.026, 0.164, 0.018)),
        origin=Origin(xyz=(0.185, 0.0, 0.052)),
        material=parkerized,
        name="upper_limit_flag",
    )
    first.visual(
        Box((0.026, 0.036, 0.058)),
        origin=Origin(xyz=(0.185, 0.086, 0.022)),
        material=parkerized,
        name="guide_shoe_pos",
    )
    first.visual(
        Box((0.026, 0.036, 0.058)),
        origin=Origin(xyz=(0.185, -0.086, 0.022)),
        material=parkerized,
        name="guide_shoe_neg",
    )
    first.visual(
        Cylinder(radius=0.011, length=0.188),
        origin=_y_cylinder_origin(0.185, 0.0, 0.052),
        material=fastener,
        name="shoe_crossbolt",
    )
    for y, suffix in ((0.132, "pos"), (-0.132, "neg")):
        first.visual(
            Box((0.048, 0.030, 0.026)),
            origin=Origin(xyz=(0.185, y, -0.052)),
            material=bronze,
            name=f"linear_bearing_pad_{suffix}",
        )
        first.visual(
            Box((0.044, 0.088, 0.018)),
            origin=Origin(xyz=(0.185, 0.088 if y > 0 else -0.088, -0.039)),
            material=parkerized,
            name=f"pad_bridge_{suffix}",
        )
        first.visual(
            Box((0.044, 0.018, 0.096)),
            origin=Origin(xyz=(0.185, 0.043 if y > 0 else -0.043, -0.007)),
            material=parkerized,
            name=f"pad_web_{suffix}",
        )

    second = model.part("second_plunger")
    second.visual(
        _tube_mesh(
            [
                (0.020, -0.330),
                (0.020, 0.150),
                (0.029, 0.150),
                (0.029, 0.190),
                (0.034, 0.190),
                (0.034, 0.238),
            ],
            0.012,
            "second_plunger_tube",
            segments=72,
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="stage_body",
    )
    second.visual(
        _tube_mesh(
            [(0.0245, -0.016), (0.0245, 0.016)],
            0.012,
            "second_rear_bearing_band",
            segments=64,
        ),
        origin=Origin(xyz=(-0.305, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bronze,
        name="rear_bearing_band",
    )
    second.visual(
        Box((0.020, 0.115, 0.016)),
        origin=Origin(xyz=(0.175, 0.0, 0.037)),
        material=parkerized,
        name="return_stop_tab",
    )
    second.visual(
        Cylinder(radius=0.0085, length=0.132),
        origin=_y_cylinder_origin(0.175, 0.0, 0.037),
        material=fastener,
        name="tab_pin",
    )

    core = model.part("core_rod")
    core.visual(
        Cylinder(radius=0.010, length=0.560),
        origin=_x_cylinder_origin(-0.065, 0.0, 0.0),
        material=polished,
        name="rod_shank",
    )
    core.visual(
        Cylinder(radius=0.0124, length=0.030),
        origin=_x_cylinder_origin(-0.320, 0.0, 0.0),
        material=bronze,
        name="rear_bearing_band",
    )
    core.visual(
        Cylinder(radius=0.017, length=0.035),
        origin=_x_cylinder_origin(0.215, 0.0, 0.0),
        material=machined,
        name="rod_collar",
    )
    core.visual(
        Box((0.040, 0.022, 0.052)),
        origin=Origin(xyz=(0.245, 0.026, 0.0)),
        material=machined,
        name="clevis_cheek_pos",
    )
    core.visual(
        Box((0.040, 0.022, 0.052)),
        origin=Origin(xyz=(0.245, -0.026, 0.0)),
        material=machined,
        name="clevis_cheek_neg",
    )
    core.visual(
        Box((0.030, 0.074, 0.052)),
        origin=Origin(xyz=(0.210, 0.0, 0.0)),
        material=machined,
        name="clevis_bridge",
    )
    core.visual(
        Cylinder(radius=0.010, length=0.090),
        origin=_y_cylinder_origin(0.255, 0.0, 0.0),
        material=fastener,
        name="clevis_pin",
    )

    model.articulation(
        "base_to_first",
        ArticulationType.PRISMATIC,
        parent=base,
        child=first,
        origin=Origin(xyz=(0.145, 0.0, 0.225)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=700.0, velocity=0.18, lower=0.0, upper=0.220),
        motion_properties=MotionProperties(damping=14.0, friction=18.0),
    )
    model.articulation(
        "first_to_second",
        ArticulationType.PRISMATIC,
        parent=first,
        child=second,
        origin=Origin(xyz=(0.265, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=430.0, velocity=0.16, lower=0.0, upper=0.160),
        motion_properties=MotionProperties(damping=10.0, friction=10.0),
    )
    model.articulation(
        "second_to_core",
        ArticulationType.PRISMATIC,
        parent=second,
        child=core,
        origin=Origin(xyz=(0.255, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.14, lower=0.0, upper=0.120),
        motion_properties=MotionProperties(damping=8.0, friction=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    first = object_model.get_part("first_plunger")
    second = object_model.get_part("second_plunger")
    core = object_model.get_part("core_rod")
    j1 = object_model.get_articulation("base_to_first")
    j2 = object_model.get_articulation("first_to_second")
    j3 = object_model.get_articulation("second_to_core")

    ctx.allow_overlap(
        first,
        second,
        elem_a="stage_body",
        elem_b="rear_bearing_band",
        reason="The second-stage bronze rear bearing band is intentionally captured inside the first plunger bore.",
    )
    ctx.allow_overlap(
        second,
        core,
        elem_a="stage_body",
        elem_b="rear_bearing_band",
        reason="The core rod rear bearing band is intentionally seated in the second plunger bore to show sliding support.",
    )

    ctx.expect_within(
        first,
        base,
        axes="yz",
        inner_elem="stage_body",
        outer_elem="outer_sleeve",
        margin=0.004,
        name="first plunger is centered in stationary sleeve envelope",
    )
    ctx.expect_overlap(
        first,
        base,
        axes="x",
        elem_a="stage_body",
        elem_b="outer_sleeve",
        min_overlap=0.22,
        name="first plunger retained in outer sleeve at rest",
    )
    ctx.expect_overlap(
        second,
        first,
        axes="x",
        elem_a="rear_bearing_band",
        elem_b="stage_body",
        min_overlap=0.025,
        name="second bearing band retained in first plunger at rest",
    )
    ctx.expect_overlap(
        core,
        second,
        axes="x",
        elem_a="rear_bearing_band",
        elem_b="stage_body",
        min_overlap=0.025,
        name="core bearing band retained in second plunger at rest",
    )

    first_rest = ctx.part_world_position(first)
    second_rest = ctx.part_world_position(second)
    core_rest = ctx.part_world_position(core)
    with ctx.pose({j1: 0.220, j2: 0.160, j3: 0.120}):
        ctx.expect_overlap(
            first,
            base,
            axes="x",
            elem_a="stage_body",
            elem_b="outer_sleeve",
            min_overlap=0.10,
            name="first plunger remains captured at full travel",
        )
        ctx.expect_overlap(
            second,
            first,
            axes="x",
            elem_a="rear_bearing_band",
            elem_b="stage_body",
            min_overlap=0.025,
            name="second bearing band remains captured at full travel",
        )
        ctx.expect_overlap(
            core,
            second,
            axes="x",
            elem_a="rear_bearing_band",
            elem_b="stage_body",
            min_overlap=0.025,
            name="core bearing band remains captured at full travel",
        )
        first_ext = ctx.part_world_position(first)
        second_ext = ctx.part_world_position(second)
        core_ext = ctx.part_world_position(core)

    ctx.check(
        "all plungers extend along positive x",
        first_rest is not None
        and second_rest is not None
        and core_rest is not None
        and first_ext is not None
        and second_ext is not None
        and core_ext is not None
        and first_ext[0] > first_rest[0] + 0.20
        and second_ext[0] > second_rest[0] + 0.35
        and core_ext[0] > core_rest[0] + 0.48,
        details=f"rest={(first_rest, second_rest, core_rest)} extended={(first_ext, second_ext, core_ext)}",
    )

    return ctx.report()


object_model = build_object_model()
