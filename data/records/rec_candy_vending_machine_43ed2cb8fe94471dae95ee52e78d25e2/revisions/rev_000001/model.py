from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _globe_shell_mesh():
    radius = 0.155
    height = 0.250
    center_z = height / 2.0
    thickness = 0.005
    z_min = 0.0
    z_max = height

    outer = []
    inner = []
    for i in range(25):
        z = z_min + (z_max - z_min) * i / 24.0
        r = math.sqrt(max(radius * radius - (z - center_z) ** 2, 0.0))
        outer.append((r, z))
        inner.append((max(r - thickness, 0.001), z))

    return LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=96,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_gumball_machine")

    red_cast = model.material("red_cast_enamel", rgba=(0.72, 0.04, 0.03, 1.0))
    dark_red = model.material("dark_cast_shadow", rgba=(0.32, 0.01, 0.01, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.84, 0.82, 0.76, 1.0))
    brass = model.material("warm_brass", rgba=(0.92, 0.63, 0.24, 1.0))
    black = model.material("black_slot", rgba=(0.02, 0.018, 0.015, 1.0))
    glass = model.material("clear_glass", rgba=(0.74, 0.93, 1.0, 0.30))

    # Heavy one-piece cast plinth, pedestal, collar, front coin housing and chute.
    base = model.part("base")
    base_profile = [
        (0.000, 0.000),
        (0.122, 0.000),
        (0.132, 0.014),
        (0.128, 0.040),
        (0.105, 0.060),
        (0.080, 0.108),
        (0.083, 0.168),
        (0.100, 0.220),
        (0.112, 0.252),
        (0.096, 0.265),
        (0.000, 0.265),
    ]
    base.visual(
        mesh_from_geometry(LatheGeometry(base_profile, segments=96), "cast_base"),
        origin=Origin(),
        material=red_cast,
        name="cast_base",
    )
    base.visual(
        Box((0.165, 0.060, 0.198)),
        origin=Origin(xyz=(0.0, -0.100, 0.148)),
        material=red_cast,
        name="coin_housing",
    )
    base.visual(
        Box((0.150, 0.010, 0.182)),
        origin=Origin(xyz=(0.0, -0.136, 0.155)),
        material=chrome,
        name="coin_faceplate",
    )
    base.visual(
        Box((0.072, 0.004, 0.010)),
        origin=Origin(xyz=(-0.020, -0.143, 0.226)),
        material=black,
        name="coin_slot",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(-0.065, -0.139, 0.230), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="upper_screw_0",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.065, -0.139, 0.230), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="upper_screw_1",
    )
    base.visual(
        Cylinder(radius=0.025, length=0.012),
        origin=Origin(xyz=(0.0, -0.145, 0.150), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="knob_boss",
    )
    base.visual(
        Box((0.096, 0.078, 0.012)),
        origin=Origin(xyz=(0.0, -0.160, 0.075), rpy=(0.32, 0.0, 0.0)),
        material=chrome,
        name="chute_floor",
    )
    base.visual(
        Box((0.010, 0.078, 0.032)),
        origin=Origin(xyz=(-0.053, -0.160, 0.083), rpy=(0.32, 0.0, 0.0)),
        material=chrome,
        name="chute_side_0",
    )
    base.visual(
        Box((0.010, 0.078, 0.032)),
        origin=Origin(xyz=(0.053, -0.160, 0.083), rpy=(0.32, 0.0, 0.0)),
        material=chrome,
        name="chute_side_1",
    )
    base.visual(
        Box((0.115, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -0.128, 0.102)),
        material=chrome,
        name="chute_throat",
    )
    base.visual(
        Cylinder(radius=0.070, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
        material=dark_red,
        name="dispense_plate",
    )

    # A mechanically separate globe assembly clamped to the base collar.
    globe = model.part("globe")
    globe.visual(
        mesh_from_geometry(_globe_shell_mesh(), "clear_globe_shell"),
        origin=Origin(),
        material=glass,
        name="clear_shell",
    )
    globe.visual(
        mesh_from_geometry(TorusGeometry(0.091, 0.006, radial_segments=96, tubular_segments=12), "lower_globe_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=brass,
        name="lower_ring",
    )
    globe.visual(
        mesh_from_geometry(TorusGeometry(0.091, 0.006, radial_segments=96, tubular_segments=12), "upper_globe_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        material=brass,
        name="upper_ring",
    )
    globe.visual(
        Box((0.070, 0.026, 0.016)),
        origin=Origin(xyz=(0.0, 0.105, 0.262)),
        material=brass,
        name="cap_hinge_mount",
    )

    # Front rotating dispense knob, centered on a horizontal shaft.
    knob = model.part("dispense_knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.064,
                0.046,
                body_style="lobed",
                base_diameter=0.050,
                top_diameter=0.060,
                grip=KnobGrip(style="ribbed", count=12, depth=0.003),
                center=False,
            ),
            "dispense_knob_mesh",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.0, -0.046, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="center_plug",
    )

    # Coin-return cover is a separate front-mounted flap beside the knob.
    return_cover = model.part("return_cover")
    return_cover.visual(
        Box((0.046, 0.006, 0.064)),
        origin=Origin(xyz=(0.023, -0.003, 0.0)),
        material=chrome,
        name="return_flap",
    )
    return_cover.visual(
        Cylinder(radius=0.0055, length=0.070),
        origin=Origin(xyz=(0.0, -0.003, 0.0)),
        material=chrome,
        name="return_hinge_barrel",
    )
    return_cover.visual(
        Box((0.030, 0.004, 0.008)),
        origin=Origin(xyz=(0.025, -0.007, -0.018)),
        material=black,
        name="pull_lip",
    )

    # Refill cap: a hinged metal lid above the globe opening, not fused to it.
    top_cap = model.part("refill_cap")
    top_cap.visual(
        Cylinder(radius=0.101, length=0.018),
        origin=Origin(xyz=(0.0, -0.115, 0.0)),
        material=brass,
        name="cap_disk",
    )
    top_cap.visual(
        Cylinder(radius=0.006, length=0.078),
        origin=Origin(xyz=(0.0, 0.000, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="cap_hinge_barrel",
    )
    top_cap.visual(
        Box((0.050, 0.030, 0.006)),
        origin=Origin(xyz=(0.0, -0.018, -0.002)),
        material=brass,
        name="cap_hinge_leaf",
    )

    model.articulation(
        "base_to_globe",
        ArticulationType.FIXED,
        parent=base,
        child=globe,
        origin=Origin(xyz=(0.0, 0.0, 0.269)),
    )
    model.articulation(
        "knob_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=knob,
        origin=Origin(xyz=(0.0, -0.151, 0.150)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "return_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=return_cover,
        origin=Origin(xyz=(0.050, -0.143, 0.154)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=3.0, lower=0.0, upper=1.05),
    )
    model.articulation(
        "cap_hinge",
        ArticulationType.REVOLUTE,
        parent=globe,
        child=top_cap,
        origin=Origin(xyz=(0.0, 0.105, 0.266)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=2.5, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    def _aabb_center(box):
        if box is None:
            return None
        lo, hi = box
        return (
            (lo[0] + hi[0]) / 2.0,
            (lo[1] + hi[1]) / 2.0,
            (lo[2] + hi[2]) / 2.0,
        )

    base = object_model.get_part("base")
    globe = object_model.get_part("globe")
    knob = object_model.get_part("dispense_knob")
    return_cover = object_model.get_part("return_cover")
    top_cap = object_model.get_part("refill_cap")
    knob_spin = object_model.get_articulation("knob_spin")
    return_hinge = object_model.get_articulation("return_hinge")
    cap_hinge = object_model.get_articulation("cap_hinge")

    ctx.allow_overlap(
        globe,
        top_cap,
        elem_a="cap_hinge_mount",
        elem_b="cap_hinge_barrel",
        reason="The refill-cap hinge barrel is intentionally captured inside the rear hinge mount.",
    )
    ctx.allow_overlap(
        globe,
        top_cap,
        elem_a="cap_hinge_mount",
        elem_b="cap_hinge_leaf",
        reason="The rear hinge leaf is intentionally seated into the hinge mount as a compact cast bracket.",
    )
    ctx.expect_overlap(
        top_cap,
        globe,
        axes="xz",
        elem_a="cap_hinge_barrel",
        elem_b="cap_hinge_mount",
        min_overlap=0.010,
        name="refill cap barrel is retained by hinge mount",
    )
    ctx.expect_overlap(
        top_cap,
        globe,
        axes="xz",
        elem_a="cap_hinge_leaf",
        elem_b="cap_hinge_mount",
        min_overlap=0.005,
        name="refill cap leaf is seated in hinge bracket",
    )

    ctx.expect_gap(
        globe,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.010,
        positive_elem="lower_ring",
        negative_elem="cast_base",
        name="globe sits visibly separate on cast collar",
    )
    ctx.expect_gap(
        top_cap,
        globe,
        axis="z",
        min_gap=0.001,
        max_gap=0.025,
        positive_elem="cap_disk",
        negative_elem="upper_ring",
        name="refill cap hovers above globe opening",
    )
    ctx.expect_overlap(
        knob,
        base,
        axes="xz",
        elem_a="knob_body",
        elem_b="knob_boss",
        min_overlap=0.020,
        name="knob is centered on front shaft boss",
    )
    ctx.expect_gap(
        base,
        return_cover,
        axis="y",
        max_penetration=0.002,
        positive_elem="coin_faceplate",
        negative_elem="return_flap",
        name="coin return flap is front-mounted beside knob",
    )

    rest_cap_pos = _aabb_center(ctx.part_element_world_aabb(top_cap, elem="cap_disk"))
    with ctx.pose({cap_hinge: 1.0}):
        open_cap_pos = _aabb_center(ctx.part_element_world_aabb(top_cap, elem="cap_disk"))
        ctx.expect_gap(
            top_cap,
            globe,
            axis="z",
            max_penetration=0.005,
            positive_elem="cap_disk",
            negative_elem="upper_ring",
            name="open cap keeps only hinge edge near globe ring",
        )
    ctx.check(
        "rear hinge opens refill cap upward",
        rest_cap_pos is not None
        and open_cap_pos is not None
        and open_cap_pos[2] > rest_cap_pos[2] + 0.020,
        details=f"rest={rest_cap_pos}, open={open_cap_pos}",
    )

    rest_cover_pos = _aabb_center(ctx.part_element_world_aabb(return_cover, elem="return_flap"))
    with ctx.pose({return_hinge: 0.8}):
        open_cover_pos = _aabb_center(ctx.part_element_world_aabb(return_cover, elem="return_flap"))
    ctx.check(
        "coin return cover swings outward",
        rest_cover_pos is not None
        and open_cover_pos is not None
        and open_cover_pos[1] < rest_cover_pos[1] - 0.010,
        details=f"rest={rest_cover_pos}, open={open_cover_pos}",
    )

    with ctx.pose({knob_spin: math.pi}):
        ctx.expect_overlap(
            knob,
            base,
            axes="xz",
            elem_a="knob_body",
            elem_b="knob_boss",
            min_overlap=0.020,
            name="continuous knob stays on horizontal shaft while rotating",
        )

    return ctx.report()


object_model = build_object_model()
