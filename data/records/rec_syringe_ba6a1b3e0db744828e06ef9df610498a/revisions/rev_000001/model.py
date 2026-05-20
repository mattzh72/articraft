from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


AXIAL = Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def axial_origin(x: float, y: float = 0.0, z: float = 0.0) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def barrel_shell_mesh():
    # Revolved around local +Z, then mounted with local Z along the syringe axis.
    # The lip flares, rear guide land, and thin wall keep the barrel visibly hollow.
    outer = [
        (0.0210, -0.074),
        (0.0180, -0.066),
        (0.0180, 0.060),
        (0.0205, 0.071),
    ]
    inner = [
        (0.0146, -0.071),
        (0.0146, 0.060),
        (0.0128, 0.068),
    ]
    return LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=64,
        start_cap="round",
        end_cap="round",
        lip_samples=5,
    )


def nozzle_mesh():
    # A solid old-style luer-inspired stepped metal nozzle with a fine outlet tip.
    profile = [
        (0.0000, 0.000),
        (0.0080, 0.000),
        (0.0080, 0.012),
        (0.0058, 0.016),
        (0.0046, 0.040),
        (0.0028, 0.052),
        (0.0015, 0.058),
        (0.0000, 0.058),
    ]
    return LatheGeometry(profile, segments=48, closed=True)


def guide_ring_mesh():
    # Rear guide/yoke ring: a real clearance hole for the sliding plunger rod.
    outer = [
        (0.0220, -0.008),
        (0.0220, 0.008),
    ]
    inner = [
        (0.0060, -0.008),
        (0.0060, 0.008),
    ]
    return LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=48,
        start_cap="flat",
        end_cap="flat",
        lip_samples=2,
    )


def adapter_ring_mesh(outer_radius: float, inner_radius: float, length: float):
    half = length / 2.0
    outer = [(outer_radius, -half), (outer_radius, half)]
    inner = [(inner_radius, -half), (inner_radius, half)]
    return LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=56,
        start_cap="flat",
        end_cap="flat",
        lip_samples=2,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_legacy_syringe")

    glass = model.material("smoky_graduated_glass", rgba=(0.72, 0.90, 1.0, 0.38))
    nickel = model.material("brushed_nickel", rgba=(0.68, 0.66, 0.60, 1.0))
    dark = model.material("black_enamel", rgba=(0.015, 0.014, 0.012, 1.0))
    rubber = model.material("aged_black_rubber", rgba=(0.02, 0.018, 0.015, 1.0))
    amber = model.material("amber_service_plates", rgba=(0.42, 0.30, 0.16, 1.0))

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_geometry(barrel_shell_mesh(), "hollow_barrel_shell"),
        origin=AXIAL,
        material=glass,
        name="barrel_shell",
    )

    # External clamp bands and pragmatic reinforcements at the highly loaded ends.
    for idx, x in enumerate((-0.060, -0.024, 0.024, 0.062)):
        barrel.visual(
            Cylinder(radius=0.0198, length=0.0040),
            origin=axial_origin(x),
            material=nickel if idx in (0, 3) else dark,
            name=f"clamp_band_{idx}",
        )

    barrel.visual(
        mesh_from_geometry(nozzle_mesh(), "stepped_nozzle_tip"),
        origin=axial_origin(0.070),
        material=nickel,
        name="nozzle_tip",
    )
    barrel.visual(
        Cylinder(radius=0.027, length=0.004),
        origin=axial_origin(0.071),
        material=nickel,
        name="front_adapter_flange",
    )
    barrel.visual(
        mesh_from_geometry(guide_ring_mesh(), "rear_guide_ring"),
        origin=axial_origin(-0.079),
        material=nickel,
        name="rear_guide",
    )
    barrel.visual(
        mesh_from_geometry(adapter_ring_mesh(0.028, 0.006, 0.006), "rear_adapter_ring"),
        origin=axial_origin(-0.087),
        material=nickel,
        name="rear_adapter_ring",
    )

    # Legacy finger flange wings are split around a real central clearance hole.
    for suffix, y in (("upper", 0.034), ("lower", -0.034)):
        barrel.visual(
            Box((0.012, 0.020, 0.012)),
            origin=Origin(xyz=(-0.087, y, 0.0)),
            material=nickel,
            name=f"{suffix}_finger_flange",
        )

    # Service hatches sitting on shallow saddles so they read as mounted, not pasted on.
    for side, zc, mat_name in (("top", 0.0203, amber), ("bottom", -0.0203, amber)):
        barrel.visual(
            Box((0.052, 0.023, 0.0030)),
            origin=Origin(xyz=(-0.006, 0.0, zc)),
            material=mat_name,
            name=f"{side}_hatch",
        )
        barrel.visual(
            Box((0.058, 0.026, 0.0020)),
            origin=Origin(xyz=(-0.006, 0.0, zc * 0.965)),
            material=nickel,
            name=f"{side}_hatch_saddle",
        )
        bolt_z = zc + (0.0020 if zc > 0 else -0.0020)
        bolt_rpy = (0.0, 0.0, 0.0) if zc > 0 else (math.pi, 0.0, 0.0)
        for i, bx in enumerate((-0.026, 0.014)):
            for j, by in enumerate((-0.008, 0.008)):
                barrel.visual(
                    Cylinder(radius=0.0022, length=0.0024),
                    origin=Origin(xyz=(bx, by, bolt_z), rpy=bolt_rpy),
                    material=dark,
                    name=f"{side}_hatch_bolt_{i}_{j}",
                )

    # Graduations: etched black enamel ticks on one side of the transparent barrel.
    for i, x in enumerate([v * 0.010 - 0.055 for v in range(12)]):
        major = i % 5 == 0
        barrel.visual(
            Box((0.0012, 0.0016, 0.014 if major else 0.008)),
            origin=Origin(xyz=(x, 0.0185, 0.004 if major else 0.002)),
            material=dark,
            name=f"graduation_{i}",
        )

    # Face bolts around front and rear adapters.  Their heads overlap the adapter
    # faces very slightly, like real countersunk hardware seated into metal.
    for ring, x, outward in (("front", 0.0715, 1.0), ("rear", -0.0875, -1.0)):
        for i, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
            y = 0.024 * math.cos(angle)
            z = 0.024 * math.sin(angle)
            barrel.visual(
                Cylinder(radius=0.0025, length=0.0022),
                origin=axial_origin(x + outward * 0.0005, y, z),
                material=dark,
                name=f"{ring}_adapter_bolt_{i}",
            )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0042, length=0.178),
        origin=axial_origin(0.021),
        material=nickel,
        name="plunger_rod",
    )
    plunger.visual(
        Box((0.145, 0.0028, 0.0100)),
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
        material=nickel,
        name="rod_flat_rib",
    )
    plunger.visual(
        Box((0.145, 0.0100, 0.0028)),
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
        material=nickel,
        name="rod_cross_rib",
    )
    plunger.visual(
        Cylinder(radius=0.0100, length=0.007),
        origin=axial_origin(-0.054),
        material=nickel,
        name="stop_collar",
    )
    plunger.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=axial_origin(-0.073),
        material=nickel,
        name="thumb_pad",
    )
    plunger.visual(
        Cylinder(radius=0.0075, length=0.018),
        origin=axial_origin(-0.062),
        material=nickel,
        name="thumb_stem",
    )
    plunger.visual(
        Cylinder(radius=0.0132, length=0.012),
        origin=axial_origin(0.111),
        material=rubber,
        name="piston_seal",
    )
    plunger.visual(
        Cylinder(radius=0.0100, length=0.006),
        origin=axial_origin(0.101),
        material=nickel,
        name="piston_retainer",
    )

    model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(-0.073, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.18, lower=0.0, upper=0.075),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("barrel_to_plunger")

    ctx.expect_within(
        plunger,
        barrel,
        axes="yz",
        inner_elem="plunger_rod",
        outer_elem="barrel_shell",
        margin=0.001,
        name="plunger rod is coaxial in the barrel bore",
    )
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="x",
        elem_a="piston_seal",
        elem_b="barrel_shell",
        min_overlap=0.010,
        name="piston remains inside barrel at rest",
    )

    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: 0.075}):
        ctx.expect_within(
            plunger,
            barrel,
            axes="yz",
            inner_elem="plunger_rod",
            outer_elem="barrel_shell",
            margin=0.001,
            name="extended plunger remains coaxial",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="x",
            elem_a="piston_seal",
            elem_b="barrel_shell",
            min_overlap=0.010,
            name="extended piston remains retained in barrel",
        )
        extended_pos = ctx.part_world_position(plunger)

    ctx.check(
        "prismatic joint retracts plunger rearward",
        rest_pos is not None and extended_pos is not None and extended_pos[0] < rest_pos[0] - 0.070,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
