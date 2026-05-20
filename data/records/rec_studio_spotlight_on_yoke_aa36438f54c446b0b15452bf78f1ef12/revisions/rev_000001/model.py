from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_studio_spotlight")

    powder = model.material("matte_black_powdercoat", rgba=(0.015, 0.017, 0.018, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.72, 0.70, 0.65, 1.0))
    rubber = model.material("black_epdm_seal", rgba=(0.002, 0.002, 0.002, 1.0))
    glass = model.material("sealed_smoked_glass", rgba=(0.55, 0.72, 0.80, 0.38))
    warm = model.material("warm_reflector", rgba=(0.95, 0.82, 0.46, 1.0))

    # Root: a heavy outdoor base and mast.  All visuals touch or slightly seat
    # into the neighboring member so the load path reads as one welded stand.
    stand = model.part("stand")
    stand.visual(Box((0.76, 0.50, 0.070)), origin=Origin(xyz=(0.0, 0.0, 0.035)), material=powder, name="base_plate")
    stand.visual(Box((0.30, 0.22, 0.030)), origin=Origin(xyz=(0.0, 0.0, 0.085)), material=powder, name="raised_plinth")
    stand.visual(Cylinder(radius=0.046, length=0.760), origin=Origin(xyz=(0.0, 0.0, 0.470)), material=powder, name="mast_tube")
    stand.visual(Box((0.22, 0.17, 0.025)), origin=Origin(xyz=(0.0, 0.0, 0.8475)), material=powder, name="top_flange")
    for i, (x, y) in enumerate(((-0.29, -0.18), (-0.29, 0.18), (0.29, -0.18), (0.29, 0.18))):
        stand.visual(Box((0.105, 0.070, 0.012)), origin=Origin(xyz=(x, y, -0.006)), material=rubber, name=f"rubber_foot_{i}")
        stand.visual(Cylinder(radius=0.018, length=0.010), origin=Origin(xyz=(x, y, 0.075)), material=stainless, name=f"base_bolt_{i}")

    # A single fabricated trunnion yoke: two protected side cheeks joined by a
    # bridge/base, with a real clear span around the spotlight can.
    yoke = model.part("yoke")
    yoke_body = TrunnionYokeGeometry(
        (0.60, 0.13, 0.54),
        span_width=0.430,
        trunnion_diameter=0.090,
        trunnion_center_z=0.390,
        base_thickness=0.070,
        corner_radius=0.012,
        center=False,
    )
    yoke_body.rotate_z(pi / 2.0)
    yoke.visual(mesh_from_geometry(yoke_body, "yoke_body_mesh"), origin=Origin(xyz=(0.0, 0.0, 0.860)), material=powder, name="yoke_body")
    for i, (x, y) in enumerate(((-0.035, -0.205), (0.035, -0.205), (-0.035, 0.205), (0.035, 0.205))):
        yoke.visual(Cylinder(radius=0.014, length=0.010), origin=Origin(xyz=(x, y, 0.935)), material=stainless, name=f"yoke_bolt_{i}")

    # The moving can frame is centered on the trunnion axis.  Its metal shell is
    # a revolved thin-wall housing with lip overhangs instead of a solid plug.
    can = model.part("spotlight_can")
    shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.145, -0.240),
            (0.170, -0.185),
            (0.176, 0.220),
            (0.197, 0.306),
            (0.190, 0.350),
        ],
        inner_profile=[
            (0.106, -0.218),
            (0.137, -0.165),
            (0.150, 0.225),
            (0.160, 0.303),
            (0.160, 0.326),
        ],
        segments=80,
        start_cap="round",
        end_cap="round",
        lip_samples=10,
    )
    shell.rotate_y(pi / 2.0)
    can.visual(mesh_from_geometry(shell, "can_shell_mesh"), material=powder, name="body_shell")
    can.visual(Cylinder(radius=0.128, length=0.030), origin=Origin(xyz=(-0.245, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=powder, name="sealed_rear_cap")
    can.visual(Cylinder(radius=0.048, length=0.026), origin=Origin(xyz=(-0.270, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=rubber, name="rear_gland")
    can.visual(Cylinder(radius=0.022, length=0.066), origin=Origin(xyz=(-0.312, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=rubber, name="sealed_cable_stub")
    can.visual(Cylinder(radius=0.151, length=0.012), origin=Origin(xyz=(0.115, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=warm, name="reflector_disk")
    can.visual(Cylinder(radius=0.164, length=0.018), origin=Origin(xyz=(0.316, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=glass, name="front_lens")
    gasket = TorusGeometry(radius=0.162, tube=0.0065, radial_segments=16, tubular_segments=72)
    gasket.rotate_y(pi / 2.0)
    can.visual(mesh_from_geometry(gasket, "front_gasket_mesh"), origin=Origin(xyz=(0.308, 0.0, 0.0)), material=rubber, name="front_gasket")
    can.visual(Box((0.245, 0.420, 0.018)), origin=Origin(xyz=(0.305, 0.0, 0.198)), material=powder, name="rain_hood")
    can.visual(Box((0.235, 0.015, 0.070)), origin=Origin(xyz=(0.305, 0.205, 0.161)), material=powder, name="drip_lip_0")
    can.visual(Box((0.235, 0.015, 0.070)), origin=Origin(xyz=(0.305, -0.205, 0.161)), material=powder, name="drip_lip_1")
    can.visual(Cylinder(radius=0.032, length=0.130), origin=Origin(xyz=(0.0, 0.225, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)), material=stainless, name="trunnion_pin_0")
    can.visual(Cylinder(radius=0.032, length=0.130), origin=Origin(xyz=(0.0, -0.225, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)), material=stainless, name="trunnion_pin_1")
    can.visual(Cylinder(radius=0.136, length=0.020), origin=Origin(xyz=(0.0, 0.210, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)), material=stainless, name="friction_disc_0")
    can.visual(Cylinder(radius=0.136, length=0.020), origin=Origin(xyz=(0.0, -0.210, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)), material=stainless, name="friction_disc_1")

    # Two hand-lock knobs sit outside the yoke cheeks and rotate about the same
    # protected tilt axis as the trunnion hardware.
    for index, side in enumerate((1.0, -1.0)):
        knob = model.part(f"lock_knob_{index}")
        knob_mesh = KnobGeometry(
            0.080,
            0.040,
            body_style="lobed",
            base_diameter=0.060,
            top_diameter=0.072,
            crown_radius=0.002,
            grip=KnobGrip(style="ribbed", count=10, depth=0.0012),
            center=True,
        )
        knob_mesh.rotate_x(-side * pi / 2.0)
        knob.visual(mesh_from_geometry(knob_mesh, f"lock_knob_{index}_mesh"), origin=Origin(xyz=(0.0, side * 0.023, 0.0)), material=powder, name="grip")
        knob.visual(Cylinder(radius=0.060, length=0.008), origin=Origin(xyz=(0.0, side * 0.006, 0.0), rpy=(-side * pi / 2.0, 0.0, 0.0)), material=stainless, name="washer")
        knob.visual(Cylinder(radius=0.0085, length=0.020), origin=Origin(xyz=(0.0, side * 0.018, 0.0), rpy=(-side * pi / 2.0, 0.0, 0.0)), material=stainless, name="threaded_stem")

    model.articulation("stand_to_yoke", ArticulationType.FIXED, parent=stand, child=yoke, origin=Origin())
    model.articulation(
        "tilt_axis",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=can,
        origin=Origin(xyz=(0.0, 0.0, 1.250)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=75.0, velocity=0.8, lower=-0.65, upper=0.85),
        motion_properties=MotionProperties(damping=0.25, friction=4.0),
    )
    model.articulation(
        "lock_knob_joint_0",
        ArticulationType.CONTINUOUS,
        parent=yoke,
        child="lock_knob_0",
        origin=Origin(xyz=(0.0, 0.298, 1.250)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0),
        motion_properties=MotionProperties(damping=0.05, friction=0.6),
    )
    model.articulation(
        "lock_knob_joint_1",
        ArticulationType.CONTINUOUS,
        parent=yoke,
        child="lock_knob_1",
        origin=Origin(xyz=(0.0, -0.298, 1.250)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0),
        motion_properties=MotionProperties(damping=0.05, friction=0.6),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    yoke = object_model.get_part("yoke")
    can = object_model.get_part("spotlight_can")
    lock0 = object_model.get_part("lock_knob_0")
    lock1 = object_model.get_part("lock_knob_1")
    tilt = object_model.get_articulation("tilt_axis")

    ctx.allow_overlap(
        can,
        yoke,
        elem_a="friction_disc_0",
        elem_b="yoke_body",
        reason="The side friction disc is intentionally seated into the protected yoke cheek to represent a tightened weatherproof tilt lock.",
    )
    ctx.allow_overlap(
        can,
        yoke,
        elem_a="friction_disc_1",
        elem_b="yoke_body",
        reason="The opposite friction disc is intentionally seated into the protected yoke cheek to represent the paired tilt lock.",
    )

    ctx.expect_gap(yoke, stand, axis="z", positive_elem="yoke_body", negative_elem="top_flange", max_gap=0.002, max_penetration=0.001, name="yoke base seats on stand flange")
    ctx.expect_within(can, yoke, axes="y", inner_elem="body_shell", outer_elem="yoke_body", margin=0.003, name="can is captured between side yokes")
    ctx.expect_overlap(can, yoke, axes="xz", elem_a="friction_disc_0", elem_b="yoke_body", min_overlap=0.040, name="first friction disc bears on yoke cheek")
    ctx.expect_overlap(can, yoke, axes="xz", elem_a="friction_disc_1", elem_b="yoke_body", min_overlap=0.040, name="second friction disc bears on yoke cheek")
    ctx.expect_overlap(can, yoke, axes="y", elem_a="trunnion_pin_0", elem_b="yoke_body", min_overlap=0.040, name="first trunnion passes through yoke cheek")
    ctx.expect_overlap(can, yoke, axes="y", elem_a="trunnion_pin_1", elem_b="yoke_body", min_overlap=0.040, name="second trunnion passes through yoke cheek")
    ctx.expect_gap(lock0, yoke, axis="y", positive_elem="washer", negative_elem="yoke_body", max_gap=0.004, max_penetration=0.002, name="first lock washer bears on yoke")
    ctx.expect_gap(yoke, lock1, axis="y", positive_elem="yoke_body", negative_elem="washer", max_gap=0.004, max_penetration=0.002, name="second lock washer bears on yoke")

    rest_lens = ctx.part_element_world_aabb(can, elem="front_lens")
    with ctx.pose({tilt: 0.85}):
        raised_lens = ctx.part_element_world_aabb(can, elem="front_lens")
        ctx.expect_within(can, yoke, axes="y", inner_elem="body_shell", outer_elem="yoke_body", margin=0.010, name="tilted can remains between yokes")

    rest_z = None if rest_lens is None else (rest_lens[0][2] + rest_lens[1][2]) * 0.5
    raised_z = None if raised_lens is None else (raised_lens[0][2] + raised_lens[1][2]) * 0.5
    ctx.check(
        "positive tilt raises beam end",
        rest_z is not None and raised_z is not None and raised_z > rest_z + 0.09,
        details=f"front_lens_rest_z={rest_z}, front_lens_raised_z={raised_z}",
    )

    return ctx.report()


object_model = build_object_model()
