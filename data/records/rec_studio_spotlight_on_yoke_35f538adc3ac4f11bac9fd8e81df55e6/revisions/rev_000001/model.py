from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _base_shape() -> cq.Workplane:
    """Rounded desktop base with an integral rear fold-hinge clevis."""
    plate = (
        cq.Workplane("XY")
        .box(0.260, 0.165, 0.026)
        .edges("|Z")
        .fillet(0.018)
        .translate((0.0, 0.0, 0.013))
    )
    ear_a = cq.Workplane("XY").box(0.028, 0.008, 0.047).translate((-0.095, 0.026, 0.049))
    ear_b = cq.Workplane("XY").box(0.028, 0.008, 0.047).translate((-0.095, -0.026, 0.049))
    base = plate.union(ear_a).union(ear_b)
    hinge_clearance = (
        cq.Workplane("XZ")
        .center(-0.095, 0.046)
        .cylinder(0.080, 0.0085, centered=(True, True, True))
    )
    return base.cut(hinge_clearance)


def _yoke_shape() -> cq.Workplane:
    """One-piece U yoke: side plates, lower bridge, swivel collar, and bored pivot bosses."""
    collar = cq.Workplane("XY").cylinder(0.012, 0.032, centered=(True, True, True)).translate((0.0, 0.0, 0.006))
    neck = cq.Workplane("XY").box(0.030, 0.050, 0.020).translate((0.015, 0.0, 0.016))
    bridge = cq.Workplane("XY").box(0.065, 0.145, 0.018).translate((0.032, 0.0, 0.009))
    arm_a = cq.Workplane("XY").box(0.026, 0.010, 0.136).translate((0.055, 0.064, 0.081))
    arm_b = cq.Workplane("XY").box(0.026, 0.010, 0.136).translate((0.055, -0.064, 0.081))
    boss_a = (
        cq.Workplane("XZ")
        .center(0.055, 0.080)
        .cylinder(0.008, 0.020, centered=(True, True, True))
        .translate((0.0, 0.070, 0.0))
    )
    boss_b = (
        cq.Workplane("XZ")
        .center(0.055, 0.080)
        .cylinder(0.008, 0.020, centered=(True, True, True))
        .translate((0.0, -0.070, 0.0))
    )
    yoke = collar.union(neck).union(bridge).union(arm_a).union(arm_b).union(boss_a).union(boss_b)
    pivot_bore = (
        cq.Workplane("XZ")
        .center(0.055, 0.080)
        .cylinder(0.180, 0.014, centered=(True, True, True))
    )
    return yoke.cut(pivot_bore)


def _can_shell_geometry() -> LatheGeometry:
    # Revolved around local Z, then rotated so the optical axis is local +X.
    outer_profile = [
        (0.036, -0.064),
        (0.041, -0.045),
        (0.045, 0.030),
        (0.052, 0.086),
    ]
    inner_profile = [
        (0.027, -0.058),
        (0.032, -0.040),
        (0.038, 0.030),
        (0.045, 0.078),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=64,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(math.pi / 2.0)


def _reflector_geometry() -> LatheGeometry:
    outer_profile = [(0.011, -0.042), (0.043, 0.065)]
    inner_profile = [(0.008, -0.039), (0.040, 0.061)]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=48,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_yoke_spotlight")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    graphite = model.material("graphite", rgba=(0.12, 0.125, 0.13, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.63, 0.62, 0.58, 1.0))
    warm_glass = model.material("warm_glass", rgba=(1.0, 0.82, 0.42, 0.48))
    rubber = model.material("rubber", rgba=(0.02, 0.02, 0.022, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "base_plate"),
        material=graphite,
        name="base_shell",
    )
    for idx, (x, y) in enumerate(((-0.092, -0.055), (-0.092, 0.055), (0.094, -0.055), (0.094, 0.055))):
        base.visual(
            Cylinder(radius=0.011, length=0.006),
            origin=Origin(xyz=(x, y, -0.003)),
            material=rubber,
            name=f"foot_{idx}",
        )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.010, length=0.037),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="fold_barrel",
    )
    mast.visual(
        Cylinder(radius=0.0085, length=0.072),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="hinge_pin",
    )
    mast.visual(
        Cylinder(radius=0.009, length=0.125),
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        material=matte_black,
        name="upright_tube",
    )
    mast.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.139)),
        material=graphite,
        name="pan_cap",
    )

    yoke = model.part("yoke")
    yoke.visual(
        mesh_from_cadquery(_yoke_shape(), "yoke_frame"),
        material=matte_black,
        name="yoke_shell",
    )
    yoke.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.020, tube=0.0022, radial_segments=20, tubular_segments=36),
            "friction_ring_0",
        ),
        origin=Origin(xyz=(0.055, 0.0745, 0.080), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="friction_ring_0",
    )
    yoke.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.020, tube=0.0022, radial_segments=20, tubular_segments=36),
            "friction_ring_1",
        ),
        origin=Origin(xyz=(0.055, -0.0745, 0.080), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="friction_ring_1",
    )

    can = model.part("can")
    can.visual(
        mesh_from_geometry(_can_shell_geometry(), "spotlight_can_shell"),
        material=matte_black,
        name="can_shell",
    )
    can.visual(
        mesh_from_geometry(_reflector_geometry(), "spotlight_reflector"),
        material=satin_metal,
        name="reflector",
    )
    can.visual(
        Cylinder(radius=0.046, length=0.006),
        origin=Origin(xyz=(0.080, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_glass,
        name="front_lens",
    )
    can.visual(
        Cylinder(radius=0.031, length=0.008),
        origin=Origin(xyz=(-0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="rear_cap",
    )
    can.visual(
        Cylinder(radius=0.0065, length=0.150),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="trunnion_shaft",
    )
    for idx, x in enumerate((-0.038, -0.022, -0.006, 0.010, 0.026)):
        can.visual(
            Box((0.010, 0.028, 0.008)),
            origin=Origin(xyz=(x, 0.0, 0.044)),
            material=graphite,
            name=f"cooling_fin_{idx}",
        )

    knob_geometry = KnobGeometry(
        0.030,
        0.018,
        body_style="lobed",
        base_diameter=0.022,
        top_diameter=0.030,
        crown_radius=0.0015,
    )
    for idx, side in enumerate((1.0, -1.0)):
        knob = model.part(f"knob_{idx}")
        knob.visual(
            mesh_from_geometry(knob_geometry, f"lock_knob_{idx}"),
            origin=Origin(rpy=(side * math.pi / 2.0, 0.0, 0.0)),
            material=matte_black,
            name="grip",
        )
        knob.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(0.0, -side * 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_metal,
            name="center_boss",
        )

    fold = model.articulation(
        "base_to_mast",
        ArticulationType.REVOLUTE,
        parent=base,
        child=mast,
        origin=Origin(xyz=(-0.095, 0.0, 0.046)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.0, lower=0.0, upper=0.60),
    )
    pan = model.articulation(
        "mast_to_yoke",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-2.6, upper=2.6),
    )
    tilt = model.articulation(
        "yoke_to_can",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=can,
        origin=Origin(xyz=(0.055, 0.0, 0.080)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.2, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "can_to_knob_0",
        ArticulationType.CONTINUOUS,
        parent=can,
        child="knob_0",
        origin=Origin(xyz=(0.0, 0.083, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=5.0),
    )
    model.articulation(
        "can_to_knob_1",
        ArticulationType.CONTINUOUS,
        parent=can,
        child="knob_1",
        origin=Origin(xyz=(0.0, -0.083, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=5.0),
    )

    # Names kept here make it easy for targeted tests and probing to refer to the primary motions.
    assert fold.name and pan.name and tilt.name
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    yoke = object_model.get_part("yoke")
    can = object_model.get_part("can")
    knob_0 = object_model.get_part("knob_0")
    knob_1 = object_model.get_part("knob_1")
    tilt = object_model.get_articulation("yoke_to_can")
    fold = object_model.get_articulation("base_to_mast")
    pan = object_model.get_articulation("mast_to_yoke")

    ctx.allow_overlap(
        base,
        "mast",
        elem_a="base_shell",
        elem_b="hinge_pin",
        reason="The folding stand hinge uses a captured steel pin represented through the compact clevis bores.",
    )
    ctx.expect_overlap(
        "mast",
        base,
        axes="yz",
        elem_a="hinge_pin",
        elem_b="base_shell",
        min_overlap=0.010,
        name="fold hinge pin is captured by base clevis",
    )

    ctx.expect_within(
        can,
        yoke,
        axes="y",
        inner_elem="can_shell",
        outer_elem="yoke_shell",
        margin=0.002,
        name="can body clears between side yokes",
    )
    ctx.expect_overlap(
        can,
        yoke,
        axes="xz",
        elem_a="trunnion_shaft",
        elem_b="yoke_shell",
        min_overlap=0.010,
        name="trunnion captured through bored yoke bosses",
    )
    ctx.expect_contact(
        knob_0,
        yoke,
        contact_tol=0.0015,
        name="first lock knob bears on yoke friction ring",
    )
    ctx.expect_contact(
        knob_1,
        yoke,
        contact_tol=0.0015,
        name="second lock knob bears on yoke friction ring",
    )
    ctx.expect_gap(
        can,
        base,
        axis="z",
        min_gap=0.045,
        name="upright lamp clears desktop base",
    )

    with ctx.pose({tilt: -0.65}):
        up_lens = ctx.part_element_world_aabb(can, elem="front_lens")
    with ctx.pose({tilt: 0.65}):
        down_lens = ctx.part_element_world_aabb(can, elem="front_lens")
    up_z = (up_lens[0][2] + up_lens[1][2]) * 0.5 if up_lens else None
    down_z = (down_lens[0][2] + down_lens[1][2]) * 0.5 if down_lens else None
    ctx.check(
        "tilt joint aims beam up and down",
        up_z is not None and down_z is not None and up_z > down_z + 0.070,
        details=f"up_lens_z={up_z}, down_lens_z={down_z}",
    )

    with ctx.pose({pan: 1.1}):
        panned_lens = ctx.part_element_world_aabb(can, elem="front_lens")
    rest_lens = ctx.part_element_world_aabb(can, elem="front_lens")
    rest_y = (rest_lens[0][1] + rest_lens[1][1]) * 0.5 if rest_lens else None
    pan_y = (panned_lens[0][1] + panned_lens[1][1]) * 0.5 if panned_lens else None
    ctx.check(
        "pan swivel sweeps the beam sideways",
        rest_y is not None and pan_y is not None and abs(pan_y - rest_y) > 0.045,
        details=f"rest_lens_y={rest_y}, panned_lens_y={pan_y}",
    )

    with ctx.pose({fold: 0.60}):
        ctx.expect_gap(
            can,
            base,
            axis="z",
            min_gap=0.030,
            name="stowed fold keeps lamp above base",
        )
        ctx.expect_overlap(
            can,
            base,
            axes="xy",
            min_overlap=0.030,
            name="folded assembly stays within compact footprint",
        )

    return ctx.report()


object_model = build_object_model()
