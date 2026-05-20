from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_cylinder(outer_radius: float, inner_radius: float, length: float):
    """A centered, open-ended tube with its optical axis on local Z."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length / 2.0, both=True)
    )


def _flared_tube(
    rear_outer: float,
    front_outer: float,
    rear_inner: float,
    front_inner: float,
    length: float,
):
    """A front lens shade/hood tube that runs from local z=0 to z=length."""
    outer = (
        cq.Workplane("XY")
        .circle(rear_outer)
        .workplane(offset=length)
        .circle(front_outer)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .circle(rear_inner)
        .workplane(offset=length)
        .circle(front_inner)
        .loft(combine=True)
    )
    return outer.cut(inner)


def _radial_box_origin(radius: float, angle: float, z: float) -> Origin:
    """Place a box with local +X radial and local +Y tangential."""
    return Origin(
        xyz=(radius * math.cos(angle), radius * math.sin(angle), z),
        rpy=(0.0, 0.0, angle),
    )


def _add_radial_box(
    part,
    *,
    name: str,
    radius: float,
    angle: float,
    z: float,
    size: tuple[float, float, float],
    material: Material,
):
    part.visual(
        Box(size),
        origin=_radial_box_origin(radius, angle, z),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cinema_prime_lens")

    matte_black = model.material("matte_black", rgba=(0.005, 0.005, 0.004, 1.0))
    satin_black = model.material("satin_black", rgba=(0.018, 0.017, 0.015, 1.0))
    rubber = model.material("knurled_rubber", rgba=(0.002, 0.002, 0.002, 1.0))
    dark_metal = model.material("dark_anodized_metal", rgba=(0.035, 0.035, 0.033, 1.0))
    bare_metal = model.material("brushed_mount_metal", rgba=(0.55, 0.52, 0.46, 1.0))
    engraving = model.material("filled_engraving", rgba=(0.82, 0.82, 0.76, 1.0))
    glass = model.material("coated_glass", rgba=(0.10, 0.24, 0.32, 0.72))
    amber = model.material("amber_index_dot", rgba=(1.0, 0.48, 0.08, 1.0))

    barrel = model.part("barrel")

    barrel.visual(
        mesh_from_cadquery(_annular_cylinder(0.052, 0.034, 0.150), "control_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=matte_black,
        name="control_sleeve",
    )
    barrel.visual(
        mesh_from_cadquery(_annular_cylinder(0.050, 0.034, 0.082), "rear_barrel"),
        origin=Origin(xyz=(0.0, 0.0, -0.102)),
        material=satin_black,
        name="rear_barrel",
    )
    barrel.visual(
        mesh_from_cadquery(_annular_cylinder(0.060, 0.036, 0.012), "rear_shoulder"),
        origin=Origin(xyz=(0.0, 0.0, -0.081)),
        material=dark_metal,
        name="rear_shoulder",
    )
    barrel.visual(
        mesh_from_cadquery(_annular_cylinder(0.061, 0.052, 0.006), "iris_rear_stop"),
        origin=Origin(xyz=(0.0, 0.0, -0.073)),
        material=dark_metal,
        name="iris_rear_stop",
    )
    barrel.visual(
        mesh_from_cadquery(_annular_cylinder(0.061, 0.052, 0.006), "iris_front_stop"),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=dark_metal,
        name="iris_front_stop",
    )
    barrel.visual(
        mesh_from_cadquery(_annular_cylinder(0.068, 0.052, 0.006), "focus_rear_stop"),
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
        material=dark_metal,
        name="focus_rear_stop",
    )
    barrel.visual(
        mesh_from_cadquery(_annular_cylinder(0.068, 0.052, 0.006), "focus_front_stop"),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=dark_metal,
        name="focus_front_stop",
    )
    barrel.visual(
        mesh_from_cadquery(_annular_cylinder(0.063, 0.038, 0.012), "front_shoulder"),
        origin=Origin(xyz=(0.0, 0.0, 0.072)),
        material=dark_metal,
        name="front_shoulder",
    )
    barrel.visual(
        mesh_from_cadquery(_annular_cylinder(0.075, 0.045, 0.054), "front_housing"),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=matte_black,
        name="front_housing",
    )
    barrel.visual(
        mesh_from_cadquery(_annular_cylinder(0.083, 0.052, 0.014), "front_flange"),
        origin=Origin(xyz=(0.0, 0.0, 0.139)),
        material=dark_metal,
        name="front_flange",
    )
    barrel.visual(
        Cylinder(radius=0.043, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.137)),
        material=glass,
        name="front_element",
    )
    barrel.visual(
        mesh_from_cadquery(_annular_cylinder(0.053, 0.041, 0.006), "front_retainer"),
        origin=Origin(xyz=(0.0, 0.0, 0.137)),
        material=dark_metal,
        name="front_retainer",
    )
    barrel.visual(
        mesh_from_cadquery(_annular_cylinder(0.058, 0.035, 0.018), "rear_mount"),
        origin=Origin(xyz=(0.0, 0.0, -0.151)),
        material=bare_metal,
        name="rear_mount",
    )
    barrel.visual(
        Cylinder(radius=0.033, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.162)),
        material=glass,
        name="rear_element",
    )
    barrel.visual(
        mesh_from_cadquery(_annular_cylinder(0.037, 0.031, 0.004), "rear_retainer"),
        origin=Origin(xyz=(0.0, 0.0, -0.162)),
        material=dark_metal,
        name="rear_retainer",
    )

    for i, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        _add_radial_box(
            barrel,
            name=f"front_bayonet_lug_{i}",
            radius=0.083,
            angle=angle,
            z=0.142,
            size=(0.008, 0.026, 0.006),
            material=dark_metal,
        )
        _add_radial_box(
            barrel,
            name=f"rear_mount_tab_{i}",
            radius=0.057,
            angle=angle + math.pi / 8.0,
            z=-0.161,
            size=(0.010, 0.020, 0.005),
            material=bare_metal,
        )

    # Fixed witness marks between the moving focus and iris bands.
    _add_radial_box(
        barrel,
        name="witness_line",
        radius=0.0528,
        angle=math.pi / 2.0,
        z=-0.029,
        size=(0.004, 0.005, 0.020),
        material=engraving,
    )
    _add_radial_box(
        barrel,
        name="witness_dot",
        radius=0.0528,
        angle=math.pi / 2.0,
        z=-0.012,
        size=(0.004, 0.006, 0.006),
        material=amber,
    )

    lever_angle = -0.42
    pivot_radius = 0.070
    pivot_xyz = (
        pivot_radius * math.cos(lever_angle),
        pivot_radius * math.sin(lever_angle),
        -0.151,
    )
    _add_radial_box(
        barrel,
        name="lever_bridge",
        radius=0.064,
        angle=lever_angle,
        z=-0.151,
        size=(0.020, 0.014, 0.012),
        material=dark_metal,
    )
    barrel.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(xyz=pivot_xyz),
        material=bare_metal,
        name="lever_pivot",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_cadquery(_annular_cylinder(0.068, 0.056, 0.078), "focus_band"),
        material=rubber,
        name="focus_band",
    )
    for i in range(48):
        angle = 2.0 * math.pi * i / 48.0
        _add_radial_box(
            focus_ring,
            name=f"focus_grip_{i}",
            radius=0.0684,
            angle=angle,
            z=0.0,
            size=(0.0032, 0.0038, 0.070),
            material=rubber,
        )
    for i in range(12):
        angle = 2.0 * math.pi * i / 12.0
        tick_len = 0.018 if i % 3 == 0 else 0.010
        _add_radial_box(
            focus_ring,
            name=f"focus_scale_tick_{i}",
            radius=0.0701,
            angle=angle,
            z=0.030,
            size=(0.0014, 0.0030, tick_len),
            material=engraving,
        )

    iris_ring = model.part("iris_ring")
    iris_ring.visual(
        mesh_from_cadquery(_annular_cylinder(0.061, 0.055, 0.032), "iris_band"),
        material=satin_black,
        name="iris_band",
    )
    for i in range(9):
        angle = 2.0 * math.pi * i / 9.0 + 0.10
        _add_radial_box(
            iris_ring,
            name=f"iris_mark_{i}",
            radius=0.0616,
            angle=angle,
            z=0.006,
            size=(0.0012, 0.0030, 0.012),
            material=engraving,
        )
    _add_radial_box(
        iris_ring,
        name="iris_index_dot",
        radius=0.0610,
        angle=math.pi / 2.0,
        z=-0.010,
        size=(0.0020, 0.005, 0.005),
        material=amber,
    )

    lens_shade = model.part("lens_shade")
    lens_shade.visual(
        mesh_from_cadquery(
            _flared_tube(0.067, 0.091, 0.057, 0.080, 0.084),
            "shade_body",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=matte_black,
        name="shade_body",
    )
    lens_shade.visual(
        mesh_from_cadquery(_annular_cylinder(0.071, 0.057, 0.012), "shade_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark_metal,
        name="shade_collar",
    )
    for i, angle in enumerate((0.12, 2.0 * math.pi / 3.0 + 0.12, 4.0 * math.pi / 3.0 + 0.12)):
        _add_radial_box(
            lens_shade,
            name=f"shade_bayonet_key_{i}",
            radius=0.070,
            angle=angle,
            z=0.009,
            size=(0.008, 0.025, 0.007),
            material=dark_metal,
        )
    _add_radial_box(
        lens_shade,
        name="shade_alignment_dot",
        radius=0.0710,
        angle=math.pi / 2.0,
        z=0.013,
        size=(0.0020, 0.006, 0.006),
        material=engraving,
    )

    lock_lever = model.part("lock_lever")
    lock_lever.visual(
        Box((0.008, 0.045, 0.005)),
        origin=Origin(xyz=(0.006, 0.023, 0.008)),
        material=bare_metal,
        name="lever_blade",
    )
    lock_lever.visual(
        Box((0.010, 0.012, 0.005)),
        origin=Origin(xyz=(0.006, 0.002, 0.008)),
        material=dark_metal,
        name="lever_root",
    )

    model.articulation(
        "barrel_to_focus",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=-2.1, upper=2.1),
    )
    model.articulation(
        "barrel_to_iris",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=iris_ring,
        origin=Origin(xyz=(0.0, 0.0, -0.054)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.2, lower=0.0, upper=1.45),
    )
    model.articulation(
        "barrel_to_shade",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=lens_shade,
        origin=Origin(xyz=(0.0, 0.0, 0.142)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.0, lower=0.0, upper=0.35),
    )
    model.articulation(
        "barrel_to_lever",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=lock_lever,
        origin=Origin(xyz=pivot_xyz, rpy=(0.0, 0.0, lever_angle)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=2.0, lower=-0.45, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    barrel = object_model.get_part("barrel")
    focus_ring = object_model.get_part("focus_ring")
    iris_ring = object_model.get_part("iris_ring")
    lens_shade = object_model.get_part("lens_shade")
    lock_lever = object_model.get_part("lock_lever")
    focus_joint = object_model.get_articulation("barrel_to_focus")
    iris_joint = object_model.get_articulation("barrel_to_iris")
    shade_joint = object_model.get_articulation("barrel_to_shade")
    lever_joint = object_model.get_articulation("barrel_to_lever")

    ctx.expect_overlap(
        focus_ring,
        barrel,
        axes="z",
        min_overlap=0.070,
        elem_a="focus_band",
        elem_b="control_sleeve",
        name="focus ring surrounds barrel sleeve",
    )
    ctx.expect_overlap(
        iris_ring,
        barrel,
        axes="z",
        min_overlap=0.030,
        elem_a="iris_band",
        elem_b="control_sleeve",
        name="iris ring surrounds rear sleeve",
    )
    ctx.expect_gap(
        lens_shade,
        barrel,
        axis="z",
        max_gap=0.006,
        max_penetration=0.000001,
        positive_elem="shade_body",
        negative_elem="front_flange",
        name="shade seats just in front of bayonet flange",
    )
    ctx.expect_gap(
        lock_lever,
        barrel,
        axis="z",
        min_gap=0.0,
        max_gap=0.002,
        positive_elem="lever_blade",
        negative_elem="lever_pivot",
        name="locking lever bears on pivot post",
    )

    ctx.check(
        "focus ring has cinema-scale throw",
        focus_joint.motion_limits.lower <= -2.0 and focus_joint.motion_limits.upper >= 2.0,
    )
    ctx.check(
        "iris ring has limited aperture throw",
        iris_joint.motion_limits.lower == 0.0 and iris_joint.motion_limits.upper >= 1.2,
    )
    ctx.check(
        "shade uses short bayonet twist",
        0.25 <= shade_joint.motion_limits.upper <= 0.45,
    )
    ctx.check(
        "lock lever swings both ways",
        lever_joint.motion_limits.lower < 0.0 < lever_joint.motion_limits.upper,
    )

    with ctx.pose({shade_joint: shade_joint.motion_limits.upper}):
        ctx.expect_gap(
            lens_shade,
            barrel,
            axis="z",
            max_gap=0.006,
            max_penetration=0.000001,
            positive_elem="shade_body",
            negative_elem="front_flange",
            name="twisted shade remains seated",
        )

    return ctx.report()


object_model = build_object_model()
