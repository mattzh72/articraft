from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _annular_tube(outer_radius: float, inner_radius: float, length: float, mesh_name: str):
    """Hollow lathed tube whose local axis is +Z before placement."""
    chamfer = min(0.0012, length * 0.18, (outer_radius - inner_radius) * 0.45)
    outer_profile = [
        (outer_radius - chamfer * 0.35, -length / 2.0),
        (outer_radius, -length / 2.0 + chamfer),
        (outer_radius, length / 2.0 - chamfer),
        (outer_radius - chamfer * 0.35, length / 2.0),
    ]
    inner_profile = [
        (inner_radius, -length / 2.0),
        (inner_radius, length / 2.0),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        mesh_name,
    )


def _tube_origin(x_center: float) -> Origin:
    # SDK/CadQuery cylinders are Z-axis forms; rotate them onto the lens X axis.
    return Origin(xyz=(x_center, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="macro_lens")

    model.material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    model.material("matte_black", rgba=(0.055, 0.058, 0.060, 1.0))
    model.material("rubber_black", rgba=(0.006, 0.006, 0.007, 1.0))
    model.material("anodized_metal", rgba=(0.58, 0.59, 0.57, 1.0))
    model.material("dark_glass", rgba=(0.08, 0.16, 0.18, 0.62))
    model.material("coated_glass", rgba=(0.18, 0.46, 0.50, 0.48))
    model.material("white_paint", rgba=(0.86, 0.88, 0.82, 1.0))

    outer_barrel = model.part("outer_barrel")
    outer_barrel.visual(
        _annular_tube(0.032, 0.029, 0.115, "outer_sleeve"),
        origin=_tube_origin(0.0575),
        material="satin_black",
        name="outer_sleeve",
    )
    outer_barrel.visual(
        _annular_tube(0.036, 0.030, 0.006, "rear_ring_stop"),
        origin=_tube_origin(0.030),
        material="matte_black",
        name="rear_ring_stop",
    )
    outer_barrel.visual(
        _annular_tube(0.036, 0.030, 0.006, "front_ring_stop"),
        origin=_tube_origin(0.080),
        material="matte_black",
        name="front_ring_stop",
    )
    outer_barrel.visual(
        _annular_tube(0.035, 0.028, 0.010, "front_bezel"),
        origin=_tube_origin(0.110),
        material="matte_black",
        name="front_bezel",
    )
    outer_barrel.visual(
        _annular_tube(0.033, 0.021, 0.014, "bayonet_mount"),
        origin=_tube_origin(-0.007),
        material="anodized_metal",
        name="bayonet_mount",
    )
    outer_barrel.visual(
        Cylinder(radius=0.020, length=0.003),
        origin=_tube_origin(-0.0155),
        material="dark_glass",
        name="rear_glass",
    )

    # Three bayonet lugs, slightly seated into the rear metal ring.
    for lug_index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        radius = 0.0335
        outer_barrel.visual(
            Box((0.010, 0.021, 0.007)),
            origin=Origin(
                xyz=(-0.017, radius * math.sin(angle), radius * math.cos(angle)),
                rpy=(-angle, 0.0, 0.0),
            ),
            material="anodized_metal",
            name=f"bayonet_lug_{lug_index}",
        )

    outer_barrel.visual(
        Cylinder(radius=0.0026, length=0.009),
        origin=Origin(xyz=(-0.0185, 0.0, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="anodized_metal",
        name="locking_pin",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        _annular_tube(0.038, 0.033, 0.044, "grip_sleeve"),
        origin=_tube_origin(0.0),
        material="rubber_black",
        name="grip_sleeve",
    )
    # Raised longitudinal grip ribs around the rotating rubber ring.
    for rib_index in range(32):
        angle = 2.0 * math.pi * rib_index / 32.0
        radius = 0.0392
        focus_ring.visual(
            Box((0.036, 0.0018, 0.0045)),
            origin=Origin(
                xyz=(0.0, radius * math.sin(angle), radius * math.cos(angle)),
                rpy=(-angle, 0.0, 0.0),
            ),
            material="matte_black",
            name=f"grip_rib_{rib_index}",
        )
    for mark_index, angle in enumerate((-0.34, 0.0, 0.34)):
        radius = 0.03855
        focus_ring.visual(
            Box((0.010 + mark_index * 0.003, 0.0012, 0.0012)),
            origin=Origin(
                xyz=(-0.012 + mark_index * 0.012, radius * math.sin(angle), radius * math.cos(angle)),
                rpy=(-angle, 0.0, 0.0),
            ),
            material="white_paint",
            name=f"focus_mark_{mark_index}",
        )

    inner_barrel = model.part("inner_barrel")
    inner_barrel.visual(
        _annular_tube(0.027, 0.023, 0.120, "inner_tube"),
        origin=_tube_origin(0.0),
        material="matte_black",
        name="inner_tube",
    )
    inner_barrel.visual(
        _annular_tube(0.028, 0.021, 0.009, "front_retainer"),
        origin=_tube_origin(0.0645),
        material="satin_black",
        name="front_retainer",
    )
    inner_barrel.visual(
        Cylinder(radius=0.0215, length=0.004),
        origin=_tube_origin(0.0625),
        material="coated_glass",
        name="front_glass",
    )
    inner_barrel.visual(
        Cylinder(radius=0.018, length=0.003),
        origin=_tube_origin(0.044),
        material="dark_glass",
        name="aperture_glass",
    )
    for groove_index, x_pos in enumerate((-0.030, -0.018, -0.006, 0.006)):
        inner_barrel.visual(
            _annular_tube(0.0278, 0.0232, 0.002, f"extension_groove_{groove_index}"),
            origin=_tube_origin(x_pos),
            material="satin_black",
            name=f"extension_groove_{groove_index}",
        )
    inner_barrel.visual(
        _annular_tube(0.029, 0.0262, 0.004, "guide_bearing"),
        origin=_tube_origin(-0.052),
        material="matte_black",
        name="guide_bearing",
    )
    inner_barrel.visual(
        _annular_tube(0.0235, 0.0175, 0.004, "aperture_retainer"),
        origin=_tube_origin(0.044),
        material="satin_black",
        name="aperture_retainer",
    )

    model.articulation(
        "focus_rotation",
        ArticulationType.REVOLUTE,
        parent=outer_barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )
    model.articulation(
        "barrel_extension",
        ArticulationType.PRISMATIC,
        parent=outer_barrel,
        child=inner_barrel,
        origin=Origin(xyz=(0.105, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.05, lower=0.0, upper=0.040),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_barrel")
    focus = object_model.get_part("focus_ring")
    inner = object_model.get_part("inner_barrel")
    focus_joint = object_model.get_articulation("focus_rotation")
    extension_joint = object_model.get_articulation("barrel_extension")

    ctx.expect_overlap(
        focus,
        outer,
        axes="x",
        elem_a="grip_sleeve",
        elem_b="outer_sleeve",
        min_overlap=0.040,
        name="focus ring sits at the outer barrel mid-section",
    )
    ctx.expect_contact(
        focus,
        outer,
        elem_a="grip_sleeve",
        elem_b="rear_ring_stop",
        contact_tol=0.0015,
        name="rear stop axially captures the focus ring",
    )
    ctx.expect_contact(
        focus,
        outer,
        elem_a="grip_sleeve",
        elem_b="front_ring_stop",
        contact_tol=0.0015,
        name="front stop axially captures the focus ring",
    )
    ctx.expect_within(
        inner,
        outer,
        axes="yz",
        inner_elem="inner_tube",
        outer_elem="outer_sleeve",
        margin=0.0,
        name="inner barrel is centered inside the sleeve bore",
    )
    ctx.expect_overlap(
        inner,
        outer,
        axes="x",
        elem_a="inner_tube",
        elem_b="outer_sleeve",
        min_overlap=0.060,
        name="inner barrel remains well inserted when collapsed",
    )

    rest_pos = ctx.part_world_position(inner)
    with ctx.pose({focus_joint: math.radians(100.0), extension_joint: 0.040}):
        ctx.expect_overlap(
            inner,
            outer,
            axes="x",
            elem_a="inner_tube",
            elem_b="outer_sleeve",
            min_overlap=0.025,
            name="inner barrel retains insertion at close focus",
        )
        extended_pos = ctx.part_world_position(inner)

    ctx.check(
        "close-focus pose extends the inner barrel",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.035,
        details=f"rest={rest_pos}, extended={extended_pos}, joint={extension_joint.name}",
    )

    return ctx.report()


object_model = build_object_model()
