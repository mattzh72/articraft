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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_tube(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """A watertight hollow cylinder running from z=0 to z=length."""
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(length)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """Low-profile rounded rectangular clamp pieces."""
    x, y, z = size
    return cq.Workplane("XY").box(x, y, z).edges("|Z").fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="carbon_fibre_dropper_seatpost")

    carbon = model.material("satin_carbon", rgba=(0.015, 0.016, 0.017, 1.0))
    weave_dark = model.material("subtle_black_weave", rgba=(0.004, 0.004, 0.005, 1.0))
    weave_sheen = model.material("graphite_weave_sheen", rgba=(0.105, 0.115, 0.12, 1.0))
    anodized = model.material("black_anodized_alloy", rgba=(0.01, 0.011, 0.012, 1.0))
    stanchion = model.material("polished_black_stanchion", rgba=(0.025, 0.027, 0.030, 1.0))
    titanium = model.material("brushed_titanium", rgba=(0.62, 0.60, 0.55, 1.0))
    rubber = model.material("matte_black_seal", rgba=(0.002, 0.002, 0.002, 1.0))

    outer_radius = 0.0160
    outer_bore_radius = 0.0136
    outer_length = 0.430
    collar_outer_radius = 0.0200
    collar_bore_radius = 0.0142
    collar_length = 0.038
    collar_z = outer_length - 0.025

    outer_tube = model.part("outer_tube")
    outer_tube.visual(
        mesh_from_cadquery(
            _annular_tube(outer_radius, outer_bore_radius, outer_length),
            "outer_tube_shell",
            tolerance=0.00045,
            angular_tolerance=0.06,
        ),
        material=carbon,
        name="outer_shell",
    )
    outer_tube.visual(
        mesh_from_cadquery(
            _annular_tube(collar_outer_radius, collar_bore_radius, collar_length),
            "upper_seal_collar",
            tolerance=0.0004,
            angular_tolerance=0.06,
        ),
        origin=Origin(xyz=(0.0, 0.0, collar_z)),
        material=anodized,
        name="seal_collar",
    )
    outer_tube.visual(
        mesh_from_cadquery(
            _annular_tube(0.0165, outer_bore_radius, 0.010),
            "lower_reinforced_lip",
            tolerance=0.00045,
            angular_tolerance=0.06,
        ),
        material=anodized,
        name="lower_lip",
    )
    outer_tube.visual(
        mesh_from_cadquery(
            _annular_tube(0.0158, 0.0130, 0.010),
            "rubber_wiper",
            tolerance=0.00035,
            angular_tolerance=0.06,
        ),
        origin=Origin(xyz=(0.0, 0.0, outer_length + 0.004)),
        material=rubber,
        name="wiper_seal",
    )

    # Raised, shallow tangent strips give the dark tube a woven carbon-fibre read
    # without turning the post into a heavy patterned mesh.
    strip_radius = outer_radius + 0.00012
    for i in range(18):
        theta = i * (2.0 * math.pi / 18.0)
        z = 0.060 + (i % 6) * 0.055
        pitch = math.radians(23.0 if i % 2 == 0 else -23.0)
        material = weave_sheen if i % 3 == 0 else weave_dark
        outer_tube.visual(
            Box((0.0009, 0.0038, 0.112)),
            origin=Origin(
                xyz=(strip_radius * math.cos(theta), strip_radius * math.sin(theta), z),
                rpy=(pitch, 0.0, theta),
            ),
            material=material,
            name=f"weave_strip_{i}",
        )

    inner_post = model.part("inner_post")
    inner_radius = 0.0122
    inner_length = 0.530
    inner_center_z = -0.125
    inner_post.visual(
        Cylinder(radius=inner_radius, length=inner_length),
        origin=Origin(xyz=(0.0, 0.0, inner_center_z)),
        material=stanchion,
        name="sliding_shaft",
    )
    for name, z, length in (
        ("upper_guide_bushing", -0.020, 0.020),
        ("lower_guide_bushing", -0.310, 0.016),
    ):
        inner_post.visual(
            Cylinder(radius=0.01372, length=length),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=anodized,
            name=name,
        )
    inner_post.visual(
        Cylinder(radius=0.0142, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=anodized,
        name="head_socket",
    )
    inner_post.visual(
        mesh_from_cadquery(
            _rounded_box((0.095, 0.046, 0.018), 0.006),
            "inline_clamp_base",
            tolerance=0.00035,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.162)),
        material=anodized,
        name="clamp_base",
    )
    inner_post.visual(
        mesh_from_cadquery(
            _rounded_box((0.084, 0.044, 0.008), 0.004),
            "low_profile_top_plate",
            tolerance=0.00035,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.189)),
        material=anodized,
        name="top_plate",
    )
    for y in (-0.0175, 0.0175):
        inner_post.visual(
            Box((0.086, 0.0085, 0.012)),
            origin=Origin(xyz=(0.0, y, 0.176)),
            material=anodized,
            name=f"rail_cradle_{'neg' if y < 0.0 else 'pos'}",
        )
        inner_post.visual(
            Cylinder(radius=0.0032, length=0.112),
            origin=Origin(xyz=(0.0, y, 0.183), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=titanium,
            name=f"saddle_rail_{'neg' if y < 0.0 else 'pos'}",
        )

    for x in (-0.030, 0.030):
        inner_post.visual(
            Cylinder(radius=0.0060, length=0.006),
            origin=Origin(xyz=(x, 0.0, 0.1959)),
            material=titanium,
            name=f"bolt_head_{'front' if x > 0.0 else 'rear'}",
        )
        inner_post.visual(
            Cylinder(radius=0.0032, length=0.0015),
            origin=Origin(xyz=(x, 0.0, 0.1995)),
            material=weave_dark,
            name=f"hex_recess_{'front' if x > 0.0 else 'rear'}",
        )

    for i, z in enumerate((-0.020, 0.020, 0.060, 0.100)):
        inner_post.visual(
            Box((0.0008, 0.009, 0.0018)),
            origin=Origin(xyz=(inner_radius + 0.00015, 0.0, z), rpy=(0.0, 0.0, 0.0)),
            material=weave_sheen,
            name=f"height_mark_{i}",
        )

    model.articulation(
        "tube_to_post",
        ArticulationType.PRISMATIC,
        parent=outer_tube,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, outer_length)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.35, lower=0.0, upper=0.160),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_tube")
    inner = object_model.get_part("inner_post")
    slide = object_model.get_articulation("tube_to_post")

    for bushing in ("upper_guide_bushing", "lower_guide_bushing"):
        ctx.allow_overlap(
            outer,
            inner,
            elem_a="outer_shell",
            elem_b=bushing,
            reason=(
                "The hidden guide bushing is modeled with a tiny radial preload "
                "against the bore so the prismatic post is physically captured."
            ),
        )
        ctx.expect_within(
            inner,
            outer,
            axes="xy",
            inner_elem=bushing,
            outer_elem="outer_shell",
            margin=0.0,
            name=f"{bushing} stays coaxial in the outer tube",
        )
        ctx.expect_overlap(
            inner,
            outer,
            axes="z",
            elem_a=bushing,
            elem_b="outer_shell",
            min_overlap=0.010,
            name=f"{bushing} remains inside the tube bore",
        )

    ctx.expect_within(
        inner,
        outer,
        axes="xy",
        inner_elem="sliding_shaft",
        outer_elem="outer_shell",
        margin=0.0,
        name="sliding shaft is centered inside the outer tube envelope",
    )
    ctx.expect_overlap(
        inner,
        outer,
        axes="z",
        elem_a="sliding_shaft",
        elem_b="outer_shell",
        min_overlap=0.250,
        name="collapsed post retains a long insertion",
    )

    rest_pos = ctx.part_world_position(inner)
    with ctx.pose({slide: 0.160}):
        ctx.expect_within(
            inner,
            outer,
            axes="xy",
            inner_elem="sliding_shaft",
            outer_elem="outer_shell",
            margin=0.0,
            name="extended shaft stays coaxial with the outer tube",
        )
        ctx.expect_overlap(
            inner,
            outer,
            axes="z",
            elem_a="sliding_shaft",
            elem_b="outer_shell",
            min_overlap=0.120,
            name="extended post remains safely inserted",
        )
        extended_pos = ctx.part_world_position(inner)

    ctx.check(
        "prismatic joint raises the saddle clamp",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.150,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
