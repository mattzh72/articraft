from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_actuation_dropper_seatpost")

    anodized_black = Material("anodized_black", color=(0.015, 0.016, 0.018, 1.0))
    satin_black = Material("satin_black", color=(0.03, 0.032, 0.036, 1.0))
    brushed_aluminum = Material("brushed_aluminum", color=(0.70, 0.72, 0.70, 1.0))
    dark_rubber = Material("dark_rubber", color=(0.01, 0.01, 0.012, 1.0))
    lever_red = Material("lever_red", color=(0.75, 0.035, 0.025, 1.0))
    steel = Material("steel", color=(0.55, 0.55, 0.52, 1.0))

    # Dropper posts are bicycle-seatpost sized: roughly 31.6 mm lower post,
    # a polished moving stanchion, and a larger clamp collar around the seal.
    outer_radius = 0.018
    outer_bore_radius = 0.0162
    outer_height = 0.335
    collar_outer_radius = 0.026
    collar_bore_radius = 0.0164
    collar_z0 = 0.304
    collar_z1 = 0.356
    hinge_z = 0.338
    hinge_reach = 0.043

    outer = model.part("outer_tube")
    outer.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [(outer_radius, 0.0), (outer_radius, outer_height)],
                [(outer_bore_radius, 0.0), (outer_bore_radius, outer_height)],
                segments=80,
                start_cap="flat",
                end_cap="flat",
            ),
            "outer_sleeve",
        ),
        material=anodized_black,
        name="outer_sleeve",
    )
    outer.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [(collar_outer_radius, collar_z0), (collar_outer_radius, collar_z1)],
                [(collar_bore_radius, collar_z0), (collar_bore_radius, collar_z1)],
                segments=80,
                start_cap="flat",
                end_cap="flat",
            ),
            "collar_ring",
        ),
        material=satin_black,
        name="collar_ring",
    )
    # Split clamp ears and a through bolt give the collar a believable clamp
    # construction instead of a featureless ring.
    outer.visual(
        Box((0.014, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, -0.030, 0.331)),
        material=satin_black,
        name="clamp_ear_0",
    )
    outer.visual(
        Box((0.014, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, -0.043, 0.331)),
        material=satin_black,
        name="clamp_ear_1",
    )
    outer.visual(
        Cylinder(radius=0.0032, length=0.032),
        origin=Origin(xyz=(0.0, -0.0365, 0.337), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="clamp_bolt",
    )

    for index, side in enumerate((1.0, -1.0)):
        # Each actuator gets its own clevis-like boss attached to the collar.
        outer.visual(
            Box((0.014, 0.036, 0.018)),
            origin=Origin(xyz=(side * 0.030, 0.0, hinge_z)),
            material=satin_black,
            name=f"hinge_boss_{index}",
        )
        outer.visual(
            Box((0.016, 0.006, 0.026)),
            origin=Origin(xyz=(side * hinge_reach, 0.014, hinge_z)),
            material=satin_black,
            name=f"hinge_cheek_{index}_0",
        )
        outer.visual(
            Box((0.016, 0.006, 0.026)),
            origin=Origin(xyz=(side * hinge_reach, -0.014, hinge_z)),
            material=satin_black,
            name=f"hinge_cheek_{index}_1",
        )
        outer.visual(
            Cylinder(radius=0.0026, length=0.040),
            origin=Origin(
                xyz=(side * hinge_reach, 0.0, hinge_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=steel,
            name=f"hinge_pin_{index}",
        )

    inner = model.part("inner_tube")
    inner.visual(
        Cylinder(radius=0.0162, length=0.345),
        # Child frame is at the collar lip; the tube extends downward as retained
        # insertion and upward as the exposed telescoping stanchion.
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=brushed_aluminum,
        name="sliding_tube",
    )
    inner.visual(
        Cylinder(radius=0.0172, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.166)),
        material=satin_black,
        name="top_cap",
    )
    inner.visual(
        Box((0.052, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.179)),
        material=satin_black,
        name="saddle_cradle",
    )
    inner.visual(
        Cylinder(radius=0.0042, length=0.064),
        origin=Origin(xyz=(0.0, 0.017, 0.190), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="saddle_rail_0",
    )
    inner.visual(
        Cylinder(radius=0.0042, length=0.064),
        origin=Origin(xyz=(0.0, -0.017, 0.190), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="saddle_rail_1",
    )
    inner.visual(
        Box((0.040, 0.040, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.197)),
        material=dark_rubber,
        name="rail_clamp_cap",
    )

    model.articulation(
        "outer_to_inner",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=inner,
        origin=Origin(xyz=(0.0, 0.0, outer_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.55, lower=0.0, upper=0.125),
    )

    for index, side in enumerate((1.0, -1.0)):
        lever = model.part(f"lever_{index}")
        lever.visual(
            Cylinder(radius=0.0046, length=0.018),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_black,
            name="hinge_barrel",
        )
        lever.visual(
            mesh_from_geometry(
                CapsuleGeometry(radius=0.0048, length=0.064, radial_segments=24),
                f"lever_arm_{index}",
            ),
            origin=Origin(xyz=(0.038, 0.0, -0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=lever_red,
            name="lever_arm",
        )
        lever.visual(
            Box((0.040, 0.013, 0.018)),
            origin=Origin(xyz=(0.086, 0.0, -0.012)),
            material=lever_red,
            name="finger_paddle",
        )
        lever.visual(
            Cylinder(radius=0.0030, length=0.014),
            origin=Origin(xyz=(0.089, 0.0, -0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_rubber,
            name="grip_ridge",
        )

        model.articulation(
            f"outer_to_lever_{index}",
            ArticulationType.REVOLUTE,
            parent=outer,
            child=lever,
            origin=Origin(
                xyz=(side * hinge_reach, 0.0, hinge_z),
                rpy=(0.0, 0.0, 0.0 if side > 0.0 else math.pi),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=18.0, velocity=4.0, lower=0.0, upper=0.62),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_tube")
    inner = object_model.get_part("inner_tube")
    slide = object_model.get_articulation("outer_to_inner")

    ctx.check(
        "dual lever and telescoping joints present",
        len(object_model.articulations) == 3,
        details=f"articulations={len(object_model.articulations)}",
    )

    ctx.allow_overlap(
        outer,
        inner,
        elem_a="outer_sleeve",
        elem_b="sliding_tube",
        reason=(
            "The inner telescoping member is intentionally captured inside the "
            "hollow outer sleeve; this scoped overlap represents the retained "
            "sliding fit at the hidden insertion."
        ),
    )

    # The hinge pin is intentionally modeled as the parent-side axle captured
    # inside each lever barrel.
    for index in (0, 1):
        lever = object_model.get_part(f"lever_{index}")
        hinge = object_model.get_articulation(f"outer_to_lever_{index}")
        ctx.allow_overlap(
            outer,
            lever,
            elem_a=f"hinge_pin_{index}",
            elem_b="hinge_barrel",
            reason="The parent-side hinge pin is captured inside the lever barrel bore.",
        )
        ctx.expect_within(
            outer,
            lever,
            axes="xz",
            inner_elem=f"hinge_pin_{index}",
            outer_elem="hinge_barrel",
            margin=0.0008,
            name=f"hinge pin {index} is radially inside its barrel",
        )
        ctx.expect_overlap(
            outer,
            lever,
            axes="y",
            elem_a=f"hinge_pin_{index}",
            elem_b="hinge_barrel",
            min_overlap=0.016,
            name=f"hinge pin {index} passes through the lever barrel",
        )
        rest_aabb = ctx.part_world_aabb(lever)
        with ctx.pose({hinge: 0.62}):
            raised_aabb = ctx.part_world_aabb(lever)
        ctx.check(
            f"lever {index} pivots upward about its collar hinge",
            rest_aabb is not None
            and raised_aabb is not None
            and raised_aabb[1][2] > rest_aabb[1][2] + 0.025,
            details=f"rest={rest_aabb}, raised={raised_aabb}",
        )

    ctx.expect_within(
        inner,
        outer,
        axes="xy",
        inner_elem="sliding_tube",
        outer_elem="collar_ring",
        margin=0.0,
        name="inner tube is centered within the collar bore footprint",
    )
    ctx.expect_overlap(
        inner,
        outer,
        axes="z",
        elem_a="sliding_tube",
        elem_b="outer_sleeve",
        min_overlap=0.16,
        name="collapsed inner tube has retained insertion",
    )
    rest_pos = ctx.part_world_position(inner)
    with ctx.pose({slide: 0.125}):
        ctx.expect_overlap(
            inner,
            outer,
            axes="z",
            elem_a="sliding_tube",
            elem_b="outer_sleeve",
            min_overlap=0.045,
            name="extended inner tube remains inserted in the outer tube",
        )
        extended_pos = ctx.part_world_position(inner)
    ctx.check(
        "inner tube extends upward on the prismatic axis",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.12,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
