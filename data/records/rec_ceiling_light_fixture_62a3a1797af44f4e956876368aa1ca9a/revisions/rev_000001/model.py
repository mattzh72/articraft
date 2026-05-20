from __future__ import annotations

import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_head_track_lighting")

    matte_black = Material("matte_black", color=(0.005, 0.005, 0.004, 1.0))
    satin_black = Material("satin_black", color=(0.02, 0.018, 0.015, 1.0))
    warm_white = Material("warm_white_powdercoat", color=(0.86, 0.84, 0.78, 1.0))
    dark_slot = Material("recessed_black_slot", color=(0.0, 0.0, 0.0, 1.0))
    copper = Material("copper_contacts", color=(0.80, 0.37, 0.12, 1.0))
    glass = Material("warm_diffuser_glass", color=(1.0, 0.82, 0.38, 0.72))

    rail = model.part("ceiling_rail")
    rail.visual(
        Box((2.20, 0.090, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=matte_black,
        name="rail_body",
    )
    rail.visual(
        Box((2.24, 0.120, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.0215)),
        material=satin_black,
        name="ceiling_flange",
    )
    rail.visual(
        Box((0.020, 0.094, 0.039)),
        origin=Origin(xyz=(-1.110, 0.0, 0.0)),
        material=satin_black,
        name="end_cap_0",
    )
    rail.visual(
        Box((0.020, 0.094, 0.039)),
        origin=Origin(xyz=(1.110, 0.0, 0.0)),
        material=satin_black,
        name="end_cap_1",
    )
    rail.visual(
        Box((2.08, 0.010, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, -0.0165)),
        material=dark_slot,
        name="center_recess",
    )
    for side, y in enumerate((-0.026, 0.026)):
        rail.visual(
            Box((2.00, 0.006, 0.002)),
            origin=Origin(xyz=(0.0, y, -0.0165)),
            material=copper,
            name=f"contact_strip_{side}",
        )

    fixture_x = [-0.80, -0.40, 0.0, 0.40, 0.80]
    rail_bottom_z = -0.0175

    for i, x in enumerate(fixture_x):
        clip = model.part(f"clip_{i}")
        clip.visual(
            Box((0.140, 0.100, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material=satin_black,
            name="top_slide_pad",
        )
        for side, y in enumerate((-0.053, 0.053)):
            clip.visual(
                Box((0.140, 0.012, 0.038)),
                origin=Origin(xyz=(0.0, y, 0.018)),
                material=satin_black,
                name=f"rail_lip_{side}",
            )
        clip.visual(
            Cylinder(radius=0.033, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, -0.020)),
            material=satin_black,
            name="swivel_socket",
        )

        model.articulation(
            f"rail_to_clip_{i}",
            ArticulationType.PRISMATIC,
            parent=rail,
            child=clip,
            origin=Origin(xyz=(x, 0.0, rail_bottom_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=20.0, velocity=0.35, lower=-0.12, upper=0.12),
        )

        swivel = model.part(f"swivel_{i}")
        swivel.visual(
            Cylinder(radius=0.026, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, -0.004)),
            material=satin_black,
            name="turntable_disc",
        )
        swivel.visual(
            Cylinder(radius=0.012, length=0.154),
            origin=Origin(xyz=(0.0, 0.0, -0.077)),
            material=satin_black,
            name="drop_stem",
        )
        swivel.visual(
            Box((0.036, 0.150, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, -0.161)),
            material=satin_black,
            name="yoke_bridge",
        )
        for side, y in enumerate((-0.072, 0.072)):
            swivel.visual(
                Box((0.026, 0.012, 0.124)),
                origin=Origin(xyz=(0.0, y, -0.230)),
                material=satin_black,
                name=f"yoke_arm_{side}",
            )

        model.articulation(
            f"clip_to_swivel_{i}",
            ArticulationType.REVOLUTE,
            parent=clip,
            child=swivel,
            origin=Origin(xyz=(0.0, 0.0, -0.029)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-math.pi, upper=math.pi),
        )

        head = model.part(f"head_{i}")
        head.visual(
            Cylinder(radius=0.058, length=0.026),
            origin=Origin(xyz=(0.0, 0.0, -0.013)),
            material=satin_black,
            name="head_collar",
        )
        head.visual(
            Cylinder(radius=0.055, length=0.135),
            origin=Origin(xyz=(0.0, 0.0, -0.0875)),
            material=warm_white,
            name="spot_body",
        )
        head.visual(
            Cylinder(radius=0.060, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.151)),
            material=satin_black,
            name="front_bezel",
        )
        head.visual(
            Cylinder(radius=0.047, length=0.005),
            origin=Origin(xyz=(0.0, 0.0, -0.1585)),
            material=glass,
            name="warm_lens",
        )
        head.visual(
            Cylinder(radius=0.011, length=0.132),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_black,
            name="tilt_trunnion",
        )

        model.articulation(
            f"swivel_to_head_{i}",
            ArticulationType.REVOLUTE,
            parent=swivel,
            child=head,
            origin=Origin(xyz=(0.0, 0.0, -0.230)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=1.5, lower=-1.0, upper=1.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail = object_model.get_part("ceiling_rail")

    slide_joints = [object_model.get_articulation(f"rail_to_clip_{i}") for i in range(5)]
    yaw_joints = [object_model.get_articulation(f"clip_to_swivel_{i}") for i in range(5)]
    tilt_joints = [object_model.get_articulation(f"swivel_to_head_{i}") for i in range(5)]

    ctx.check("five sliding rail clips", len(slide_joints) == 5)
    ctx.check("five vertical swivel joints", len(yaw_joints) == 5)
    ctx.check("five head tilt joints", len(tilt_joints) == 5)

    for i, slide in enumerate(slide_joints):
        clip = object_model.get_part(f"clip_{i}")
        swivel = object_model.get_part(f"swivel_{i}")
        head = object_model.get_part(f"head_{i}")
        yaw = yaw_joints[i]
        tilt = tilt_joints[i]

        ctx.check(
            f"clip_{i} slides along rail",
            slide.articulation_type == ArticulationType.PRISMATIC and tuple(slide.axis) == (1.0, 0.0, 0.0),
        )
        ctx.check(
            f"swivel_{i} rotates vertically",
            yaw.articulation_type == ArticulationType.REVOLUTE and tuple(yaw.axis) == (0.0, 0.0, 1.0),
        )
        ctx.check(
            f"head_{i} tilts horizontally",
            tilt.articulation_type == ArticulationType.REVOLUTE and tuple(tilt.axis) == (0.0, 1.0, 0.0),
        )
        ctx.expect_gap(
            rail,
            clip,
            axis="z",
            positive_elem="rail_body",
            negative_elem="top_slide_pad",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"clip_{i} bears on rail underside",
        )
        ctx.expect_overlap(
            clip,
            rail,
            axes="xy",
            elem_a="top_slide_pad",
            elem_b="rail_body",
            min_overlap=0.08,
            name=f"clip_{i} captures rail footprint",
        )

        rest = ctx.part_world_position(clip)
        with ctx.pose({slide: 0.10}):
            moved = ctx.part_world_position(clip)
        ctx.check(
            f"clip_{i} translates forward on prismatic joint",
            rest is not None and moved is not None and moved[0] > rest[0] + 0.095,
            details=f"rest={rest}, moved={moved}",
        )

        lens_rest = ctx.part_element_world_aabb(head, elem="warm_lens")
        with ctx.pose({tilt: 0.70}):
            lens_tilted = ctx.part_element_world_aabb(head, elem="warm_lens")
        rest_center_x = None if lens_rest is None else 0.5 * (lens_rest[0][0] + lens_rest[1][0])
        tilted_center_x = None if lens_tilted is None else 0.5 * (lens_tilted[0][0] + lens_tilted[1][0])
        ctx.check(
            f"head_{i} pitch changes aim",
            rest_center_x is not None
            and tilted_center_x is not None
            and abs(tilted_center_x - rest_center_x) > 0.035,
            details=f"rest={lens_rest}, tilted={lens_tilted}",
        )

        if i == 0:
            ctx.expect_gap(
                swivel,
                head,
                axis="y",
                positive_elem="yoke_arm_1",
                negative_elem="head_collar",
                min_gap=0.004,
                max_gap=0.012,
                name="positive yoke clears collar",
            )
            ctx.expect_gap(
                head,
                swivel,
                axis="y",
                positive_elem="head_collar",
                negative_elem="yoke_arm_0",
                min_gap=0.004,
                max_gap=0.012,
                name="negative yoke clears collar",
            )

    return ctx.report()


object_model = build_object_model()
