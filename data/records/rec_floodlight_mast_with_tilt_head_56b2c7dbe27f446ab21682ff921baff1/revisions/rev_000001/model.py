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
    model = ArticulatedObject(name="four_head_floodlight_mast")

    galvanised = Material("galvanised_steel", rgba=(0.55, 0.58, 0.57, 1.0))
    dark_steel = Material("dark_powdercoat", rgba=(0.06, 0.065, 0.07, 1.0))
    rubber_black = Material("black_gasket", rgba=(0.005, 0.005, 0.004, 1.0))
    lens_glass = Material("warm_frosted_glass", rgba=(1.0, 0.86, 0.45, 0.62))
    led_warm = Material("warm_led_cells", rgba=(1.0, 0.93, 0.62, 1.0))
    bolt_dark = Material("dark_bolts", rgba=(0.015, 0.015, 0.014, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((0.70, 0.70, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=galvanised,
        name="base_plate",
    )
    mast.visual(
        Cylinder(radius=0.085, length=7.45),
        origin=Origin(xyz=(0.0, 0.0, 3.785)),
        material=galvanised,
        name="round_pole",
    )
    mast.visual(
        Cylinder(radius=0.12, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 7.55)),
        material=galvanised,
        name="top_collar",
    )
    mast.visual(
        Box((3.55, 0.14, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 7.62)),
        material=galvanised,
        name="top_cross_arm",
    )
    mast.visual(
        Box((0.34, 0.20, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 7.58)),
        material=galvanised,
        name="arm_center_saddle",
    )
    lamp_xs = (-1.20, -0.40, 0.40, 1.20)
    pivot_z = 7.14
    pivot_y = 0.0

    for idx, x in enumerate(lamp_xs):
        mast.visual(
            Box((0.62, 0.16, 0.08)),
            origin=Origin(xyz=(x, -0.01, 7.555)),
            material=galvanised,
            name=f"arm_clamp_{idx}",
        )
        mast.visual(
            Box((0.08, 0.08, 0.36)),
            origin=Origin(xyz=(x, -0.115, 7.37)),
            material=galvanised,
            name=f"drop_post_{idx}",
        )
        mast.visual(
            Box((0.63, 0.12, 0.07)),
            origin=Origin(xyz=(x, -0.055, pivot_z + 0.165)),
            material=galvanised,
            name=f"yoke_bridge_{idx}",
        )
        mast.visual(
            Box((0.038, 0.18, 0.34)),
            origin=Origin(xyz=(x - 0.31, 0.02, pivot_z)),
            material=galvanised,
            name=f"yoke_cheek_{idx}_0",
        )
        mast.visual(
            Box((0.038, 0.18, 0.34)),
            origin=Origin(xyz=(x + 0.31, 0.02, pivot_z)),
            material=galvanised,
            name=f"yoke_cheek_{idx}_1",
        )
        mast.visual(
            Cylinder(radius=0.058, length=0.026),
            origin=Origin(
                xyz=(x - 0.338, 0.02, pivot_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=bolt_dark,
            name=f"pivot_pad_{idx}_0",
        )
        mast.visual(
            Cylinder(radius=0.058, length=0.026),
            origin=Origin(
                xyz=(x + 0.338, 0.02, pivot_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=bolt_dark,
            name=f"pivot_pad_{idx}_1",
        )

        lamp = model.part(f"lamp_{idx}")
        lamp.visual(
            Cylinder(radius=0.026, length=0.63),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=galvanised,
            name="trunnion_pin",
        )
        lamp.visual(
            Box((0.048, 0.22, 0.18)),
            origin=Origin(xyz=(-0.235, 0.095, -0.075)),
            material=dark_steel,
            name="side_lug_0",
        )
        lamp.visual(
            Box((0.048, 0.22, 0.18)),
            origin=Origin(xyz=(0.235, 0.095, -0.075)),
            material=dark_steel,
            name="side_lug_1",
        )
        lamp.visual(
            Box((0.50, 0.48, 0.24)),
            origin=Origin(xyz=(0.0, 0.34, -0.14)),
            material=dark_steel,
            name="lamp_body",
        )
        lamp.visual(
            Box((0.54, 0.075, 0.035)),
            origin=Origin(xyz=(0.0, 0.54, -0.005)),
            material=dark_steel,
            name="top_visor",
        )
        lamp.visual(
            Box((0.46, 0.018, 0.18)),
            origin=Origin(xyz=(0.0, 0.576, -0.14)),
            material=lens_glass,
            name="front_lens",
        )
        lamp.visual(
            Box((0.54, 0.030, 0.030)),
            origin=Origin(xyz=(0.0, 0.586, -0.025)),
            material=rubber_black,
            name="upper_bezel",
        )
        lamp.visual(
            Box((0.54, 0.030, 0.030)),
            origin=Origin(xyz=(0.0, 0.586, -0.255)),
            material=rubber_black,
            name="lower_bezel",
        )
        lamp.visual(
            Box((0.032, 0.030, 0.24)),
            origin=Origin(xyz=(-0.266, 0.586, -0.14)),
            material=rubber_black,
            name="side_bezel_0",
        )
        lamp.visual(
            Box((0.032, 0.030, 0.24)),
            origin=Origin(xyz=(0.266, 0.586, -0.14)),
            material=rubber_black,
            name="side_bezel_1",
        )

        for fin_idx, fin_x in enumerate((-0.18, -0.108, -0.036, 0.036, 0.108, 0.18)):
            lamp.visual(
                Box((0.022, 0.075, 0.17)),
                origin=Origin(xyz=(fin_x, 0.085, -0.14)),
                material=dark_steel,
                name=f"heat_fin_{fin_idx}",
            )
        for led_idx, (lx, lz) in enumerate(
            ((-0.115, -0.095), (0.115, -0.095), (-0.115, -0.185), (0.115, -0.185))
        ):
            lamp.visual(
                Cylinder(radius=0.032, length=0.012),
                origin=Origin(xyz=(lx, 0.586, lz), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=led_warm,
                name=f"led_cell_{led_idx}",
            )

        model.articulation(
            f"mast_to_lamp_{idx}",
            ArticulationType.REVOLUTE,
            parent=mast,
            child=lamp,
            origin=Origin(xyz=(x, pivot_y, pivot_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=120.0, velocity=0.7, lower=-0.60, upper=0.55),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    lamp_parts = [object_model.get_part(f"lamp_{idx}") for idx in range(4)]
    joints = [object_model.get_articulation(f"mast_to_lamp_{idx}") for idx in range(4)]

    ctx.check(
        "four individual lamp heads",
        len(lamp_parts) == 4 and len(joints) == 4,
        details=f"lamp_parts={len(lamp_parts)}, joints={len(joints)}",
    )
    ctx.check(
        "each lamp has its own tilt revolute",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=", ".join(str(j.articulation_type) for j in joints),
    )

    for idx, (lamp, joint) in enumerate(zip(lamp_parts, joints)):
        for cheek_idx in (0, 1):
            ctx.allow_overlap(
                mast,
                lamp,
                elem_a=f"yoke_cheek_{idx}_{cheek_idx}",
                elem_b="trunnion_pin",
                reason=(
                    "The floodlight trunnion pin is intentionally captured through the "
                    "solid proxy cheek, representing the real bored pivot hole."
                ),
            )
            ctx.expect_overlap(
                mast,
                lamp,
                axes="x",
                elem_a=f"yoke_cheek_{idx}_{cheek_idx}",
                elem_b="trunnion_pin",
                min_overlap=0.012,
                name=f"lamp_{idx} trunnion captured in cheek_{cheek_idx}",
            )

        ctx.expect_gap(
            lamp,
            mast,
            axis="x",
            positive_elem="trunnion_pin",
            negative_elem=f"pivot_pad_{idx}_0",
            min_gap=0.0,
            max_gap=0.055,
            name=f"lamp_{idx} pin nearly reaches first yoke pad",
        )
        ctx.expect_gap(
            mast,
            lamp,
            axis="x",
            positive_elem=f"pivot_pad_{idx}_1",
            negative_elem="trunnion_pin",
            min_gap=0.0,
            max_gap=0.055,
            name=f"lamp_{idx} pin nearly reaches second yoke pad",
        )

        rest_lens = ctx.part_element_world_aabb(lamp, elem="front_lens")
        with ctx.pose({joint: 0.45}):
            up_lens = ctx.part_element_world_aabb(lamp, elem="front_lens")
        with ctx.pose({joint: -0.45}):
            down_lens = ctx.part_element_world_aabb(lamp, elem="front_lens")

        if rest_lens is not None and up_lens is not None and down_lens is not None:
            rest_z = (rest_lens[0][2] + rest_lens[1][2]) * 0.5
            up_z = (up_lens[0][2] + up_lens[1][2]) * 0.5
            down_z = (down_lens[0][2] + down_lens[1][2]) * 0.5
            ctx.check(
                f"lamp_{idx} tilt changes aiming elevation",
                up_z > rest_z + 0.18 and down_z < rest_z - 0.12,
                details=f"down={down_z:.3f}, rest={rest_z:.3f}, up={up_z:.3f}",
            )
        else:
            ctx.fail(f"lamp_{idx} lens aabb available", "front_lens AABB could not be measured")

    return ctx.report()


object_model = build_object_model()
