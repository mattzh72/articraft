from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="parking_lot_boom_gate")

    galvanized = Material("galvanized_steel", rgba=(0.62, 0.65, 0.66, 1.0))
    dark_steel = Material("dark_powder_coated_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    black_rubber = Material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    safety_white = Material("safety_white", rgba=(0.92, 0.90, 0.84, 1.0))
    reflective_red = Material("reflective_red", rgba=(0.90, 0.04, 0.025, 1.0))
    reflective_yellow = Material("reflective_yellow", rgba=(1.00, 0.72, 0.07, 1.0))

    # Static post, service housing, base plate, and the fixed fork of the hinge.
    post = model.part("post")
    post.visual(
        Box((0.55, 0.45, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_steel,
        name="base_plate",
    )
    post.visual(
        Box((0.34, 0.28, 1.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
        material=galvanized,
        name="post_housing",
    )
    post.visual(
        Box((0.24, 0.014, 0.46)),
        origin=Origin(xyz=(0.0, -0.147, 0.56)),
        material=dark_steel,
        name="service_panel",
    )
    post.visual(
        Box((0.08, 0.018, 0.035)),
        origin=Origin(xyz=(0.0, -0.158, 0.80)),
        material=black_rubber,
        name="panel_pull",
    )
    post.visual(
        Box((0.42, 0.32, 0.185)),
        origin=Origin(xyz=(0.04, 0.0, 1.1775)),
        material=galvanized,
        name="head_cap",
    )
    post.visual(
        Box((0.08, 0.24, 0.16)),
        origin=Origin(xyz=(0.23, 0.0, 1.19)),
        material=dark_steel,
        name="hinge_mount_plate",
    )
    post.visual(
        Box((0.22, 0.045, 0.30)),
        origin=Origin(xyz=(0.35, -0.105, 1.34)),
        material=dark_steel,
        name="hinge_cheek_0",
    )
    post.visual(
        Box((0.22, 0.045, 0.30)),
        origin=Origin(xyz=(0.35, 0.105, 1.34)),
        material=dark_steel,
        name="hinge_cheek_1",
    )
    for i, (x, y) in enumerate(
        ((-0.20, -0.15), (-0.20, 0.15), (0.20, -0.15), (0.20, 0.15))
    ):
        post.visual(
            Cylinder(radius=0.026, length=0.014),
            origin=Origin(xyz=(x, y, 0.039)),
            material=galvanized,
            name=f"anchor_bolt_{i}",
        )

    # Moving boom frame.  The child frame is the hinge pin axis; at q=0 the
    # long rectangular tube points along +X and the counterweight tail along -X.
    boom = model.part("boom")
    boom.visual(
        Box((0.20, 0.10, 0.22)),
        origin=Origin(xyz=(0.03, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_lug",
    )
    boom.visual(
        Cylinder(radius=0.035, length=0.35),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="pivot_pin",
    )
    boom.visual(
        Cylinder(radius=0.058, length=0.018),
        origin=Origin(xyz=(0.0, 0.170, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="pin_head",
    )
    hex_nut = mesh_from_geometry(
        CylinderGeometry(0.060, 0.040, radial_segments=6),
        "lock_nut_hex",
    )
    boom.visual(
        hex_nut,
        origin=Origin(xyz=(0.0, -0.175, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="lock_nut",
    )
    boom.visual(
        Box((3.90, 0.080, 0.100)),
        origin=Origin(xyz=(2.05, 0.0, 0.0)),
        material=safety_white,
        name="main_tube",
    )
    boom.visual(
        Box((0.58, 0.080, 0.100)),
        origin=Origin(xyz=(-0.24, 0.0, 0.0)),
        material=safety_white,
        name="tail_tube",
    )
    boom.visual(
        Box((0.33, 0.22, 0.22)),
        origin=Origin(xyz=(-0.40, 0.0, 0.13)),
        material=dark_steel,
        name="counterweight",
    )
    boom.visual(
        Box((0.20, 0.12, 0.070)),
        origin=Origin(xyz=(-0.40, 0.0, 0.055)),
        material=galvanized,
        name="counterweight_clamp",
    )
    boom.visual(
        Box((3.35, 0.006, 0.035)),
        origin=Origin(xyz=(2.24, -0.0435, -0.020)),
        material=reflective_yellow,
        name="reflective_skirt",
    )
    for i, x in enumerate((0.78, 1.55, 2.32, 3.09)):
        boom.visual(
            Box((0.34, 0.008, 0.050)),
            origin=Origin(xyz=(x, -0.047, -0.018), rpy=(0.0, 0.0, 0.30)),
            material=reflective_red,
            name=f"red_marker_{i}",
        )
    boom.visual(
        Box((0.11, 0.098, 0.118)),
        origin=Origin(xyz=(4.055, 0.0, 0.0)),
        material=black_rubber,
        name="tip_cap",
    )

    model.articulation(
        "post_to_boom",
        ArticulationType.REVOLUTE,
        parent=post,
        child=boom,
        origin=Origin(xyz=(0.36, 0.0, 1.34)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.65, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    post = object_model.get_part("post")
    boom = object_model.get_part("boom")
    joint = object_model.get_articulation("post_to_boom")

    for cheek in ("hinge_cheek_0", "hinge_cheek_1"):
        ctx.allow_overlap(
            post,
            boom,
            elem_a=cheek,
            elem_b="pivot_pin",
            reason="The hinge pin is intentionally captured through the fixed fork cheek.",
        )
        ctx.expect_overlap(
            post,
            boom,
            axes="xyz",
            elem_a=cheek,
            elem_b="pivot_pin",
            min_overlap=0.010,
            name=f"{cheek} captures the pivot pin",
        )

    ctx.expect_gap(
        boom,
        post,
        axis="z",
        positive_elem="tail_tube",
        negative_elem="head_cap",
        min_gap=0.010,
        name="tail clears the post head",
    )
    ctx.expect_overlap(
        boom,
        post,
        axes="xz",
        elem_a="pivot_lug",
        elem_b="hinge_cheek_0",
        min_overlap=0.050,
        name="pivot lug sits inside the fork span",
    )

    closed_tip = ctx.part_element_world_aabb(boom, elem="tip_cap")
    with ctx.pose({joint: 1.20}):
        raised_tip = ctx.part_element_world_aabb(boom, elem="tip_cap")

    ctx.check(
        "boom raises upward at upper pose",
        closed_tip is not None
        and raised_tip is not None
        and raised_tip[0][2] > closed_tip[0][2] + 2.0,
        details=f"closed_tip={closed_tip}, raised_tip={raised_tip}",
    )

    return ctx.report()


object_model = build_object_model()
