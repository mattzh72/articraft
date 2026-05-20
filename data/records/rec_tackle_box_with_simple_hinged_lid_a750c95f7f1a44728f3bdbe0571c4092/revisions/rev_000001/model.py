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
    model = ArticulatedObject(name="plain_hinged_tackle_box")

    blue = Material("blue_molded_plastic", rgba=(0.05, 0.22, 0.75, 1.0))
    dark_blue = Material("darker_blue_edges", rgba=(0.03, 0.12, 0.42, 1.0))
    tray_gray = Material("light_gray_tray", rgba=(0.72, 0.74, 0.72, 1.0))
    pin_dark = Material("dark_hinge_shadow", rgba=(0.03, 0.035, 0.04, 1.0))

    body = model.part("body")

    # Overall body footprint: about 34 cm deep by 46 cm wide by 14 cm tall.
    # The body is built as a real open shell rather than a solid block.
    body.visual(
        Box((0.340, 0.460, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_blue,
        name="bottom_shell",
    )
    body.visual(
        Box((0.012, 0.460, 0.128)),
        origin=Origin(xyz=(-0.164, 0.0, 0.076)),
        material=blue,
        name="rear_wall",
    )
    body.visual(
        Box((0.012, 0.460, 0.128)),
        origin=Origin(xyz=(0.164, 0.0, 0.076)),
        material=blue,
        name="front_wall",
    )
    for y, name in ((-0.224, "side_wall_0"), (0.224, "side_wall_1")):
        body.visual(
            Box((0.340, 0.012, 0.128)),
            origin=Origin(xyz=(0.0, y, 0.076)),
            material=blue,
            name=name,
        )

    # A fixed internal organizer tray with molded dividers, deliberately part
    # of the body assembly and not a second moving tray.
    body.visual(
        Box((0.318, 0.438, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.092)),
        material=tray_gray,
        name="tray_floor",
    )
    body.visual(
        Box((0.304, 0.010, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.107)),
        material=tray_gray,
        name="long_divider",
    )
    for x, y, sx, sy, name in (
        (-0.070, -0.105, 0.010, 0.195, "divider_0"),
        (0.075, -0.105, 0.010, 0.195, "divider_1"),
        (-0.035, 0.105, 0.010, 0.195, "divider_2"),
        (0.110, 0.105, 0.010, 0.195, "divider_3"),
        (0.000, 0.000, 0.304, 0.010, "cross_divider"),
    ):
        body.visual(
            Box((sx, sy, 0.026)),
            origin=Origin(xyz=(x, y, 0.107)),
            material=tray_gray,
            name=name,
        )

    hinge_x = -0.185
    hinge_z = 0.160
    hinge_r = 0.009
    cylinder_to_y = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))

    body.visual(
        Cylinder(radius=0.0035, length=0.440),
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z), rpy=cylinder_to_y.rpy),
        material=pin_dark,
        name="hinge_pin",
    )

    # Fixed hinge knuckles alternate with the lid knuckles along the rear edge.
    for y, length, name in (
        (-0.175, 0.070, "body_knuckle_0"),
        (0.000, 0.070, "body_knuckle_1"),
        (0.175, 0.070, "body_knuckle_2"),
    ):
        body.visual(
            Cylinder(radius=hinge_r, length=length),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=cylinder_to_y.rpy),
            material=dark_blue,
            name=name,
        )
        body.visual(
            Box((0.026, length, 0.016)),
            origin=Origin(xyz=(hinge_x + 0.012, y, hinge_z - 0.015)),
            material=dark_blue,
            name=f"hinge_leaf_{name[-1]}",
        )

    # Paired rear side cheeks bracket the lid panel without being another joint.
    for y, sign, suffix in ((-0.260, -1.0, "0"), (0.260, 1.0, "1")):
        body.visual(
            Box((0.062, 0.008, 0.066)),
            origin=Origin(xyz=(-0.178, y, 0.169)),
            material=dark_blue,
            name=f"side_cheek_{suffix}",
        )
        body.visual(
            Box((0.062, 0.046, 0.014)),
            origin=Origin(xyz=(-0.178, sign * 0.242, 0.134)),
            material=dark_blue,
            name=f"cheek_foot_{suffix}",
        )
        body.visual(
            Cylinder(radius=0.012, length=0.004),
            origin=Origin(
                xyz=(-0.185, sign * 0.266, hinge_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=pin_dark,
            name=f"cheek_boss_{suffix}",
        )

    lid = model.part("lid")
    lid.visual(
        Box((0.340, 0.480, 0.014)),
        # The child frame is on the hinge axis; the closed panel extends forward.
        origin=Origin(xyz=(0.204, 0.0, -0.010)),
        material=blue,
        name="lid_panel",
    )
    # Downturned lip on the front and shorter side flanges keep the lid plain
    # while reading as a molded tackle-box cover.
    lid.visual(
        Box((0.012, 0.480, 0.040)),
        origin=Origin(xyz=(0.376, 0.0, -0.034)),
        material=dark_blue,
        name="front_lip",
    )
    for y, name in ((-0.242, "side_lip_0"), (0.242, "side_lip_1")):
        lid.visual(
            Box((0.280, 0.010, 0.040)),
            origin=Origin(xyz=(0.232, y, -0.034)),
            material=dark_blue,
            name=name,
        )

    for y, length, name in (
        (-0.0875, 0.085, "lid_knuckle_0"),
        (0.0875, 0.085, "lid_knuckle_1"),
    ):
        lid.visual(
            Cylinder(radius=hinge_r, length=length),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=cylinder_to_y.rpy),
            material=blue,
            name=name,
        )
        lid.visual(
            Box((0.029, length, 0.010)),
            origin=Origin(xyz=(0.0205, y, -0.008)),
            material=blue,
            name=f"knuckle_web_{name[-1]}",
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        # With the closed lid extending along local +X from the hinge, -Y makes
        # positive motion lift the front edge upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    for knuckle_name in ("lid_knuckle_0", "lid_knuckle_1"):
        ctx.allow_overlap(
            body,
            lid,
            elem_a="hinge_pin",
            elem_b=knuckle_name,
            reason="The dark hinge pin intentionally passes through the lid knuckle barrel.",
        )
        ctx.expect_within(
            body,
            lid,
            axes="xz",
            inner_elem="hinge_pin",
            outer_elem=knuckle_name,
            margin=0.0,
            name=f"{knuckle_name} captures hinge pin",
        )
        ctx.expect_overlap(
            body,
            lid,
            axes="y",
            elem_a="hinge_pin",
            elem_b=knuckle_name,
            min_overlap=0.075,
            name=f"{knuckle_name} has retained pin length",
        )

    ctx.check(
        "single lid articulation",
        len(object_model.articulations) == 1,
        details=f"articulations={len(object_model.articulations)}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="front_wall",
            min_gap=0.001,
            max_gap=0.006,
            name="closed lid sits just above body rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="tray_floor",
            min_overlap=0.30,
            name="closed lid covers internal tray",
        )

    with ctx.pose({hinge: 1.20}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_lip",
            negative_elem="front_wall",
            min_gap=0.13,
            name="opened lid front edge rises above body",
        )

    return ctx.report()


object_model = build_object_model()
