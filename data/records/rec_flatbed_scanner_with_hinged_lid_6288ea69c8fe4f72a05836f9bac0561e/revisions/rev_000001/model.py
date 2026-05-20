from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flatbed_document_scanner")

    case_mat = model.material("warm_white_plastic", rgba=(0.86, 0.86, 0.80, 1.0))
    dark_mat = model.material("charcoal_bezel", rgba=(0.04, 0.045, 0.05, 1.0))
    glass_mat = model.material("slightly_blue_glass", rgba=(0.45, 0.75, 0.92, 0.42))
    rubber_mat = model.material("soft_white_pad", rgba=(0.93, 0.93, 0.88, 1.0))
    metal_mat = model.material("satin_hinge_metal", rgba=(0.55, 0.56, 0.54, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.56, 0.40, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=case_mat,
        name="case_shell",
    )
    body.visual(
        Box((0.52, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, 0.206, 0.061)),
        material=case_mat,
        name="rear_hinge_ledge",
    )
    body.visual(
        Box((0.54, 0.032, 0.016)),
        origin=Origin(xyz=(0.0, -0.184, 0.063)),
        material=dark_mat,
        name="front_lip",
    )
    body.visual(
        Box((0.48, 0.040, 0.012)),
        origin=Origin(xyz=(0.0, -0.160, 0.061)),
        material=dark_mat,
        name="front_bezel",
    )
    body.visual(
        Box((0.48, 0.050, 0.012)),
        origin=Origin(xyz=(0.0, 0.135, 0.061)),
        material=dark_mat,
        name="rear_bezel",
    )
    body.visual(
        Box((0.035, 0.300, 0.012)),
        origin=Origin(xyz=(-0.2225, -0.010, 0.061)),
        material=dark_mat,
        name="bezel_0",
    )
    body.visual(
        Box((0.035, 0.300, 0.012)),
        origin=Origin(xyz=(0.2225, -0.010, 0.061)),
        material=dark_mat,
        name="bezel_1",
    )
    body.visual(
        Box((0.400, 0.250, 0.004)),
        origin=Origin(xyz=(0.0, -0.015, 0.057)),
        material=glass_mat,
        name="glass_platen",
    )
    body.visual(
        Box((0.390, 0.010, 0.0018)),
        origin=Origin(xyz=(0.0, 0.103, 0.0599)),
        material=case_mat,
        name="registration_strip",
    )

    hinge_axis_y = 0.213
    hinge_axis_z = 0.085
    hinge_centers = (-0.160, 0.160)
    hinge_outer_len = 0.022
    hinge_moving_len = 0.026
    hinge_gap = 0.0
    hinge_seg_offset = hinge_moving_len / 2.0 + hinge_gap + hinge_outer_len / 2.0
    barrel_radius = 0.011
    barrel_rot = Origin(rpy=(0.0, math.pi / 2.0, 0.0))

    for i, hinge_x in enumerate(hinge_centers):
        for side, x_offset in enumerate((-hinge_seg_offset, hinge_seg_offset)):
            x = hinge_x + x_offset
            body.visual(
                Box((hinge_outer_len, 0.018, 0.022)),
                origin=Origin(xyz=(x, hinge_axis_y, 0.066)),
                material=metal_mat,
                name=f"hinge_mount_{i}_{side}",
            )
            body.visual(
                Cylinder(radius=barrel_radius, length=hinge_outer_len),
                origin=Origin(
                    xyz=(x, hinge_axis_y, hinge_axis_z),
                    rpy=barrel_rot.rpy,
                ),
                material=metal_mat,
                name=f"fixed_knuckle_{i}_{side}",
            )
        body.visual(
            Cylinder(radius=0.004, length=0.004),
            origin=Origin(
                xyz=(hinge_x - 0.037, hinge_axis_y, hinge_axis_z),
                rpy=barrel_rot.rpy,
            ),
            material=metal_mat,
            name=f"pin_head_{i}_0",
        )
        body.visual(
            Cylinder(radius=0.004, length=0.004),
            origin=Origin(
                xyz=(hinge_x + 0.037, hinge_axis_y, hinge_axis_z),
                rpy=barrel_rot.rpy,
            ),
            material=metal_mat,
            name=f"pin_head_{i}_1",
        )

    lid = model.part("lid")
    lid.visual(
        Box((0.50, 0.360, 0.024)),
        origin=Origin(xyz=(0.0, -0.193, -0.002)),
        material=case_mat,
        name="lid_panel",
    )
    lid.visual(
        Box((0.382, 0.232, 0.006)),
        origin=Origin(xyz=(0.0, -0.228, -0.017)),
        material=rubber_mat,
        name="pressure_pad",
    )
    for i, hinge_x in enumerate(hinge_centers):
        lid.visual(
            Cylinder(radius=barrel_radius, length=hinge_moving_len),
            origin=Origin(xyz=(hinge_x, 0.0, 0.0), rpy=barrel_rot.rpy),
            material=metal_mat,
            name=f"moving_knuckle_{i}",
        )
        lid.visual(
            Box((hinge_moving_len, 0.030, 0.006)),
            origin=Origin(xyz=(hinge_x, -0.014, -0.006)),
            material=metal_mat,
            name=f"moving_leaf_{i}",
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="glass_platen",
        min_overlap=0.20,
        name="closed lid covers the glass platen footprint",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="pressure_pad",
        negative_elem="glass_platen",
        min_gap=0.004,
        max_gap=0.010,
        name="pressure pad is just above the platen when shut",
    )

    closed_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({hinge: 1.05}):
        open_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="glass_platen",
            min_gap=0.020,
            name="opened lid clears the glass",
        )

    ctx.check(
        "rear hinge opens the lid upward",
        closed_panel is not None
        and open_panel is not None
        and open_panel[1][2] > closed_panel[1][2] + 0.20,
        details=f"closed_panel={closed_panel}, open_panel={open_panel}",
    )
    ctx.check(
        "two visible hinge knuckles are modeled on the lid",
        lid.get_visual("moving_knuckle_0") is not None
        and lid.get_visual("moving_knuckle_1") is not None,
    )

    return ctx.report()


object_model = build_object_model()
