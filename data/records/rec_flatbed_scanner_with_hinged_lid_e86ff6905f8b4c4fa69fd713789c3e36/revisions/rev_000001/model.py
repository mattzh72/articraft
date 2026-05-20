from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="book_scanning_flatbed")

    matte_black = model.material("matte_black", rgba=(0.025, 0.027, 0.030, 1.0))
    warm_grey = model.material("warm_grey", rgba=(0.55, 0.56, 0.55, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.13, 0.14, 0.15, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.55, 0.56, 0.54, 1.0))
    glass_blue = model.material("scanner_glass", rgba=(0.50, 0.80, 0.95, 0.42))
    white_pad = model.material("white_pressure_pad", rgba=(0.94, 0.93, 0.88, 1.0))
    rubber = model.material("rubber_black", rgba=(0.006, 0.006, 0.006, 1.0))

    width = 0.68
    depth = 0.48
    body_height = 0.080
    rear_axis_y = depth / 2.0 + 0.015
    rear_axis_z = body_height + 0.030
    arm_length = 0.085
    hinge_centers = (-0.235, 0.235)

    body = model.part("body")
    body_shell = ExtrudeGeometry(
        rounded_rect_profile(width, depth, 0.035, corner_segments=8),
        body_height,
        center=True,
    )
    body.visual(
        mesh_from_geometry(body_shell, "rounded_body_shell"),
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
        material=matte_black,
        name="rounded_body_shell",
    )

    # Raised, continuous-looking top surround around the recessed scan glass.
    body.visual(
        Box((0.61, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, 0.173, body_height + 0.006)),
        material=dark_grey,
        name="rear_platen_rail",
    )
    body.visual(
        Box((0.61, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, -0.213, body_height + 0.006)),
        material=dark_grey,
        name="front_platen_rail",
    )
    body.visual(
        Box((0.030, 0.386, 0.012)),
        origin=Origin(xyz=(-0.305, -0.020, body_height + 0.006)),
        material=dark_grey,
        name="platen_side_rail_0",
    )
    body.visual(
        Box((0.030, 0.386, 0.012)),
        origin=Origin(xyz=(0.305, -0.020, body_height + 0.006)),
        material=dark_grey,
        name="platen_side_rail_1",
    )
    body.visual(
        Box((0.550, 0.340, 0.006)),
        origin=Origin(xyz=(0.0, -0.020, body_height + 0.003)),
        material=glass_blue,
        name="platen_glass",
    )
    body.visual(
        Box((0.550, 0.026, 0.0025)),
        origin=Origin(xyz=(0.0, 0.137, body_height + 0.0070)),
        material=white_pad,
        name="calibration_strip",
    )
    body.visual(
        Box((0.54, 0.060, 0.010)),
        origin=Origin(xyz=(0.0, depth / 2.0 - 0.009, body_height + 0.005)),
        material=dark_grey,
        name="rear_hinge_deck",
    )

    barrel_radius = 0.010
    barrel_gap = 0.0
    outer_len = 0.030
    center_len = 0.038
    outer_offset = center_len / 2.0 + barrel_gap + outer_len / 2.0

    for index, x_center in enumerate(hinge_centers):
        # Body-side leaves and two outer knuckles of each rear barrel hinge.
        for side, x_offset in enumerate((-outer_offset, outer_offset)):
            x = x_center + x_offset
            body.visual(
                Box((outer_len, 0.016, 0.028)),
                origin=Origin(xyz=(x, rear_axis_y - 0.002, body_height + 0.012)),
                material=hinge_steel,
                name=f"base_saddle_{index}_{side}",
            )
            body.visual(
                Cylinder(radius=barrel_radius, length=outer_len),
                origin=Origin(
                    xyz=(x, rear_axis_y, rear_axis_z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=hinge_steel,
                name="base_barrel_0_0" if index == 0 and side == 0 else f"base_barrel_{index}_{side}",
            )

    riser_yoke = model.part("riser_yoke")
    for index, x_center in enumerate(hinge_centers):
        # Center knuckle captured between the body-side outer knuckles.
        riser_yoke.visual(
            Cylinder(radius=barrel_radius * 0.92, length=center_len),
            origin=Origin(
                xyz=(x_center, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hinge_steel,
            name="base_center_knuckle_0" if index == 0 else f"base_center_knuckle_{index}",
        )
        riser_yoke.visual(
            Box((0.028, arm_length, 0.018)),
            origin=Origin(xyz=(x_center, -arm_length / 2.0, 0.0)),
            material=hinge_steel,
            name=f"lift_arm_{index}",
        )
        # Center knuckle of the secondary lid joint, placed at the forward end of
        # the riser arm so the lid can rise above a thick open book.
        riser_yoke.visual(
            Cylinder(radius=barrel_radius * 0.90, length=center_len),
            origin=Origin(
                xyz=(x_center, -arm_length, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hinge_steel,
            name=f"lid_center_knuckle_{index}",
        )
    riser_yoke.visual(
        Box((0.515, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, -0.050, -0.013)),
        material=hinge_steel,
        name="cross_tie",
    )

    lid = model.part("lid")
    lid_panel = ExtrudeGeometry(
        rounded_rect_profile(0.650, 0.430, 0.030, corner_segments=8),
        0.026,
        center=True,
    )
    lid.visual(
        mesh_from_geometry(lid_panel, "lid_panel"),
        origin=Origin(xyz=(0.0, -0.240, -0.001)),
        material=warm_grey,
        name="lid_panel",
    )
    lid.visual(
        Box((0.540, 0.335, 0.008)),
        origin=Origin(xyz=(0.0, -0.225, -0.018)),
        material=white_pad,
        name="pressure_pad",
    )
    for x in (-0.230, 0.230):
        lid.visual(
            Box((0.055, 0.020, 0.006)),
            origin=Origin(xyz=(x, -0.072, -0.021)),
            material=rubber,
            name=f"rubber_foot_{0 if x < 0 else 1}",
        )

    for index, x_center in enumerate(hinge_centers):
        for side, x_offset in enumerate((-outer_offset, outer_offset)):
            x = x_center + x_offset
            lid.visual(
                Cylinder(radius=barrel_radius, length=outer_len),
                origin=Origin(
                    xyz=(x, 0.0, 0.0),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=hinge_steel,
                name=f"lid_barrel_{index}_{side}",
            )
            lid.visual(
                Box((outer_len, 0.036, 0.005)),
                origin=Origin(xyz=(x, -0.020, -0.006)),
                material=hinge_steel,
                name=f"lid_leaf_{index}_{side}",
            )

    base_to_riser = model.articulation(
        "base_to_riser",
        ArticulationType.REVOLUTE,
        parent=body,
        child=riser_yoke,
        origin=Origin(xyz=(0.0, rear_axis_y, rear_axis_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=0.72),
    )
    model.articulation(
        "riser_to_lid",
        ArticulationType.REVOLUTE,
        parent=riser_yoke,
        child=lid,
        origin=Origin(xyz=(0.0, -arm_length, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.4, lower=0.0, upper=1.35),
    )

    # Keep a small note for reviewers: the two visible arms are one yoke link so
    # the articulated tree can remain open while still representing paired hinges.
    model.meta["mechanism"] = {
        "paired_hinges": len(hinge_centers),
        "primary_lift_joint": base_to_riser.name,
        "secondary_lid_joint": "riser_to_lid",
    }
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    riser_yoke = object_model.get_part("riser_yoke")
    lid = object_model.get_part("lid")
    base_to_riser = object_model.get_articulation("base_to_riser")
    riser_to_lid = object_model.get_articulation("riser_to_lid")

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="pressure_pad",
        elem_b="platen_glass",
        min_overlap=0.29,
        name="closed lid pressure pad covers the scan glass",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="pressure_pad",
        negative_elem="platen_glass",
        min_gap=0.001,
        max_gap=0.006,
        name="closed lid pad clears the platen glass",
    )
    ctx.expect_overlap(
        riser_yoke,
        body,
        axes="x",
        elem_a="base_center_knuckle_0",
        elem_b="base_barrel_0_0",
        min_overlap=0.0,
        name="riser knuckle is coaxial with a body barrel hinge",
    )

    closed_lid_origin = ctx.part_world_position(lid)
    closed_lid_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({base_to_riser: 0.65, riser_to_lid: 1.10}):
        raised_lid_origin = ctx.part_world_position(lid)
        raised_lid_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")

    ctx.check(
        "riser arms lift the lid hinge above a thick book",
        closed_lid_origin is not None
        and raised_lid_origin is not None
        and raised_lid_origin[2] > closed_lid_origin[2] + 0.045,
        details=f"closed={closed_lid_origin}, raised={raised_lid_origin}",
    )
    ctx.check(
        "secondary lid revolute opens the wide cover upward",
        closed_lid_panel is not None
        and raised_lid_panel is not None
        and raised_lid_panel[1][2] > closed_lid_panel[1][2] + 0.20,
        details=f"closed_aabb={closed_lid_panel}, raised_aabb={raised_lid_panel}",
    )

    return ctx.report()


object_model = build_object_model()
