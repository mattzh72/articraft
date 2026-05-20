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
    model = ArticulatedObject(name="calibration_sewing_box")

    model.material("powder_coat", rgba=(0.18, 0.22, 0.26, 1.0))
    model.material("machined_datum", rgba=(0.72, 0.74, 0.72, 1.0))
    model.material("dark_gap", rgba=(0.015, 0.017, 0.018, 1.0))
    model.material("hinge_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    model.material("engraving_black", rgba=(0.02, 0.02, 0.018, 1.0))
    model.material("screw_black", rgba=(0.08, 0.08, 0.075, 1.0))
    model.material("interior_blue", rgba=(0.08, 0.16, 0.24, 1.0))

    body = model.part("body")
    lid = model.part("lid")

    # Body: a real open tray, not a solid block.  The flat capped rim surfaces
    # are intentionally datum-like; the black strips are the designed lid gap.
    body.visual(Box((0.320, 0.220, 0.012)), origin=Origin(xyz=(0.0, 0.0, 0.006)), material="powder_coat", name="floor")
    body.visual(Box((0.012, 0.220, 0.098)), origin=Origin(xyz=(0.154, 0.0, 0.061)), material="powder_coat", name="front_wall")
    body.visual(Box((0.012, 0.220, 0.098)), origin=Origin(xyz=(-0.154, 0.0, 0.061)), material="powder_coat", name="rear_wall")
    body.visual(Box((0.320, 0.012, 0.098)), origin=Origin(xyz=(0.0, 0.104, 0.061)), material="powder_coat", name="side_wall_0")
    body.visual(Box((0.320, 0.012, 0.098)), origin=Origin(xyz=(0.0, -0.104, 0.061)), material="powder_coat", name="side_wall_1")

    body.visual(Box((0.320, 0.018, 0.002)), origin=Origin(xyz=(0.0, 0.101, 0.109)), material="machined_datum", name="datum_rail_0")
    body.visual(Box((0.320, 0.018, 0.002)), origin=Origin(xyz=(0.0, -0.101, 0.109)), material="machined_datum", name="datum_rail_1")
    body.visual(Box((0.018, 0.220, 0.002)), origin=Origin(xyz=(0.151, 0.0, 0.109)), material="machined_datum", name="datum_rail_front")
    body.visual(Box((0.018, 0.220, 0.002)), origin=Origin(xyz=(-0.151, 0.0, 0.109)), material="machined_datum", name="datum_rail_rear")
    body.visual(Box((0.014, 0.205, 0.001)), origin=Origin(xyz=(0.143, 0.0, 0.110)), material="dark_gap", name="front_gap_strip")
    body.visual(Box((0.300, 0.008, 0.001)), origin=Origin(xyz=(0.0, 0.095, 0.110)), material="dark_gap", name="side_gap_strip_0")
    body.visual(Box((0.300, 0.008, 0.001)), origin=Origin(xyz=(0.0, -0.095, 0.110)), material="dark_gap", name="side_gap_strip_1")

    # Sewing-box organization features are tied to the floor so the body reads
    # as one connected mechanical assembly.
    body.visual(Box((0.012, 0.178, 0.045)), origin=Origin(xyz=(0.020, 0.0, 0.0345)), material="interior_blue", name="center_divider")
    body.visual(Box((0.205, 0.012, 0.035)), origin=Origin(xyz=(-0.030, -0.040, 0.0295)), material="interior_blue", name="needle_divider")
    for idx, y in enumerate((-0.065, 0.000, 0.065)):
        body.visual(Cylinder(radius=0.006, length=0.045), origin=Origin(xyz=(-0.095, y, 0.0345)), material="interior_blue", name=f"spool_post_{idx}")

    # Front calibration-stop bosses for repeatable lid seating.
    for idx, y in enumerate((-0.073, 0.073)):
        body.visual(Box((0.008, 0.032, 0.026)), origin=Origin(xyz=(0.164, y, 0.075)), material="machined_datum", name=f"stop_boss_{idx}")

    hinge_x = -0.170
    hinge_z = 0.123
    body_knuckles = [(-0.078, 0.034), (0.0, 0.042), (0.078, 0.034)]
    lid_knuckles = [(-0.041, 0.034), (0.041, 0.034)]
    for idx, (y, length) in enumerate(body_knuckles):
        body.visual(
            Cylinder(radius=0.007, length=length),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="hinge_steel",
            name=f"body_knuckle_{idx}",
        )
        body.visual(
            Box((0.024, length, 0.012)),
            origin=Origin(xyz=(-0.164, y, 0.116)),
            material="hinge_steel",
            name=f"body_hinge_arm_{idx}",
        )

    # Lid geometry is authored in the hinge frame.  In the closed pose, the panel
    # extends forward from the pin line and leaves a controlled 1.5 mm datum gap.
    lid.visual(Box((0.330, 0.232, 0.014)), origin=Origin(xyz=(0.181, 0.0, -0.003)), material="powder_coat", name="lid_panel")
    lid.visual(Box((0.180, 0.082, 0.002)), origin=Origin(xyz=(0.205, 0.0, 0.005)), material="machined_datum", name="top_datum_plate")
    lid.visual(Box((0.010, 0.220, 0.020)), origin=Origin(xyz=(0.340, 0.0, -0.018)), material="powder_coat", name="front_lip")
    lid.visual(Box((0.210, 0.010, 0.008)), origin=Origin(xyz=(0.175, 0.055, -0.014)), material="powder_coat", name="underside_rib_0")
    lid.visual(Box((0.210, 0.010, 0.008)), origin=Origin(xyz=(0.175, -0.055, -0.014)), material="powder_coat", name="underside_rib_1")
    lid.visual(Box((0.010, 0.150, 0.008)), origin=Origin(xyz=(0.285, 0.0, -0.014)), material="powder_coat", name="underside_rib_front")
    for idx, (y, length) in enumerate(lid_knuckles):
        lid.visual(
            Cylinder(radius=0.007, length=length),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="hinge_steel",
            name=f"lid_knuckle_{idx}",
        )
        lid.visual(
            Box((0.018, length, 0.012)),
            origin=Origin(xyz=(0.008, y, -0.003)),
            material="hinge_steel",
            name=f"lid_hinge_arm_{idx}",
        )

    # Datum/index language: black engraved ticks are slightly seated into their
    # carrier surfaces so they remain mechanically connected.
    for idx, y in enumerate((-0.032, -0.016, 0.0, 0.016, 0.032)):
        lid.visual(Box((0.040, 0.0015, 0.001)), origin=Origin(xyz=(0.205, y, 0.006)), material="engraving_black", name=f"index_mark_{idx}")
    for idx, x in enumerate((-0.092, -0.046, 0.0, 0.046, 0.092)):
        body.visual(Box((0.002, 0.020, 0.001)), origin=Origin(xyz=(x, -0.101, 0.1104)), material="engraving_black", name=f"rim_tick_{idx}")

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.75),
    )

    # Two external stop screws rotate on their own axes.  They are mounted to the
    # front bosses rather than left free in space, and sit outside the lid sweep.
    for idx, y in enumerate((-0.073, 0.073)):
        screw = model.part(f"stop_screw_{idx}")
        screw.visual(
            Cylinder(radius=0.004, length=0.012),
            origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="hinge_steel",
            name="screw_shank",
        )
        screw.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="screw_black",
            name="knurled_head",
        )
        screw.visual(
            Box((0.001, 0.017, 0.001)),
            origin=Origin(xyz=(0.0205, 0.0, 0.0)),
            material="engraving_black",
            name="screw_index",
        )
        model.articulation(
            f"stop_screw_{idx}_spin",
            ArticulationType.REVOLUTE,
            parent=body,
            child=screw,
            origin=Origin(xyz=(0.168, y, 0.075)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.5, velocity=6.0, lower=-math.pi, upper=math.pi),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="front_gap_strip",
            min_gap=0.001,
            max_gap=0.0035,
            name="closed lid has controlled datum gap",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="datum_rail_front",
            min_overlap=0.010,
            name="lid covers front datum rail",
        )

    rest_pos = ctx.part_world_position(lid)
    rest_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({hinge: 1.35}):
        open_pos = ctx.part_world_position(lid)
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    ctx.check(
        "hinge axis stays fixed",
        rest_pos is not None and open_pos is not None and abs(rest_pos[0] - open_pos[0]) < 1e-6 and abs(rest_pos[2] - open_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, open={open_pos}",
    )
    ctx.check(
        "lid rotates upward",
        rest_aabb is not None and open_aabb is not None and open_aabb[1][2] > rest_aabb[1][2] + 0.18,
        details=f"rest_aabb={rest_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
