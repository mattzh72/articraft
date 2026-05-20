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
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_gooseneck_faucet")

    chrome = model.material("brushed_chrome", rgba=(0.72, 0.70, 0.66, 1.0))
    polished = model.material("polished_edges", rgba=(0.88, 0.86, 0.80, 1.0))
    dark = model.material("dark_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    nozzle = model.material("black_nozzle", rgba=(0.02, 0.02, 0.025, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.055, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=polished,
        name="deck_flange",
    )
    base.visual(
        Cylinder(radius=0.032, length=0.225),
        origin=Origin(xyz=(0.0, 0.0, 0.1245)),
        material=chrome,
        name="mounting_stud",
    )
    base.visual(
        Cylinder(radius=0.037, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.247)),
        material=polished,
        name="bearing_collar",
    )
    base.visual(
        Cylinder(radius=0.034, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.259)),
        material=dark,
        name="top_gasket",
    )
    base.visual(
        Cylinder(radius=0.022, length=0.050),
        origin=Origin(xyz=(0.0, 0.056, 0.140), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="valve_boss",
    )

    spout = model.part("spout")
    spout.visual(
        Cylinder(radius=0.034, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=chrome,
        name="rotating_sleeve",
    )
    spout_path = [
        (0.000, 0.0, 0.030),
        (0.000, 0.0, 0.150),
        (0.035, 0.0, 0.230),
        (0.115, 0.0, 0.265),
        (0.195, 0.0, 0.225),
        (0.235, 0.0, 0.105),
    ]
    spout.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                spout_path,
                radius=0.012,
                samples_per_segment=16,
                radial_segments=32,
            ),
            "gooseneck_tube",
        ),
        material=chrome,
        name="gooseneck_tube",
    )
    spout.visual(
        Cylinder(radius=0.019, length=0.038),
        origin=Origin(xyz=(0.235, 0.0, 0.092)),
        material=polished,
        name="outlet_collar",
    )

    spray_wand = model.part("spray_wand")
    spray_wand.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=polished,
        name="seat_shoulder",
    )
    spray_wand.visual(
        Cylinder(radius=0.0165, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=chrome,
        name="wand_body",
    )
    spray_wand.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.092)),
        material=nozzle,
        name="spray_face",
    )

    side_lever = model.part("side_lever")
    side_lever.visual(
        Cylinder(radius=0.026, length=0.022),
        origin=Origin(xyz=(0.0, 0.011, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="lever_hub",
    )
    side_lever.visual(
        Box(size=(0.014, 0.012, 0.088)),
        origin=Origin(xyz=(0.0, 0.024, 0.046)),
        material=chrome,
        name="lever_stem",
    )
    side_lever.visual(
        Box(size=(0.026, 0.014, 0.050)),
        origin=Origin(xyz=(0.0, 0.026, 0.103)),
        material=polished,
        name="lever_paddle",
    )

    model.articulation(
        "base_to_spout",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, 0.261)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0),
    )
    model.articulation(
        "base_to_side_lever",
        ArticulationType.REVOLUTE,
        parent=base,
        child=side_lever,
        origin=Origin(xyz=(0.0, 0.081, 0.140)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=2.0, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "spout_to_spray_wand",
        ArticulationType.PRISMATIC,
        parent=spout,
        child=spray_wand,
        origin=Origin(xyz=(0.235, 0.0, 0.073)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.35, lower=0.0, upper=0.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    spout = object_model.get_part("spout")
    spray_wand = object_model.get_part("spray_wand")
    side_lever = object_model.get_part("side_lever")
    spout_joint = object_model.get_articulation("base_to_spout")
    lever_joint = object_model.get_articulation("base_to_side_lever")
    wand_joint = object_model.get_articulation("spout_to_spray_wand")

    def _center_x(aabb):
        lo, hi = aabb
        return (lo[0] + hi[0]) * 0.5

    def _center_y(aabb):
        lo, hi = aabb
        return (lo[1] + hi[1]) * 0.5

    ctx.expect_contact(
        spout,
        base,
        elem_a="rotating_sleeve",
        elem_b="top_gasket",
        name="spout sleeve is seated on the bearing gasket",
    )
    ctx.expect_gap(
        spout,
        spray_wand,
        axis="z",
        positive_elem="outlet_collar",
        negative_elem="seat_shoulder",
        max_gap=0.001,
        max_penetration=1e-6,
        name="spray wand shoulder seats against the outlet collar",
    )
    ctx.expect_overlap(
        spray_wand,
        spout,
        axes="xy",
        elem_a="seat_shoulder",
        elem_b="outlet_collar",
        min_overlap=0.015,
        name="spray wand is centered in the spout outlet",
    )

    rest_wand_pos = ctx.part_world_position(spray_wand)
    with ctx.pose({wand_joint: 0.18}):
        extended_wand_pos = ctx.part_world_position(spray_wand)
        ctx.expect_overlap(
            spray_wand,
            spout,
            axes="xy",
            elem_a="seat_shoulder",
            elem_b="outlet_collar",
            min_overlap=0.015,
            name="extended spray wand remains aligned with the outlet",
        )
    ctx.check(
        "spray wand extends downward from the spout",
        rest_wand_pos is not None
        and extended_wand_pos is not None
        and extended_wand_pos[2] < rest_wand_pos[2] - 0.15,
        details=f"rest={rest_wand_pos}, extended={extended_wand_pos}",
    )

    rest_outlet = ctx.part_element_world_aabb(spout, elem="outlet_collar")
    with ctx.pose({spout_joint: math.pi / 2.0}):
        turned_outlet = ctx.part_element_world_aabb(spout, elem="outlet_collar")
    ctx.check(
        "gooseneck spout rotates continuously about the vertical stud",
        rest_outlet is not None
        and turned_outlet is not None
        and _center_x(rest_outlet) > 0.20
        and _center_y(turned_outlet) > 0.20,
        details=f"rest={rest_outlet}, turned={turned_outlet}",
    )

    rest_paddle = ctx.part_element_world_aabb(side_lever, elem="lever_paddle")
    with ctx.pose({lever_joint: 0.60}):
        rotated_paddle = ctx.part_element_world_aabb(side_lever, elem="lever_paddle")
    ctx.check(
        "side lever rotates about the valve axis",
        rest_paddle is not None
        and rotated_paddle is not None
        and _center_x(rotated_paddle) > _center_x(rest_paddle) + 0.04,
        details=f"rest={rest_paddle}, rotated={rotated_paddle}",
    )

    return ctx.report()


object_model = build_object_model()
