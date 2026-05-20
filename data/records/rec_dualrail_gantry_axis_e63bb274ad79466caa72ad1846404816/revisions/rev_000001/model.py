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
    model = ArticulatedObject(name="dual_rail_gantry_axis")

    steel = model.material("blackened_steel", rgba=(0.10, 0.11, 0.11, 1.0))
    rail_steel = model.material("ground_rail_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    machined = model.material("machined_aluminum", rgba=(0.56, 0.59, 0.58, 1.0))
    dark_plate = model.material("dark_anodized_plate", rgba=(0.16, 0.17, 0.18, 1.0))
    fastener = model.material("dark_fasteners", rgba=(0.035, 0.035, 0.035, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.012, 1.0))
    tray_mat = model.material("galvanized_cable_tray", rgba=(0.44, 0.46, 0.45, 1.0))

    rail_frame = model.part("rail_frame")

    # Straight welded/machined rail support ladder.
    for y, name, riser_name, rail_name in (
        (0.32, "rail_beam_0", "riser_0", "rail_0"),
        (-0.32, "rail_beam_1", "riser_1", "rail_1"),
    ):
        rail_frame.visual(
            Box((1.60, 0.070, 0.100)),
            origin=Origin(xyz=(0.0, y, 0.050)),
            material=steel,
            name=name,
        )
        rail_frame.visual(
            Box((1.50, 0.052, 0.040)),
            origin=Origin(xyz=(0.0, y, 0.120)),
            material=machined,
            name=riser_name,
        )
        rail_frame.visual(
            Box((1.46, 0.035, 0.032)),
            origin=Origin(xyz=(0.0, y, 0.156)),
            material=rail_steel,
            name=rail_name,
        )
        for x, suffix in ((-0.78, "neg"), (0.78, "pos")):
            rail_frame.visual(
                Box((0.070, 0.038, 0.020)),
                origin=Origin(xyz=(x * 0.968, y, 0.162)),
                material=rail_steel,
                name=f"{suffix}_stop_mount_{name[-1]}",
            )
            rail_frame.visual(
                Box((0.040, 0.095, 0.080)),
                origin=Origin(xyz=(x, y, 0.210)),
                material=dark_plate,
                name=f"{suffix}_stop_{name[-1]}",
            )
            rail_frame.visual(
                Cylinder(radius=0.014, length=0.012),
                origin=Origin(xyz=(x, y, 0.255)),
                material=rubber,
                name=f"{suffix}_bumper_{name[-1]}",
            )

    for x, name in ((-0.72, "end_tie_0"), (0.72, "end_tie_1"), (0.0, "center_tie")):
        rail_frame.visual(
            Box((0.080, 0.760, 0.080)),
            origin=Origin(xyz=(x, 0.0, 0.040)),
            material=steel,
            name=name,
        )

    for x in (-0.70, -0.25, 0.25, 0.70):
        for y in (-0.32, 0.32):
            rail_frame.visual(
                Box((0.130, 0.130, 0.020)),
                origin=Origin(xyz=(x, y, 0.010)),
                material=steel,
                name=f"anchor_pad_{x:+.2f}_{y:+.2f}",
            )
            for dy in (-0.035, 0.035):
                rail_frame.visual(
                    Cylinder(radius=0.010, length=0.008),
                    origin=Origin(xyz=(x, y + dy, 0.024)),
                    material=fastener,
                    name=f"anchor_bolt_{x:+.2f}_{y+dy:+.2f}",
                )

    # Side cable tray and its load-bearing stand-off brackets.
    rail_frame.visual(
        Box((1.22, 0.080, 0.018)),
        origin=Origin(xyz=(0.0, -0.500, 0.300)),
        material=tray_mat,
        name="cable_tray_floor",
    )
    for y, name in ((-0.540, "tray_lip_0"), (-0.460, "tray_lip_1")):
        rail_frame.visual(
            Box((1.22, 0.012, 0.055)),
            origin=Origin(xyz=(0.0, y, 0.325)),
            material=tray_mat,
            name=name,
        )
    for x in (-0.50, 0.0, 0.50):
        rail_frame.visual(
            Box((0.040, 0.185, 0.025)),
            origin=Origin(xyz=(x, -0.425, 0.155)),
            material=steel,
            name=f"tray_arm_{x:+.2f}",
        )
        rail_frame.visual(
            Box((0.035, 0.035, 0.100)),
            origin=Origin(xyz=(x, -0.355, 0.115)),
            material=steel,
            name=f"tray_post_{x:+.2f}",
        )
        rail_frame.visual(
            Box((0.035, 0.035, 0.142)),
            origin=Origin(xyz=(x, -0.500, 0.229)),
            material=steel,
            name=f"tray_riser_{x:+.2f}",
        )

    bridge = model.part("bridge_carriage")

    # Four open linear-bearing saddles ride above the two ground rails.
    for rail_index, y, bearing_specs in (
        (
            0,
            0.32,
            (
                (0, -0.26, "bearing_0_0_cap", "bearing_0_0_shoe"),
                (1, 0.26, "bearing_0_1_cap", "bearing_0_1_shoe"),
            ),
        ),
        (
            1,
            -0.32,
            (
                (0, -0.26, "bearing_1_0_cap", "bearing_1_0_shoe"),
                (1, 0.26, "bearing_1_1_cap", "bearing_1_1_shoe"),
            ),
        ),
    ):
        for x_index, x, cap_name, shoe_name in bearing_specs:
            bridge.visual(
                Box((0.130, 0.110, 0.040)),
                origin=Origin(xyz=(x, y, 0.205)),
                material=machined,
                name=cap_name,
            )
            bridge.visual(
                Box((0.060, 0.025, 0.014)),
                origin=Origin(xyz=(x, y, 0.179)),
                material=rail_steel,
                name=shoe_name,
            )
            bridge.visual(
                Box((0.130, 0.018, 0.080)),
                origin=Origin(xyz=(x, y - 0.055, 0.175)),
                material=machined,
                name=f"bearing_{rail_index}_{x_index}_jaw_0",
            )
            bridge.visual(
                Box((0.130, 0.018, 0.080)),
                origin=Origin(xyz=(x, y + 0.055, 0.175)),
                material=machined,
                name=f"bearing_{rail_index}_{x_index}_jaw_1",
            )
            for end_index, wx in enumerate((x - 0.071, x + 0.071)):
                bridge.visual(
                    Box((0.012, 0.100, 0.050)),
                    origin=Origin(xyz=(wx, y, 0.200)),
                    material=rubber,
                    name=f"rail_wiper_{rail_index}_{x_index}_{end_index}",
                )

        bridge.visual(
            Box((0.640, 0.080, 0.035)),
            origin=Origin(xyz=(0.0, y, 0.242)),
            material=dark_plate,
            name=f"bearing_tie_{rail_index}",
        )
        bridge.visual(
            Box((0.160, 0.075, 0.180)),
            origin=Origin(xyz=(0.0, y, 0.325)),
            material=dark_plate,
            name=f"side_upright_{rail_index}",
        )

    # Stiff bridge member spanning both rails, with bolted face rails for the center truck.
    bridge.visual(
        Box((0.165, 0.790, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, 0.405)),
        material=dark_plate,
        name="bridge_box_beam",
    )
    bridge.visual(
        Box((0.205, 0.790, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.4675)),
        material=machined,
        name="bridge_top_cap",
    )
    bridge.visual(
        Box((0.030, 0.790, 0.065)),
        origin=Origin(xyz=(-0.100, 0.0, 0.405)),
        material=dark_plate,
        name="rear_stiffener_web",
    )
    for y in (-0.26, 0.0, 0.26):
        bridge.visual(
            Box((0.205, 0.020, 0.105)),
            origin=Origin(xyz=(0.0, y, 0.405)),
            material=machined,
            name=f"vertical_gusset_{y:+.2f}",
        )

    for z, name in ((0.430, "upper_bridge_rail"), (0.365, "lower_bridge_rail")):
        bridge.visual(
            Box((0.030, 0.620, 0.025)),
            origin=Origin(xyz=(0.095, 0.0, z)),
            material=rail_steel,
            name=name,
        )
        for y in (-0.245, -0.080, 0.080, 0.245):
            bridge.visual(
                Cylinder(radius=0.008, length=0.010),
                origin=Origin(xyz=(0.074, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=fastener,
                name=f"{name}_screw_{y:+.2f}",
            )

    # Small hard-mounted cable tray pickup bracket on the moving carriage.
    bridge.visual(
        Box((0.045, 0.050, 0.180)),
        origin=Origin(xyz=(-0.060, -0.407, 0.390)),
        material=steel,
        name="moving_tray_post",
    )
    bridge.visual(
        Box((0.165, 0.040, 0.035)),
        origin=Origin(xyz=(-0.120, -0.440, 0.492)),
        material=steel,
        name="moving_tray_arm",
    )

    center_truck = model.part("center_truck")
    for z, name, shoe_name in (
        (0.030, "upper_linear_bearing", "upper_linear_bearing_shoe"),
        (-0.035, "lower_linear_bearing", "lower_linear_bearing_shoe"),
    ):
        center_truck.visual(
            Box((0.060, 0.140, 0.045)),
            origin=Origin(xyz=(-0.035, 0.0, z)),
            material=machined,
            name=name,
        )
        center_truck.visual(
            Box((0.006, 0.105, 0.020)),
            origin=Origin(xyz=(-0.067, 0.0, z)),
            material=rail_steel,
            name=shoe_name,
        )
        for y, suffix in ((-0.076, "end_0"), (0.076, "end_1")):
            center_truck.visual(
                Box((0.052, 0.012, 0.045)),
                origin=Origin(xyz=(-0.035, y, z)),
                material=rubber,
                name=f"{name}_wiper_{suffix}",
            )
    center_truck.visual(
        Box((0.050, 0.180, 0.125)),
        origin=Origin(xyz=(0.002, 0.0, -0.002)),
        material=dark_plate,
        name="bearing_spacer",
    )
    center_truck.visual(
        Box((0.050, 0.245, 0.265)),
        origin=Origin(xyz=(0.050, 0.0, 0.000)),
        material=dark_plate,
        name="tool_plate",
    )
    center_truck.visual(
        Box((0.040, 0.165, 0.160)),
        origin=Origin(xyz=(0.0925, 0.0, -0.005)),
        material=machined,
        name="nose_plate",
    )
    for y in (-0.075, 0.075):
        for z in (-0.075, 0.065):
            center_truck.visual(
                Cylinder(radius=0.009, length=0.010),
                origin=Origin(xyz=(0.116, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=fastener,
                name=f"tool_bolt_{y:+.2f}_{z:+.2f}",
            )

    bridge_cover = model.part("bridge_cover")
    bridge_cover.visual(
        Box((0.125, 0.310, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=machined,
        name="cover_panel",
    )
    for y in (-0.115, 0.115):
        bridge_cover.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(0.0, y, 0.015)),
            material=fastener,
            name=f"cover_screw_{y:+.2f}",
        )

    truck_cover = model.part("truck_cover")
    truck_cover.visual(
        Box((0.012, 0.170, 0.095)),
        origin=Origin(xyz=(0.006, 0.0, 0.000)),
        material=machined,
        name="cover_panel",
    )
    for y in (-0.060, 0.060):
        truck_cover.visual(
            Cylinder(radius=0.007, length=0.006),
            origin=Origin(xyz=(0.015, y, 0.032), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=fastener,
            name=f"cover_screw_{y:+.2f}",
        )

    model.articulation(
        "bridge_slide",
        ArticulationType.PRISMATIC,
        parent=rail_frame,
        child=bridge,
        origin=Origin(xyz=(-0.400, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.80, lower=0.0, upper=0.80),
    )
    model.articulation(
        "truck_slide",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=center_truck,
        origin=Origin(xyz=(0.180, -0.220, 0.400)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=260.0, velocity=0.55, lower=0.0, upper=0.44),
    )
    model.articulation(
        "bridge_cover_mount",
        ArticulationType.FIXED,
        parent=bridge,
        child=bridge_cover,
        origin=Origin(xyz=(0.0, 0.0, 0.485)),
    )
    model.articulation(
        "truck_cover_mount",
        ArticulationType.FIXED,
        parent=center_truck,
        child=truck_cover,
        origin=Origin(xyz=(0.1125, 0.0, -0.005)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail_frame = object_model.get_part("rail_frame")
    bridge = object_model.get_part("bridge_carriage")
    center_truck = object_model.get_part("center_truck")
    bridge_cover = object_model.get_part("bridge_cover")
    truck_cover = object_model.get_part("truck_cover")
    bridge_slide = object_model.get_articulation("bridge_slide")
    truck_slide = object_model.get_articulation("truck_slide")

    ctx.expect_gap(
        bridge,
        rail_frame,
        axis="z",
        positive_elem="bearing_0_0_cap",
        negative_elem="rail_0",
        min_gap=0.006,
        max_gap=0.020,
        name="bridge bearing clears rail",
    )
    ctx.expect_overlap(
        bridge,
        rail_frame,
        axes="x",
        elem_a="bearing_0_0_cap",
        elem_b="rail_0",
        min_overlap=0.080,
        name="bridge bearing remains on guide rail",
    )
    ctx.expect_contact(
        bridge,
        rail_frame,
        elem_a="bearing_0_0_shoe",
        elem_b="rail_0",
        contact_tol=0.001,
        name="bridge shoe bears on ground rail",
    )
    ctx.expect_gap(
        center_truck,
        bridge,
        axis="x",
        positive_elem="upper_linear_bearing",
        negative_elem="upper_bridge_rail",
        min_gap=0.003,
        max_gap=0.012,
        name="truck bearing clears bridge rail",
    )
    ctx.expect_overlap(
        center_truck,
        bridge,
        axes="y",
        elem_a="upper_linear_bearing",
        elem_b="upper_bridge_rail",
        min_overlap=0.120,
        name="truck bearing remains on bridge guide",
    )
    ctx.expect_contact(
        center_truck,
        bridge,
        elem_a="upper_linear_bearing_shoe",
        elem_b="upper_bridge_rail",
        contact_tol=0.001,
        name="truck shoe bears on bridge rail",
    )
    ctx.expect_contact(
        bridge_cover,
        bridge,
        elem_a="cover_panel",
        elem_b="bridge_top_cap",
        contact_tol=0.001,
        name="bridge access cover is seated",
    )
    ctx.expect_contact(
        truck_cover,
        center_truck,
        elem_a="cover_panel",
        elem_b="nose_plate",
        contact_tol=0.001,
        name="truck access cover is seated",
    )

    rest_bridge_pos = ctx.part_world_position(bridge)
    with ctx.pose({bridge_slide: 0.80}):
        extended_bridge_pos = ctx.part_world_position(bridge)
        ctx.expect_overlap(
            bridge,
            rail_frame,
            axes="x",
            elem_a="bearing_0_1_cap",
            elem_b="rail_0",
            min_overlap=0.080,
            name="extended bridge stays on rail",
        )

    rest_truck_pos = ctx.part_world_position(center_truck)
    with ctx.pose({truck_slide: 0.44}):
        extended_truck_pos = ctx.part_world_position(center_truck)
        ctx.expect_overlap(
            center_truck,
            bridge,
            axes="y",
            elem_a="upper_linear_bearing",
            elem_b="upper_bridge_rail",
            min_overlap=0.090,
            name="extended truck stays on guide",
        )

    ctx.check(
        "bridge slide travels along rails",
        rest_bridge_pos is not None
        and extended_bridge_pos is not None
        and extended_bridge_pos[0] > rest_bridge_pos[0] + 0.70,
        details=f"rest={rest_bridge_pos}, extended={extended_bridge_pos}",
    )
    ctx.check(
        "center truck travels across bridge",
        rest_truck_pos is not None
        and extended_truck_pos is not None
        and extended_truck_pos[1] > rest_truck_pos[1] + 0.35,
        details=f"rest={rest_truck_pos}, extended={extended_truck_pos}",
    )

    return ctx.report()


object_model = build_object_model()
