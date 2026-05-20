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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_watch_winder")

    powder = model.material("olive_powder_coat", rgba=(0.18, 0.24, 0.17, 1.0))
    black = model.material("matte_black_rubber", rgba=(0.01, 0.012, 0.011, 1.0))
    foam = model.material("charcoal_tool_foam", rgba=(0.025, 0.027, 0.030, 1.0))
    steel = model.material("brushed_stainless_steel", rgba=(0.63, 0.65, 0.62, 1.0))
    gunmetal = model.material("dark_gunmetal", rgba=(0.10, 0.11, 0.12, 1.0))
    uhmw = model.material("replaceable_uhmw_wear_strip", rgba=(0.76, 0.77, 0.70, 1.0))
    brass = model.material("oil_bronze_bushing", rgba=(0.78, 0.54, 0.24, 1.0))
    leather = model.material("tan_watch_cushion", rgba=(0.55, 0.34, 0.18, 1.0))
    glass = model.material("smoked_polycarbonate_window", rgba=(0.18, 0.23, 0.27, 0.38))
    label = model.material("service_yellow_label", rgba=(0.95, 0.72, 0.12, 1.0))

    case = model.part("case")

    # Chunky welded tray: real hollow box proportions, not a solid block.
    case.visual(
        Box((0.54, 0.34, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=powder,
        name="bottom_plate",
    )
    case.visual(
        Box((0.030, 0.34, 0.195)),
        origin=Origin(xyz=(-0.255, 0.0, 0.1225)),
        material=powder,
        name="rear_wall",
    )
    case.visual(
        Box((0.030, 0.34, 0.195)),
        origin=Origin(xyz=(0.255, 0.0, 0.1225)),
        material=powder,
        name="front_wall",
    )
    case.visual(
        Box((0.54, 0.025, 0.195)),
        origin=Origin(xyz=(0.0, 0.1575, 0.1225)),
        material=powder,
        name="side_wall_0",
    )
    case.visual(
        Box((0.54, 0.025, 0.195)),
        origin=Origin(xyz=(0.0, -0.1575, 0.1225)),
        material=powder,
        name="side_wall_1",
    )

    # Replaceable wear strips define the lid seating plane and protect the case lip.
    case.visual(
        Box((0.030, 0.34, 0.005)),
        origin=Origin(xyz=(-0.255, 0.0, 0.2225)),
        material=uhmw,
        name="rear_wear_strip",
    )
    case.visual(
        Box((0.030, 0.34, 0.005)),
        origin=Origin(xyz=(0.255, 0.0, 0.2225)),
        material=uhmw,
        name="front_wear_strip",
    )
    case.visual(
        Box((0.54, 0.025, 0.005)),
        origin=Origin(xyz=(0.0, 0.1575, 0.2225)),
        material=uhmw,
        name="side_wear_strip_0",
    )
    case.visual(
        Box((0.54, 0.025, 0.005)),
        origin=Origin(xyz=(0.0, -0.1575, 0.2225)),
        material=uhmw,
        name="side_wear_strip_1",
    )

    # Interior service rails and foam are bolted down to the bottom plate.
    case.visual(
        Box((0.42, 0.035, 0.012)),
        origin=Origin(xyz=(0.015, 0.103, 0.031)),
        material=steel,
        name="service_rail_0",
    )
    case.visual(
        Box((0.42, 0.035, 0.012)),
        origin=Origin(xyz=(0.015, -0.103, 0.031)),
        material=steel,
        name="service_rail_1",
    )
    case.visual(
        Box((0.34, 0.18, 0.018)),
        origin=Origin(xyz=(0.030, 0.0, 0.034)),
        material=foam,
        name="foam_insert",
    )
    case.visual(
        Box((0.16, 0.020, 0.004)),
        origin=Origin(xyz=(0.150, 0.0, 0.045)),
        material=label,
        name="service_label",
    )

    # Front welded grab handle with real standoff blocks.
    case.visual(
        Cylinder(radius=0.012, length=0.155),
        origin=Origin(xyz=(0.294, 0.0, 0.115), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="front_grab_bar",
    )
    for idx, y in enumerate((-0.080, 0.080)):
        case.visual(
            Box((0.026, 0.020, 0.040)),
            origin=Origin(xyz=(0.282, y, 0.115)),
            material=gunmetal,
            name=f"handle_standoff_{idx}",
        )

    # Lid hinge: case-side knuckles sit on two welded support points.
    for idx, y in enumerate((-0.129, 0.129)):
        case.visual(
            Box((0.012, 0.068, 0.036)),
            origin=Origin(xyz=(-0.276, y, 0.238)),
            material=gunmetal,
            name=f"hinge_support_{idx}",
        )
        case.visual(
            Cylinder(radius=0.011, length=0.065),
            origin=Origin(
                xyz=(-0.285, y, 0.245),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=steel,
            name=f"case_hinge_knuckle_{idx}",
        )
        case.visual(
            Box((0.012, 0.046, 0.018)),
            origin=Origin(xyz=(-0.276, y, 0.214)),
            material=gunmetal,
            name=f"hinge_gusset_{idx}",
        )

    # Bearing supports for the rotating watch cradle.  The shaft is clearanced
    # through bronze torus bushings instead of passing through a solid proxy.
    bearing_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.024, tube=0.006, radial_segments=18, tubular_segments=36),
        "bronze_bearing_ring",
    )
    for idx, (bearing_name, x) in enumerate((("rear_bearing", -0.105), ("front_bearing", 0.105))):
        case.visual(
            Box((0.040, 0.082, 0.070)),
            origin=Origin(xyz=(x, 0.0, 0.060)),
            material=gunmetal,
            name=f"bearing_pedestal_{idx}",
        )
        case.visual(
            bearing_mesh,
            origin=Origin(xyz=(x, 0.0, 0.125), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brass,
            name=bearing_name,
        )
        case.visual(
            Box((0.046, 0.090, 0.008)),
            origin=Origin(xyz=(x, 0.0, 0.095)),
            material=steel,
            name=f"bearing_cap_plate_{idx}",
        )

    # Rugged feet are bonded into the bottom plate at the four corners.
    for ix, x in enumerate((-0.205, 0.205)):
        for iy, y in enumerate((-0.120, 0.120)):
            case.visual(
                Box((0.070, 0.050, 0.020)),
                origin=Origin(xyz=(x, y, -0.010)),
                material=black,
                name=f"rubber_foot_{ix}_{iy}",
            )

    lid = model.part("lid")
    # The lid part frame is the hinge pin line.  Closed geometry extends in +X.
    lid.visual(
        Box((0.035, 0.340, 0.035)),
        origin=Origin(xyz=(0.0325, 0.0, -0.0025)),
        material=powder,
        name="rear_rail",
    )
    lid.visual(
        Box((0.035, 0.340, 0.035)),
        origin=Origin(xyz=(0.5375, 0.0, -0.0025)),
        material=powder,
        name="front_rail",
    )
    lid.visual(
        Box((0.540, 0.035, 0.035)),
        origin=Origin(xyz=(0.285, 0.1525, -0.0025)),
        material=powder,
        name="side_rail_0",
    )
    lid.visual(
        Box((0.540, 0.035, 0.035)),
        origin=Origin(xyz=(0.285, -0.1525, -0.0025)),
        material=powder,
        name="side_rail_1",
    )
    lid.visual(
        Box((0.470, 0.270, 0.006)),
        origin=Origin(xyz=(0.285, 0.0, 0.018)),
        material=glass,
        name="inspection_window",
    )
    lid.visual(
        Box((0.410, 0.020, 0.012)),
        origin=Origin(xyz=(0.285, 0.0, 0.028)),
        material=gunmetal,
        name="window_crossbar",
    )
    lid.visual(
        Cylinder(radius=0.011, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="lid_hinge_knuckle",
    )
    lid.visual(
        Box((0.030, 0.180, 0.010)),
        origin=Origin(xyz=(0.012, 0.0, -0.004)),
        material=gunmetal,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Box((0.115, 0.055, 0.014)),
        origin=Origin(xyz=(0.520, 0.0, 0.025)),
        material=gunmetal,
        name="front_latch_plate",
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.011, length=0.250),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="shaft",
    )
    cradle.visual(
        Cylinder(radius=0.084, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="cradle_disk",
    )
    cradle.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(-0.078, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="rear_stop_collar",
    )
    cradle.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.078, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="front_stop_collar",
    )
    cradle.visual(
        Box((0.040, 0.112, 0.070)),
        origin=Origin(xyz=(0.041, 0.0, 0.0)),
        material=leather,
        name="watch_cushion",
    )
    cradle.visual(
        Box((0.008, 0.150, 0.018)),
        origin=Origin(xyz=(0.058, 0.0, 0.040)),
        material=black,
        name="elastic_strap",
    )
    cradle.visual(
        Box((0.010, 0.048, 0.018)),
        origin=Origin(xyz=(0.060, 0.074, 0.040)),
        material=steel,
        name="strap_buckle_0",
    )
    cradle.visual(
        Box((0.010, 0.048, 0.018)),
        origin=Origin(xyz=(0.060, -0.074, 0.040)),
        material=steel,
        name="strap_buckle_1",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lid,
        origin=Origin(xyz=(-0.285, 0.0, 0.245)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.35),
    )
    model.articulation(
        "cradle_spin",
        ArticulationType.CONTINUOUS,
        parent=case,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=1.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_hinge = object_model.get_articulation("lid_hinge")
    cradle_spin = object_model.get_articulation("cradle_spin")

    ctx.check(
        "primary mechanisms are articulated",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and cradle_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"lid={lid_hinge.articulation_type}, cradle={cradle_spin.articulation_type}",
    )
    ctx.expect_gap(
        lid,
        case,
        axis="z",
        positive_elem="front_rail",
        negative_elem="front_wear_strip",
        max_gap=0.001,
        max_penetration=0.0001,
        name="closed lid seats on replaceable front wear strip",
    )
    ctx.expect_overlap(
        lid,
        case,
        axes="xy",
        elem_a="front_rail",
        elem_b="front_wear_strip",
        min_overlap=0.020,
        name="front rail bears on the case lip",
    )
    ctx.expect_within(
        cradle,
        case,
        axes="yz",
        inner_elem="shaft",
        outer_elem="front_bearing",
        margin=0.001,
        name="shaft is centered inside the front bearing envelope",
    )
    ctx.expect_within(
        cradle,
        case,
        axes="yz",
        inner_elem="shaft",
        outer_elem="rear_bearing",
        margin=0.001,
        name="shaft is centered inside the rear bearing envelope",
    )
    ctx.expect_overlap(
        cradle,
        case,
        axes="x",
        elem_a="shaft",
        elem_b="front_bearing",
        min_overlap=0.004,
        name="shaft passes through the front support point",
    )
    ctx.expect_overlap(
        cradle,
        case,
        axes="x",
        elem_a="shaft",
        elem_b="rear_bearing",
        min_overlap=0.004,
        name="shaft passes through the rear support point",
    )

    closed_front = ctx.part_element_world_aabb(lid, elem="front_rail")
    with ctx.pose({lid_hinge: 1.0}):
        open_front = ctx.part_element_world_aabb(lid, elem="front_rail")
    ctx.check(
        "lid hinge lifts the front rail",
        closed_front is not None
        and open_front is not None
        and open_front[0][2] > closed_front[0][2] + 0.10,
        details=f"closed={closed_front}, open={open_front}",
    )

    rest_cushion = ctx.part_element_world_aabb(cradle, elem="watch_cushion")
    with ctx.pose({cradle_spin: math.pi / 2.0}):
        spun_cushion = ctx.part_element_world_aabb(cradle, elem="watch_cushion")
    ctx.check(
        "cradle spin rotates the asymmetric watch cushion",
        rest_cushion is not None
        and spun_cushion is not None
        and (spun_cushion[1][2] - spun_cushion[0][2])
        > (rest_cushion[1][2] - rest_cushion[0][2]) + 0.025,
        details=f"rest={rest_cushion}, spun={spun_cushion}",
    )

    return ctx.report()


object_model = build_object_model()
