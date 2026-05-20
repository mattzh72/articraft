from __future__ import annotations

from math import pi

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_disk(outer_radius: float, inner_radius: float, thickness: float) -> cq.Workplane:
    """A centered annular cylinder with its axis on local +Z."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
        .translate((0.0, 0.0, -thickness / 2.0))
    )


def _drum_geometry() -> cq.Workplane:
    """Open stainless drum: a tubular shell plus three internal wash lifters."""
    outer_radius = 0.245
    inner_radius = 0.210
    length = 0.340
    drum = _annular_disk(outer_radius, inner_radius, length)

    # Three long, raised lifters are merged into the drum shell so the rotating
    # part visibly has features that make its continuous spin legible.
    for angle_deg in (0.0, 120.0, 240.0):
        rib = (
            cq.Workplane("XY")
            .box(0.055, 0.030, length * 0.88)
            .translate((inner_radius - 0.010, 0.0, 0.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        drum = drum.union(rib)

    # A shallow rear lip suggests depth while keeping the front open to view.
    rear_lip = (
        cq.Workplane("XY")
        .circle(outer_radius * 0.96)
        .circle(inner_radius * 0.70)
        .extrude(0.018)
        .translate((0.0, 0.0, -length / 2.0 - 0.009))
    )
    return drum.union(rear_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_load_washer")

    white = model.material("warm_white_enamel", rgba=(0.96, 0.95, 0.91, 1.0))
    shadow = model.material("deep_shadow", rgba=(0.015, 0.016, 0.018, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.62, 0.65, 0.66, 1.0))
    glass = model.material("blue_tinted_glass", rgba=(0.55, 0.86, 1.0, 0.34))
    plastic = model.material("drawer_plastic", rgba=(0.90, 0.91, 0.90, 1.0))
    dark_panel = model.material("smoked_control_panel", rgba=(0.03, 0.04, 0.05, 1.0))

    width = 0.720
    depth = 0.620
    height = 0.950
    wall = 0.035
    front_y = -depth / 2.0
    drum_z = 0.500
    drum_y = -0.020

    body = model.part("body")

    # Cabinet shell made from real panels.  The front is segmented to leave a
    # porthole area and a top-left dispenser slot instead of hiding moving parts
    # inside a solid block.
    body.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, height / 2.0)),
        material=white,
        name="side_panel_0",
    )
    body.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, height / 2.0)),
        material=white,
        name="side_panel_1",
    )
    body.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, height - wall / 2.0)),
        material=white,
        name="top_panel",
    )
    body.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=white,
        name="bottom_panel",
    )
    body.visual(
        Box((width, wall, height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, height / 2.0)),
        material=white,
        name="rear_panel",
    )

    # Front frame pieces around the porthole.
    body.visual(
        Box((width, wall, 0.205)),
        origin=Origin(xyz=(0.0, front_y + wall / 2.0, 0.1025)),
        material=white,
        name="front_lower_band",
    )
    body.visual(
        Box((0.135, wall, 0.540)),
        origin=Origin(xyz=(-0.2925, front_y + wall / 2.0, 0.470)),
        material=white,
        name="front_side_band_0",
    )
    body.visual(
        Box((0.135, wall, 0.540)),
        origin=Origin(xyz=(0.2925, front_y + wall / 2.0, 0.470)),
        material=white,
        name="front_side_band_1",
    )
    body.visual(
        Box((width, wall, 0.060)),
        origin=Origin(xyz=(0.0, front_y + wall / 2.0, 0.735)),
        material=white,
        name="front_mid_band",
    )

    # Top control/dispenser fascia; the missing rectangular area is the drawer
    # pocket on the upper-left front face.
    body.visual(
        Box((width, wall, 0.085)),
        origin=Origin(xyz=(0.0, front_y + wall / 2.0, 0.9075)),
        material=white,
        name="front_top_band",
    )
    body.visual(
        Box((0.420, wall, 0.095)),
        origin=Origin(xyz=(0.130, front_y + wall / 2.0, 0.8275)),
        material=white,
        name="control_fascia",
    )
    body.visual(
        Box((0.040, wall, 0.095)),
        origin=Origin(xyz=(-0.340, front_y + wall / 2.0, 0.8275)),
        material=white,
        name="drawer_slot_jamb",
    )
    body.visual(
        Box((0.185, 0.006, 0.050)),
        origin=Origin(xyz=(0.150, front_y - 0.002, 0.828)),
        material=dark_panel,
        name="display_window",
    )

    # Dark interior seen through the porthole, plus a rubber boot/gasket.
    body.visual(
        Cylinder(radius=0.255, length=0.018),
        origin=Origin(xyz=(0.0, front_y + 0.032, drum_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=shadow,
        name="drum_shadow",
    )
    body.visual(
        mesh_from_cadquery(_annular_disk(0.285, 0.235, 0.020), "porthole_gasket"),
        origin=Origin(xyz=(0.0, front_y - 0.008, drum_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="porthole_gasket",
    )
    body.visual(
        Cylinder(radius=0.055, length=0.130),
        origin=Origin(xyz=(0.0, 0.220, drum_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="rear_bearing",
    )

    # Cabinet-side leaves for the two visible barrel hinges.
    for suffix, z in (("upper", drum_z + 0.220), ("lower", drum_z - 0.220)):
        body.visual(
            Box((0.090, 0.065, 0.080)),
            origin=Origin(xyz=(-0.327, front_y - 0.0325, z)),
            material=stainless,
            name=f"{suffix}_hinge_leaf",
        )

    drum = model.part("drum")
    drum.visual(
        mesh_from_cadquery(_drum_geometry(), "perforated_drum_shell", tolerance=0.0007),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.024, length=0.360),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="axle_shaft",
    )
    for index, angle in enumerate((0.0, pi / 3.0, 2.0 * pi / 3.0)):
        drum.visual(
            Box((0.460, 0.020, 0.025)),
            origin=Origin(xyz=(0.0, 0.130, 0.0), rpy=(0.0, angle, 0.0)),
            material=stainless,
            name=f"spider_arm_{index}",
        )
    model.articulation(
        "body_to_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.0, drum_y, drum_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=8.0),
    )

    hinge_x = -width / 2.0
    door_y = front_y - 0.055
    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_annular_disk(0.300, 0.225, 0.045), "door_outer_ring"),
        origin=Origin(xyz=(-hinge_x, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=white,
        name="door_ring",
    )
    door.visual(
        Cylinder(radius=0.230, length=0.014),
        origin=Origin(xyz=(-hinge_x, -0.018, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="glass",
    )
    door.visual(
        Box((0.040, 0.050, 0.165)),
        origin=Origin(xyz=(-hinge_x + 0.267, -0.040, 0.0)),
        material=stainless,
        name="pull_handle",
    )
    for suffix, z in (("upper", 0.220), ("lower", -0.220)):
        door.visual(
            Cylinder(radius=0.022, length=0.130),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=stainless,
            name=f"{suffix}_hinge_barrel",
        )
        door.visual(
            Box((0.220, 0.035, 0.055)),
            origin=Origin(xyz=(0.110, 0.0, z)),
            material=stainless,
            name=f"{suffix}_hinge_lug",
        )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_x, door_y, drum_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.85),
    )

    dispenser = model.part("dispenser_drawer")
    dispenser.visual(
        Box((0.240, 0.026, 0.080)),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material=white,
        name="drawer_front",
    )
    dispenser.visual(
        Box((0.205, 0.260, 0.012)),
        origin=Origin(xyz=(0.0, 0.130, -0.029)),
        material=plastic,
        name="tray_floor",
    )
    dispenser.visual(
        Box((0.012, 0.250, 0.050)),
        origin=Origin(xyz=(-0.096, 0.130, -0.005)),
        material=plastic,
        name="tray_side_0",
    )
    dispenser.visual(
        Box((0.012, 0.250, 0.050)),
        origin=Origin(xyz=(0.096, 0.130, -0.005)),
        material=plastic,
        name="tray_side_1",
    )
    dispenser.visual(
        Box((0.205, 0.012, 0.050)),
        origin=Origin(xyz=(0.0, 0.254, -0.005)),
        material=plastic,
        name="tray_back_wall",
    )
    dispenser.visual(
        Box((0.008, 0.205, 0.038)),
        origin=Origin(xyz=(-0.030, 0.135, -0.008)),
        material=plastic,
        name="tray_divider",
    )
    model.articulation(
        "body_to_dispenser",
        ArticulationType.PRISMATIC,
        parent=body,
        child=dispenser,
        origin=Origin(xyz=(-0.200, front_y - 0.006, 0.825)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.25, lower=0.0, upper=0.160),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    dispenser = object_model.get_part("dispenser_drawer")
    drum_joint = object_model.get_articulation("body_to_drum")
    door_hinge = object_model.get_articulation("body_to_door")
    drawer_slide = object_model.get_articulation("body_to_dispenser")

    ctx.allow_overlap(
        body,
        drum,
        elem_a="rear_bearing",
        elem_b="axle_shaft",
        reason="The rotating drum axle is intentionally captured inside the rear bearing sleeve.",
    )
    for suffix in ("upper", "lower"):
        ctx.allow_overlap(
            body,
            door,
            elem_a=f"{suffix}_hinge_leaf",
            elem_b=f"{suffix}_hinge_barrel",
            reason="The door hinge barrel is intentionally captured by the cabinet-side hinge leaf.",
        )
        ctx.allow_overlap(
            body,
            door,
            elem_a=f"{suffix}_hinge_leaf",
            elem_b=f"{suffix}_hinge_lug",
            reason="The interleaved hinge leaf/lug is locally embedded to show a retained two-barrel hinge.",
        )

    ctx.check(
        "drum has continuous axle",
        drum_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(drum_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={drum_joint.articulation_type}, axis={drum_joint.axis}",
    )
    ctx.check(
        "door hinge has realistic swing limits",
        door_hinge.motion_limits is not None
        and door_hinge.motion_limits.lower == 0.0
        and door_hinge.motion_limits.upper is not None
        and door_hinge.motion_limits.upper > 1.5,
        details=f"limits={door_hinge.motion_limits}",
    )
    ctx.check(
        "drawer is prismatic and pulls forward",
        drawer_slide.articulation_type == ArticulationType.PRISMATIC and tuple(drawer_slide.axis) == (0.0, -1.0, 0.0),
        details=f"type={drawer_slide.articulation_type}, axis={drawer_slide.axis}",
    )

    with ctx.pose({door_hinge: 0.0, drawer_slide: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="y",
            positive_elem="porthole_gasket",
            negative_elem="door_ring",
            min_gap=0.0,
            max_gap=0.055,
            name="closed door ring clears porthole gasket",
        )
        ctx.expect_overlap(
            drum,
            door,
            axes="xz",
            elem_a="drum_shell",
            elem_b="glass",
            min_overlap=0.30,
            name="transparent window frames drum view",
        )
        ctx.expect_within(
            drum,
            body,
            axes="xz",
            elem_a="axle_shaft",
            elem_b="rear_bearing",
            margin=0.001,
            name="drum axle is centered in bearing",
        )
        ctx.expect_overlap(
            drum,
            body,
            axes="y",
            elem_a="axle_shaft",
            elem_b="rear_bearing",
            min_overlap=0.012,
            name="drum axle remains captured by bearing",
        )
        for suffix in ("upper", "lower"):
            ctx.expect_overlap(
                body,
                door,
                axes="z",
                elem_a=f"{suffix}_hinge_leaf",
                elem_b=f"{suffix}_hinge_barrel",
                min_overlap=0.060,
                name=f"{suffix} barrel is held by hinge leaf",
            )
            ctx.expect_overlap(
                body,
                door,
                axes="z",
                elem_a=f"{suffix}_hinge_leaf",
                elem_b=f"{suffix}_hinge_lug",
                min_overlap=0.040,
                name=f"{suffix} hinge lug stays under leaf",
            )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_ring")
    drawer_front_aabb = ctx.part_element_world_aabb(dispenser, elem="drawer_front")
    ctx.check(
        "dispenser front is upper-left",
        drawer_front_aabb is not None
        and drawer_front_aabb[1][0] < -0.075
        and drawer_front_aabb[0][2] > 0.760,
        details=f"drawer_front={drawer_front_aabb}",
    )
    closed_drawer_pos = ctx.part_world_position(dispenser)
    with ctx.pose({door_hinge: 1.10, drawer_slide: 0.160}):
        opened_door_aabb = ctx.part_element_world_aabb(door, elem="door_ring")
        extended_drawer_pos = ctx.part_world_position(dispenser)
        ctx.expect_within(
            dispenser,
            body,
            axes="xz",
            elem_a="tray_floor",
            margin=0.020,
            name="extended dispenser stays within cabinet slot width",
        )
        ctx.expect_overlap(
            dispenser,
            body,
            axes="y",
            elem_a="tray_floor",
            min_overlap=0.060,
            name="extended dispenser remains guided in slot",
        )

    ctx.check(
        "door opens outward from left hinge",
        closed_door_aabb is not None
        and opened_door_aabb is not None
        and opened_door_aabb[0][1] < closed_door_aabb[0][1] - 0.10,
        details=f"closed={closed_door_aabb}, opened={opened_door_aabb}",
    )
    ctx.check(
        "drawer extends from top-left face",
        closed_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] < closed_drawer_pos[1] - 0.12,
        details=f"closed={closed_drawer_pos}, extended={extended_drawer_pos}",
    )

    return ctx.report()


object_model = build_object_model()
