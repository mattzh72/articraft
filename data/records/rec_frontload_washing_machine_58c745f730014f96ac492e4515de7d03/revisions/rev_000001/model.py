from __future__ import annotations

import math

import cadquery as cq
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
    mesh_from_cadquery,
)


def _annular_y_mesh(
    outer_radius: float,
    inner_radius: float,
    depth: float,
    center: tuple[float, float, float],
):
    """A tube/ring whose axis is local/world Y, authored in meters."""
    cx, cy, cz = center
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(depth)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((cx, cy + depth / 2.0, cz))
    )


def _disk_y_mesh(radius: float, depth: float, center: tuple[float, float, float]):
    """A solid disk whose axis is local/world Y, authored in meters."""
    cx, cy, cz = center
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(depth)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((cx, cy + depth / 2.0, cz))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_load_washer")

    white = model.material("warm_white_enamel", rgba=(0.92, 0.93, 0.91, 1.0))
    dark = model.material("dark_recess", rgba=(0.02, 0.022, 0.025, 1.0))
    rubber = model.material("charcoal_rubber", rgba=(0.03, 0.032, 0.035, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.63, 0.66, 0.67, 1.0))
    glass = model.material("blue_tinted_glass", rgba=(0.50, 0.78, 0.95, 0.38))
    plastic = model.material("satin_silver_plastic", rgba=(0.74, 0.74, 0.72, 1.0))
    black = model.material("gloss_black", rgba=(0.0, 0.0, 0.0, 1.0))

    # Overall residential appliance proportions: about 65 cm wide/deep and
    # 85 cm tall, with a hollow central bay for the drum rather than a solid box.
    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.035, 0.650, 0.850)),
        origin=Origin(xyz=(-0.3075, 0.0, 0.425)),
        material=white,
        name="side_panel_0",
    )
    cabinet.visual(
        Box((0.035, 0.650, 0.850)),
        origin=Origin(xyz=(0.3075, 0.0, 0.425)),
        material=white,
        name="side_panel_1",
    )
    cabinet.visual(
        Box((0.650, 0.650, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.8325)),
        material=white,
        name="top_panel",
    )
    cabinet.visual(
        Box((0.650, 0.650, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=white,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((0.650, 0.035, 0.850)),
        origin=Origin(xyz=(0.0, 0.3075, 0.425)),
        material=white,
        name="back_panel",
    )
    cabinet.visual(
        Box((0.650, 0.035, 0.170)),
        origin=Origin(xyz=(0.0, -0.3075, 0.765)),
        material=white,
        name="control_panel",
    )
    cabinet.visual(
        Box((0.650, 0.035, 0.170)),
        origin=Origin(xyz=(0.0, -0.3075, 0.085)),
        material=white,
        name="front_toe_panel",
    )
    cabinet.visual(
        Box((0.125, 0.035, 0.510)),
        origin=Origin(xyz=(-0.2625, -0.3075, 0.425)),
        material=white,
        name="front_stile_0",
    )
    cabinet.visual(
        Box((0.125, 0.035, 0.510)),
        origin=Origin(xyz=(0.2625, -0.3075, 0.425)),
        material=white,
        name="front_stile_1",
    )
    cabinet.visual(
        mesh_from_cadquery(
            _annular_y_mesh(0.280, 0.218, 0.028, (0.0, -0.335, 0.435)),
            "front_opening_ring",
            tolerance=0.0008,
            angular_tolerance=0.05,
        ),
        material=white,
        name="front_opening_ring",
    )
    cabinet.visual(
        mesh_from_cadquery(
            _annular_y_mesh(0.232, 0.190, 0.030, (0.0, -0.350, 0.435)),
            "gasket_ring",
            tolerance=0.0008,
            angular_tolerance=0.05,
        ),
        material=rubber,
        name="gasket_ring",
    )
    cabinet.visual(
        Box((0.210, 0.006, 0.052)),
        origin=Origin(xyz=(-0.135, -0.328, 0.765)),
        material=black,
        name="display_window",
    )
    cabinet.visual(
        Box((0.145, 0.008, 0.060)),
        origin=Origin(xyz=(-0.240, -0.329, 0.765)),
        material=white,
        name="detergent_drawer_face",
    )
    cabinet.visual(
        Box((0.024, 0.055, 0.150)),
        origin=Origin(xyz=(-0.314, -0.3525, 0.535)),
        material=plastic,
        name="hinge_mount_0",
    )
    cabinet.visual(
        Box((0.024, 0.055, 0.150)),
        origin=Origin(xyz=(-0.314, -0.3525, 0.335)),
        material=plastic,
        name="hinge_mount_1",
    )

    drum = model.part("drum")
    drum.visual(
        mesh_from_cadquery(
            _annular_y_mesh(0.205, 0.184, 0.420, (0.0, 0.0, 0.0)),
            "drum_shell",
            tolerance=0.0008,
            angular_tolerance=0.05,
        ),
        material=stainless,
        name="drum_shell",
    )
    drum.visual(
        mesh_from_cadquery(
            _annular_y_mesh(0.218, 0.178, 0.030, (0.0, -0.225, 0.0)),
            "drum_front_lip",
            tolerance=0.0008,
            angular_tolerance=0.05,
        ),
        material=stainless,
        name="drum_front_lip",
    )
    drum.visual(
        mesh_from_cadquery(
            _disk_y_mesh(0.060, 0.035, (0.0, -0.245, 0.0)),
            "axle_hub",
            tolerance=0.0008,
            angular_tolerance=0.05,
        ),
        material=plastic,
        name="axle_hub",
    )
    drum.visual(
        Cylinder(radius=0.022, length=0.520),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="central_axle",
    )
    drum.visual(
        Cylinder(radius=0.030, length=0.110),
        origin=Origin(xyz=(0.0, 0.265, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="rear_axle_stub",
    )
    for i, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        drum.visual(
            Box((0.170, 0.016, 0.016)),
            origin=Origin(
                xyz=(math.cos(angle) * 0.105, -0.245, math.sin(angle) * 0.105),
                rpy=(0.0, -angle, 0.0),
            ),
            material=stainless,
            name=f"hub_spoke_{i}",
        )
    for i, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        radial = 0.178
        drum.visual(
            Box((0.042, 0.320, 0.026)),
            origin=Origin(
                xyz=(math.sin(angle) * radial, -0.005, math.cos(angle) * radial),
                rpy=(0.0, angle, 0.0),
            ),
            material=stainless,
            name=f"baffle_{i}",
        )

    model.articulation(
        "cabinet_to_drum",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0.0, -0.030, 0.435)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=12.0),
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(
            _annular_y_mesh(0.265, 0.182, 0.055, (0.310, 0.0, 0.0)),
            "door_frame",
            tolerance=0.0008,
            angular_tolerance=0.05,
        ),
        material=plastic,
        name="door_frame",
    )
    door.visual(
        mesh_from_cadquery(
            _disk_y_mesh(0.188, 0.014, (0.310, -0.002, 0.0)),
            "glass_bowl",
            tolerance=0.0008,
            angular_tolerance=0.05,
        ),
        material=glass,
        name="glass_bowl",
    )
    door.visual(
        Cylinder(radius=0.018, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=plastic,
        name="hinge_barrel_0",
    )
    door.visual(
        Cylinder(radius=0.018, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, -0.115)),
        material=plastic,
        name="hinge_barrel_1",
    )
    door.visual(
        Box((0.095, 0.022, 0.046)),
        origin=Origin(xyz=(0.047, 0.0, 0.115)),
        material=plastic,
        name="hinge_leaf_0",
    )
    door.visual(
        Box((0.095, 0.022, 0.046)),
        origin=Origin(xyz=(0.047, 0.0, -0.115)),
        material=plastic,
        name="hinge_leaf_1",
    )
    door.visual(
        Box((0.038, 0.045, 0.125)),
        origin=Origin(xyz=(0.545, -0.037, 0.0)),
        material=plastic,
        name="pull_handle",
    )

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-0.310, -0.398, 0.435)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=0.0, upper=1.85),
    )

    knob = model.part("cycle_knob")
    knob.visual(
        Cylinder(radius=0.038, length=0.035),
        origin=Origin(xyz=(0.0, -0.0175, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=plastic,
        name="knob_cap",
    )
    knob.visual(
        Box((0.006, 0.004, 0.026)),
        origin=Origin(xyz=(0.0, -0.0365, 0.020)),
        material=black,
        name="pointer_mark",
    )
    model.articulation(
        "cabinet_to_cycle_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=knob,
        origin=Origin(xyz=(0.205, -0.325, 0.765)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0),
    )

    start_button = model.part("start_button")
    start_button.visual(
        Box((0.048, 0.012, 0.024)),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material=plastic,
        name="button_cap",
    )
    model.articulation(
        "cabinet_to_start_button",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=start_button,
        origin=Origin(xyz=(0.275, -0.325, 0.765)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.08, lower=0.0, upper=0.006),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    knob = object_model.get_part("cycle_knob")
    button = object_model.get_part("start_button")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    drum_axle = object_model.get_articulation("cabinet_to_drum")

    ctx.expect_gap(
        cabinet,
        door,
        axis="y",
        min_gap=0.004,
        max_gap=0.040,
        positive_elem="gasket_ring",
        negative_elem="door_frame",
        name="closed door sits just proud of gasket",
    )
    ctx.expect_overlap(
        door,
        cabinet,
        axes="xz",
        min_overlap=0.34,
        elem_a="door_frame",
        elem_b="gasket_ring",
        name="porthole door covers the rubber gasket",
    )
    ctx.expect_gap(
        drum,
        cabinet,
        axis="y",
        min_gap=0.035,
        max_gap=0.100,
        positive_elem="drum_front_lip",
        negative_elem="gasket_ring",
        name="drum lip is recessed behind front gasket",
    )
    ctx.expect_gap(
        cabinet,
        knob,
        axis="y",
        min_gap=0.0,
        max_gap=0.006,
        positive_elem="control_panel",
        negative_elem="knob_cap",
        name="cycle knob is mounted proud of the panel",
    )
    ctx.expect_gap(
        cabinet,
        button,
        axis="y",
        min_gap=0.0,
        max_gap=0.006,
        positive_elem="control_panel",
        negative_elem="button_cap",
        name="start button is flush and pressable",
    )

    ctx.check(
        "door hinge is on left vertical edge",
        tuple(round(v, 3) for v in door_hinge.axis) == (0.0, 0.0, -1.0),
        details=f"axis={door_hinge.axis}",
    )
    ctx.check(
        "drum axle runs front to back",
        tuple(round(v, 3) for v in drum_axle.axis) == (0.0, 1.0, 0.0),
        details=f"axis={drum_axle.axis}",
    )

    rest_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.25}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door opens outward from the left hinge",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < rest_aabb[0][1] - 0.12,
        details=f"rest={rest_aabb}, open={open_aabb}",
    )

    def _aabb_center_xz(aabb):
        return ((aabb[0][0] + aabb[1][0]) / 2.0, (aabb[0][2] + aabb[1][2]) / 2.0)

    baffle_rest = ctx.part_element_world_aabb(drum, elem="baffle_0")
    with ctx.pose({drum_axle: 1.0}):
        baffle_spun = ctx.part_element_world_aabb(drum, elem="baffle_0")
    if baffle_rest is not None and baffle_spun is not None:
        rest_xz = _aabb_center_xz(baffle_rest)
        spun_xz = _aabb_center_xz(baffle_spun)
        moved = math.hypot(spun_xz[0] - rest_xz[0], spun_xz[1] - rest_xz[1])
    else:
        moved = 0.0
    ctx.check(
        "drum baffle moves when axle rotates",
        moved > 0.10,
        details=f"baffle displacement around axle={moved:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
