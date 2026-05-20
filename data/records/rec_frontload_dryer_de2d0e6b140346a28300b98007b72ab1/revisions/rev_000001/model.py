from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, segments: int = 96) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_laundromat_dryer")

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    black = model.material("matte_black", rgba=(0.015, 0.015, 0.012, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.009, 0.008, 1.0))
    glass = model.material("smoked_glass", rgba=(0.35, 0.55, 0.70, 0.42))
    label_blue = model.material("blue_display", rgba=(0.02, 0.11, 0.20, 1.0))

    body_w = 1.10
    body_d = 0.90
    body_h = 1.55
    wall = 0.035
    center_z = 0.80
    front_y = -body_d / 2.0 - wall / 2.0

    body = model.part("body")
    # Wide stainless cabinet built as a hollow sheet-metal box, so the drum is
    # not hidden inside a solid block.  Panels intentionally overlap at folded
    # seams, matching a heavy commercial appliance cabinet.
    body.visual(
        Box((wall, body_d, body_h)),
        origin=Origin(xyz=(-body_w / 2.0 + wall / 2.0, 0.0, body_h / 2.0)),
        material=stainless,
        name="side_panel_0",
    )
    body.visual(
        Box((wall, body_d, body_h)),
        origin=Origin(xyz=(body_w / 2.0 - wall / 2.0, 0.0, body_h / 2.0)),
        material=stainless,
        name="side_panel_1",
    )
    body.visual(
        Box((body_w, body_d, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=stainless,
        name="bottom_panel",
    )
    body.visual(
        Box((body_w, body_d, wall)),
        origin=Origin(xyz=(0.0, 0.0, body_h - wall / 2.0)),
        material=stainless,
        name="top_panel",
    )
    body.visual(
        Box((body_w, wall, body_h)),
        origin=Origin(xyz=(0.0, body_d / 2.0 - wall / 2.0, body_h / 2.0)),
        material=stainless,
        name="rear_panel",
    )

    front_panel_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(body_w, body_h, 0.035, corner_segments=10),
        [_circle_profile(0.365, 128)],
        wall,
        center=True,
    )
    body.visual(
        mesh_from_geometry(front_panel_geom, "front_panel"),
        origin=Origin(xyz=(0.0, front_y, center_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="front_panel",
    )
    body.visual(
        Box((0.72, 0.018, 0.18)),
        origin=Origin(xyz=(0.04, front_y - 0.026, 1.34)),
        material=black,
        name="control_panel",
    )
    body.visual(
        Box((0.16, 0.020, 0.075)),
        origin=Origin(xyz=(-0.28, front_y - 0.038, 1.35)),
        material=dark_steel,
        name="coin_receiver",
    )
    body.visual(
        Box((0.18, 0.010, 0.045)),
        origin=Origin(xyz=(0.00, front_y - 0.038, 1.375)),
        material=label_blue,
        name="display_window",
    )
    body.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(0.42, 0.18, 0.018, corner_segments=6),
                [],
                0.006,
                center=True,
            ),
            "lower_service_plate",
        ),
        origin=Origin(xyz=(0.0, front_y - 0.0205, 0.22), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="lower_service_plate",
    )
    body.visual(
        Cylinder(radius=0.055, length=0.040),
        origin=Origin(xyz=(0.0, body_d / 2.0 - wall - 0.020, center_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_bearing",
    )

    hinge_x = -0.46
    hinge_y = front_y - 0.055
    hinge_zs = (center_z - 0.27, center_z + 0.27)
    for i, hz in enumerate(hinge_zs):
        body.visual(
            Box((0.13, 0.035, 0.12)),
            origin=Origin(xyz=(hinge_x - 0.055, front_y - 0.027, hz)),
            material=dark_steel,
            name=f"hinge_mount_{i}",
        )
        body.visual(
            Cylinder(radius=0.022, length=0.14),
            origin=Origin(xyz=(hinge_x, hinge_y, hz)),
            material=dark_steel,
            name=f"hinge_barrel_{i}",
        )

    drum = model.part("drum")
    drum.visual(
        mesh_from_geometry(CylinderGeometry(0.335, 0.64, radial_segments=96, closed=False), "drum_shell"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="drum_shell",
    )
    drum.visual(
        mesh_from_geometry(TorusGeometry(0.335, 0.018, radial_segments=24, tubular_segments=96), "drum_front_rim"),
        origin=Origin(xyz=(0.0, -0.32, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="front_rim",
    )
    drum.visual(
        mesh_from_geometry(TorusGeometry(0.335, 0.018, radial_segments=24, tubular_segments=96), "drum_rear_rim"),
        origin=Origin(xyz=(0.0, 0.32, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="rear_rim",
    )
    drum.visual(
        Cylinder(radius=0.338, length=0.020),
        origin=Origin(xyz=(0.0, 0.31, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_spider_plate",
    )
    drum.visual(
        Cylinder(radius=0.025, length=0.355),
        origin=Origin(xyz=(0.0, 0.1775, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="drive_axle",
    )
    for i, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        # Raised tumbling ribs on the rotating drum interior.
        x = 0.315 * math.cos(angle)
        z = 0.315 * math.sin(angle)
        drum.visual(
            Box((0.055, 0.54, 0.026)),
            origin=Origin(xyz=(x, 0.0, z), rpy=(0.0, -angle, 0.0)),
            material=stainless,
            name=f"tumble_rib_{i}",
        )

    model.articulation(
        "body_to_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.0, 0.02, center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=240.0, velocity=7.0),
    )

    door = model.part("door")
    door.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(_circle_profile(0.405, 128), [_circle_profile(0.292, 128)], 0.055, center=True),
            "door_metal_ring",
        ),
        origin=Origin(xyz=(0.46, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="metal_ring",
    )
    door.visual(
        Cylinder(radius=0.278, length=0.014),
        origin=Origin(xyz=(0.46, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="glass_porthole",
    )
    door.visual(
        mesh_from_geometry(TorusGeometry(0.292, 0.024, radial_segments=24, tubular_segments=128), "rubber_gasket"),
        origin=Origin(xyz=(0.46, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="rubber_gasket",
    )
    for i, hz in enumerate((-0.27, 0.27)):
        door.visual(
            Box((0.246, 0.052, 0.085)),
            origin=Origin(xyz=(0.145, 0.0, hz)),
            material=dark_steel,
            name=f"hinge_leaf_{i}",
        )
    door.visual(
        Cylinder(radius=0.016, length=0.095),
        origin=Origin(xyz=(0.81, -0.068, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="handle_stem",
    )
    door.visual(
        Cylinder(radius=0.018, length=0.20),
        origin=Origin(xyz=(0.81, -0.125, 0.0)),
        material=dark_steel,
        name="handle_bar",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, center_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.4, lower=0.0, upper=1.75),
    )

    cycle_dial = model.part("cycle_dial")
    cycle_dial.visual(
        Cylinder(radius=0.045, length=0.030),
        origin=Origin(xyz=(0.0, -0.017, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="dial_cap",
    )
    model.articulation(
        "body_to_cycle_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cycle_dial,
        origin=Origin(xyz=(0.27, front_y - 0.033, 1.35)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=5.0),
    )

    start_button = model.part("start_button")
    start_button.visual(
        Cylinder(radius=0.025, length=0.020),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="button_cap",
    )
    model.articulation(
        "body_to_start_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=start_button,
        origin=Origin(xyz=(0.40, front_y - 0.033, 1.35)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.08, lower=0.0, upper=0.012),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    button = object_model.get_part("start_button")
    door_hinge = object_model.get_articulation("body_to_door")
    button_slide = object_model.get_articulation("body_to_start_button")

    ctx.expect_gap(
        body,
        door,
        axis="y",
        positive_elem="front_panel",
        negative_elem="metal_ring",
        min_gap=0.005,
        max_gap=0.060,
        name="closed door sits proud of front panel",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        elem_a="metal_ring",
        elem_b="front_panel",
        min_overlap=0.70,
        name="round door covers the drum opening",
    )
    ctx.expect_within(
        drum,
        body,
        axes="xz",
        inner_elem="drum_shell",
        outer_elem="front_panel",
        margin=0.02,
        name="drum sits inside the cabinet opening envelope",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="metal_ring")
    with ctx.pose({door_hinge: 1.20}):
        opened_aabb = ctx.part_element_world_aabb(door, elem="metal_ring")
    ctx.check(
        "door hinge opens outward",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[0][1] < closed_aabb[0][1] - 0.25,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    rest_button = ctx.part_world_position(button)
    with ctx.pose({button_slide: 0.010}):
        pushed_button = ctx.part_world_position(button)
    ctx.check(
        "start button pushes inward",
        rest_button is not None
        and pushed_button is not None
        and pushed_button[1] > rest_button[1] + 0.008,
        details=f"rest={rest_button}, pushed={pushed_button}",
    )

    return ctx.report()


object_model = build_object_model()
