from __future__ import annotations

import math

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


def _front_panel_mesh():
    """One continuous dryer front with a round loading aperture."""
    width = 0.70
    height = 0.86
    thickness = 0.035
    opening_radius = 0.265
    opening_z = 0.050

    panel = cq.Workplane("XZ").rect(width, height).extrude(thickness)
    cutter = (
        cq.Workplane("XZ")
        .center(0.0, opening_z)
        .circle(opening_radius)
        .extrude(thickness + 0.010)
    )
    return panel.cut(cutter)


def _annular_disc_mesh(outer_radius: float, inner_radius: float, thickness: float):
    """An annular door trim disc in the local XZ plane, thickened along Y."""
    return (
        cq.Workplane("XZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness / 2.0, both=True)
    )


def _drum_shell_mesh():
    """Open stainless drum shell, authored around local Z before rotation."""
    length = 0.450
    outer_radius = 0.240
    inner_radius = 0.215
    outer = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
    )
    cutter = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(length + 0.020)
        .translate((0.0, 0.0, -length / 2.0 - 0.010))
    )
    return outer.cut(cutter)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="condenser_dryer")

    light_grey = model.material("warm_grey_body", rgba=(0.62, 0.64, 0.65, 1.0))
    side_grey = model.material("slightly_darker_grey", rgba=(0.46, 0.48, 0.50, 1.0))
    dark = model.material("dark_recess", rgba=(0.035, 0.038, 0.040, 1.0))
    black_glass = model.material("smoked_glass", rgba=(0.02, 0.035, 0.045, 0.42))
    rubber = model.material("black_rubber_gasket", rgba=(0.015, 0.015, 0.016, 1.0))
    stainless = model.material("brushed_stainless_drum", rgba=(0.74, 0.76, 0.75, 1.0))
    blue_display = model.material("dim_blue_display", rgba=(0.06, 0.15, 0.22, 1.0))

    cabinet = model.part("cabinet")
    # Appliance dimensions: roughly 70 cm wide, 62 cm deep, 86 cm tall.
    cabinet.visual(
        Box((0.037, 0.640, 0.864)),
        origin=Origin(xyz=(-0.3325, -0.015, 0.0)),
        material=side_grey,
        name="side_panel_0",
    )
    cabinet.visual(
        Box((0.037, 0.640, 0.864)),
        origin=Origin(xyz=(0.3325, -0.015, 0.0)),
        material=side_grey,
        name="side_panel_1",
    )
    cabinet.visual(
        Box((0.704, 0.640, 0.044)),
        origin=Origin(xyz=(0.0, -0.015, 0.410)),
        material=light_grey,
        name="top_panel",
    )
    cabinet.visual(
        Box((0.704, 0.640, 0.054)),
        origin=Origin(xyz=(0.0, -0.015, -0.405)),
        material=light_grey,
        name="base_panel",
    )
    cabinet.visual(
        Box((0.700, 0.035, 0.860)),
        origin=Origin(xyz=(0.0, 0.2925, 0.0)),
        material=side_grey,
        name="rear_panel",
    )
    cabinet.visual(
        mesh_from_cadquery(_front_panel_mesh(), "front_panel"),
        origin=Origin(xyz=(0.0, -0.2975, 0.0)),
        material=light_grey,
        name="front_panel",
    )
    cabinet.visual(
        Box((0.600, 0.012, 0.075)),
        origin=Origin(xyz=(0.0, -0.334, 0.382)),
        material=dark,
        name="control_band",
    )
    cabinet.visual(
        Box((0.180, 0.012, 0.038)),
        origin=Origin(xyz=(-0.180, -0.341, 0.382)),
        material=blue_display,
        name="display_window",
    )
    cabinet.visual(
        Box((0.260, 0.012, 0.105)),
        origin=Origin(xyz=(0.0, -0.326, -0.260)),
        material=dark,
        name="condenser_access",
    )
    for i, z in enumerate((-0.292, -0.267, -0.242, -0.217)):
        cabinet.visual(
            Box((0.215, 0.006, 0.006)),
            origin=Origin(xyz=(0.0, -0.333, z)),
            material=light_grey,
            name=f"condenser_slit_{i}",
        )
    cabinet.visual(
        Box((0.035, 0.035, 0.040)),
        origin=Origin(xyz=(-0.270, -0.210, -0.445)),
        material=dark,
        name="foot_0",
    )
    cabinet.visual(
        Box((0.035, 0.035, 0.040)),
        origin=Origin(xyz=(0.270, -0.210, -0.445)),
        material=dark,
        name="foot_1",
    )
    cabinet.visual(
        Box((0.035, 0.035, 0.040)),
        origin=Origin(xyz=(-0.270, 0.210, -0.445)),
        material=dark,
        name="foot_2",
    )
    cabinet.visual(
        Box((0.035, 0.035, 0.040)),
        origin=Origin(xyz=(0.270, 0.210, -0.445)),
        material=dark,
        name="foot_3",
    )

    drum = model.part("drum")
    drum.visual(
        mesh_from_cadquery(_drum_shell_mesh(), "drum_shell"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.032, length=0.060),
        origin=Origin(xyz=(0.0, 0.238, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="rear_hub",
    )
    drum.visual(
        Cylinder(radius=0.225, length=0.018),
        origin=Origin(xyz=(0.0, 0.216, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="rear_drum_wall",
    )
    for i, angle in enumerate((math.pi / 2.0, math.pi * 7.0 / 6.0, math.pi * 11.0 / 6.0)):
        x = 0.205 * math.cos(angle)
        z = 0.205 * math.sin(angle)
        drum.visual(
            Box((0.032, 0.390, 0.050)),
            origin=Origin(xyz=(x, 0.0, z), rpy=(0.0, angle, 0.0)),
            material=stainless,
            name=f"drum_lifter_{i}",
        )

    model.articulation(
        "cabinet_to_drum",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=8.0),
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_annular_disc_mesh(0.285, 0.198, 0.050), "door_ring"),
        origin=Origin(xyz=(-0.285, 0.0, 0.0)),
        material=light_grey,
        name="door_ring",
    )
    door.visual(
        mesh_from_cadquery(_annular_disc_mesh(0.218, 0.198, 0.020), "rubber_seal"),
        origin=Origin(xyz=(-0.285, 0.011, 0.0)),
        material=rubber,
        name="rubber_seal",
    )
    door.visual(
        Cylinder(radius=0.204, length=0.012),
        origin=Origin(xyz=(-0.285, -0.010, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_glass,
        name="window_bowl",
    )
    door.visual(
        Box((0.060, 0.006, 0.130)),
        origin=Origin(xyz=(-0.535, -0.028, 0.0)),
        material=dark,
        name="handle_recess",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.500),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=side_grey,
        name="hinge_barrel",
    )
    cabinet.visual(
        mesh_from_cadquery(_annular_disc_mesh(0.055, 0.030, 0.030), "rear_bearing"),
        origin=Origin(xyz=(0.0, 0.270, 0.050)),
        material=side_grey,
        name="rear_bearing",
    )
    cabinet.visual(
        Box((0.018, 0.040, 0.555)),
        origin=Origin(xyz=(0.315, -0.333, 0.050)),
        material=side_grey,
        name="hinge_leaf",
    )
    for i, z in enumerate((-0.135, 0.235)):
        cabinet.visual(
            Cylinder(radius=0.014, length=0.090),
            origin=Origin(xyz=(0.302, -0.365, z)),
            material=side_grey,
            name=f"hinge_socket_{i}",
        )

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(0.285, -0.365, 0.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    drum_axle = object_model.get_articulation("cabinet_to_drum")
    door_hinge = object_model.get_articulation("cabinet_to_door")

    ctx.allow_overlap(
        cabinet,
        drum,
        elem_a="rear_bearing",
        elem_b="rear_hub",
        reason="The drum axle hub is intentionally captured in the rear bearing bushing.",
    )
    for socket_name in ("hinge_socket_0", "hinge_socket_1"):
        ctx.allow_overlap(
            cabinet,
            door,
            elem_a=socket_name,
            elem_b="hinge_barrel",
            reason="The door hinge barrel is intentionally captured by the cabinet hinge socket.",
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="z",
            elem_a="hinge_barrel",
            elem_b=socket_name,
            min_overlap=0.075,
            name=f"{socket_name} captures the hinge barrel",
        )
    ctx.expect_within(
        drum,
        cabinet,
        axes="xz",
        inner_elem="rear_hub",
        outer_elem="rear_bearing",
        margin=0.002,
        name="rear hub is centered in the bearing bushing",
    )
    ctx.expect_overlap(
        drum,
        cabinet,
        axes="y",
        elem_a="rear_hub",
        elem_b="rear_bearing",
        min_overlap=0.010,
        name="rear hub remains captured by the bearing",
    )
    ctx.expect_within(
        drum,
        cabinet,
        axes="xz",
        inner_elem="drum_shell",
        outer_elem="front_panel",
        margin=0.025,
        name="drum is centered behind the loading aperture",
    )
    ctx.expect_gap(
        cabinet,
        door,
        axis="y",
        positive_elem="front_panel",
        negative_elem="door_ring",
        min_gap=0.004,
        max_gap=0.020,
        name="closed door sits just proud of the front panel",
    )
    ctx.expect_overlap(
        door,
        cabinet,
        axes="xz",
        elem_a="door_ring",
        elem_b="front_panel",
        min_overlap=0.45,
        name="door covers the circular front opening",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="handle_recess")
    with ctx.pose({door_hinge: 1.25}):
        open_aabb = ctx.part_element_world_aabb(door, elem="handle_recess")
    ctx.check(
        "right-hinged door swings outward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.18,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    rest_aabb = ctx.part_element_world_aabb(drum, elem="drum_lifter_0")
    with ctx.pose({drum_axle: math.pi / 2.0}):
        rotated_aabb = ctx.part_element_world_aabb(drum, elem="drum_lifter_0")
    ctx.check(
        "drum rotates about the front-to-back axle",
        rest_aabb is not None
        and rotated_aabb is not None
        and abs(rotated_aabb[1][2] - rest_aabb[1][2]) > 0.12,
        details=f"rest={rest_aabb}, rotated={rotated_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
