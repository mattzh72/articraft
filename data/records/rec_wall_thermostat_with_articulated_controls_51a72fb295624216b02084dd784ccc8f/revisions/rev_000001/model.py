from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _front_plate_mesh(width: float, height: float, depth: float, radius: float, name: str):
    # Mesh extrusion is along local Z; rotate so depth becomes the thermostat's
    # front/back X direction while the rounded rectangle lies on the YZ face.
    profile = rounded_rect_profile(height, width, radius, corner_segments=10)
    return mesh_from_geometry(
        ExtrudeGeometry(profile, depth, center=True).rotate_y(math.pi / 2.0),
        name,
    )


def _annular_dial_mesh(outer_radius: float, inner_radius: float, depth: float, name: str):
    # CadQuery gives the dial a true through-bore so the fixed shaft can pass
    # through the rotating part without any collision allowance.
    ring = cq.Workplane("YZ").circle(outer_radius).circle(inner_radius).extrude(depth)
    return mesh_from_cadquery(ring, name, tolerance=0.00035, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_dial_thermostat")

    shell = model.material("warm_white_shell", rgba=(0.92, 0.90, 0.84, 1.0))
    face = model.material("satin_face", rgba=(0.82, 0.82, 0.78, 1.0))
    dark = model.material("display_black", rgba=(0.03, 0.04, 0.045, 1.0))
    glass = model.material("smoked_glass", rgba=(0.08, 0.18, 0.21, 0.55))
    dial_mat = model.material("graphite_dial", rgba=(0.12, 0.13, 0.13, 1.0))
    dial_edge = model.material("soft_rubber_grip", rgba=(0.05, 0.055, 0.055, 1.0))
    print_white = model.material("printed_white", rgba=(0.96, 0.95, 0.88, 1.0))
    amber = model.material("warm_indicator", rgba=(1.00, 0.58, 0.16, 1.0))
    metal = model.material("brushed_retainer", rgba=(0.68, 0.70, 0.70, 1.0))
    stand_mat = model.material("folding_stand_grey", rgba=(0.34, 0.35, 0.34, 1.0))

    housing = model.part("housing")
    body_depth = 0.024
    front_x = body_depth * 0.5
    rear_x = -front_x
    dial_center_z = -0.012

    housing.visual(
        _front_plate_mesh(0.132, 0.096, body_depth, 0.016, "thermostat_rounded_body"),
        material=shell,
        name="body_shell",
    )
    housing.visual(
        _front_plate_mesh(0.118, 0.082, 0.0018, 0.012, "thermostat_front_face"),
        origin=Origin(xyz=(front_x + 0.0002, 0.0, 0.0)),
        material=face,
        name="faceplate",
    )

    # Compact display window and two printed temperature rows above the dial.
    housing.visual(
        Box((0.0012, 0.063, 0.017)),
        origin=Origin(xyz=(front_x + 0.0014, 0.0, 0.035)),
        material=dark,
        name="display_recess",
    )
    housing.visual(
        Box((0.0010, 0.055, 0.011)),
        origin=Origin(xyz=(front_x + 0.0021, 0.0, 0.035)),
        material=glass,
        name="display_lens",
    )
    for i, y in enumerate((-0.021, -0.007, 0.007, 0.021)):
        housing.visual(
            Box((0.0008, 0.008, 0.0014)),
            origin=Origin(xyz=(front_x + 0.0028, y, 0.0375)),
            material=print_white,
            name=f"display_segment_{i}",
        )

    # Printed scale ticks live on the fixed faceplate, outside the rotating dial.
    tick_radius = 0.048
    for i, angle in enumerate([math.radians(a) for a in range(-120, 121, 20)]):
        y = tick_radius * math.sin(angle)
        z = dial_center_z + tick_radius * math.cos(angle)
        long_tick = i in (0, 6, 12)
        housing.visual(
            Box((0.0008, 0.0018 if not long_tick else 0.0024, 0.0058 if not long_tick else 0.0082)),
            origin=Origin(
                xyz=(front_x + 0.0011, y, z),
                rpy=(-angle, 0.0, 0.0),
            ),
            material=print_white,
            name=f"scale_tick_{i}",
        )

    # Fixed center shaft and front retainer: the dial rotates around this clear
    # physical axis with visible clearance instead of floating in front of the case.
    housing.visual(
        Cylinder(radius=0.0052, length=0.023),
        origin=Origin(xyz=(front_x + 0.0115, 0.0, dial_center_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="dial_shaft",
    )
    housing.visual(
        Cylinder(radius=0.0114, length=0.0030),
        origin=Origin(xyz=(front_x + 0.0204, 0.0, dial_center_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="dial_retainer",
    )

    # Rear hinge knuckles and bracket pads for the fold-flat desktop stand.
    for i, y in enumerate((-0.031, 0.031)):
        housing.visual(
            Cylinder(radius=0.0042, length=0.016),
            origin=Origin(xyz=(rear_x - 0.0025, y, 0.027), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"stand_hinge_knuckle_{i}",
        )
        housing.visual(
            Box((0.006, 0.018, 0.010)),
            origin=Origin(xyz=(rear_x - 0.0007, y, 0.024)),
            material=stand_mat,
            name=f"stand_hinge_pad_{i}",
        )

    dial = model.part("dial")
    dial.visual(
        _annular_dial_mesh(0.0365, 0.0090, 0.0140, "thermostat_annular_dial"),
        material=dial_mat,
        name="dial_ring",
    )
    # Finger ridges wrap the dial perimeter and are part of the rotating control.
    for i in range(24):
        angle = 2.0 * math.pi * i / 24.0
        y = 0.0375 * math.sin(angle)
        z = 0.0375 * math.cos(angle)
        dial.visual(
            Box((0.0115, 0.0020, 0.0040)),
            origin=Origin(xyz=(0.0070, y, z), rpy=(-angle, 0.0, 0.0)),
            material=dial_edge,
            name=f"grip_rib_{i}",
        )
    dial.visual(
        Box((0.0014, 0.0060, 0.0120)),
        origin=Origin(xyz=(0.0147, 0.0, 0.0290)),
        material=amber,
        name="dial_pointer",
    )

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.0035, length=0.038),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="stand_hinge_barrel",
    )
    stand.visual(
        Box((0.0032, 0.064, 0.058)),
        origin=Origin(xyz=(-0.0020, 0.0, -0.0290)),
        material=stand_mat,
        name="stand_panel",
    )
    stand.visual(
        Box((0.006, 0.066, 0.004)),
        origin=Origin(xyz=(-0.0060, 0.0, -0.0595)),
        material=stand_mat,
        name="stand_foot",
    )

    model.articulation(
        "housing_to_dial",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=dial,
        origin=Origin(xyz=(front_x + 0.0050, 0.0, dial_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.08, velocity=4.0),
    )
    model.articulation(
        "housing_to_stand",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=stand,
        origin=Origin(xyz=(rear_x - 0.0025, 0.0, 0.027)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.6, lower=0.0, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    dial = object_model.get_part("dial")
    stand = object_model.get_part("stand")
    dial_joint = object_model.get_articulation("housing_to_dial")
    stand_joint = object_model.get_articulation("housing_to_stand")

    ctx.expect_gap(
        dial,
        housing,
        axis="x",
        positive_elem="dial_ring",
        negative_elem="faceplate",
        min_gap=0.0015,
        max_gap=0.0045,
        name="dial has safe faceplate clearance",
    )
    ctx.expect_gap(
        housing,
        dial,
        axis="x",
        positive_elem="dial_retainer",
        negative_elem="dial_ring",
        max_gap=0.00025,
        max_penetration=0.00025,
        name="retainer lightly captures dial",
    )
    ctx.expect_within(
        housing,
        dial,
        axes="yz",
        inner_elem="dial_shaft",
        outer_elem="dial_ring",
        margin=0.0,
        name="shaft is centered inside dial envelope",
    )
    ctx.expect_within(
        stand,
        housing,
        axes="yz",
        inner_elem="stand_panel",
        outer_elem="body_shell",
        margin=0.001,
        name="stand folds within thermostat footprint",
    )
    ctx.expect_gap(
        housing,
        stand,
        axis="x",
        positive_elem="body_shell",
        negative_elem="stand_panel",
        min_gap=0.001,
        name="folded stand clears rear shell",
    )

    rest_dial_position = ctx.part_world_position(dial)
    with ctx.pose({dial_joint: math.pi * 0.75}):
        turned_dial_position = ctx.part_world_position(dial)
        ctx.expect_gap(
            dial,
            housing,
            axis="x",
            positive_elem="dial_ring",
            negative_elem="faceplate",
            min_gap=0.0015,
            max_gap=0.0045,
            name="rotated dial keeps face clearance",
        )
    ctx.check(
        "dial rotates about fixed center",
        rest_dial_position is not None
        and turned_dial_position is not None
        and all(abs(a - b) < 1e-7 for a, b in zip(rest_dial_position, turned_dial_position)),
        details=f"rest={rest_dial_position}, turned={turned_dial_position}",
    )

    rest_stand_aabb = ctx.part_element_world_aabb(stand, elem="stand_foot")
    with ctx.pose({stand_joint: 1.05}):
        open_stand_aabb = ctx.part_element_world_aabb(stand, elem="stand_foot")
        ctx.expect_gap(
            housing,
            stand,
            axis="x",
            positive_elem="body_shell",
            negative_elem="stand_foot",
            min_gap=0.010,
            name="opened stand swings behind body",
        )
    ctx.check(
        "stand deploys backward from flat stow",
        rest_stand_aabb is not None
        and open_stand_aabb is not None
        and open_stand_aabb[0][0] < rest_stand_aabb[0][0] - 0.025,
        details=f"rest={rest_stand_aabb}, open={open_stand_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
