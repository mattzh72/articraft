from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, *, segments: int = 72, center=(0.0, 0.0)):
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _translate_profile(profile, dx: float, dy: float):
    return [(x + dx, y + dy) for x, y in profile]


def _hole(profile):
    return list(reversed(profile))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_load_washer_steam")

    white = model.material("warm_white", rgba=(0.94, 0.94, 0.90, 1.0))
    panel_white = model.material("panel_white", rgba=(0.98, 0.98, 0.95, 1.0))
    dark = model.material("soft_black", rgba=(0.02, 0.022, 0.025, 1.0))
    glass = model.material("smoked_glass", rgba=(0.28, 0.55, 0.75, 0.36))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    chrome = model.material("chrome_trim", rgba=(0.78, 0.80, 0.82, 1.0))
    blue = model.material("steam_blue", rgba=(0.10, 0.45, 0.95, 1.0))

    body = model.part("body")

    # Real appliance scale: about 70 cm wide, 64 cm deep, 95 cm high.
    body.visual(Box((0.030, 0.640, 0.950)), origin=Origin(xyz=(-0.335, 0.0, 0.475)), material=white, name="side_panel_0")
    body.visual(Box((0.030, 0.640, 0.950)), origin=Origin(xyz=(0.335, 0.0, 0.475)), material=white, name="side_panel_1")
    body.visual(Box((0.700, 0.640, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.9325)), material=white, name="top_panel")
    body.visual(Box((0.700, 0.640, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.0175)), material=white, name="base_panel")
    body.visual(Box((0.700, 0.030, 0.950)), origin=Origin(xyz=(0.0, 0.305, 0.475)), material=white, name="rear_panel")

    front_outer = rounded_rect_profile(0.700, 0.950, 0.035, corner_segments=8)
    drawer_hole = _translate_profile(rounded_rect_profile(0.350, 0.095, 0.012, corner_segments=5), -0.170, 0.365)
    front_panel = (
        cq.Workplane("XY")
        .box(0.700, 0.950, 0.030)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .center(0.0, 0.015)
        .circle(0.245)
        .cutThruAll()
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .center(-0.170, 0.365)
        .rect(0.350, 0.095)
        .cutThruAll()
    )
    body.visual(
        mesh_from_cadquery(front_panel, "front_panel"),
        origin=Origin(xyz=(0.0, -0.335, 0.475), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=panel_white,
        name="front_panel",
    )

    gasket_ring = ExtrudeWithHolesGeometry(
        _circle_profile(0.265),
        [_hole(_circle_profile(0.210))],
        0.010,
        center=True,
    )
    body.visual(
        mesh_from_geometry(gasket_ring, "rubber_gasket"),
        origin=Origin(xyz=(0.0, -0.354, 0.490), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="rubber_gasket",
    )

    # Mounted hinge leaves on the cabinet side of the right-edge door hinge.
    body.visual(
        Box((0.065, 0.024, 0.120)),
        origin=Origin(xyz=(0.305, -0.360, 0.335)),
        material=chrome,
        name="hinge_leaf_0",
    )
    body.visual(
        Box((0.065, 0.024, 0.120)),
        origin=Origin(xyz=(0.305, -0.360, 0.645)),
        material=chrome,
        name="hinge_leaf_1",
    )

    # Top-front fascia details: display decal plus the socket under the selector knob.
    body.visual(Box((0.170, 0.008, 0.055)), origin=Origin(xyz=(0.070, -0.351, 0.842)), material=dark, name="display_panel")
    body.visual(
        Cylinder(radius=0.050, length=0.006),
        origin=Origin(xyz=(0.225, -0.351, 0.842), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="knob_socket",
    )
    body.visual(
        Cylinder(radius=0.035, length=0.205),
        origin=Origin(xyz=(0.0, 0.205, 0.490), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rear_bearing",
    )
    body.visual(
        Box((0.290, 0.300, 0.015)),
        origin=Origin(xyz=(-0.170, -0.215, 0.8825)),
        material=white,
        name="drawer_rail_top",
    )
    body.visual(
        Box((0.360, 0.300, 0.015)),
        origin=Origin(xyz=(-0.170, -0.215, 0.7975)),
        material=white,
        name="drawer_rail_bottom",
    )

    drum = model.part("drum")
    drum_shell = CylinderGeometry(0.230, 0.340, radial_segments=80, closed=False)
    drum.visual(
        mesh_from_geometry(drum_shell, "drum_shell"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="drum_shell",
    )
    drum_rim = ExtrudeWithHolesGeometry(_circle_profile(0.238), [_hole(_circle_profile(0.196))], 0.025, center=True)
    drum.visual(
        mesh_from_geometry(drum_rim, "drum_rim"),
        origin=Origin(xyz=(0.0, -0.180, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="drum_rim",
    )
    drum.visual(
        Cylinder(radius=0.232, length=0.016),
        origin=Origin(xyz=(0.0, 0.164, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="drum_back",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        drum.visual(
            Box((0.060, 0.270, 0.040)),
            origin=Origin(
                xyz=(0.215 * math.cos(angle), -0.010, 0.215 * math.sin(angle)),
                rpy=(0.0, -angle, 0.0),
            ),
            material=chrome,
            name=f"drum_baffle_{index}",
        )

    model.articulation(
        "body_to_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.0, -0.060, 0.490)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=8.0),
    )

    door = model.part("door")
    door_ring = ExtrudeWithHolesGeometry(_circle_profile(0.275), [_hole(_circle_profile(0.170))], 0.052, center=True)
    door.visual(
        mesh_from_geometry(door_ring, "door_ring"),
        origin=Origin(xyz=(-0.275, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=panel_white,
        name="door_ring",
    )
    door.visual(
        mesh_from_geometry(ExtrudeWithHolesGeometry(_circle_profile(0.205), [_hole(_circle_profile(0.172))], 0.018, center=True), "door_chrome_ring"),
        origin=Origin(xyz=(-0.275, -0.032, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="door_chrome_ring",
    )
    door.visual(
        Cylinder(radius=0.165, length=0.014),
        origin=Origin(xyz=(-0.275, -0.043, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="door_glass",
    )
    # Two exposed barrel hinge knuckles on the child door, tied back into the door ring by leaves.
    door.visual(
        Cylinder(radius=0.020, length=0.115),
        origin=Origin(xyz=(0.0, 0.0, -0.155)),
        material=chrome,
        name="hinge_barrel_0",
    )
    door.visual(
        Box((0.145, 0.030, 0.060)),
        origin=Origin(xyz=(-0.070, 0.0, -0.155)),
        material=chrome,
        name="hinge_arm_0",
    )
    door.visual(
        Cylinder(radius=0.020, length=0.115),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=chrome,
        name="hinge_barrel_1",
    )
    door.visual(
        Box((0.145, 0.030, 0.060)),
        origin=Origin(xyz=(-0.070, 0.0, 0.155)),
        material=chrome,
        name="hinge_arm_1",
    )
    door.visual(Box((0.030, 0.040, 0.190)), origin=Origin(xyz=(-0.525, -0.020, 0.0)), material=chrome, name="door_handle")

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.275, -0.388, 0.490)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=0.0, upper=1.85),
    )

    steam_drawer = model.part("steam_drawer")
    steam_drawer.visual(
        Box((0.290, 0.320, 0.070)),
        origin=Origin(xyz=(0.0, 0.150, 0.0)),
        material=panel_white,
        name="drawer_box",
    )
    steam_drawer.visual(
        Box((0.355, 0.016, 0.105)),
        origin=Origin(xyz=(0.0, -0.014, 0.0)),
        material=panel_white,
        name="drawer_face",
    )
    steam_drawer.visual(Box((0.120, 0.010, 0.012)), origin=Origin(xyz=(-0.060, -0.027, -0.022)), material=blue, name="steam_mark")
    steam_drawer.visual(Box((0.110, 0.012, 0.014)), origin=Origin(xyz=(0.070, -0.028, 0.026)), material=dark, name="drawer_grip")

    model.articulation(
        "body_to_steam_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=steam_drawer,
        origin=Origin(xyz=(-0.170, -0.365, 0.840)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.220),
    )

    control_knob = model.part("selector_knob")
    control_knob.visual(
        Cylinder(radius=0.043, length=0.030),
        origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="knob_cap",
    )
    control_knob.visual(
        Cylinder(radius=0.012, length=0.046),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="knob_stem",
    )
    control_knob.visual(Box((0.009, 0.006, 0.033)), origin=Origin(xyz=(0.0, -0.033, 0.026)), material=dark, name="knob_pointer")
    model.articulation(
        "body_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=control_knob,
        origin=Origin(xyz=(0.225, -0.386, 0.842)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    drawer = object_model.get_part("steam_drawer")
    door_joint = object_model.get_articulation("body_to_door")
    drawer_joint = object_model.get_articulation("body_to_steam_drawer")
    drum_joint = object_model.get_articulation("body_to_drum")

    ctx.allow_overlap(
        body,
        door,
        elem_a="hinge_leaf_0",
        elem_b="hinge_barrel_0",
        reason="The lower barrel hinge is intentionally captured in the cabinet leaf with a tiny local pin-like embed.",
    )
    ctx.allow_overlap(
        body,
        door,
        elem_a="hinge_leaf_1",
        elem_b="hinge_barrel_1",
        reason="The upper barrel hinge is intentionally captured in the cabinet leaf with a tiny local pin-like embed.",
    )
    ctx.allow_overlap(
        body,
        drum,
        elem_a="rear_bearing",
        elem_b="drum_back",
        reason="The rotating drum hub is intentionally nested on the rear bearing axle.",
    )
    ctx.allow_overlap(
        body,
        "selector_knob",
        elem_a="knob_socket",
        elem_b="knob_stem",
        reason="The control knob stem is intentionally seated into the front-panel socket.",
    )

    ctx.expect_overlap(body, door, axes="xyz", min_overlap=0.001, elem_a="hinge_leaf_0", elem_b="hinge_barrel_0", name="lower hinge barrel is captured")
    ctx.expect_overlap(body, door, axes="xyz", min_overlap=0.001, elem_a="hinge_leaf_1", elem_b="hinge_barrel_1", name="upper hinge barrel is captured")
    ctx.expect_overlap(body, drum, axes="xyz", min_overlap=0.002, elem_a="rear_bearing", elem_b="drum_back", name="drum rides on rear bearing")
    ctx.expect_overlap(body, "selector_knob", axes="xyz", min_overlap=0.0005, elem_a="knob_socket", elem_b="knob_stem", name="knob stem seats in socket")

    ctx.expect_gap(
        body,
        door,
        axis="y",
        positive_elem="rubber_gasket",
        negative_elem="door_ring",
        min_gap=0.001,
        max_gap=0.010,
        name="closed door ring clears gasket",
    )
    ctx.expect_overlap(door, body, axes="xz", min_overlap=0.35, elem_a="door_ring", elem_b="rubber_gasket", name="porthole door covers gasket")
    ctx.expect_overlap(drum, body, axes="xz", min_overlap=0.35, elem_a="drum_rim", elem_b="rubber_gasket", name="drum aligns with porthole")
    ctx.expect_overlap(drawer, body, axes="y", min_overlap=0.200, elem_a="drawer_box", name="steam drawer is retained when shut")
    ctx.expect_contact(body, drawer, elem_a="drawer_rail_top", elem_b="drawer_box", contact_tol=0.001, name="steam drawer rides in top guide rail")

    rest_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.20}):
        opened_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door hinge opens outward",
        rest_door_aabb is not None
        and opened_door_aabb is not None
        and opened_door_aabb[0][1] < rest_door_aabb[0][1] - 0.10,
        details=f"rest={rest_door_aabb}, opened={opened_door_aabb}",
    )

    rest_drawer = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: 0.220}):
        extended_drawer = ctx.part_world_position(drawer)
        ctx.expect_overlap(drawer, body, axes="y", min_overlap=0.060, elem_a="drawer_box", name="extended drawer keeps insertion")
    ctx.check(
        "steam drawer slides out front",
        rest_drawer is not None and extended_drawer is not None and extended_drawer[1] < rest_drawer[1] - 0.18,
        details=f"rest={rest_drawer}, extended={extended_drawer}",
    )

    ctx.check("drum axle is front back", tuple(round(v, 3) for v in drum_joint.axis) == (0.0, 1.0, 0.0), details=str(drum_joint.axis))

    return ctx.report()


object_model = build_object_model()
