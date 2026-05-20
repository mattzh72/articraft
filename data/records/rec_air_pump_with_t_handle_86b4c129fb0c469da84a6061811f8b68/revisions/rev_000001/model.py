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
    mesh_from_geometry,
    tube_from_spline_points,
)


def _ring_mesh(outer_radius: float, inner_radius: float, length: float, name: str):
    ring = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(length)
    return mesh_from_cadquery(ring, name)


def _rounded_box_mesh(size: tuple[float, float, float], fillet: float, name: str):
    sx, sy, sz = size
    solid = cq.Workplane("XY").box(sx, sy, sz)
    if fillet > 0.0:
        solid = solid.edges().fillet(fillet)
    return mesh_from_cadquery(solid, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_station_floor_pump")

    body_black = model.material("body_black", rgba=(0.14, 0.15, 0.16, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.77, 0.79, 0.81, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    dial_white = model.material("dial_white", rgba=(0.95, 0.95, 0.93, 1.0))
    warning_orange = model.material("warning_orange", rgba=(0.96, 0.45, 0.12, 1.0))
    release_red = model.material("release_red", rgba=(0.78, 0.16, 0.12, 1.0))

    barrel_height = 0.590
    barrel_base_z = 0.050
    barrel_outer_radius = 0.033
    barrel_inner_radius = 0.0285
    guide_top_z = 0.655
    gauge_center = (0.0, 0.114, 0.195)
    gauge_size = (0.110, 0.050, 0.140)
    gauge_front_y = gauge_center[1] + gauge_size[1] / 2.0
    gauge_right_x = gauge_center[0] + gauge_size[0] / 2.0

    body = model.part("body")

    body.visual(
        Cylinder(radius=0.046, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=body_black,
        name="base_hub",
    )
    body.visual(
        Box((0.150, 0.075, 0.018)),
        origin=Origin(xyz=(-0.102, 0.0, 0.009)),
        material=body_black,
        name="foot_pad_0",
    )
    body.visual(
        Box((0.150, 0.075, 0.018)),
        origin=Origin(xyz=(0.102, 0.0, 0.009)),
        material=body_black,
        name="foot_pad_1",
    )
    body.visual(
        mesh_from_cadquery(
            cq.Workplane("XY")
            .circle(barrel_outer_radius)
            .circle(barrel_inner_radius)
            .extrude(barrel_height),
            "pump_barrel_shell",
        ),
        origin=Origin(xyz=(0.0, 0.0, barrel_base_z)),
        material=body_black,
        name="barrel_shell",
    )
    body.visual(
        _ring_mesh(0.032, 0.0105, 0.030, "pump_guide_ring"),
        origin=Origin(xyz=(0.0, 0.0, guide_top_z - 0.030)),
        material=satin_steel,
        name="guide_ring",
    )
    body.visual(
        _rounded_box_mesh((0.034, 0.064, 0.150), 0.004, "gauge_bracket"),
        origin=Origin(xyz=(0.0, 0.057, 0.195)),
        material=body_black,
        name="gauge_bracket",
    )
    body.visual(
        _rounded_box_mesh(gauge_size, 0.008, "gauge_housing"),
        origin=Origin(xyz=gauge_center),
        material=body_black,
        name="gauge_housing",
    )
    body.visual(
        _ring_mesh(0.044, 0.037, 0.008, "gauge_bezel_ring"),
        origin=Origin(
            xyz=(0.0, gauge_front_y, gauge_center[2]),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_steel,
        name="gauge_bezel",
    )
    body.visual(
        Cylinder(radius=0.036, length=0.012),
        origin=Origin(
            xyz=(0.0, gauge_front_y - 0.006, gauge_center[2]),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=dial_white,
        name="dial_face",
    )
    body.visual(
        _ring_mesh(0.010, 0.0058, 0.006, "release_button_guide"),
        origin=Origin(
            xyz=(gauge_right_x, gauge_center[1] - 0.003, gauge_center[2] + 0.050),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_steel,
        name="button_guide",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.028),
        origin=Origin(xyz=(0.0, 0.045, 0.070), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="hose_port",
    )

    hose = tube_from_spline_points(
        [
            (0.0, 0.032, 0.070),
            (0.026, 0.056, 0.064),
            (0.048, 0.090, 0.100),
            (0.048, 0.103, 0.150),
            (0.040, 0.103, 0.210),
            (0.028, 0.103, 0.246),
        ],
        radius=0.007,
        samples_per_segment=18,
        radial_segments=20,
    )
    body.visual(
        mesh_from_geometry(hose, "parked_hose"),
        material=rubber_black,
        name="hose",
    )
    body.visual(
        Box((0.018, 0.028, 0.030)),
        origin=Origin(xyz=(0.046, 0.091, 0.142)),
        material=body_black,
        name="lower_clip",
    )
    body.visual(
        Box((0.018, 0.032, 0.030)),
        origin=Origin(xyz=(0.034, 0.091, 0.232)),
        material=body_black,
        name="upper_clip",
    )
    body.visual(
        Box((0.028, 0.014, 0.012)),
        origin=Origin(xyz=(0.028, 0.105, 0.254)),
        material=satin_steel,
        name="hose_head",
    )
    body.visual(
        Box((0.010, 0.004, 0.018)),
        origin=Origin(xyz=(0.038, 0.110, 0.266)),
        material=body_black,
        name="hose_head_lever",
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0085, length=0.620),
        origin=Origin(xyz=(0.0, 0.0, -0.255)),
        material=satin_steel,
        name="piston_rod",
    )
    plunger.visual(
        Cylinder(radius=0.013, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=satin_steel,
        name="handle_stem",
    )
    plunger.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=satin_steel,
        name="guide_collar",
    )
    plunger.visual(
        Box((0.042, 0.034, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.116)),
        material=body_black,
        name="handle_center",
    )
    plunger.visual(
        Cylinder(radius=0.014, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.125), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="handle_bar",
    )
    plunger.visual(
        Cylinder(radius=0.018, length=0.094),
        origin=Origin(xyz=(-0.103, 0.0, 0.125), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="grip_0",
    )
    plunger.visual(
        Cylinder(radius=0.018, length=0.094),
        origin=Origin(xyz=(0.103, 0.0, 0.125), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="grip_1",
    )

    gauge_needle = model.part("gauge_needle")
    gauge_needle.visual(
        Cylinder(radius=0.0042, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="needle_hub",
    )
    gauge_needle.visual(
        Box((0.030, 0.0024, 0.0045)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=warning_orange,
        name="needle_pointer",
    )
    gauge_needle.visual(
        Box((0.008, 0.0024, 0.003)),
        origin=Origin(xyz=(-0.006, 0.0, 0.0)),
        material=warning_orange,
        name="needle_tail",
    )

    release_button = model.part("release_button")
    release_button.visual(
        Cylinder(radius=0.0048, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=release_red,
        name="button_shaft",
    )
    release_button.visual(
        Cylinder(radius=0.0078, length=0.002),
        origin=Origin(xyz=(0.001, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=release_red,
        name="button_flange",
    )
    release_button.visual(
        Cylinder(radius=0.0072, length=0.004),
        origin=Origin(xyz=(0.0105, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=release_red,
        name="button_cap",
    )

    model.articulation(
        "body_to_plunger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, guide_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=0.0, upper=0.280),
    )
    model.articulation(
        "body_to_gauge_needle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=gauge_needle,
        origin=Origin(xyz=(0.0, gauge_front_y - 0.0025, gauge_center[2])),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=10.0),
    )
    model.articulation(
        "body_to_release_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=release_button,
        origin=Origin(
            xyz=(gauge_right_x + 0.006, gauge_center[1] - 0.003, gauge_center[2] + 0.050)
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.08, lower=0.0, upper=0.0055),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    plunger = object_model.get_part("plunger")
    gauge_needle = object_model.get_part("gauge_needle")
    release_button = object_model.get_part("release_button")

    plunger_joint = object_model.get_articulation("body_to_plunger")
    needle_joint = object_model.get_articulation("body_to_gauge_needle")
    button_joint = object_model.get_articulation("body_to_release_button")

    plunger_limits = plunger_joint.motion_limits
    button_limits = button_joint.motion_limits

    rest_plunger_pos = None
    extended_plunger_pos = None
    if plunger_limits is not None and plunger_limits.upper is not None:
        with ctx.pose({plunger_joint: 0.0}):
            ctx.expect_within(
                plunger,
                body,
                axes="xy",
                inner_elem="piston_rod",
                outer_elem="guide_ring",
                margin=0.0,
                name="piston rod stays centered in the top guide at rest",
            )
            ctx.expect_overlap(
                plunger,
                body,
                axes="z",
                elem_a="piston_rod",
                elem_b="guide_ring",
                min_overlap=0.026,
                name="piston rod remains inserted through the guide at rest",
            )
            rest_plunger_pos = ctx.part_world_position(plunger)

        with ctx.pose({plunger_joint: plunger_limits.upper}):
            ctx.expect_within(
                plunger,
                body,
                axes="xy",
                inner_elem="piston_rod",
                outer_elem="guide_ring",
                margin=0.0,
                name="piston rod stays centered in the top guide at full stroke",
            )
            ctx.expect_overlap(
                plunger,
                body,
                axes="z",
                elem_a="piston_rod",
                elem_b="guide_ring",
                min_overlap=0.026,
                name="piston rod remains retained in the guide at full stroke",
            )
            extended_plunger_pos = ctx.part_world_position(plunger)

    ctx.check(
        "plunger extends upward along the pump axis",
        rest_plunger_pos is not None
        and extended_plunger_pos is not None
        and extended_plunger_pos[2] > rest_plunger_pos[2] + 0.20,
        details=f"rest={rest_plunger_pos}, extended={extended_plunger_pos}",
    )

    button_rest_pos = None
    button_pressed_pos = None
    if button_limits is not None and button_limits.upper is not None:
        with ctx.pose({button_joint: 0.0}):
            ctx.expect_overlap(
                release_button,
                body,
                axes="yz",
                elem_a="button_shaft",
                elem_b="button_guide",
                min_overlap=0.009,
                name="release button stays aligned with its guide at rest",
            )
            button_rest_pos = ctx.part_world_position(release_button)

        with ctx.pose({button_joint: button_limits.upper}):
            ctx.expect_overlap(
                release_button,
                body,
                axes="yz",
                elem_a="button_shaft",
                elem_b="button_guide",
                min_overlap=0.009,
                name="release button stays aligned with its guide when pressed",
            )
            button_pressed_pos = ctx.part_world_position(release_button)

    ctx.check(
        "release button presses inward from the gauge housing",
        button_rest_pos is not None
        and button_pressed_pos is not None
        and button_pressed_pos[0] < button_rest_pos[0] - 0.004,
        details=f"rest={button_rest_pos}, pressed={button_pressed_pos}",
    )

    rest_pointer_aabb = ctx.part_element_world_aabb(gauge_needle, elem="needle_pointer")
    with ctx.pose({needle_joint: 1.1}):
        turned_pointer_aabb = ctx.part_element_world_aabb(gauge_needle, elem="needle_pointer")

    ctx.check(
        "gauge needle rotates on its local pivot",
        rest_pointer_aabb is not None
        and turned_pointer_aabb is not None
        and abs(turned_pointer_aabb[0][2] - rest_pointer_aabb[0][2]) > 0.01,
        details=f"rest={rest_pointer_aabb}, turned={turned_pointer_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
