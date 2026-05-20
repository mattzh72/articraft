from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)


BODY_LENGTH = 0.150
BODY_WIDTH = 0.068
BODY_HEIGHT = 0.082
DOOR_WIDTH = 0.088
DOOR_HEIGHT = 0.060
DOOR_THICKNESS = 0.005


def _rounded_rect_loop(
    width: float,
    height: float,
    radius: float,
    *,
    corner_segments: int = 5,
) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    r = min(radius, half_w * 0.98, half_h * 0.98)
    corners = (
        (half_w - r, half_h - r, 0.0, math.pi * 0.5),
        (-half_w + r, half_h - r, math.pi * 0.5, math.pi),
        (-half_w + r, -half_h + r, math.pi, math.pi * 1.5),
        (half_w - r, -half_h + r, math.pi * 1.5, math.tau),
    )

    points: list[tuple[float, float]] = []
    for corner_index, (cx, cy, a0, a1) in enumerate(corners):
        for step in range(corner_segments + 1):
            if corner_index > 0 and step == 0:
                continue
            t = step / corner_segments
            angle = a0 + (a1 - a0) * t
            points.append((cx + r * math.cos(angle), cy + r * math.sin(angle)))
    return points


def _yz_section(
    x_pos: float,
    width_y: float,
    height_z: float,
    radius: float,
    *,
    y_center: float = 0.0,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y + y_center, z + z_center)
        for y, z in _rounded_rect_loop(width_y, height_z, radius)
    ]


def _section_mesh(name: str, sections: list[list[tuple[float, float, float]]]):
    return mesh_from_geometry(repair_loft(section_loft(sections), repair="mesh"), name)


def _ring_mesh(name: str, *, inner_radius: float, outer_radius: float, length: float):
    ring = LatheGeometry.from_shell_profiles(
        [(outer_radius, -length * 0.5), (outer_radius, length * 0.5)],
        [(inner_radius, -length * 0.5), (inner_radius, length * 0.5)],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(math.pi * 0.5)
    return mesh_from_geometry(ring, name)


def _center_y(aabb) -> float | None:
    if aabb is None:
        return None
    return (aabb[0][1] + aabb[1][1]) * 0.5


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="home_video_camcorder")

    body_graphite = model.material("body_graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    trim_silver = model.material("trim_silver", rgba=(0.62, 0.64, 0.67, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.08, 1.0))
    plastic_black = model.material("plastic_black", rgba=(0.10, 0.10, 0.11, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.14, 0.20, 0.24, 0.78))

    body = model.part("body")
    body_sections = [
        _yz_section(-0.072, 0.068, 0.080, 0.012, z_center=0.003),
        _yz_section(-0.032, 0.067, 0.082, 0.013, z_center=0.004),
        _yz_section(0.014, 0.066, 0.076, 0.012, z_center=0.001),
        _yz_section(0.054, 0.063, 0.068, 0.011, z_center=-0.003),
        _yz_section(0.076, 0.058, 0.060, 0.010, z_center=-0.005),
    ]
    body.visual(
        _section_mesh("camcorder_body_shell", body_sections),
        material=body_graphite,
        name="body_shell",
    )
    body.visual(
        Box((0.082, 0.002, 0.050)),
        origin=Origin(xyz=(0.026, 0.031, 0.002)),
        material=plastic_black,
        name="display_bay",
    )
    body.visual(
        Box((0.012, 0.004, 0.010)),
        origin=Origin(xyz=(0.014, 0.033, 0.019)),
        material=plastic_black,
        name="side_button_0",
    )
    body.visual(
        Box((0.012, 0.004, 0.010)),
        origin=Origin(xyz=(0.030, 0.033, 0.003)),
        material=plastic_black,
        name="side_button_1",
    )
    body.visual(
        Box((0.020, 0.006, 0.020)),
        origin=Origin(xyz=(0.034, -0.032, 0.004)),
        material=rubber_black,
        name="strap_pad_front",
    )
    body.visual(
        Box((0.018, 0.006, 0.022)),
        origin=Origin(xyz=(-0.030, -0.032, 0.006)),
        material=rubber_black,
        name="strap_pad_rear",
    )
    body.visual(
        Box((0.074, 0.003, 0.016)),
        origin=Origin(xyz=(0.002, -0.036, 0.007)),
        material=rubber_black,
        name="hand_strap",
    )

    lens_barrel = model.part("lens_barrel")
    lens_barrel.visual(
        Cylinder(radius=0.0245, length=0.030),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=plastic_black,
        name="barrel_rear",
    )
    lens_barrel.visual(
        Cylinder(radius=0.0285, length=0.001),
        origin=Origin(xyz=(0.0105, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=rubber_black,
        name="focus_seat",
    )
    lens_barrel.visual(
        Cylinder(radius=0.0240, length=0.014),
        origin=Origin(xyz=(0.036, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=plastic_black,
        name="barrel_front",
    )
    lens_barrel.visual(
        Cylinder(radius=0.028, length=0.004),
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=trim_silver,
        name="front_bezel",
    )
    lens_barrel.visual(
        Cylinder(radius=0.0175, length=0.010),
        origin=Origin(xyz=(0.034, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=screen_glass,
        name="objective_glass",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        _ring_mesh(
            "camcorder_focus_ring",
            inner_radius=0.0255,
            outer_radius=0.0285,
            length=0.010,
        ),
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
        material=rubber_black,
        name="focus_ring_shell",
    )

    viewfinder = model.part("viewfinder")
    viewfinder_sections = [
        _yz_section(0.000, 0.024, 0.018, 0.004),
        _yz_section(-0.014, 0.026, 0.020, 0.0045, z_center=0.001),
        _yz_section(-0.028, 0.034, 0.026, 0.006, z_center=0.002),
    ]
    viewfinder.visual(
        _section_mesh("camcorder_viewfinder_shell", viewfinder_sections),
        material=rubber_black,
        name="viewfinder_shell",
    )
    viewfinder.visual(
        Box((0.010, 0.020, 0.010)),
        origin=Origin(xyz=(-0.006, 0.0, -0.004)),
        material=body_graphite,
        name="viewfinder_stem",
    )

    diopter_wheel = model.part("diopter_wheel")
    diopter_wheel.visual(
        Cylinder(radius=0.0055, length=0.0035),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=rubber_black,
        name="diopter_wheel",
    )
    diopter_wheel.visual(
        Cylinder(radius=0.0020, length=0.0060),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=trim_silver,
        name="diopter_axle",
    )

    display_door = model.part("display_door")
    door_face = ExtrudeGeometry(
        _rounded_rect_loop(DOOR_WIDTH, DOOR_HEIGHT, 0.006),
        DOOR_THICKNESS,
        center=True,
    ).rotate_x(math.pi * 0.5)
    display_door.visual(
        mesh_from_geometry(door_face, "camcorder_display_door"),
        origin=Origin(xyz=(DOOR_WIDTH * 0.5, DOOR_THICKNESS * 0.5 + 0.0004, 0.0)),
        material=body_graphite,
        name="door_panel",
    )
    display_door.visual(
        Box((0.064, 0.0010, 0.044)),
        origin=Origin(xyz=(0.047, 0.0009, 0.0)),
        material=screen_glass,
        name="lcd_screen",
    )
    display_door.visual(
        Cylinder(radius=0.0015, length=0.050),
        origin=Origin(xyz=(0.0, 0.0016, 0.0)),
        material=trim_silver,
        name="hinge_barrel",
    )

    model.articulation(
        "body_to_lens_barrel",
        ArticulationType.FIXED,
        parent=body,
        child=lens_barrel,
        origin=Origin(xyz=(0.076, 0.0, -0.002)),
    )
    model.articulation(
        "lens_barrel_to_focus_ring",
        ArticulationType.CONTINUOUS,
        parent=lens_barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.011, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=12.0),
    )
    model.articulation(
        "body_to_viewfinder",
        ArticulationType.FIXED,
        parent=body,
        child=viewfinder,
        origin=Origin(xyz=(-0.072, -0.002, 0.024)),
    )
    model.articulation(
        "viewfinder_to_diopter_wheel",
        ArticulationType.CONTINUOUS,
        parent=viewfinder,
        child=diopter_wheel,
        origin=Origin(xyz=(-0.023, 0.0188, 0.006)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.05, velocity=8.0),
    )
    model.articulation(
        "body_to_display_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=display_door,
        origin=Origin(xyz=(-0.016, 0.0342, 0.001)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.8,
            lower=0.0,
            upper=2.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lens_barrel = object_model.get_part("lens_barrel")
    focus_ring = object_model.get_part("focus_ring")
    viewfinder = object_model.get_part("viewfinder")
    diopter_wheel = object_model.get_part("diopter_wheel")
    display_door = object_model.get_part("display_door")
    door_hinge = object_model.get_articulation("body_to_display_door")

    ctx.expect_gap(
        display_door,
        body,
        axis="y",
        positive_elem="door_panel",
        negative_elem="body_shell",
        max_gap=0.003,
        max_penetration=0.0,
        name="display door closes nearly flush to the body side",
    )
    ctx.expect_overlap(
        display_door,
        body,
        axes="xz",
        elem_a="door_panel",
        elem_b="display_bay",
        min_overlap=0.040,
        name="display door covers the side display bay",
    )

    ctx.expect_origin_distance(
        focus_ring,
        lens_barrel,
        axes="yz",
        max_dist=0.0005,
        name="focus ring stays coaxial with the lens barrel",
    )
    ctx.expect_overlap(
        focus_ring,
        lens_barrel,
        axes="x",
        min_overlap=0.008,
        name="focus ring remains seated on the lens barrel",
    )

    ctx.expect_gap(
        diopter_wheel,
        viewfinder,
        axis="y",
        positive_elem="diopter_wheel",
        negative_elem="viewfinder_shell",
        max_gap=0.003,
        max_penetration=0.0,
        name="diopter wheel sits on the viewfinder sidewall",
    )
    ctx.expect_overlap(
        diopter_wheel,
        viewfinder,
        axes="xz",
        elem_a="diopter_wheel",
        elem_b="viewfinder_shell",
        min_overlap=0.004,
        name="diopter wheel lines up with the rear viewfinder",
    )

    closed_aabb = ctx.part_element_world_aabb(display_door, elem="door_panel")
    with ctx.pose({door_hinge: 1.45}):
        open_aabb = ctx.part_element_world_aabb(display_door, elem="door_panel")
    closed_center_y = _center_y(closed_aabb)
    open_center_y = _center_y(open_aabb)
    ctx.check(
        "display door swings outward from the body",
        closed_center_y is not None
        and open_center_y is not None
        and open_center_y > closed_center_y + 0.030,
        details=f"closed_center_y={closed_center_y}, open_center_y={open_center_y}",
    )

    return ctx.report()


object_model = build_object_model()
