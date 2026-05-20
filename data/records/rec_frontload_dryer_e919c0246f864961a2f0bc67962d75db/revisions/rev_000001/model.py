from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(radius: float, *, center=(0.0, 0.0), segments: int = 72):
    cx, cy = center
    return [
        (cx + radius * math.cos(2.0 * math.pi * i / segments), cy + radius * math.sin(2.0 * math.pi * i / segments))
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_load_dryer")

    enamel = model.material("warm_white_enamel", rgba=(0.92, 0.90, 0.86, 1.0))
    satin_white = model.material("satin_white_trim", rgba=(0.96, 0.95, 0.91, 1.0))
    graphite = model.material("graphite_panel", rgba=(0.10, 0.11, 0.12, 1.0))
    dark = model.material("deep_shadow", rgba=(0.015, 0.017, 0.020, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.022, 0.024, 1.0))
    steel = model.material("brushed_steel", rgba=(0.66, 0.67, 0.65, 1.0))
    chrome = model.material("soft_chrome", rgba=(0.80, 0.82, 0.80, 1.0))
    glass = model.material("smoked_blue_glass", rgba=(0.20, 0.34, 0.45, 0.42))
    led_green = model.material("green_led", rgba=(0.25, 0.95, 0.55, 1.0))

    cabinet = model.part("cabinet")

    # The dryer body is roughly full-size: a wide, deep appliance cabinet with a
    # separate front sheet that has a real circular loading opening.
    side_panel = Box((0.045, 0.730, 0.960))
    cabinet.visual(side_panel, origin=Origin(xyz=(-0.355, 0.0, 0.500)), material=enamel, name="side_panel_0")
    cabinet.visual(side_panel, origin=Origin(xyz=(0.355, 0.0, 0.500)), material=enamel, name="side_panel_1")
    cabinet.visual(Box((0.735, 0.720, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.9825)), material=enamel, name="top_panel")
    cabinet.visual(Box((0.735, 0.720, 0.040)), origin=Origin(xyz=(0.0, 0.0, 0.020)), material=enamel, name="base_panel")
    cabinet.visual(Box((0.735, 0.035, 0.960)), origin=Origin(xyz=(0.0, 0.3675, 0.500)), material=enamel, name="rear_panel")

    front_panel_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.710, 0.760, 0.028, corner_segments=10),
            [_circle_profile(0.246, center=(0.0, 0.035), segments=96)],
            0.018,
            cap=True,
            center=True,
        ),
        "front_panel",
    )
    cabinet.visual(
        front_panel_mesh,
        origin=Origin(xyz=(0.0, -0.367, 0.445), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_white,
        name="front_panel",
    )

    kick_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.610, 0.070, 0.012, corner_segments=6), 0.010, cap=True, center=True),
        "toe_kick",
    )
    cabinet.visual(kick_mesh, origin=Origin(xyz=(0.0, -0.380, 0.105), rpy=(math.pi / 2.0, 0.0, 0.0)), material=graphite, name="toe_kick")

    opening_bezel_mesh = mesh_from_geometry(
        BezelGeometry(
            (0.460, 0.460),
            (0.555, 0.555),
            0.030,
            opening_shape="circle",
            outer_shape="circle",
        ),
        "opening_bezel",
    )
    cabinet.visual(
        opening_bezel_mesh,
        origin=Origin(xyz=(0.0, -0.386, 0.480), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_white,
        name="opening_bezel",
    )

    gasket_mesh = mesh_from_geometry(
        BezelGeometry(
            (0.420, 0.420),
            (0.505, 0.505),
            0.120,
            opening_shape="circle",
            outer_shape="circle",
        ),
        "door_gasket",
    )
    cabinet.visual(
        gasket_mesh,
        origin=Origin(xyz=(0.0, -0.315, 0.480), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="door_gasket",
    )

    drum_lip_mesh = mesh_from_geometry(
        BezelGeometry((0.360, 0.360), (0.455, 0.455), 0.035, opening_shape="circle", outer_shape="circle"),
        "drum_lip",
    )
    cabinet.visual(drum_lip_mesh, origin=Origin(xyz=(0.0, -0.244, 0.480), rpy=(math.pi / 2.0, 0.0, 0.0)), material=steel, name="drum_lip")
    cabinet.visual(Cylinder(radius=0.210, length=0.016), origin=Origin(xyz=(0.0, -0.219, 0.480), rpy=(math.pi / 2.0, 0.0, 0.0)), material=dark, name="drum_back")

    for row, z in enumerate((0.405, 0.440, 0.480, 0.520, 0.555)):
        width = 0.260 - abs(z - 0.480) * 1.2
        count = 5 if row in (1, 2, 3) else 3
        for i in range(count):
            x = (i - (count - 1) / 2.0) * (width / max(count - 1, 1))
            cabinet.visual(
                Cylinder(radius=0.0075, length=0.004),
                origin=Origin(xyz=(x, -0.224, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=steel,
                name=f"drum_perforation_{row}_{i}",
            )

    console_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.660, 0.155, 0.020, corner_segments=8), 0.026, cap=True, center=True),
        "control_panel",
    )
    cabinet.visual(console_mesh, origin=Origin(xyz=(0.0, -0.389, 0.858), rpy=(math.pi / 2.0, 0.0, 0.0)), material=graphite, name="control_panel")
    cabinet.visual(Box((0.235, 0.006, 0.060)), origin=Origin(xyz=(0.155, -0.404, 0.882)), material=dark, name="display_window")
    for i, x in enumerate((0.070, 0.105, 0.140, 0.175)):
        cabinet.visual(Box((0.020, 0.004, 0.006)), origin=Origin(xyz=(x, -0.408, 0.890)), material=led_green, name=f"display_segment_{i}")
    cabinet.visual(Box((0.105, 0.005, 0.004)), origin=Origin(xyz=(0.260, -0.408, 0.864)), material=led_green, name="cycle_status_line")

    cabinet.visual(Box((0.030, 0.030, 0.455)), origin=Origin(xyz=(-0.349, -0.375, 0.500)), material=chrome, name="hinge_leaf")
    cabinet.visual(Cylinder(radius=0.020, length=0.105), origin=Origin(xyz=(-0.349, -0.410, 0.480)), material=chrome, name="hinge_barrel_0")
    cabinet.visual(Cylinder(radius=0.020, length=0.075), origin=Origin(xyz=(-0.349, -0.410, 0.695)), material=chrome, name="hinge_barrel_1")

    for i, x in enumerate((-0.300, 0.300)):
        cabinet.visual(Cylinder(radius=0.030, length=0.018), origin=Origin(xyz=(x, -0.260, 0.009)), material=rubber, name=f"foot_{i}")

    door = model.part("door")
    door_ring_mesh = mesh_from_geometry(
        BezelGeometry((0.420, 0.420), (0.620, 0.620), 0.075, opening_shape="circle", outer_shape="circle"),
        "door_ring",
    )
    door.visual(door_ring_mesh, origin=Origin(xyz=(0.349, -0.049, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=satin_white, name="door_ring")
    door.visual(Cylinder(radius=0.218, length=0.014), origin=Origin(xyz=(0.349, -0.092, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=glass, name="door_glass")
    door.visual(Cylinder(radius=0.016, length=0.230), origin=Origin(xyz=(0.614, -0.114, 0.0)), material=chrome, name="front_handle")
    door.visual(Box((0.055, 0.036, 0.026)), origin=Origin(xyz=(0.586, -0.097, 0.095)), material=chrome, name="handle_post_0")
    door.visual(Box((0.055, 0.036, 0.026)), origin=Origin(xyz=(0.586, -0.097, -0.095)), material=chrome, name="handle_post_1")
    door.visual(Cylinder(radius=0.018, length=0.110), origin=Origin(xyz=(0.0, -0.041, 0.1075)), material=chrome, name="door_hinge_barrel_0")
    door.visual(Cylinder(radius=0.018, length=0.110), origin=Origin(xyz=(0.0, -0.041, -0.1075)), material=chrome, name="door_hinge_barrel_1")
    door.visual(Box((0.092, 0.024, 0.030)), origin=Origin(xyz=(0.045, -0.044, 0.128)), material=chrome, name="hinge_strap_0")
    door.visual(Box((0.092, 0.024, 0.030)), origin=Origin(xyz=(0.045, -0.044, -0.128)), material=chrome, name="hinge_strap_1")

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-0.349, -0.392, 0.480)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=0.0, upper=2.05),
    )

    dial = model.part("dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.090,
            0.036,
            body_style="skirted",
            top_diameter=0.070,
            grip=KnobGrip(style="fluted", count=28, depth=0.0014),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
            center=False,
        ),
        "dial",
    )
    dial.visual(dial_mesh, origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)), material=chrome, name="dial_cap")
    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=dial,
        origin=Origin(xyz=(-0.205, -0.401, 0.865)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0),
    )

    start_button = model.part("start_button")
    start_button.visual(Cylinder(radius=0.027, length=0.020), origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=chrome, name="button_cap")
    model.articulation(
        "start_press",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=start_button,
        origin=Origin(xyz=(0.300, -0.401, 0.865)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=0.012),
    )

    for i, x in enumerate((0.040, 0.105, 0.170)):
        button = model.part(f"option_button_{i}")
        button.visual(Box((0.044, 0.014, 0.026)), origin=Origin(xyz=(0.0, -0.007, 0.0)), material=chrome, name="button_cap")
        model.articulation(
            f"option_press_{i}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x, -0.401, 0.805)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.010),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("door_hinge")
    start_button = object_model.get_part("start_button")
    start_press = object_model.get_articulation("start_press")

    ctx.expect_overlap(
        door,
        cabinet,
        axes="xz",
        elem_a="door_ring",
        elem_b="opening_bezel",
        min_overlap=0.40,
        name="closed door surrounds the loading opening",
    )
    ctx.expect_gap(
        cabinet,
        door,
        axis="y",
        positive_elem="opening_bezel",
        negative_elem="door_ring",
        min_gap=0.0,
        max_gap=0.010,
        name="closed door sits just proud of the front bezel",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_ring")
    with ctx.pose({door_hinge: 1.55}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_ring")
    ctx.check(
        "door swings outward from the front",
        closed_aabb is not None and open_aabb is not None and open_aabb[0][1] < closed_aabb[0][1] - 0.20,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    rest_pos = ctx.part_world_position(start_button)
    with ctx.pose({start_press: 0.012}):
        pressed_pos = ctx.part_world_position(start_button)
    ctx.check(
        "start button presses inward",
        rest_pos is not None and pressed_pos is not None and pressed_pos[1] > rest_pos[1] + 0.009,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    dial = object_model.get_articulation("dial_spin")
    ctx.check(
        "cycle dial rotates about the front normal",
        tuple(round(v, 3) for v in dial.axis) == (0.0, -1.0, 0.0),
        details=f"axis={dial.axis}",
    )

    return ctx.report()


object_model = build_object_model()
