from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="grill_combo_microwave")

    stainless = Material("brushed_stainless", rgba=(0.62, 0.64, 0.63, 1.0))
    dark = Material("black_glass", rgba=(0.02, 0.025, 0.028, 1.0))
    liner = Material("warm_cavity_liner", rgba=(0.80, 0.78, 0.70, 1.0))
    glass = Material("smoky_glass", rgba=(0.12, 0.18, 0.22, 0.48))
    button_mat = Material("soft_black_buttons", rgba=(0.015, 0.015, 0.016, 1.0))
    white = Material("white_marking", rgba=(0.92, 0.92, 0.86, 1.0))
    red_heat = Material("dark_grill_element", rgba=(0.22, 0.04, 0.02, 1.0))
    tray_glass = Material("pale_green_glass", rgba=(0.65, 0.86, 0.78, 0.46))

    width = 0.56
    depth = 0.42
    height = 0.32
    wall = 0.025
    front_y = -depth / 2.0
    floor_top = -height / 2.0 + wall

    cabinet = model.part("cabinet")

    # One connected outer shell: open cavity on the left, solid control tower on the right.
    cabinet.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, -height / 2.0 + wall / 2.0)),
        material=stainless,
        name="bottom_wall",
    )
    cabinet.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, height / 2.0 - wall / 2.0)),
        material=stainless,
        name="top_wall",
    )
    cabinet.visual(
        Box((width, wall, height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, 0.0)),
        material=stainless,
        name="rear_wall",
    )
    cabinet.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, 0.0)),
        material=stainless,
        name="side_wall",
    )
    cabinet.visual(
        Box((0.14, depth, height)),
        origin=Origin(xyz=(width / 2.0 - 0.07, 0.0, 0.0)),
        material=stainless,
        name="control_tower",
    )
    cabinet.visual(
        Box((0.120, 0.004, 0.285)),
        origin=Origin(xyz=(0.210, front_y - 0.002, 0.0)),
        material=dark,
        name="control_face",
    )

    # Light-colored microwave cavity liner and grill element under the ceiling.
    cabinet.visual(
        Box((0.390, 0.340, 0.004)),
        origin=Origin(xyz=(-0.070, 0.005, floor_top + 0.002)),
        material=liner,
        name="cavity_floor",
    )
    cabinet.visual(
        Box((0.390, 0.004, 0.245)),
        origin=Origin(xyz=(-0.070, depth / 2.0 - wall - 0.002, 0.0)),
        material=liner,
        name="cavity_back",
    )
    cabinet.visual(
        Box((0.390, 0.340, 0.004)),
        origin=Origin(xyz=(-0.070, 0.005, height / 2.0 - wall - 0.002)),
        material=liner,
        name="cavity_ceiling",
    )
    for i, y in enumerate((-0.075, -0.025, 0.025, 0.075)):
        cabinet.visual(
            Box((0.285, 0.010, 0.006)),
            origin=Origin(xyz=(-0.070, y, height / 2.0 - wall - 0.005)),
            material=red_heat,
            name=f"grill_bar_{i}",
        )

    # Fixed hinge knuckles on the cabinet side, separated from the moving middle knuckle.
    hinge_x = -0.285
    hinge_y = front_y - 0.014
    for name, z in (("hinge_socket_low", -0.095), ("hinge_socket_high", 0.095)):
        cabinet.visual(
            Cylinder(radius=0.008, length=0.060),
            origin=Origin(xyz=(hinge_x, hinge_y, z)),
            material=stainless,
            name=name,
        )
        cabinet.visual(
            Box((0.010, 0.014, 0.052)),
            origin=Origin(xyz=(hinge_x + 0.004, front_y - 0.003, z)),
            material=stainless,
            name=f"{name}_bracket",
        )

    # Central support captures the turntable through its annular hub.
    spindle_x = -0.070
    spindle_y = 0.020
    tray_joint_z = floor_top + 0.030
    cabinet.visual(
        Cylinder(radius=0.032, length=0.024),
        origin=Origin(xyz=(spindle_x, spindle_y, floor_top + 0.012)),
        material=stainless,
        name="support_base",
    )
    cabinet.visual(
        Cylinder(radius=0.010, length=0.048),
        origin=Origin(xyz=(spindle_x, spindle_y, floor_top + 0.024)),
        material=stainless,
        name="spindle",
    )
    cabinet.visual(
        Cylinder(radius=0.030, length=0.004),
        origin=Origin(xyz=(spindle_x, spindle_y, tray_joint_z + 0.016)),
        material=stainless,
        name="top_clip",
    )

    # Door frame with a smoky window, handle, and a moving hinge knuckle.
    door = model.part("door")
    door_w = 0.400
    door_h = 0.270
    door_t = 0.020
    door_offset = 0.012
    rail = 0.034
    door.visual(
        Box((rail, door_t, door_h)),
        origin=Origin(xyz=(door_offset + rail / 2.0, 0.0, 0.0)),
        material=dark,
        name="hinge_rail",
    )
    door.visual(
        Box((rail, door_t, door_h)),
        origin=Origin(xyz=(door_offset + door_w - rail / 2.0, 0.0, 0.0)),
        material=dark,
        name="latch_rail",
    )
    door.visual(
        Box((door_w, door_t, rail)),
        origin=Origin(xyz=(door_offset + door_w / 2.0, 0.0, door_h / 2.0 - rail / 2.0)),
        material=dark,
        name="top_rail",
    )
    door.visual(
        Box((door_w, door_t, rail)),
        origin=Origin(xyz=(door_offset + door_w / 2.0, 0.0, -door_h / 2.0 + rail / 2.0)),
        material=dark,
        name="bottom_rail",
    )
    door.visual(
        Box((0.340, 0.006, 0.202)),
        origin=Origin(xyz=(door_offset + 0.195, -0.003, 0.0)),
        material=glass,
        name="window_glass",
    )
    for i, z in enumerate((-0.050, 0.0, 0.050)):
        door.visual(
            Box((0.300, 0.003, 0.004)),
            origin=Origin(xyz=(door_offset + 0.195, -0.007, z)),
            material=dark,
            name=f"mesh_line_{i}",
        )
    door.visual(
        Cylinder(radius=0.008, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=stainless,
        name="hinge_knuckle",
    )
    door.visual(
        Box((0.020, 0.010, 0.070)),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material=stainless,
        name="hinge_leaf",
    )
    handle_x = door_offset + door_w - 0.050
    door.visual(
        Box((0.016, 0.018, 0.170)),
        origin=Origin(xyz=(handle_x, -0.047, 0.0)),
        material=stainless,
        name="handle_grip",
    )
    for i, z in enumerate((-0.060, 0.060)):
        door.visual(
            Box((0.038, 0.045, 0.018)),
            origin=Origin(xyz=(handle_x, -0.025, z)),
            material=stainless,
            name=f"handle_standoff_{i}",
        )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.85),
    )

    # Annular glass tray: a central hole clears the spindle while the top clip captures it.
    tray_shape = (
        cq.Workplane("XY")
        .circle(0.145)
        .circle(0.022)
        .extrude(0.008)
        .union(cq.Workplane("XY").circle(0.145).circle(0.133).extrude(0.004).translate((0, 0, 0.008)))
        .union(cq.Workplane("XY").circle(0.045).circle(0.022).extrude(0.010).translate((0, 0, -0.006)))
    )
    turntable = model.part("turntable")
    turntable.visual(
        mesh_from_cadquery(tray_shape, "glass_tray"),
        origin=Origin(),
        material=tray_glass,
        name="glass_tray",
    )
    model.articulation(
        "turntable_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=turntable,
        origin=Origin(xyz=(spindle_x, spindle_y, tray_joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=3.0),
    )

    # Chunky rotary selector knob and its visible pointer mark.
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.064,
            0.030,
            body_style="skirted",
            top_diameter=0.052,
            skirt=KnobSkirt(0.074, 0.007, flare=0.05, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=20, depth=0.0015),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "selector_knob_mesh",
    )
    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        knob_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="selector_knob",
    )
    selector_knob.visual(
        Box((0.007, 0.003, 0.020)),
        origin=Origin(xyz=(0.0, -0.031, 0.022)),
        material=white,
        name="pointer_mark",
    )
    model.articulation(
        "knob_turn",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=selector_knob,
        origin=Origin(xyz=(0.210, front_y - 0.004, -0.084)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=4.0, lower=-2.4, upper=2.4),
    )

    # Separate short-travel push buttons on the control strip.
    button_positions = [
        (0.188, 0.094),
        (0.232, 0.094),
        (0.188, 0.050),
        (0.232, 0.050),
        (0.188, 0.006),
        (0.232, 0.006),
    ]
    for i, (x, z) in enumerate(button_positions):
        button = model.part(f"button_{i}")
        button.visual(
            Box((0.034, 0.014, 0.026)),
            origin=Origin(xyz=(0.0, -0.007, 0.0)),
            material=button_mat,
            name="button_cap",
        )
        model.articulation(
            f"button_press_{i}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x, front_y - 0.004, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.20, lower=0.0, upper=0.006),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    turntable = object_model.get_part("turntable")
    knob = object_model.get_part("selector_knob")
    door_hinge = object_model.get_articulation("door_hinge")
    turntable_spin = object_model.get_articulation("turntable_spin")
    knob_turn = object_model.get_articulation("knob_turn")

    # The door covers the cavity window area at rest and swings outward about the side hinge.
    ctx.expect_overlap(
        door,
        cabinet,
        axes="xz",
        elem_a="window_glass",
        elem_b="cavity_back",
        min_overlap=0.15,
        name="door window covers the cavity opening",
    )
    closed_box = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.25}):
        open_box = ctx.part_world_aabb(door)
    if closed_box is not None and open_box is not None:
        closed_center_y = (closed_box[0][1] + closed_box[1][1]) * 0.5
        open_center_y = (open_box[0][1] + open_box[1][1]) * 0.5
        ctx.check(
            "door opens outward from the front",
            open_center_y < closed_center_y - 0.050,
            details=f"closed_y={closed_center_y:.3f}, open_y={open_center_y:.3f}",
        )
    else:
        ctx.fail("door opens outward from the front", "door AABB was unavailable")

    # The annular tray remains centered and captured by the fixed spindle/clip while rotating.
    ctx.expect_within(
        turntable,
        cabinet,
        axes="xy",
        inner_elem="glass_tray",
        outer_elem="cavity_floor",
        margin=0.0,
        name="turntable sits within the oven floor",
    )
    ctx.expect_overlap(
        cabinet,
        turntable,
        axes="z",
        elem_a="spindle",
        elem_b="glass_tray",
        min_overlap=0.010,
        name="spindle passes through tray hub height",
    )
    ctx.expect_gap(
        cabinet,
        turntable,
        axis="z",
        positive_elem="top_clip",
        negative_elem="glass_tray",
        min_gap=0.001,
        max_gap=0.004,
        name="top clip captures tray above",
    )
    with ctx.pose({turntable_spin: math.pi / 2.0}):
        ctx.expect_within(
            turntable,
            cabinet,
            axes="xy",
            inner_elem="glass_tray",
            outer_elem="cavity_floor",
            margin=0.0,
            name="rotated turntable remains captured on spindle",
        )

    # The knob has an off-center pointer mark that visibly rotates with the part.
    pointer_rest = ctx.part_element_world_aabb(knob, elem="pointer_mark")
    with ctx.pose({knob_turn: 1.1}):
        pointer_turned = ctx.part_element_world_aabb(knob, elem="pointer_mark")
    if pointer_rest is not None and pointer_turned is not None:
        rest_x = (pointer_rest[0][0] + pointer_rest[1][0]) * 0.5
        turned_x = (pointer_turned[0][0] + pointer_turned[1][0]) * 0.5
        ctx.check(
            "selector pointer rotates around knob axis",
            abs(turned_x - rest_x) > 0.010,
            details=f"rest_x={rest_x:.3f}, turned_x={turned_x:.3f}",
        )
    else:
        ctx.fail("selector pointer rotates around knob axis", "pointer AABB was unavailable")

    for i in range(6):
        button = object_model.get_part(f"button_{i}")
        joint = object_model.get_articulation(f"button_press_{i}")
        ctx.allow_overlap(
            cabinet,
            button,
            elem_a="control_face",
            elem_b="button_cap",
            reason="The button cap intentionally retracts a few millimeters into its panel recess during the prismatic press.",
        )
        rest_box = ctx.part_world_aabb(button)
        with ctx.pose({joint: 0.006}):
            ctx.expect_gap(
                cabinet,
                button,
                axis="y",
                positive_elem="control_face",
                negative_elem="button_cap",
                max_penetration=0.0065,
                name=f"button_{i} has short recessed press travel",
            )
            pressed_box = ctx.part_world_aabb(button)
        if rest_box is not None and pressed_box is not None:
            rest_y = (rest_box[0][1] + rest_box[1][1]) * 0.5
            pressed_y = (pressed_box[0][1] + pressed_box[1][1]) * 0.5
            ctx.check(
                f"button_{i} translates inward",
                pressed_y > rest_y + 0.004,
                details=f"rest_y={rest_y:.3f}, pressed_y={pressed_y:.3f}",
            )
        else:
            ctx.fail(f"button_{i} translates inward", "button AABB was unavailable")

    return ctx.report()


object_model = build_object_model()
