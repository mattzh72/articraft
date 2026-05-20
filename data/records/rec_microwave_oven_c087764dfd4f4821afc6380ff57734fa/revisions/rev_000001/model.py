from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
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
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="built_in_microwave")

    # The model uses a conventional appliance frame:
    # X = width, Y = depth (negative Y is the front), Z = height.
    steel = model.material("brushed_stainless", rgba=(0.62, 0.63, 0.60, 1.0))
    dark = model.material("black_enamel", rgba=(0.02, 0.022, 0.025, 1.0))
    panel_black = model.material("gloss_black_panel", rgba=(0.01, 0.012, 0.016, 1.0))
    glass = model.material("smoked_glass", rgba=(0.08, 0.12, 0.16, 0.38))
    button_mat = model.material("soft_touch_buttons", rgba=(0.16, 0.17, 0.18, 1.0))
    knob_mat = model.material("satin_black_knob", rgba=(0.03, 0.032, 0.035, 1.0))
    white = model.material("white_markings", rgba=(0.90, 0.92, 0.88, 1.0))

    housing = model.part("housing")

    def add_box(
        name: str,
        size: tuple[float, float, float],
        xyz: tuple[float, float, float],
        material: Material,
    ) -> None:
        housing.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)

    # Wide built-in metal cabinet, open at the front where the cavity and
    # right-hand control area are visible.
    add_box("top_shell", (0.62, 0.44, 0.035), (0.0, 0.0, 0.1725), steel)
    add_box("bottom_shell", (0.62, 0.44, 0.035), (0.0, 0.0, -0.1725), steel)
    add_box("side_shell_0", (0.035, 0.44, 0.38), (-0.2925, 0.0, 0.0), steel)
    add_box("side_shell_1", (0.035, 0.44, 0.38), (0.2925, 0.0, 0.0), steel)
    add_box("back_shell", (0.62, 0.035, 0.38), (0.0, 0.2025, 0.0), steel)

    # Cavity liner and front frame.  The dark liner leaves a real hollow volume
    # behind the glass door instead of filling the oven with a solid block.
    add_box("cavity_back", (0.42, 0.012, 0.26), (-0.065, 0.162, 0.0), dark)
    add_box("cavity_floor", (0.43, 0.35, 0.018), (-0.065, -0.025, -0.144), dark)
    add_box("cavity_ceiling", (0.43, 0.35, 0.018), (-0.065, -0.025, 0.144), dark)
    add_box("cavity_wall_0", (0.018, 0.35, 0.27), (-0.284, -0.025, 0.0), dark)
    add_box("cavity_wall_1", (0.018, 0.35, 0.27), (0.154, -0.025, 0.0), dark)
    add_box("cavity_top_trim", (0.47, 0.014, 0.030), (-0.065, -0.226, 0.151), steel)
    add_box("cavity_bottom_trim", (0.47, 0.014, 0.030), (-0.065, -0.226, -0.151), steel)
    add_box("cavity_side_trim_0", (0.030, 0.014, 0.32), (-0.292, -0.226, 0.0), steel)
    add_box("cavity_side_trim_1", (0.030, 0.014, 0.32), (0.162, -0.226, 0.0), steel)

    # Fixed control fascia on the right-hand side of the front.  Static display,
    # labels, and vents make it read as a built-in appliance panel; individual
    # buttons and the rotary control are separate articulated children.
    add_box("control_panel", (0.125, 0.027, 0.32), (0.231, -0.2325, 0.0), panel_black)
    add_box("display_window", (0.086, 0.003, 0.036), (0.231, -0.2475, 0.142), glass)
    add_box("display_glow", (0.050, 0.0015, 0.010), (0.218, -0.249, 0.143), white)
    for i, z in enumerate((-0.126, -0.138, -0.150)):
        add_box(f"vent_slot_{i}", (0.082, 0.003, 0.004), (0.231, -0.2475, z), dark)

    # Drive spindle for the rotating glass turntable.
    housing.visual(
        Cylinder(radius=0.018, length=0.025),
        origin=Origin(xyz=(-0.065, -0.035, -0.1325)),
        material=dark,
        name="drive_spindle",
    )

    # Side-hinged door.  Its frame origin is the hinge line so the child frame
    # coincides with the vertical revolute joint axis at q=0.
    door = model.part("door")
    door_frame = BezelGeometry(
        (0.315, 0.205),
        (0.430, 0.300),
        0.030,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.015,
        outer_corner_radius=0.018,
        face=BezelFace(style="radiused_step", front_lip=0.004, fillet=0.002),
    )
    door.visual(
        mesh_from_geometry(door_frame, "door_frame"),
        # Bezel local Z becomes appliance -Y, so the frame thickness is the
        # door depth while local Y becomes the vertical height.
        origin=Origin(xyz=(0.222, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="door_frame",
    )
    door.visual(
        Box((0.335, 0.006, 0.222)),
        origin=Origin(xyz=(0.222, -0.004, 0.0)),
        material=glass,
        name="door_glass",
    )
    door.visual(
        Box((0.026, 0.018, 0.220)),
        origin=Origin(xyz=(0.405, -0.026, 0.0)),
        material=panel_black,
        name="door_handle",
    )
    door.visual(
        Cylinder(radius=0.011, length=0.312),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    door.visual(
        Box((0.030, 0.012, 0.265)),
        origin=Origin(xyz=(0.017, -0.004, 0.0)),
        material=steel,
        name="hinge_leaf",
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(-0.285, -0.248, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    # Pop-out rotary control knob on the front panel.  The mesh's local Z axis is
    # rotated to front-to-back, so the revolute axis is the appliance Y axis.
    knob = model.part("knob")
    knob_geom = KnobGeometry(
        0.055,
        0.034,
        body_style="skirted",
        top_diameter=0.042,
        skirt=KnobSkirt(0.062, 0.006, flare=0.04, chamfer=0.001),
        grip=KnobGrip(style="fluted", count=20, depth=0.0013),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    knob.visual(
        mesh_from_geometry(knob_geom, "control_knob"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_mat,
        name="knob_cap",
    )
    model.articulation(
        "knob_turn",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=knob,
        origin=Origin(xyz=(0.205, -0.246, 0.075)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=6.0, lower=-2.4, upper=2.4),
    )

    # Five distinct short-travel push buttons in a vertical row beside the knob.
    button_zs = (0.088, 0.050, 0.012, -0.026, -0.064)
    for i, z in enumerate(button_zs):
        button = model.part(f"button_{i}")
        button.visual(
            Box((0.034, 0.011, 0.019)),
            origin=Origin(xyz=(0.0, -0.0055, 0.0)),
            material=button_mat,
            name="button_cap",
        )
        button.visual(
            Box((0.018, 0.003, 0.004)),
            origin=Origin(xyz=(-0.002, -0.0118, 0.0)),
            material=white,
            name="button_mark",
        )
        model.articulation(
            f"button_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=button,
            origin=Origin(xyz=(0.268, -0.246, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.20, lower=0.0, upper=0.008),
        )

    # Glass turntable on a vertical continuous axis inside the cavity.  The small
    # off-center mark makes the rotation visibly legible.
    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.145, length=0.010),
        origin=Origin(),
        material=glass,
        name="glass_disk",
    )
    turntable.visual(
        Cylinder(radius=0.022, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=dark,
        name="center_hub",
    )
    turntable.visual(
        Box((0.075, 0.012, 0.002)),
        origin=Origin(xyz=(0.061, 0.0, 0.006)),
        material=white,
        name="rotation_mark",
    )
    model.articulation(
        "turntable_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=turntable,
        origin=Origin(xyz=(-0.065, -0.035, -0.115)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    knob = object_model.get_part("knob")
    turntable = object_model.get_part("turntable")
    door_hinge = object_model.get_articulation("door_hinge")
    knob_turn = object_model.get_articulation("knob_turn")
    turntable_spin = object_model.get_articulation("turntable_spin")

    def close_axis(a: tuple[float, float, float], b: tuple[float, float, float]) -> bool:
        return all(abs(a[i] - b[i]) < 1e-6 for i in range(3))

    def aabb_center_y(aabb) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][1] + aabb[1][1])

    def aabb_center_x(aabb) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][0] + aabb[1][0])

    ctx.check(
        "door uses a vertical side hinge",
        door_hinge.articulation_type == ArticulationType.REVOLUTE
        and close_axis(tuple(door_hinge.axis), (0.0, 0.0, -1.0))
        and door_hinge.motion_limits is not None
        and door_hinge.motion_limits.upper is not None
        and door_hinge.motion_limits.upper > 1.4,
        details=f"type={door_hinge.articulation_type}, axis={door_hinge.axis}, limits={door_hinge.motion_limits}",
    )
    ctx.expect_gap(
        housing,
        door,
        axis="y",
        positive_elem="cavity_top_trim",
        negative_elem="door_frame",
        max_gap=0.002,
        max_penetration=0.001,
        name="closed door seats against front trim",
    )

    closed_handle = ctx.part_element_world_aabb(door, elem="door_handle")
    with ctx.pose({door_hinge: 1.20}):
        open_handle = ctx.part_element_world_aabb(door, elem="door_handle")
    ctx.check(
        "door swings outward from the cavity",
        closed_handle is not None
        and open_handle is not None
        and open_handle[0][1] < closed_handle[0][1] - 0.12,
        details=f"closed_handle={closed_handle}, open_handle={open_handle}",
    )

    ctx.check(
        "knob rotates front-to-back",
        knob_turn.articulation_type == ArticulationType.REVOLUTE
        and close_axis(tuple(knob_turn.axis), (0.0, 1.0, 0.0))
        and knob_turn.motion_limits is not None
        and knob_turn.motion_limits.lower < 0.0
        and knob_turn.motion_limits.upper > 0.0,
        details=f"type={knob_turn.articulation_type}, axis={knob_turn.axis}, limits={knob_turn.motion_limits}",
    )
    ctx.expect_gap(
        housing,
        knob,
        axis="y",
        positive_elem="control_panel",
        negative_elem="knob_cap",
        max_gap=0.002,
        max_penetration=0.001,
        name="knob sits proud on the control panel",
    )

    buttons = [object_model.get_part(f"button_{i}") for i in range(5)]
    button_joints = [object_model.get_articulation(f"button_{i}_slide") for i in range(5)]
    ctx.check(
        "five buttons are individual plungers",
        all(j.articulation_type == ArticulationType.PRISMATIC for j in button_joints)
        and all(close_axis(tuple(j.axis), (0.0, 1.0, 0.0)) for j in button_joints),
        details=", ".join(f"{j.name}:{j.articulation_type}/{j.axis}" for j in button_joints),
    )
    ctx.expect_gap(
        housing,
        buttons[0],
        axis="y",
        positive_elem="control_panel",
        negative_elem="button_cap",
        max_gap=0.001,
        max_penetration=0.001,
        name="button caps start flush with panel face",
    )
    rest_button = ctx.part_world_position(buttons[0])
    with ctx.pose({button_joints[0]: 0.008}):
        pressed_button = ctx.part_world_position(buttons[0])
    ctx.check(
        "button plunger travels inward",
        rest_button is not None
        and pressed_button is not None
        and pressed_button[1] > rest_button[1] + 0.006,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    ctx.check(
        "turntable has a vertical spin axis",
        turntable_spin.articulation_type == ArticulationType.CONTINUOUS
        and close_axis(tuple(turntable_spin.axis), (0.0, 0.0, 1.0)),
        details=f"type={turntable_spin.articulation_type}, axis={turntable_spin.axis}",
    )
    ctx.expect_contact(
        turntable,
        housing,
        elem_a="glass_disk",
        elem_b="drive_spindle",
        contact_tol=0.001,
        name="turntable rests on central drive spindle",
    )
    rest_mark = ctx.part_element_world_aabb(turntable, elem="rotation_mark")
    with ctx.pose({turntable_spin: math.pi / 2.0}):
        spun_mark = ctx.part_element_world_aabb(turntable, elem="rotation_mark")
    ctx.check(
        "turntable marker rotates with the glass plate",
        aabb_center_y(rest_mark) is not None
        and aabb_center_y(spun_mark) is not None
        and aabb_center_x(rest_mark) is not None
        and aabb_center_x(spun_mark) is not None
        and aabb_center_y(spun_mark) > aabb_center_y(rest_mark) + 0.04
        and aabb_center_x(spun_mark) < aabb_center_x(rest_mark) - 0.04,
        details=f"rest_mark={rest_mark}, spun_mark={spun_mark}",
    )

    return ctx.report()


object_model = build_object_model()
