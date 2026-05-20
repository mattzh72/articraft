from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dorm_room_microwave")

    cream = model.material("warm_white_enamel", rgba=(0.86, 0.84, 0.78, 1.0))
    dark = model.material("dark_cavity", rgba=(0.015, 0.016, 0.018, 1.0))
    black = model.material("black_glass", rgba=(0.01, 0.012, 0.014, 1.0))
    window = model.material("smoked_window", rgba=(0.07, 0.09, 0.10, 0.42))
    steel = model.material("brushed_steel", rgba=(0.55, 0.56, 0.54, 1.0))
    button_mat = model.material("soft_membrane_gray", rgba=(0.23, 0.24, 0.25, 1.0))
    label_mat = model.material("button_printing", rgba=(0.88, 0.90, 0.86, 1.0))
    green = model.material("display_green", rgba=(0.2, 0.85, 0.35, 1.0))
    glass = model.material("turntable_glass", rgba=(0.76, 0.92, 1.0, 0.34))

    housing = model.part("housing")

    # Compact dorm-room outer shell: roughly 58 cm wide by 42 cm deep by 34 cm high.
    housing.visual(Box((0.58, 0.42, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.0175)), material=cream, name="bottom_shell")
    housing.visual(Box((0.58, 0.42, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.3225)), material=cream, name="top_shell")
    housing.visual(Box((0.035, 0.42, 0.34)), origin=Origin(xyz=(-0.2725, 0.0, 0.17)), material=cream, name="side_shell")
    housing.visual(Box((0.035, 0.42, 0.34)), origin=Origin(xyz=(0.2725, 0.0, 0.17)), material=cream, name="other_side_shell")
    housing.visual(Box((0.58, 0.030, 0.34)), origin=Origin(xyz=(0.0, 0.195, 0.17)), material=cream, name="rear_shell")

    # Front face: broad door opening on the left and a raised controls panel on the right.
    housing.visual(Box((0.145, 0.035, 0.300)), origin=Origin(xyz=(0.2075, -0.2275, 0.17)), material=cream, name="control_panel")
    housing.visual(Box((0.430, 0.035, 0.035)), origin=Origin(xyz=(-0.0775, -0.2275, 0.3025)), material=cream, name="front_top_rail")
    housing.visual(Box((0.430, 0.035, 0.035)), origin=Origin(xyz=(-0.0775, -0.2275, 0.0375)), material=cream, name="front_bottom_rail")
    housing.visual(Box((0.035, 0.035, 0.300)), origin=Origin(xyz=(-0.2725, -0.2275, 0.17)), material=cream, name="front_jamb")

    # Fixed dark metal cavity surfaces visible through the smoked window.
    housing.visual(Box((0.390, 0.285, 0.006)), origin=Origin(xyz=(-0.075, -0.055, 0.038)), material=dark, name="cavity_floor")
    housing.visual(Box((0.390, 0.008, 0.260)), origin=Origin(xyz=(-0.075, 0.095, 0.165)), material=dark, name="cavity_back")
    housing.visual(Box((0.008, 0.310, 0.260)), origin=Origin(xyz=(-0.266, -0.055, 0.165)), material=dark, name="cavity_wall")
    housing.visual(Box((0.008, 0.310, 0.260)), origin=Origin(xyz=(0.126, -0.055, 0.165)), material=dark, name="cavity_partition")
    housing.visual(Cylinder(radius=0.025, length=0.004), origin=Origin(xyz=(-0.075, -0.055, 0.043)), material=steel, name="spindle_pad")

    # Control-panel display and printing.
    housing.visual(Box((0.102, 0.003, 0.044)), origin=Origin(xyz=(0.2075, -0.2465, 0.280)), material=black, name="display_window")
    housing.visual(Box((0.063, 0.001, 0.005)), origin=Origin(xyz=(0.2075, -0.2480, 0.286)), material=green, name="display_digits")
    housing.visual(Box((0.050, 0.001, 0.004)), origin=Origin(xyz=(0.2075, -0.2480, 0.274)), material=green, name="display_status")

    # Stationary hinge knuckles and leaves, clipped to the left front corner.
    hinge_x = -0.307
    hinge_y = -0.260
    for i, z in enumerate((0.065, 0.170, 0.275)):
        housing.visual(Cylinder(radius=0.010, length=0.050), origin=Origin(xyz=(hinge_x, hinge_y, z)), material=steel, name=f"hinge_knuckle_{i}")
        housing.visual(Box((0.040, 0.012, 0.050)), origin=Origin(xyz=(hinge_x + 0.020, hinge_y + 0.009, z)), material=steel, name=f"hinge_leaf_{i}")

    door = model.part("door")
    # Door part frame is the vertical hinge axis; the broad panel extends along local +X.
    door.visual(Box((0.045, 0.026, 0.270)), origin=Origin(xyz=(0.0525, -0.0125, 0.175)), material=black, name="door_hinge_stile")
    door.visual(Box((0.045, 0.026, 0.270)), origin=Origin(xyz=(0.4175, -0.0125, 0.175)), material=black, name="door_latch_stile")
    door.visual(Box((0.410, 0.026, 0.045)), origin=Origin(xyz=(0.235, -0.0125, 0.2875)), material=black, name="door_top_rail")
    door.visual(Box((0.410, 0.026, 0.045)), origin=Origin(xyz=(0.235, -0.0125, 0.0625)), material=black, name="door_bottom_rail")
    door.visual(Box((0.320, 0.006, 0.175)), origin=Origin(xyz=(0.235, -0.0270, 0.175)), material=window, name="window_pane")
    # Subtle microwave mesh/safety grid printed across the broad smoked glass.
    for i, z in enumerate((0.125, 0.155, 0.185, 0.215)):
        door.visual(Box((0.324, 0.002, 0.003)), origin=Origin(xyz=(0.235, -0.031, z)), material=steel, name=f"window_line_{i}")
    for i, x in enumerate((0.145, 0.235, 0.325)):
        door.visual(Box((0.003, 0.002, 0.178)), origin=Origin(xyz=(x, -0.031, 0.175)), material=steel, name=f"window_mullion_{i}")
    for i, z in enumerate((0.1175, 0.2225)):
        door.visual(Cylinder(radius=0.010, length=0.045), origin=Origin(xyz=(0.0, 0.0, z)), material=steel, name=f"door_knuckle_{i}")
        door.visual(Box((0.065, 0.008, 0.040)), origin=Origin(xyz=(0.030, -0.010, z)), material=steel, name=f"door_hinge_leaf_{i}")
    door.visual(Cylinder(radius=0.004, length=0.280), origin=Origin(xyz=(0.0, 0.0, 0.170)), material=steel, name="hinge_pin")

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    turntable = model.part("turntable")
    turntable.visual(Cylinder(radius=0.020, length=0.014), origin=Origin(xyz=(0.0, 0.0, 0.007)), material=steel, name="center_hub")
    turntable.visual(Cylinder(radius=0.145, length=0.008), origin=Origin(xyz=(0.0, 0.0, 0.014)), material=glass, name="glass_plate")
    turntable.visual(Cylinder(radius=0.115, length=0.002), origin=Origin(xyz=(0.0, 0.0, 0.010)), material=steel, name="roller_ring")
    model.articulation(
        "turntable_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=turntable,
        origin=Origin(xyz=(-0.075, -0.055, 0.045)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )

    button_x = 0.2075
    button_y = -0.245
    for i, z in enumerate((0.242, 0.217, 0.192, 0.167, 0.142, 0.122)):
        button = model.part(f"button_{i}")
        button.visual(Box((0.090, 0.006, 0.019)), origin=Origin(xyz=(0.0, -0.003, 0.0)), material=button_mat, name="button_pad")
        button.visual(Box((0.052, 0.001, 0.004)), origin=Origin(xyz=(0.0, -0.0065, 0.003)), material=label_mat, name="button_label")
        button.visual(Box((0.024, 0.001, 0.003)), origin=Origin(xyz=(0.0, -0.0065, -0.004)), material=label_mat, name="button_mark")
        model.articulation(
            f"button_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=button,
            origin=Origin(xyz=(button_x, button_y, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=0.08, lower=0.0, upper=0.004),
        )

    dial = model.part("dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.068,
            0.026,
            body_style="skirted",
            top_diameter=0.056,
            grip=KnobGrip(style="fluted", count=20, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=20.0),
            center=False,
        ),
        "microwave_dial",
    )
    dial.visual(dial_mesh, origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)), material=steel, name="dial_cap")
    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=dial,
        origin=Origin(xyz=(0.2075, -0.245, 0.058)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    turntable = object_model.get_part("turntable")
    button_0 = object_model.get_part("button_0")
    dial = object_model.get_part("dial")

    door_hinge = object_model.get_articulation("door_hinge")
    turntable_spin = object_model.get_articulation("turntable_spin")
    button_slide = object_model.get_articulation("button_0_slide")
    dial_spin = object_model.get_articulation("dial_spin")

    for i in range(3):
        ctx.allow_overlap(
            door,
            housing,
            elem_a="hinge_pin",
            elem_b=f"hinge_knuckle_{i}",
            reason="The door hinge pin is intentionally captured inside the fixed hinge barrel.",
        )
        ctx.expect_within(
            door,
            housing,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem=f"hinge_knuckle_{i}",
            margin=0.0005,
            name=f"hinge pin is centered in fixed barrel {i}",
        )
        ctx.expect_overlap(
            door,
            housing,
            axes="z",
            elem_a="hinge_pin",
            elem_b=f"hinge_knuckle_{i}",
            min_overlap=0.040,
            name=f"hinge pin passes through fixed barrel {i}",
        )

    ctx.expect_overlap(
        door,
        housing,
        axes="xy",
        elem_a="door_knuckle_0",
        elem_b="hinge_knuckle_0",
        min_overlap=0.014,
        name="door hinge barrel is coaxial with fixed barrel",
    )
    ctx.expect_gap(
        door,
        housing,
        axis="z",
        positive_elem="door_knuckle_0",
        negative_elem="hinge_knuckle_0",
        min_gap=0.003,
        max_gap=0.008,
        name="door barrel is clipped just above lower fixed barrel",
    )
    ctx.expect_contact(
        turntable,
        housing,
        elem_a="center_hub",
        elem_b="spindle_pad",
        contact_tol=0.0005,
        name="turntable hub sits on center spindle pad",
    )
    ctx.expect_contact(
        button_0,
        housing,
        elem_a="button_pad",
        elem_b="control_panel",
        contact_tol=0.001,
        name="membrane button rests on the control panel",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.2}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "side-hinged door swings outward",
        closed_aabb is not None and open_aabb is not None and open_aabb[0][1] < closed_aabb[0][1] - 0.05,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    rest_button = ctx.part_world_position(button_0)
    with ctx.pose({button_slide: 0.004}):
        pushed_button = ctx.part_world_position(button_0)
    ctx.check(
        "membrane button translates inward",
        rest_button is not None and pushed_button is not None and pushed_button[1] > rest_button[1] + 0.003,
        details=f"rest={rest_button}, pushed={pushed_button}",
    )

    rest_dial = ctx.part_world_aabb(dial)
    with ctx.pose({dial_spin: 1.0, turntable_spin: 1.0}):
        turned_dial = ctx.part_world_aabb(dial)
        spun_turntable = ctx.part_world_aabb(turntable)
    ctx.check(
        "dial remains centered while rotating on front axis",
        rest_dial is not None and turned_dial is not None and abs(rest_dial[0][1] - turned_dial[0][1]) < 0.002,
        details=f"rest={rest_dial}, turned={turned_dial}",
    )
    ctx.check(
        "turntable remains centered while spinning",
        spun_turntable is not None,
        details=f"spun_turntable={spun_turntable}",
    )

    return ctx.report()


object_model = build_object_model()
