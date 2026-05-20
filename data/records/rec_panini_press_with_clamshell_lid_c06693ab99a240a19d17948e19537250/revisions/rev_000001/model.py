from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


Y_AXIS_CYLINDER = Origin(rpy=(-1.5707963267948966, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_contact_grill")

    stainless = model.material("stainless", rgba=(0.70, 0.72, 0.74, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    cast_plate = model.material("cast_plate", rgba=(0.11, 0.11, 0.12, 1.0))
    handle_black = model.material("handle_black", rgba=(0.08, 0.08, 0.08, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    lower_frame = model.part("lower_frame")
    for x_center in (0.055, 0.365):
        for y_center in (-0.145, 0.145):
            lower_frame.visual(
                Box((0.042, 0.030, 0.012)),
                origin=Origin(xyz=(x_center, y_center, 0.006)),
                material=rubber,
                name=f"foot_{'front' if x_center > 0.2 else 'rear'}_{'right' if y_center > 0 else 'left'}",
            )

    lower_frame.visual(
        Box((0.338, 0.294, 0.012)),
        origin=Origin(xyz=(0.196, 0.0, 0.018)),
        material=dark_steel,
        name="base_pan",
    )
    lower_frame.visual(
        Box((0.420, 0.022, 0.125)),
        origin=Origin(xyz=(0.210, -0.179, 0.0745)),
        material=stainless,
        name="left_shell",
    )
    lower_frame.visual(
        Box((0.420, 0.022, 0.125)),
        origin=Origin(xyz=(0.210, 0.179, 0.0745)),
        material=stainless,
        name="right_shell",
    )
    lower_frame.visual(
        Box((0.026, 0.336, 0.110)),
        origin=Origin(xyz=(0.407, 0.0, 0.067)),
        material=stainless,
        name="front_shell",
    )
    lower_frame.visual(
        Box((0.030, 0.336, 0.108)),
        origin=Origin(xyz=(0.015, 0.0, 0.066)),
        material=stainless,
        name="rear_shell",
    )
    lower_frame.visual(
        Box((0.308, 0.266, 0.026)),
        origin=Origin(xyz=(0.201, 0.0, 0.145)),
        material=cast_plate,
        name="lower_plate",
    )
    lower_frame.visual(
        Box((0.308, 0.040, 0.018)),
        origin=Origin(xyz=(0.201, -0.148, 0.129)),
        material=dark_steel,
        name="plate_rail_left",
    )
    lower_frame.visual(
        Box((0.308, 0.040, 0.018)),
        origin=Origin(xyz=(0.201, 0.148, 0.129)),
        material=dark_steel,
        name="plate_rail_right",
    )
    lower_frame.visual(
        Box((0.034, 0.266, 0.018)),
        origin=Origin(xyz=(0.032, 0.0, 0.129)),
        material=dark_steel,
        name="plate_rail_rear",
    )
    lower_frame.visual(
        Box((0.060, 0.250, 0.028)),
        origin=Origin(xyz=(0.365, 0.0, 0.126)),
        material=dark_steel,
        name="grease_trough",
    )
    lower_frame.visual(
        Box((0.050, 0.038, 0.120)),
        origin=Origin(xyz=(0.040, -0.160, 0.177)),
        material=stainless,
        name="hinge_tower_left",
    )
    lower_frame.visual(
        Box((0.050, 0.038, 0.120)),
        origin=Origin(xyz=(0.040, 0.160, 0.177)),
        material=stainless,
        name="hinge_tower_right",
    )
    lower_frame.visual(
        Box((0.026, 0.290, 0.040)),
        origin=Origin(xyz=(0.026, 0.0, 0.160)),
        material=dark_steel,
        name="hinge_crossmember",
    )
    lower_frame.visual(
        Cylinder(radius=0.015, length=0.044),
        origin=Origin(xyz=(0.028, -0.140, 0.222), rpy=Y_AXIS_CYLINDER.rpy),
        material=dark_steel,
        name="hinge_support_left",
    )
    lower_frame.visual(
        Cylinder(radius=0.015, length=0.044),
        origin=Origin(xyz=(0.028, 0.140, 0.222), rpy=Y_AXIS_CYLINDER.rpy),
        material=dark_steel,
        name="hinge_support_right",
    )

    top_platen = model.part("top_platen")
    top_platen.visual(
        Cylinder(radius=0.013, length=0.236),
        origin=Origin(rpy=Y_AXIS_CYLINDER.rpy),
        material=dark_steel,
        name="hinge_bar",
    )
    top_platen.visual(
        Box((0.110, 0.220, 0.032)),
        origin=Origin(xyz=(0.065, 0.0, 0.000)),
        material=dark_steel,
        name="rear_spine",
    )
    top_platen.visual(
        Box((0.220, 0.240, 0.028)),
        origin=Origin(xyz=(0.170, 0.0, -0.026)),
        material=dark_steel,
        name="plate_bridge",
    )
    top_platen.visual(
        Box((0.304, 0.266, 0.024)),
        origin=Origin(xyz=(0.170, 0.0, -0.050)),
        material=cast_plate,
        name="upper_plate",
    )
    top_platen.visual(
        Box((0.220, 0.280, 0.056)),
        origin=Origin(xyz=(0.195, 0.0, 0.004)),
        material=stainless,
        name="top_shell",
    )
    top_platen.visual(
        Box((0.060, 0.296, 0.040)),
        origin=Origin(xyz=(0.335, 0.0, -0.006)),
        material=stainless,
        name="front_nose",
    )
    top_platen.visual(
        Box((0.030, 0.022, 0.048)),
        origin=Origin(xyz=(0.350, -0.102, -0.030)),
        material=dark_steel,
        name="handle_post_left",
    )
    top_platen.visual(
        Box((0.030, 0.022, 0.048)),
        origin=Origin(xyz=(0.350, 0.102, -0.030)),
        material=dark_steel,
        name="handle_post_right",
    )
    top_platen.visual(
        Cylinder(radius=0.012, length=0.204),
        origin=Origin(xyz=(0.350, 0.0, -0.050), rpy=Y_AXIS_CYLINDER.rpy),
        material=handle_black,
        name="handle_bar",
    )
    top_platen.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(0.235, 0.142, -0.002), rpy=Y_AXIS_CYLINDER.rpy),
        material=dark_steel,
        name="knob_mount",
    )

    side_knob = model.part("side_knob")
    side_knob.visual(
        Cylinder(radius=0.004, length=0.006),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=Y_AXIS_CYLINDER.rpy),
        material=dark_steel,
        name="knob_shaft",
    )
    side_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.030,
                0.016,
                body_style="skirted",
                top_diameter=0.024,
                skirt=KnobSkirt(0.034, 0.004, flare=0.06),
                grip=KnobGrip(style="fluted", count=12, depth=0.0008),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0005),
            ),
            "side_knob_shell",
        ),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=Y_AXIS_CYLINDER.rpy),
        material=handle_black,
        name="knob_shell",
    )

    lower_frame.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(0.265, 0.193, 0.088), rpy=Y_AXIS_CYLINDER.rpy),
        material=dark_steel,
        name="dial_mount",
    )

    side_dial = model.part("side_dial")
    side_dial.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=Y_AXIS_CYLINDER.rpy),
        material=dark_steel,
        name="dial_shaft",
    )
    side_dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.056,
                0.018,
                body_style="skirted",
                top_diameter=0.046,
                skirt=KnobSkirt(0.064, 0.004, flare=0.05),
                grip=KnobGrip(style="fluted", count=18, depth=0.0009),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006),
            ),
            "side_dial_shell",
        ),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=Y_AXIS_CYLINDER.rpy),
        material=handle_black,
        name="dial_shell",
    )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_frame,
        child=top_platen,
        origin=Origin(xyz=(0.028, 0.0, 0.222)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.6, lower=0.0, upper=1.65),
    )
    model.articulation(
        "knob_spin",
        ArticulationType.CONTINUOUS,
        parent=top_platen,
        child=side_knob,
        origin=Origin(xyz=(0.235, 0.151, -0.002)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=lower_frame,
        child=side_dial,
        origin=Origin(xyz=(0.265, 0.206, 0.088)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_frame = object_model.get_part("lower_frame")
    top_platen = object_model.get_part("top_platen")
    side_knob = object_model.get_part("side_knob")
    side_dial = object_model.get_part("side_dial")
    rear_hinge = object_model.get_articulation("rear_hinge")
    knob_spin = object_model.get_articulation("knob_spin")
    dial_spin = object_model.get_articulation("dial_spin")

    limits = rear_hinge.motion_limits
    upper = 1.35 if limits is None or limits.upper is None else limits.upper

    with ctx.pose({rear_hinge: 0.0}):
        ctx.expect_gap(
            top_platen,
            lower_frame,
            axis="z",
            positive_elem="upper_plate",
            negative_elem="lower_plate",
            min_gap=0.001,
            max_gap=0.006,
            name="upper plate sits just above lower plate when closed",
        )
        ctx.expect_overlap(
            top_platen,
            lower_frame,
            axes="xy",
            elem_a="upper_plate",
            elem_b="lower_plate",
            min_overlap=0.240,
            name="upper and lower plates overlap in the cooking footprint",
        )
        ctx.expect_contact(
            top_platen,
            lower_frame,
            elem_a="hinge_bar",
            elem_b="hinge_support_right",
            name="rear hinge bar stays seated in the right hinge support",
        )
        ctx.expect_contact(
            side_knob,
            top_platen,
            elem_a="knob_shaft",
            elem_b="knob_mount",
            name="side browning knob stays mounted on its shaft boss",
        )
        ctx.expect_gap(
            side_knob,
            top_platen,
            axis="y",
            positive_elem="knob_shell",
            negative_elem="top_shell",
            min_gap=0.003,
            max_gap=0.010,
            name="side knob cap stays visually separate from the top housing shell",
        )
        ctx.expect_contact(
            side_dial,
            lower_frame,
            elem_a="dial_shaft",
            elem_b="dial_mount",
            name="side browning dial stays mounted on its wall boss",
        )
        ctx.expect_gap(
            side_dial,
            lower_frame,
            axis="y",
            positive_elem="dial_shell",
            negative_elem="right_shell",
            min_gap=0.006,
            max_gap=0.014,
            name="right side dial stays clearly proud of the housing wall",
        )

    closed_handle = None
    open_handle = None
    with ctx.pose({rear_hinge: 0.0}):
        closed_handle = ctx.part_element_world_aabb(top_platen, elem="handle_bar")
    with ctx.pose({rear_hinge: upper}):
        open_handle = ctx.part_element_world_aabb(top_platen, elem="handle_bar")
        ctx.expect_gap(
            top_platen,
            lower_frame,
            axis="z",
            positive_elem="upper_plate",
            negative_elem="lower_plate",
            min_gap=0.080,
            name="opened top platen lifts clear of the lower plate",
        )

    handle_closed_z = None
    handle_open_z = None
    if closed_handle is not None:
        handle_closed_z = 0.5 * (float(closed_handle[0][2]) + float(closed_handle[1][2]))
    if open_handle is not None:
        handle_open_z = 0.5 * (float(open_handle[0][2]) + float(open_handle[1][2]))
    ctx.check(
        "front handle rises when the top platen opens",
        handle_closed_z is not None
        and handle_open_z is not None
        and handle_open_z > handle_closed_z + 0.12,
        details=f"closed_z={handle_closed_z}, open_z={handle_open_z}",
    )

    knob_rest = None
    knob_spun = None
    with ctx.pose({rear_hinge: 0.0, knob_spin: 0.0}):
        knob_rest = ctx.part_world_position(side_knob)
    with ctx.pose({rear_hinge: 0.0, knob_spin: 2.2}):
        knob_spun = ctx.part_world_position(side_knob)
    ctx.check(
        "side knob rotates about a fixed shaft center",
        knob_rest is not None
        and knob_spun is not None
        and max(abs(knob_rest[i] - knob_spun[i]) for i in range(3)) <= 1e-6,
        details=f"rest={knob_rest}, spun={knob_spun}",
    )

    dial_rest = None
    dial_spun = None
    with ctx.pose({dial_spin: 0.0}):
        dial_rest = ctx.part_world_position(side_dial)
    with ctx.pose({dial_spin: 2.6}):
        dial_spun = ctx.part_world_position(side_dial)
    ctx.check(
        "side dial rotates about a fixed center axis",
        dial_rest is not None
        and dial_spun is not None
        and max(abs(dial_rest[i] - dial_spun[i]) for i in range(3)) <= 1e-6,
        details=f"rest={dial_rest}, spun={dial_spun}",
    )

    return ctx.report()


object_model = build_object_model()
