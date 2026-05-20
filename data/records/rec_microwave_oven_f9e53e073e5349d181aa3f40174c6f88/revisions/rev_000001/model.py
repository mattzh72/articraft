from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="over_range_drop_down_microwave")

    stainless = Material("brushed_stainless", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_trim = Material("dark_glass_and_trim", rgba=(0.02, 0.025, 0.028, 1.0))
    smoked_glass = Material("smoked_window", rgba=(0.03, 0.05, 0.06, 0.55))
    black_rubber = Material("black_button_rubber", rgba=(0.01, 0.012, 0.014, 1.0))
    screen_green = Material("green_display", rgba=(0.15, 0.9, 0.45, 1.0))
    warm_wood = Material("cabinet_wood", rgba=(0.62, 0.45, 0.28, 1.0))

    body = model.part("body")

    # Cabinet and over-range microwave carcass.  The body is one fixed root part:
    # a cabinet slab sits directly on the metal case to make the under-cabinet
    # mounting relationship visible.
    body.visual(
        Box((0.90, 0.50, 0.080)),
        origin=Origin(xyz=(0.0, 0.02, 0.379)),
        material=warm_wood,
        name="cabinet_slab",
    )
    body.visual(
        Box((0.78, 0.42, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.327)),
        material=stainless,
        name="top_panel",
    )
    body.visual(
        Box((0.78, 0.42, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=stainless,
        name="bottom_panel",
    )
    body.visual(
        Box((0.026, 0.42, 0.340)),
        origin=Origin(xyz=(-0.377, 0.0, 0.170)),
        material=stainless,
        name="side_panel_0",
    )
    body.visual(
        Box((0.026, 0.42, 0.340)),
        origin=Origin(xyz=(0.377, 0.0, 0.170)),
        material=stainless,
        name="side_panel_1",
    )
    body.visual(
        Box((0.78, 0.024, 0.340)),
        origin=Origin(xyz=(0.0, 0.204, 0.170)),
        material=stainless,
        name="rear_panel",
    )

    # Front fascia: a control strip above a recessed dark cooking opening and a
    # lower sill carrying the drop-down hinge.
    body.visual(
        Box((0.740, 0.024, 0.086)),
        origin=Origin(xyz=(0.0, -0.208, 0.287)),
        material=dark_trim,
        name="control_strip",
    )
    body.visual(
        Box((0.030, 0.024, 0.196)),
        origin=Origin(xyz=(-0.365, -0.208, 0.143)),
        material=stainless,
        name="front_jamb_0",
    )
    body.visual(
        Box((0.030, 0.024, 0.196)),
        origin=Origin(xyz=(0.365, -0.208, 0.143)),
        material=stainless,
        name="front_jamb_1",
    )
    body.visual(
        Box((0.740, 0.024, 0.040)),
        origin=Origin(xyz=(0.0, -0.208, 0.030)),
        material=stainless,
        name="lower_sill",
    )
    body.visual(
        Box((0.730, 0.006, 0.170)),
        origin=Origin(xyz=(0.0, -0.212, 0.148)),
        material=dark_trim,
        name="cavity_shadow",
    )

    # Display, vents, and underside range features are seated into the fixed
    # fascia/underside so the appliance reads as an over-range microwave.
    body.visual(
        Box((0.180, 0.006, 0.040)),
        origin=Origin(xyz=(-0.260, -0.222, 0.296)),
        material=screen_green,
        name="clock_display",
    )
    for i in range(7):
        body.visual(
            Box((0.034, 0.006, 0.010)),
            origin=Origin(xyz=(-0.032 + 0.046 * i, -0.222, 0.318)),
            material=stainless,
            name=f"vent_slot_{i}",
        )
    body.visual(
        Box((0.285, 0.130, 0.004)),
        origin=Origin(xyz=(-0.190, -0.050, -0.001)),
        material=dark_trim,
        name="grease_filter_0",
    )
    body.visual(
        Box((0.285, 0.130, 0.004)),
        origin=Origin(xyz=(0.190, -0.050, -0.001)),
        material=dark_trim,
        name="grease_filter_1",
    )

    # Exposed hinge knuckles at the lower front edge.  The root-side knuckles are
    # split at the ends, leaving a central barrel on the door.
    for x in (-0.255, 0.255):
        body.visual(
            Box((0.090, 0.028, 0.018)),
            origin=Origin(xyz=(x, -0.226, 0.054)),
            material=stainless,
            name=f"hinge_bracket_{0 if x < 0 else 1}",
        )
        body.visual(
            Cylinder(radius=0.012, length=0.090),
            origin=Origin(xyz=(x, -0.240, 0.060), rpy=(0.0, pi / 2.0, 0.0)),
            material=stainless,
            name=f"body_hinge_knuckle_{0 if x < 0 else 1}",
        )

    door = model.part("door")
    # The door frame is authored in a hinge-line child frame.  At q=0 it extends
    # upward from the lower hinge; positive rotation about +X drops the top edge
    # outward and downward.
    door.visual(
        Box((0.410, 0.030, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=stainless,
        name="bottom_rail",
    )
    door.visual(
        Box((0.060, 0.030, 0.036)),
        origin=Origin(xyz=(-0.330, 0.0, 0.018)),
        material=stainless,
        name="bottom_rail_0",
    )
    door.visual(
        Box((0.060, 0.030, 0.036)),
        origin=Origin(xyz=(0.330, 0.0, 0.018)),
        material=stainless,
        name="bottom_rail_1",
    )
    door.visual(
        Box((0.720, 0.030, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.162)),
        material=stainless,
        name="top_rail",
    )
    door.visual(
        Box((0.042, 0.030, 0.180)),
        origin=Origin(xyz=(-0.339, 0.0, 0.090)),
        material=stainless,
        name="side_rail_0",
    )
    door.visual(
        Box((0.042, 0.030, 0.180)),
        origin=Origin(xyz=(0.339, 0.0, 0.090)),
        material=stainless,
        name="side_rail_1",
    )
    door.visual(
        Box((0.650, 0.006, 0.120)),
        origin=Origin(xyz=(0.0, -0.013, 0.090)),
        material=smoked_glass,
        name="glass",
    )
    door.visual(
        Box((0.650, 0.004, 0.120)),
        origin=Origin(xyz=(0.0, 0.017, 0.090)),
        material=dark_trim,
        name="rear_liner",
    )
    for i in range(5):
        door.visual(
            Box((0.610, 0.003, 0.004)),
            origin=Origin(xyz=(0.0, -0.014, 0.052 + i * 0.019)),
            material=stainless,
            name=f"window_screen_{i}",
        )
    door.visual(
        Cylinder(radius=0.012, length=0.420),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless,
        name="hinge_barrel",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.560),
        origin=Origin(xyz=(0.0, -0.052, 0.160), rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless,
        name="handle_bar",
    )
    for x in (-0.255, 0.255):
        door.visual(
            Cylinder(radius=0.007, length=0.040),
            origin=Origin(xyz=(x, -0.034, 0.160), rpy=(pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"handle_standoff_{0 if x < 0 else 1}",
        )

    door_hinge = model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, -0.240, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.35),
    )

    # Individual keypad buttons are separate movable parts on short inward
    # prismatic guides.  Their caps sit flush against the front panel at rest.
    button_positions = [
        (0.140, 0.299),
        (0.205, 0.299),
        (0.270, 0.299),
        (0.335, 0.299),
        (0.140, 0.267),
        (0.205, 0.267),
        (0.270, 0.267),
        (0.335, 0.267),
    ]
    for index, (x, z) in enumerate(button_positions):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.052, 0.012, 0.020)),
            origin=Origin(xyz=(0.0, -0.006, 0.0)),
            material=black_rubber,
            name="cap",
        )
        model.articulation(
            f"button_{index}_slide",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, -0.220, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.12, lower=0.0, upper=0.008),
        )

    # Keep a reference in model metadata for tests/readability without changing
    # the public articulation names.
    model.meta["primary_hinge"] = door_hinge.name
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("door_hinge")

    ctx.check(
        "door hinge rotates downward from lower edge",
        tuple(door_hinge.axis) == (1.0, 0.0, 0.0)
        and door_hinge.motion_limits is not None
        and door_hinge.motion_limits.lower == 0.0
        and door_hinge.motion_limits.upper >= 1.2,
        details=f"axis={door_hinge.axis}, limits={door_hinge.motion_limits}",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="y",
            positive_elem="cavity_shadow",
            negative_elem="rear_liner",
            min_gap=0.0,
            max_gap=0.006,
            name="closed door sits just in front of cooking opening",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            elem_a="rear_liner",
            elem_b="cavity_shadow",
            min_overlap=0.10,
            name="closed door covers recessed cooking opening",
        )

    closed_top = ctx.part_element_world_aabb(door, elem="top_rail")
    with ctx.pose({door_hinge: 1.20}):
        open_top = ctx.part_element_world_aabb(door, elem="top_rail")
    ctx.check(
        "drop-down door moves outward and down",
        closed_top is not None
        and open_top is not None
        and open_top[0][1] < closed_top[0][1] - 0.12
        and open_top[1][2] < closed_top[1][2] - 0.035,
        details=f"closed_top={closed_top}, open_top={open_top}",
    )

    for index in range(8):
        button = object_model.get_part(f"button_{index}")
        slide = object_model.get_articulation(f"button_{index}_slide")
        ctx.check(
            f"button_{index} has short inward guide",
            slide.articulation_type == ArticulationType.PRISMATIC
            and tuple(slide.axis) == (0.0, 1.0, 0.0)
            and slide.motion_limits is not None
            and 0.005 <= slide.motion_limits.upper <= 0.012,
            details=f"type={slide.articulation_type}, axis={slide.axis}, limits={slide.motion_limits}",
        )
        if index in (0, 7):
            ctx.expect_contact(
                button,
                body,
                elem_a="cap",
                elem_b="control_strip",
                contact_tol=0.001,
                name=f"button_{index} cap is seated in front panel",
            )

    slide_0 = object_model.get_articulation("button_0_slide")
    button_0 = object_model.get_part("button_0")
    rest_button = ctx.part_world_position(button_0)
    with ctx.pose({slide_0: 0.008}):
        pressed_button = ctx.part_world_position(button_0)
        ctx.expect_gap(
            body,
            button_0,
            axis="y",
            positive_elem="control_strip",
            negative_elem="cap",
            max_penetration=0.009,
            max_gap=0.001,
            name="pressed button travels into control strip",
        )
    ctx.check(
        "button guide travel is inward",
        rest_button is not None
        and pressed_button is not None
        and pressed_button[1] > rest_button[1] + 0.006,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    return ctx.report()


object_model = build_object_model()
