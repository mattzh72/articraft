from __future__ import annotations

import math

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
    model = ArticulatedObject(name="freestanding_gas_range")

    # Overall scale: a common 30 inch freestanding range, about 0.76 m wide,
    # 0.66 m deep, with a cooktop at counter height and a raised rear panel.
    stainless = model.material("brushed_stainless", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_enamel = model.material("black_enamel", rgba=(0.02, 0.02, 0.018, 1.0))
    cast_iron = model.material("matte_cast_iron", rgba=(0.005, 0.005, 0.005, 1.0))
    glass = model.material("smoked_glass", rgba=(0.02, 0.03, 0.04, 0.45))
    brass = model.material("brass_burner", rgba=(0.75, 0.52, 0.22, 1.0))
    shadow = model.material("oven_shadow", rgba=(0.0, 0.0, 0.0, 1.0))

    body = model.part("range_body")
    # Main freestanding cabinet and side panels.
    body.visual(
        Box((0.76, 0.60, 0.76)),
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
        material=stainless,
        name="cabinet_shell",
    )
    # Black recessed toe kick, connected into the lower cabinet face.
    body.visual(
        Box((0.68, 0.035, 0.075)),
        origin=Origin(xyz=(0.0, -0.317, 0.095)),
        material=shadow,
        name="toe_recess",
    )
    # Slightly proud black oven cavity reveal behind the door perimeter.
    body.visual(
        Box((0.66, 0.018, 0.53)),
        origin=Origin(xyz=(0.0, -0.306, 0.455)),
        material=shadow,
        name="oven_reveal",
    )
    body.visual(
        Box((0.72, 0.040, 0.028)),
        origin=Origin(xyz=(0.0, -0.319, 0.157)),
        material=cast_iron,
        name="hinge_leaf",
    )
    # Counter-height cooktop slab with a rear lip.
    body.visual(
        Box((0.79, 0.66, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.865)),
        material=dark_enamel,
        name="cooktop",
    )
    body.visual(
        Box((0.79, 0.030, 0.060)),
        origin=Origin(xyz=(0.0, 0.315, 0.900)),
        material=dark_enamel,
        name="rear_cooktop_lip",
    )
    # Raised rear control panel / backsplash, visibly attached to the rear lip.
    body.visual(
        Box((0.79, 0.065, 0.235)),
        origin=Origin(xyz=(0.0, 0.330, 1.005)),
        material=stainless,
        name="backguard_panel",
    )
    body.visual(
        Box((0.79, 0.012, 0.055)),
        origin=Origin(xyz=(0.0, 0.294, 0.915)),
        material=dark_enamel,
        name="control_face_band",
    )

    # Four gas burner heads and brass inner caps sitting below the grates.
    burner_centers = [
        (-0.19, -0.13),
        (0.19, -0.13),
        (-0.19, 0.115),
        (0.19, 0.115),
    ]
    for idx, (x, y) in enumerate(burner_centers):
        body.visual(
            Cylinder(radius=0.052, length=0.016),
            origin=Origin(xyz=(x, y, 0.896)),
            material=cast_iron,
            name=f"burner_head_{idx}",
        )
        body.visual(
            Cylinder(radius=0.034, length=0.016),
            origin=Origin(xyz=(x, y, 0.911)),
            material=brass,
            name=f"burner_cap_{idx}",
        )

    # Four individual removable-looking grate castings.  Each grate is one
    # connected grid: an outer square ring with short inward fingers that leave
    # the central burner cap clear.
    grate_size_x = 0.255
    grate_size_y = 0.225
    rail = 0.022
    grate_height = 0.024
    for idx, (x, y) in enumerate(burner_centers):
        grate = model.part(f"grate_{idx}")
        zc = grate_height / 2.0
        grate.visual(
            Box((grate_size_x, rail, grate_height)),
            origin=Origin(xyz=(0.0, grate_size_y / 2.0 - rail / 2.0, zc)),
            material=cast_iron,
            name="rear_rail",
        )
        grate.visual(
            Box((grate_size_x, rail, grate_height)),
            origin=Origin(xyz=(0.0, -grate_size_y / 2.0 + rail / 2.0, zc)),
            material=cast_iron,
            name="front_rail",
        )
        grate.visual(
            Box((rail, grate_size_y, grate_height)),
            origin=Origin(xyz=(grate_size_x / 2.0 - rail / 2.0, 0.0, zc)),
            material=cast_iron,
            name="side_rail_0",
        )
        grate.visual(
            Box((rail, grate_size_y, grate_height)),
            origin=Origin(xyz=(-grate_size_x / 2.0 + rail / 2.0, 0.0, zc)),
            material=cast_iron,
            name="side_rail_1",
        )
        # Short pan-support fingers.
        grate.visual(
            Box((0.018, 0.026, 0.020)),
            origin=Origin(xyz=(0.0, grate_size_y / 2.0 - rail - 0.013, zc + 0.002)),
            material=cast_iron,
            name="rear_finger",
        )
        grate.visual(
            Box((0.018, 0.026, 0.020)),
            origin=Origin(xyz=(0.0, -grate_size_y / 2.0 + rail + 0.013, zc + 0.002)),
            material=cast_iron,
            name="front_finger",
        )
        grate.visual(
            Box((0.039, 0.018, 0.020)),
            origin=Origin(xyz=(grate_size_x / 2.0 - rail - 0.0195, 0.0, zc + 0.002)),
            material=cast_iron,
            name="side_finger_0",
        )
        grate.visual(
            Box((0.039, 0.018, 0.020)),
            origin=Origin(xyz=(-grate_size_x / 2.0 + rail + 0.0195, 0.0, zc + 0.002)),
            material=cast_iron,
            name="side_finger_1",
        )
        model.articulation(
            f"body_to_grate_{idx}",
            ArticulationType.FIXED,
            parent=body,
            child=grate,
            origin=Origin(xyz=(x, y, 0.890)),
        )

    # Bottom-hinged oven door.  The child part frame is the hinge axis; at q=0
    # the door is vertical and extends in local +Z from the full-width hinge.
    door = model.part("oven_door")
    door.visual(
        Box((0.67, 0.040, 0.165)),
        origin=Origin(xyz=(0.0, -0.020, 0.0825)),
        material=stainless,
        name="lower_panel",
    )
    door.visual(
        Box((0.67, 0.040, 0.080)),
        origin=Origin(xyz=(0.0, -0.020, 0.485)),
        material=stainless,
        name="top_panel",
    )
    door.visual(
        Box((0.065, 0.040, 0.520)),
        origin=Origin(xyz=(-0.3025, -0.020, 0.260)),
        material=stainless,
        name="side_frame_0",
    )
    door.visual(
        Box((0.065, 0.040, 0.520)),
        origin=Origin(xyz=(0.3025, -0.020, 0.260)),
        material=stainless,
        name="side_frame_1",
    )
    door.visual(
        Box((0.540, 0.012, 0.300)),
        origin=Origin(xyz=(0.0, -0.046, 0.310)),
        material=glass,
        name="window_panel",
    )
    door.visual(
        Box((0.500, 0.006, 0.260)),
        origin=Origin(xyz=(0.0, -0.039, 0.310)),
        material=shadow,
        name="window_shadow_gasket",
    )
    door.visual(
        Cylinder(radius=0.017, length=0.70),
        origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="full_width_hinge_barrel",
    )
    # Handle with two standoffs tied into the upper door face.
    door.visual(
        Box((0.036, 0.045, 0.036)),
        origin=Origin(xyz=(-0.250, -0.057, 0.468)),
        material=stainless,
        name="handle_standoff_0",
    )
    door.visual(
        Box((0.036, 0.045, 0.036)),
        origin=Origin(xyz=(0.250, -0.057, 0.468)),
        material=stainless,
        name="handle_standoff_1",
    )
    door.visual(
        Cylinder(radius=0.018, length=0.610),
        origin=Origin(xyz=(0.0, -0.087, 0.468), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="handle_bar",
    )
    model.articulation(
        "body_to_oven_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, -0.325, 0.175)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=0.0, upper=1.45),
    )

    # Five separate rotary controls on the rear panel: four burner valves and
    # an oven selector.  Their cap faces sit proud of the backguard front face.
    for idx, x in enumerate((-0.28, -0.14, 0.0, 0.14, 0.28)):
        knob = model.part(f"knob_{idx}")
        knob.visual(
            Cylinder(radius=0.024, length=0.030),
            origin=Origin(xyz=(0.0, 0.0, 0.015)),
            material=dark_enamel,
            name="knob_cap",
        )
        knob.visual(
            Box((0.006, 0.036, 0.004)),
            origin=Origin(xyz=(0.0, -0.020, 0.032)),
            material=stainless,
            name="pointer_mark",
        )
        model.articulation(
            f"panel_to_knob_{idx}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(x, 0.2975, 0.980), rpy=(math.pi / 2.0, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.5, velocity=3.0, lower=-2.35, upper=2.35),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("range_body")
    door = object_model.get_part("oven_door")
    hinge = object_model.get_articulation("body_to_oven_door")

    ctx.allow_overlap(
        body,
        door,
        elem_a="hinge_leaf",
        elem_b="full_width_hinge_barrel",
        reason="The full-width hinge barrel is intentionally captured by the fixed body hinge leaf.",
    )
    ctx.expect_gap(
        door,
        body,
        axis="z",
        max_gap=0.003,
        max_penetration=0.015,
        positive_elem="full_width_hinge_barrel",
        negative_elem="hinge_leaf",
        name="hinge barrel is seated in body hinge leaf",
    )

    # Prompt-critical checks: four separate top grates are seated on the
    # cooktop, and the oven door's bottom hinge moves the windowed door outward.
    for idx in range(4):
        grate = object_model.get_part(f"grate_{idx}")
        ctx.expect_gap(
            grate,
            body,
            axis="z",
            min_gap=-0.001,
            max_gap=0.003,
            positive_elem="front_rail",
            negative_elem="cooktop",
            name=f"grate_{idx} rests on cooktop",
        )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="y",
            min_gap=0.001,
            max_gap=0.035,
            positive_elem="oven_reveal",
            negative_elem="lower_panel",
            name="closed oven door sits proud of front",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            min_overlap=0.28,
            elem_a="window_panel",
            elem_b="oven_reveal",
            name="windowed door covers oven opening",
        )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({hinge: 1.25}):
        open_aabb = ctx.part_world_aabb(door)
        ctx.check(
            "oven door hinges downward and outward",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[0][1] < closed_aabb[0][1] - 0.20
            and open_aabb[1][2] < closed_aabb[1][2] - 0.10,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
