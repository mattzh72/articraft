from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    KnobBore,
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
    rounded_rect_profile,
    superellipse_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="freestanding_five_burner_gas_stove")

    stainless = Material("brushed_stainless", rgba=(0.68, 0.68, 0.63, 1.0))
    dark_enamel = Material("black_enamel", rgba=(0.015, 0.014, 0.013, 1.0))
    cast_iron = Material("matte_cast_iron", rgba=(0.025, 0.024, 0.023, 1.0))
    black_glass = Material("black_glass", rgba=(0.0, 0.01, 0.018, 1.0))
    warm_brass = Material("warm_brass", rgba=(0.90, 0.58, 0.22, 1.0))
    rubber = Material("dark_rubber", rgba=(0.01, 0.01, 0.01, 1.0))

    round_ring_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            superellipse_profile(0.118, 0.118, exponent=2.0, segments=64),
            [superellipse_profile(0.076, 0.076, exponent=2.0, segments=48)],
            0.006,
            center=True,
        ),
        "round_burner_ring",
    )
    oval_ring_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.118, 0.286, 0.057, corner_segments=12),
            [rounded_rect_profile(0.070, 0.232, 0.034, corner_segments=10)],
            0.006,
            center=True,
        ),
        "central_oval_burner_ring",
    )
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.054,
            0.030,
            body_style="skirted",
            top_diameter=0.039,
            skirt=KnobSkirt(0.064, 0.007, flare=0.06, chamfer=0.0012),
            grip=KnobGrip(style="fluted", count=22, depth=0.0015),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
            bore=KnobBore(style="d_shaft", diameter=0.007, flat_depth=0.0012),
        ),
        "gas_range_knob",
    )

    body = model.part("body")
    # Lower appliance case and the rolled front control rail.
    body.visual(
        Box((0.740, 0.690, 0.750)),
        origin=Origin(xyz=(0.0, -0.005, 0.375)),
        material=stainless,
        name="lower_case",
    )
    body.visual(
        Box((0.772, 0.060, 0.108)),
        origin=Origin(xyz=(0.0, -0.356, 0.794)),
        material=stainless,
        name="front_rail",
    )
    body.visual(
        Box((0.805, 0.742, 0.046)),
        origin=Origin(xyz=(0.0, -0.020, 0.865)),
        material=dark_enamel,
        name="cooktop_pan",
    )
    body.visual(
        Box((0.790, 0.045, 0.215)),
        origin=Origin(xyz=(0.0, 0.343, 0.988)),
        material=stainless,
        name="rear_splash",
    )
    body.visual(
        Box((0.690, 0.014, 0.060)),
        origin=Origin(xyz=(0.0, -0.354, 0.705)),
        material=black_glass,
        name="oven_shadow_recess",
    )
    body.visual(
        Box((0.760, 0.665, 0.040)),
        origin=Origin(xyz=(0.0, 0.000, 0.020)),
        material=rubber,
        name="toe_plinth",
    )
    # Small hinge brackets at the lower door axis visually ground the door hinge.
    for x in (-0.385, 0.385):
        body.visual(
            Box((0.030, 0.052, 0.054)),
            origin=Origin(xyz=(x, -0.369, 0.160)),
            material=cast_iron,
            name=f"hinge_mount_{0 if x < 0 else 1}",
        )
        body.visual(
            Cylinder(radius=0.018, length=0.055),
            origin=Origin(xyz=(x, -0.400, 0.160), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=cast_iron,
            name=f"hinge_bracket_{0 if x < 0 else 1}",
        )

    # Four round burners and a lengthwise central oval burner.
    round_burners = [
        (-0.235, -0.195, 0.050),
        (0.235, -0.195, 0.050),
        (-0.235, 0.170, 0.043),
        (0.235, 0.170, 0.043),
    ]
    for index, (x, y, cap_radius) in enumerate(round_burners):
        body.visual(
            round_ring_mesh,
            origin=Origin(xyz=(x, y, 0.891)),
            material=cast_iron,
            name=f"round_ring_{index}",
        )
        body.visual(
            Cylinder(radius=cap_radius, length=0.016),
            origin=Origin(xyz=(x, y, 0.896)),
            material=warm_brass,
            name=f"burner_cap_{index}",
        )
        body.visual(
            Box((0.205, 0.019, 0.010)),
            origin=Origin(xyz=(x, y, 0.909)),
            material=cast_iron,
            name=f"grate_cross_x_{index}",
        )
        body.visual(
            Box((0.019, 0.205, 0.010)),
            origin=Origin(xyz=(x, y, 0.909)),
            material=cast_iron,
            name=f"grate_cross_y_{index}",
        )

    body.visual(
        oval_ring_mesh,
        origin=Origin(xyz=(0.0, -0.015, 0.891)),
        material=cast_iron,
        name="oval_ring",
    )
    body.visual(
        Box((0.056, 0.175, 0.016)),
        origin=Origin(xyz=(0.0, -0.015, 0.896)),
        material=warm_brass,
        name="oval_center_cap",
    )
    for i, y in enumerate((-0.102, 0.072)):
        body.visual(
            Cylinder(radius=0.028, length=0.016),
            origin=Origin(xyz=(0.0, y, 0.896)),
            material=warm_brass,
            name=f"oval_end_cap_{i}",
        )
    body.visual(
        Box((0.025, 0.318, 0.010)),
        origin=Origin(xyz=(0.0, -0.015, 0.909)),
        material=cast_iron,
        name="oval_grate_long",
    )
    for i, y in enumerate((-0.115, 0.085)):
        body.visual(
            Box((0.172, 0.020, 0.010)),
            origin=Origin(xyz=(0.0, y, 0.909)),
            material=cast_iron,
            name=f"oval_grate_cross_{i}",
        )

    # Subtle white tick marks on the rail emphasize that the knob parts are controls.
    for i, x in enumerate((-0.280, -0.140, 0.0, 0.140, 0.280)):
        body.visual(
            Box((0.004, 0.002, 0.022)),
            origin=Origin(xyz=(x, -0.388, 0.842 if i == 2 else 0.833)),
            material=Material("white_tick", rgba=(0.92, 0.90, 0.84, 1.0)),
            name=f"knob_tick_{i}",
        )

    knob_positions = [
        (-0.280, -0.408, 0.778),
        (-0.140, -0.408, 0.790),
        (0.000, -0.408, 0.797),
        (0.140, -0.408, 0.790),
        (0.280, -0.408, 0.778),
    ]
    for i, xyz in enumerate(knob_positions):
        knob = model.part(f"knob_{i}")
        knob.visual(
            knob_mesh,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name="knob_cap",
        )
        model.articulation(
            f"body_to_knob_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=xyz),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.6, velocity=6.0),
        )

    oven_door = model.part("oven_door")
    oven_door.visual(
        Box((0.660, 0.035, 0.500)),
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        material=stainless,
        name="door_panel",
    )
    oven_door.visual(
        Box((0.475, 0.006, 0.245)),
        origin=Origin(xyz=(0.0, -0.0205, 0.260)),
        material=black_glass,
        name="door_window",
    )
    oven_door.visual(
        Cylinder(radius=0.017, length=0.500),
        origin=Origin(xyz=(0.0, -0.058, 0.420), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="door_handle_bar",
    )
    for i, x in enumerate((-0.235, 0.235)):
        oven_door.visual(
            Cylinder(radius=0.011, length=0.046),
            origin=Origin(xyz=(x, -0.037, 0.420), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"handle_standoff_{i}",
        )
    for i, x in enumerate((-0.3425, 0.3425)):
        oven_door.visual(
            Cylinder(radius=0.017, length=0.035),
            origin=Origin(xyz=(x, 0.0035, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=cast_iron,
            name=f"door_hinge_knuckle_{i}",
        )
    model.articulation(
        "body_to_oven_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=oven_door,
        origin=Origin(xyz=(0.0, -0.4035, 0.160)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    door = object_model.get_part("oven_door")
    body = object_model.get_part("body")
    door_joint = object_model.get_articulation("body_to_oven_door")
    knobs = [object_model.get_part(f"knob_{i}") for i in range(5)]
    knob_joints = [object_model.get_articulation(f"body_to_knob_{i}") for i in range(5)]

    ctx.check("five separate burner knobs", len(knobs) == 5)
    for i, joint in enumerate(knob_joints):
        ctx.check(
            f"knob {i} uses a continuous front-back axis",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and abs(joint.axis[1]) > 0.99
            and abs(joint.axis[0]) < 0.01
            and abs(joint.axis[2]) < 0.01,
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )
        ctx.expect_contact(
            knobs[i],
            body,
            elem_a="knob_cap",
            elem_b="front_rail",
            contact_tol=0.003,
            name=f"knob {i} seats on the front rail",
        )

    knob_aabbs = [ctx.part_world_aabb(knob) for knob in knobs]
    knob_centers = [
        (
            (aabb[0][0] + aabb[1][0]) / 2.0,
            (aabb[0][2] + aabb[1][2]) / 2.0,
        )
        for aabb in knob_aabbs
        if aabb is not None
    ]
    ctx.check(
        "knobs form a shallow upward arc",
        len(knob_centers) == 5
        and knob_centers[0][1] < knob_centers[1][1] < knob_centers[2][1]
        and knob_centers[4][1] < knob_centers[3][1] < knob_centers[2][1],
        details=f"knob centers={knob_centers}",
    )

    ctx.expect_gap(
        body,
        door,
        axis="y",
        min_gap=-0.001,
        max_gap=0.020,
        positive_elem="front_rail",
        negative_elem="door_panel",
        name="closed oven door sits just in front of the rail",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.25}):
        open_aabb = ctx.part_world_aabb(door)
        ctx.expect_gap(
            door,
            body,
            axis="z",
            min_gap=0.010,
            positive_elem="door_panel",
            negative_elem="toe_plinth",
            name="open oven door stays above the floor plinth",
        )
    ctx.check(
        "oven door opens downward and forward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] < closed_aabb[1][2] - 0.20
        and open_aabb[0][1] < closed_aabb[0][1] - 0.35,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
