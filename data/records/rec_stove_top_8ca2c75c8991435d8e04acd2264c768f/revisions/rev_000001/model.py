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
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_WIDTH = 0.58
BODY_DEPTH = 0.46
SHELL_HEIGHT = 0.040
TOP_PLATE_THICKNESS = 0.006
DECK_TOP_Z = 0.042

FRONT_BURNER_X = 0.150
FRONT_BURNER_Y = -0.035
REAR_BURNER_Y = 0.125

KNOB_XS = (-0.150, 0.0, 0.150)
KNOB_AXIS_Y = -(BODY_DEPTH / 2.0)


def _add_burner(
    model: ArticulatedObject,
    body,
    name: str,
    *,
    xy: tuple[float, float],
    flange_radius: float,
    head_radius: float,
    cap_radius: float,
    grate_span: float,
) -> None:
    burner = model.part(name)
    cast_iron = "cast_iron"
    burner_head = "burner_steel"

    burner.visual(
        Cylinder(radius=flange_radius, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=burner_head,
        name="flange",
    )
    burner.visual(
        Cylinder(radius=head_radius, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=cast_iron,
        name="head",
    )
    burner.visual(
        Cylinder(radius=cap_radius, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=cast_iron,
        name="cap",
    )

    post_offset = flange_radius * 0.72
    for post_index, post_xyz in enumerate(
        (
            (post_offset, 0.0, 0.0115),
            (-post_offset, 0.0, 0.0115),
            (0.0, post_offset, 0.0115),
            (0.0, -post_offset, 0.0115),
        )
    ):
        burner.visual(
            Box((0.010, 0.020, 0.019)),
            origin=Origin(xyz=post_xyz),
            material=cast_iron,
            name=f"post_{post_index}",
        )

    burner.visual(
        Box((grate_span, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=cast_iron,
        name="grate_x",
    )
    burner.visual(
        Box((0.012, grate_span, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=cast_iron,
        name="grate_y",
    )

    model.articulation(
        f"body_to_{name}",
        ArticulationType.FIXED,
        parent=body,
        child=burner,
        origin=Origin(xyz=(xy[0], xy[1], DECK_TOP_Z)),
    )


def _add_knob(
    model: ArticulatedObject,
    body,
    knob_mesh,
    *,
    name: str,
    x: float,
) -> None:
    knob = model.part(name)
    knob.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="burner_steel",
        name="shaft",
    )
    knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="knob_black",
        name="knob_shell",
    )

    model.articulation(
        f"body_to_{name}",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(x, KNOB_AXIS_Y, 0.020)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triangular_gas_cooktop")

    model.material("body_black", rgba=(0.15, 0.15, 0.16, 1.0))
    model.material("control_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("stainless", rgba=(0.74, 0.76, 0.78, 1.0))
    model.material("cast_iron", rgba=(0.17, 0.17, 0.18, 1.0))
    model.material("burner_steel", rgba=(0.40, 0.42, 0.45, 1.0))
    model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, SHELL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, SHELL_HEIGHT / 2.0)),
        material="body_black",
        name="housing_shell",
    )
    body.visual(
        Box((0.54, 0.39, TOP_PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.015, 0.039)),
        material="stainless",
        name="top_plate",
    )
    body.visual(
        Box((0.50, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, -0.220, 0.018)),
        material="control_black",
        name="control_strip",
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.050,
            0.028,
            body_style="skirted",
            top_diameter=0.040,
            skirt=KnobSkirt(0.060, 0.007, flare=0.10),
            grip=KnobGrip(style="fluted", count=14, depth=0.0015),
            indicator=KnobIndicator(
                style="line",
                mode="engraved",
                depth=0.0009,
                angle_deg=0.0,
            ),
            center=False,
        ),
        "cooktop_knob",
    )

    _add_burner(
        model,
        body,
        "burner_0",
        xy=(-FRONT_BURNER_X, FRONT_BURNER_Y),
        flange_radius=0.055,
        head_radius=0.036,
        cap_radius=0.021,
        grate_span=0.122,
    )
    _add_burner(
        model,
        body,
        "burner_1",
        xy=(FRONT_BURNER_X, FRONT_BURNER_Y),
        flange_radius=0.055,
        head_radius=0.036,
        cap_radius=0.021,
        grate_span=0.122,
    )
    _add_burner(
        model,
        body,
        "burner_2",
        xy=(0.0, REAR_BURNER_Y),
        flange_radius=0.060,
        head_radius=0.040,
        cap_radius=0.024,
        grate_span=0.132,
    )

    for knob_index, knob_x in enumerate(KNOB_XS):
        _add_knob(model, body, knob_mesh, name=f"knob_{knob_index}", x=knob_x)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")

    burners = [object_model.get_part(f"burner_{index}") for index in range(3)]
    knobs = [object_model.get_part(f"knob_{index}") for index in range(3)]
    knob_joints = [object_model.get_articulation(f"body_to_knob_{index}") for index in range(3)]

    for burner in burners:
        ctx.expect_contact(
            burner,
            body,
            name=f"{burner.name}_rests_on_cooktop",
        )

    for knob in knobs:
        ctx.expect_gap(
            body,
            knob,
            axis="y",
            max_gap=0.0015,
            max_penetration=0.0,
            name=f"{knob.name}_sits_flush_to_front_strip",
        )

    ctx.expect_origin_distance(
        "burner_0",
        "burner_1",
        axes="x",
        min_dist=0.26,
        name="front burners are widely spaced",
    )
    ctx.expect_origin_gap(
        "burner_2",
        "burner_0",
        axis="y",
        min_gap=0.14,
        name="rear burner sits behind left front burner",
    )
    ctx.expect_origin_gap(
        "burner_2",
        "burner_1",
        axis="y",
        min_gap=0.14,
        name="rear burner sits behind right front burner",
    )

    burner_positions = [ctx.part_world_position(burner) for burner in burners]
    if all(position is not None for position in burner_positions):
        left_burner, right_burner, rear_burner = burner_positions
        ctx.check(
            "front burner pair is symmetric about the cooktop center",
            abs(left_burner[0] + right_burner[0]) <= 0.01
            and abs(left_burner[1] - right_burner[1]) <= 0.005,
            details=f"left={left_burner}, right={right_burner}",
        )
        ctx.check(
            "rear burner is centered between the front burners",
            abs(rear_burner[0]) <= 0.01 and rear_burner[1] > left_burner[1] + 0.14,
            details=f"rear={rear_burner}, front_left={left_burner}",
        )

    for joint in knob_joints:
        ctx.check(
            f"{joint.name}_is_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={joint.articulation_type}",
        )

    center_knob = object_model.get_part("knob_1")
    center_joint = object_model.get_articulation("body_to_knob_1")
    rest_position = ctx.part_world_position(center_knob)
    with ctx.pose({center_joint: pi / 2.0}):
        posed_position = ctx.part_world_position(center_knob)
        ctx.expect_gap(
            body,
            center_knob,
            axis="y",
            max_gap=0.0015,
            max_penetration=0.0,
            name="center knob remains seated when turned",
        )

    ctx.check(
        "center knob rotates in place",
        rest_position is not None
        and posed_position is not None
        and abs(rest_position[0] - posed_position[0]) <= 1e-6
        and abs(rest_position[1] - posed_position[1]) <= 1e-6
        and abs(rest_position[2] - posed_position[2]) <= 1e-6,
        details=f"rest={rest_position}, posed={posed_position}",
    )

    return ctx.report()


object_model = build_object_model()
