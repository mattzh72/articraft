from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BODY_WIDTH = 0.76
BODY_DEPTH = 0.51
BODY_HEIGHT = 0.078
TOP_THICKNESS = 0.008
TOP_Z = BODY_HEIGHT + TOP_THICKNESS

GRATE_BAR = 0.010
GRATE_UNDERSIDE_Z = 0.118
GRATE_TOP_Z = GRATE_UNDERSIDE_Z + GRATE_BAR / 2.0
GRATE_LEG_HEIGHT = GRATE_UNDERSIDE_Z - TOP_Z
GRATE_LEG_OVERLAP = 0.004

LID_HINGE_Y = 0.215
LID_HINGE_Z = 0.142
LID_OPEN_ANGLE = math.radians(100.0)

SMALL_BURNERS = (
    (-0.225, -0.070),
    (-0.225, 0.125),
    (0.225, -0.070),
    (0.225, 0.125),
)
KNOB_XS = (-0.26, -0.13, 0.0, 0.13, 0.26)


def _box(part, size, xyz, material, *, name: str | None = None, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _cylinder(
    part,
    radius: float,
    length: float,
    xyz,
    material,
    *,
    name: str | None = None,
    rpy=(0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_small_burner(body, x: float, y: float, index: int, burner_black, steel) -> None:
    _cylinder(
        body,
        radius=0.048,
        length=0.004,
        xyz=(x, y, TOP_Z + 0.002),
        material=steel,
        name=f"burner_bowl_{index}",
    )
    _cylinder(
        body,
        radius=0.031,
        length=0.010,
        xyz=(x, y, TOP_Z + 0.0085),
        material=burner_black,
        name=f"burner_crown_{index}",
    )
    _cylinder(
        body,
        radius=0.022,
        length=0.009,
        xyz=(x, y, TOP_Z + 0.0170),
        material=burner_black,
        name=f"burner_cap_{index}",
    )


def _add_wok_burner(body, burner_black, steel) -> None:
    _cylinder(
        body,
        radius=0.075,
        length=0.005,
        xyz=(0.0, 0.025, TOP_Z + 0.0025),
        material=steel,
        name="wok_bowl",
    )
    _cylinder(
        body,
        radius=0.050,
        length=0.011,
        xyz=(0.0, 0.025, TOP_Z + 0.0080),
        material=burner_black,
        name="wok_outer_crown",
    )
    _cylinder(
        body,
        radius=0.026,
        length=0.010,
        xyz=(0.0, 0.025, TOP_Z + 0.0150),
        material=burner_black,
        name="wok_inner_crown",
    )
    _cylinder(
        body,
        radius=0.036,
        length=0.008,
        xyz=(0.0, 0.025, TOP_Z + 0.0220),
        material=burner_black,
        name="wok_cap",
    )


def _add_rect_grate(part, width: float, depth: float, grate_iron, *, prefix: str) -> None:
    side_x = width / 2.0 - GRATE_BAR / 2.0
    side_y = depth / 2.0 - GRATE_BAR / 2.0

    _box(part, (width, GRATE_BAR, GRATE_BAR), (0.0, -side_y, GRATE_TOP_Z), grate_iron, name=f"{prefix}_front")
    _box(part, (width, GRATE_BAR, GRATE_BAR), (0.0, side_y, GRATE_TOP_Z), grate_iron, name=f"{prefix}_rear")
    _box(part, (GRATE_BAR, depth, GRATE_BAR), (-side_x, 0.0, GRATE_TOP_Z), grate_iron, name=f"{prefix}_left")
    _box(part, (GRATE_BAR, depth, GRATE_BAR), (side_x, 0.0, GRATE_TOP_Z), grate_iron, name=f"{prefix}_right")

    leg_x = side_x - 0.002
    leg_y = side_y - 0.002
    leg_height = GRATE_LEG_HEIGHT + GRATE_LEG_OVERLAP
    for ix, sign_x in enumerate((-1.0, 1.0)):
        for iy, sign_y in enumerate((-1.0, 1.0)):
            _box(
                part,
                (0.012, 0.012, leg_height),
                (sign_x * leg_x, sign_y * leg_y, TOP_Z + leg_height / 2.0),
                grate_iron,
                name=f"{prefix}_leg_{ix}_{iy}",
            )


def _build_left_grate(model: ArticulatedObject, grate_iron) -> None:
    left_grate = model.part("left_grate")
    _add_rect_grate(left_grate, 0.24, 0.30, grate_iron, prefix="left_frame")
    _box(left_grate, (0.228, GRATE_BAR, GRATE_BAR), (0.0, 0.0, GRATE_TOP_Z), grate_iron, name="left_bridge")
    _box(left_grate, (GRATE_BAR, 0.288, GRATE_BAR), (-0.050, 0.0, GRATE_TOP_Z), grate_iron, name="left_rib_0")
    _box(left_grate, (GRATE_BAR, 0.288, GRATE_BAR), (0.050, 0.0, GRATE_TOP_Z), grate_iron, name="left_rib_1")
    model.articulation(
        "body_to_left_grate",
        ArticulationType.FIXED,
        parent="body",
        child=left_grate,
        origin=Origin(xyz=(-0.235, 0.025, 0.0)),
    )


def _build_right_grate(model: ArticulatedObject, grate_iron) -> None:
    right_grate = model.part("right_grate")
    _add_rect_grate(right_grate, 0.24, 0.30, grate_iron, prefix="right_frame")
    _box(right_grate, (0.228, GRATE_BAR, GRATE_BAR), (0.0, 0.0, GRATE_TOP_Z), grate_iron, name="right_bridge")
    _box(right_grate, (GRATE_BAR, 0.288, GRATE_BAR), (-0.050, 0.0, GRATE_TOP_Z), grate_iron, name="right_rib_0")
    _box(right_grate, (GRATE_BAR, 0.288, GRATE_BAR), (0.050, 0.0, GRATE_TOP_Z), grate_iron, name="right_rib_1")
    model.articulation(
        "body_to_right_grate",
        ArticulationType.FIXED,
        parent="body",
        child=right_grate,
        origin=Origin(xyz=(0.235, 0.025, 0.0)),
    )


def _build_center_grate(model: ArticulatedObject, grate_iron) -> None:
    center_grate = model.part("center_grate")
    _add_rect_grate(center_grate, 0.22, 0.24, grate_iron, prefix="center_frame")

    inner_side = 0.12 / 2.0 - GRATE_BAR / 2.0
    _box(center_grate, (0.12, GRATE_BAR, GRATE_BAR), (0.0, -inner_side, GRATE_TOP_Z), grate_iron, name="center_inner_front")
    _box(center_grate, (0.12, GRATE_BAR, GRATE_BAR), (0.0, inner_side, GRATE_TOP_Z), grate_iron, name="center_inner_rear")
    _box(center_grate, (GRATE_BAR, 0.12, GRATE_BAR), (-inner_side, 0.0, GRATE_TOP_Z), grate_iron, name="center_inner_left")
    _box(center_grate, (GRATE_BAR, 0.12, GRATE_BAR), (inner_side, 0.0, GRATE_TOP_Z), grate_iron, name="center_inner_right")

    _box(center_grate, (0.218, GRATE_BAR, GRATE_BAR), (0.0, 0.0, GRATE_TOP_Z), grate_iron, name="center_cross")
    _box(center_grate, (GRATE_BAR, 0.218, GRATE_BAR), (0.0, 0.0, GRATE_TOP_Z), grate_iron, name="center_spine")

    model.articulation(
        "body_to_center_grate",
        ArticulationType.FIXED,
        parent="body",
        child=center_grate,
        origin=Origin(xyz=(0.0, 0.025, 0.0)),
    )


def _build_knob(model: ArticulatedObject, index: int, x: float, knob_body, knob_cap, marker) -> None:
    knob = model.part(f"knob_{index}")
    _cylinder(
        knob,
        radius=0.023,
        length=0.028,
        xyz=(0.0, -0.014, 0.0),
        material=knob_body,
        name="knob_shell",
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )
    _cylinder(
        knob,
        radius=0.015,
        length=0.004,
        xyz=(0.0, -0.030, 0.0),
        material=knob_cap,
        name="knob_face",
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )
    _box(
        knob,
        (0.003, 0.002, 0.015),
        (0.0, -0.033, 0.010),
        marker,
        name="indicator",
    )

    model.articulation(
        f"body_to_knob_{index}",
        ArticulationType.CONTINUOUS,
        parent="body",
        child=knob,
        origin=Origin(xyz=(x, -BODY_DEPTH / 2.0, 0.043)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=12.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_burner_gas_hob")

    steel = model.material("steel", rgba=(0.77, 0.79, 0.80, 1.0))
    fascia = model.material("fascia", rgba=(0.19, 0.20, 0.21, 1.0))
    burner_black = model.material("burner_black", rgba=(0.13, 0.13, 0.14, 1.0))
    grate_iron = model.material("grate_iron", rgba=(0.08, 0.08, 0.09, 1.0))
    knob_body = model.material("knob_body", rgba=(0.10, 0.10, 0.11, 1.0))
    knob_cap = model.material("knob_cap", rgba=(0.70, 0.72, 0.74, 1.0))
    marker = model.material("marker", rgba=(0.90, 0.91, 0.92, 1.0))
    glass = model.material("glass", rgba=(0.52, 0.68, 0.74, 0.28))

    body = model.part("body")
    _box(body, (BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT), (0.0, 0.0, BODY_HEIGHT / 2.0), steel, name="body_shell")
    _box(
        body,
        (BODY_WIDTH, BODY_DEPTH, TOP_THICKNESS),
        (0.0, 0.0, BODY_HEIGHT + TOP_THICKNESS / 2.0),
        steel,
        name="top_panel",
    )
    _box(body, (BODY_WIDTH, 0.060, 0.024), (0.0, -0.220, 0.064), fascia, name="front_fascia")
    _box(body, (0.70, 0.018, 0.018), (0.0, 0.226, 0.095), fascia, name="rear_hinge_rail")
    _box(body, (0.030, 0.018, 0.022), (-0.325, 0.226, 0.097), fascia, name="rear_hinge_block_0")
    _box(body, (0.030, 0.018, 0.022), (0.325, 0.226, 0.097), fascia, name="rear_hinge_block_1")

    for burner_index, (x, y) in enumerate(SMALL_BURNERS):
        _add_small_burner(body, x, y, burner_index, burner_black, steel)
    _add_wok_burner(body, burner_black, steel)

    _build_left_grate(model, grate_iron)
    _build_center_grate(model, grate_iron)
    _build_right_grate(model, grate_iron)

    lid = model.part("lid")
    _box(lid, (0.74, 0.34, 0.005), (0.0, -0.170, 0.0025), glass, name="lid_glass")
    _box(lid, (0.74, 0.020, 0.014), (0.0, -0.010, 0.007), fascia, name="lid_rail")
    _box(lid, (0.020, 0.024, 0.034), (-0.330, -0.008, -0.017), fascia, name="lid_bracket_0")
    _box(lid, (0.020, 0.024, 0.034), (0.330, -0.008, -0.017), fascia, name="lid_bracket_1")
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, LID_HINGE_Y, LID_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=LID_OPEN_ANGLE,
        ),
    )

    for index, x in enumerate(KNOB_XS):
        _build_knob(model, index, x, knob_body, knob_cap, marker)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    center_grate = object_model.get_part("center_grate")
    lid_hinge = object_model.get_articulation("body_to_lid")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            center_grate,
            axes="xy",
            elem_a="lid_glass",
            min_overlap=0.18,
            name="closed lid covers the wok grate footprint",
        )
        ctx.expect_gap(
            lid,
            center_grate,
            axis="z",
            positive_elem="lid_glass",
            min_gap=0.006,
            max_gap=0.030,
            name="closed lid clears the grate tops",
        )

    open_aabb = None
    with ctx.pose({lid_hinge: LID_OPEN_ANGLE}):
        open_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "lid opens up behind the cooking field",
        open_aabb is not None and open_aabb[1][2] > 0.45 and open_aabb[0][1] > 0.12,
        details=f"open lid aabb={open_aabb}",
    )

    knob_positions = []
    for index in range(5):
        knob = object_model.get_part(f"knob_{index}")
        joint = object_model.get_articulation(f"body_to_knob_{index}")
        ctx.expect_gap(
            body,
            knob,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"knob_{index} seats against the fascia",
        )
        knob_positions.append(ctx.part_world_position(knob))
        limits = joint.motion_limits
        ctx.check(
            f"knob_{index} uses continuous rotation",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"type={joint.articulation_type}, limits={limits}",
        )

    ctx.check(
        "five knobs form a front control row",
        all(position is not None and position[1] < -0.24 for position in knob_positions)
        and all(
            knob_positions[index] is not None
            and knob_positions[index + 1] is not None
            and knob_positions[index][0] < knob_positions[index + 1][0]
            for index in range(len(knob_positions) - 1)
        ),
        details=f"knob_positions={knob_positions}",
    )

    wok_aabb = ctx.part_element_world_aabb(body, elem="wok_cap")
    small_aabb = ctx.part_element_world_aabb(body, elem="burner_cap_0")
    wok_width = None
    small_width = None
    if wok_aabb is not None:
        wok_width = wok_aabb[1][0] - wok_aabb[0][0]
    if small_aabb is not None:
        small_width = small_aabb[1][0] - small_aabb[0][0]
    ctx.check(
        "central wok burner is visibly larger than the side burners",
        wok_width is not None and small_width is not None and wok_width > small_width + 0.020,
        details=f"wok_width={wok_width}, small_width={small_width}",
    )

    return ctx.report()


object_model = build_object_model()
