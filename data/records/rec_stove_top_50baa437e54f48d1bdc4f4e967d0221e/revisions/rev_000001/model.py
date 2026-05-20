from __future__ import annotations

import math

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
BODY_DEPTH = 0.50
BODY_HEIGHT = 0.058
DECK_TOP_Z = 0.062
HINGE_Y = 0.216
HINGE_Z = 0.096
COVER_WIDTH = 0.544
COVER_DEPTH = 0.360
COVER_THICKNESS = 0.006

KNOB_XS = (-0.192, -0.064, 0.064, 0.192)
KNOB_Y = -(BODY_DEPTH / 2.0) - 0.001
KNOB_Z = 0.031


def _add_burner(
    body,
    *,
    x: float,
    y: float,
    seat_radius: float,
    crown_radius: float,
    cap_radius: float,
    prefix: str,
    trim,
    dark,
) -> None:
    body.visual(
        Cylinder(radius=seat_radius, length=0.004),
        origin=Origin(xyz=(x, y, DECK_TOP_Z + 0.001)),
        material=trim,
        name=f"{prefix}_seat",
    )
    body.visual(
        Cylinder(radius=crown_radius, length=0.010),
        origin=Origin(xyz=(x, y, DECK_TOP_Z + 0.0085)),
        material=dark,
        name=f"{prefix}_crown",
    )
    body.visual(
        Cylinder(radius=cap_radius, length=0.008),
        origin=Origin(xyz=(x, y, DECK_TOP_Z + 0.0165)),
        material=dark,
        name=f"{prefix}_cap",
    )


def _add_grate(body, *, center_x: float, prefix: str, finish) -> None:
    top_z = 0.083
    frame_thickness = 0.010
    frame_width = 0.164
    frame_depth = 0.192
    side_x = frame_width / 2.0 - frame_thickness / 2.0
    end_y = frame_depth / 2.0 - frame_thickness / 2.0

    leg_height = top_z - DECK_TOP_Z + 0.003
    leg_z = DECK_TOP_Z + leg_height / 2.0 - 0.0005
    leg_size = (0.012, 0.012, leg_height)
    for leg_index, (dx, dy) in enumerate(
        (
            (-side_x, -end_y),
            (side_x, -end_y),
            (-side_x, end_y),
            (side_x, end_y),
        )
    ):
        body.visual(
            Box(leg_size),
            origin=Origin(xyz=(center_x + dx, dy, leg_z)),
            material=finish,
            name=f"{prefix}_leg_{leg_index}",
        )

    bar_size_x = (frame_width, frame_thickness, frame_thickness)
    bar_size_y = (frame_thickness, frame_depth, frame_thickness)
    for bar_name, xyz, size in (
        (f"{prefix}_front_bar", (center_x, -end_y, top_z), bar_size_x),
        (f"{prefix}_rear_bar", (center_x, end_y, top_z), bar_size_x),
        (f"{prefix}_outer_bar", (center_x - side_x, 0.0, top_z), bar_size_y),
        (f"{prefix}_inner_bar", (center_x + side_x, 0.0, top_z), bar_size_y),
        (prefix, (center_x, 0.0, top_z), (frame_width - 0.020, frame_thickness, frame_thickness)),
    ):
        body.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=finish,
            name=bar_name,
        )


def _make_knob_mesh():
    return mesh_from_geometry(
        KnobGeometry(
            0.042,
            0.024,
            body_style="skirted",
            top_diameter=0.032,
            edge_radius=0.0015,
            skirt=KnobSkirt(0.050, 0.005, flare=0.06),
            grip=KnobGrip(style="fluted", count=16, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
            center=False,
        ),
        "apartment_cooktop_knob",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="glass_cover_gas_cooktop")

    enamel = model.material("enamel_black", rgba=(0.10, 0.10, 0.11, 1.0))
    burner_dark = model.material("burner_dark", rgba=(0.15, 0.15, 0.16, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.75, 0.77, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.20, 0.21, 0.22, 1.0))
    glass = model.material("smoked_glass", rgba=(0.44, 0.56, 0.58, 0.33))

    knob_mesh = _make_knob_mesh()

    body = model.part("cooktop_body")
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0)),
        material=enamel,
        name="base_shell",
    )
    body.visual(
        Box((0.548, 0.350, 0.008)),
        origin=Origin(xyz=(0.0, 0.025, DECK_TOP_Z - 0.004)),
        material=steel,
        name="deck_plate",
    )
    body.visual(
        Box((BODY_WIDTH, 0.062, 0.024)),
        origin=Origin(xyz=(0.0, -0.219, 0.052)),
        material=enamel,
        name="front_fascia",
    )
    body.visual(
        Box((0.558, 0.038, 0.028)),
        origin=Origin(xyz=(0.0, 0.214, 0.074)),
        material=trim_dark,
        name="rear_hinge_riser",
    )
    body.visual(
        Cylinder(radius=0.008, length=BODY_WIDTH - 0.036),
        origin=Origin(xyz=(0.0, 0.220, 0.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="hinge_bar",
    )

    burner_specs = (
        (-0.140, -0.078, 0.053, 0.036, 0.023, "front_burner_0"),
        (0.140, -0.078, 0.053, 0.036, 0.023, "front_burner_1"),
        (-0.140, 0.080, 0.046, 0.031, 0.020, "rear_burner_0"),
        (0.140, 0.080, 0.046, 0.031, 0.020, "rear_burner_1"),
    )
    for x, y, seat_radius, crown_radius, cap_radius, prefix in burner_specs:
        _add_burner(
            body,
            x=x,
            y=y,
            seat_radius=seat_radius,
            crown_radius=crown_radius,
            cap_radius=cap_radius,
            prefix=prefix,
            trim=steel,
            dark=burner_dark,
        )

    _add_grate(body, center_x=-0.140, prefix="grate_0", finish=enamel)
    _add_grate(body, center_x=0.140, prefix="grate_1", finish=enamel)

    cover = model.part("glass_cover")
    cover.visual(
        Box((COVER_WIDTH, COVER_DEPTH, COVER_THICKNESS)),
        origin=Origin(xyz=(0.0, -(COVER_DEPTH / 2.0), COVER_THICKNESS / 2.0)),
        material=glass,
        name="cover_glass",
    )
    cover.visual(
        Box((COVER_WIDTH, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, -0.010, 0.004)),
        material=trim_dark,
        name="rear_leaf",
    )
    cover.visual(
        Cylinder(radius=0.006, length=COVER_WIDTH - 0.060),
        origin=Origin(xyz=(0.0, 0.004, -0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="hinge_knuckle",
    )
    cover.visual(
        Box((COVER_WIDTH * 0.82, 0.016, 0.012)),
        origin=Origin(xyz=(0.0, -(COVER_DEPTH - 0.008), 0.010)),
        material=trim_dark,
        name="front_handle",
    )

    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    for knob_index, knob_x in enumerate(KNOB_XS):
        knob = model.part(f"front_knob_{knob_index}")
        knob.visual(
            knob_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=enamel,
            name="knob_shell",
        )
        model.articulation(
            f"body_to_front_knob_{knob_index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(knob_x, -(BODY_DEPTH / 2.0), KNOB_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.4,
                velocity=6.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("cooktop_body")
    cover = object_model.get_part("glass_cover")
    cover_hinge = object_model.get_articulation("body_to_cover")
    knob_joint_names = [f"body_to_front_knob_{index}" for index in range(4)]

    cover_limits = cover_hinge.motion_limits
    ctx.check(
        "cover hinge opens upward over a real range",
        cover_hinge.articulation_type == ArticulationType.REVOLUTE
        and cover_limits is not None
        and cover_limits.lower == 0.0
        and cover_limits.upper is not None
        and cover_limits.upper >= math.radians(100.0),
        details=f"type={cover_hinge.articulation_type!r}, limits={cover_limits!r}",
    )

    knob_joints = [object_model.get_articulation(name) for name in knob_joint_names]
    ctx.check(
        "four front knobs use continuous rotation joints",
        len(knob_joints) == 4
        and all(
            joint.articulation_type == ArticulationType.CONTINUOUS
            and joint.motion_limits is not None
            and joint.motion_limits.lower is None
            and joint.motion_limits.upper is None
            for joint in knob_joints
        ),
        details=str(
            [
                (
                    joint.name,
                    joint.articulation_type,
                    None if joint.motion_limits is None else joint.motion_limits.lower,
                    None if joint.motion_limits is None else joint.motion_limits.upper,
                )
                for joint in knob_joints
            ]
        ),
    )

    with ctx.pose({cover_hinge: 0.0}):
        ctx.expect_overlap(
            cover,
            body,
            axes="xy",
            elem_a="cover_glass",
            elem_b="deck_plate",
            min_overlap=0.320,
            name="closed cover spans the burner field",
        )
        ctx.expect_gap(
            cover,
            body,
            axis="z",
            positive_elem="cover_glass",
            min_gap=0.006,
            max_gap=0.014,
            name="closed cover clears the cooktop hardware",
        )
        for knob_index in range(4):
            knob = object_model.get_part(f"front_knob_{knob_index}")
            ctx.expect_gap(
                body,
                knob,
                axis="y",
                positive_elem="front_fascia",
                negative_elem="knob_shell",
                min_gap=0.0,
                max_gap=0.0005,
                name=f"front_knob_{knob_index} sits just off the fascia",
            )

    closed_cover_aabb = ctx.part_element_world_aabb(cover, elem="cover_glass")
    with ctx.pose({cover_hinge: cover_limits.upper if cover_limits is not None else math.radians(105.0)}):
        open_cover_aabb = ctx.part_element_world_aabb(cover, elem="cover_glass")

    ctx.check(
        "open cover lifts high above the burner deck",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][2] > closed_cover_aabb[1][2] + 0.18,
        details=f"closed={closed_cover_aabb!r}, open={open_cover_aabb!r}",
    )

    return ctx.report()


object_model = build_object_model()
