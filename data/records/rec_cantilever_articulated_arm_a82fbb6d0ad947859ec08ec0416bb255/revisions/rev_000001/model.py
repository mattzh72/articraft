from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


SHOULDER_X = 0.17
SHOULDER_Z = 0.72
SHOULDER_LEN = 0.46
FORELINK_LEN = 0.36
WRIST_LEN = 0.095
POST_RADIUS = 0.055


def _circle_profile(
    cx: float,
    cy: float,
    radius: float,
    *,
    segments: int = 48,
) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * cos(2.0 * pi * index / segments),
            cy + radius * sin(2.0 * pi * index / segments),
        )
        for index in range(segments)
    ]


def _capsule_profile(
    length: float,
    radius: float,
    *,
    segments: int = 32,
) -> list[tuple[float, float]]:
    """Plan-view capsule from a pivot at x=0 to a carried pivot at x=length."""
    half = max(6, segments // 2)
    points: list[tuple[float, float]] = []
    for index in range(half + 1):
        angle = -pi / 2.0 + pi * index / half
        points.append((length + radius * cos(angle), radius * sin(angle)))
    for index in range(half + 1):
        angle = pi / 2.0 + pi * index / half
        points.append((radius * cos(angle), radius * sin(angle)))
    return points


def _link_plate_mesh(
    name: str,
    *,
    length: float,
    radius: float,
    thickness: float,
    start_hole_radius: float,
) -> object:
    """Single flat arm plate with a clearance bore at its parent joint."""
    geom = ExtrudeWithHolesGeometry(
        _capsule_profile(length, radius),
        [_circle_profile(0.0, 0.0, start_hole_radius, segments=40)],
        height=thickness,
        center=True,
    )
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_post_cantilever_arm")

    safety_yellow = model.material("safety_yellow", rgba=(0.92, 0.58, 0.08, 1.0))
    graphite = model.material("graphite", rgba=(0.12, 0.13, 0.14, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.24, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    blue_face = model.material("blue_face", rgba=(0.08, 0.20, 0.44, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    support = model.part("support")
    support.visual(
        Box((0.42, 0.30, 0.045)),
        origin=Origin(xyz=(-0.035, 0.0, 0.0225)),
        material=dark_steel,
        name="floor_plate",
    )
    support.visual(
        Cylinder(radius=POST_RADIUS, length=0.72),
        origin=Origin(xyz=(0.0, 0.0, 0.395)),
        material=graphite,
        name="post_tube",
    )
    support.visual(
        Cylinder(radius=0.076, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=brushed_steel,
        name="base_collar",
    )
    support.visual(
        Box((0.170, 0.128, 0.085)),
        origin=Origin(xyz=(0.105, 0.0, SHOULDER_Z - 0.020)),
        material=graphite,
        name="shoulder_block",
    )
    support.visual(
        Box((0.120, 0.070, 0.130)),
        origin=Origin(xyz=(0.115, 0.0, SHOULDER_Z - 0.030)),
        material=graphite,
        name="post_gusset",
    )
    support.visual(
        Cylinder(radius=0.042, length=0.170),
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z + 0.030)),
        material=brushed_steel,
        name="shoulder_pin",
    )
    support.visual(
        Cylinder(radius=0.068, length=0.022),
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z + 0.004)),
        material=dark_steel,
        name="lower_bearing_cap",
    )
    support.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z + 0.119)),
        material=brushed_steel,
        name="shoulder_nut",
    )
    # Small anchor bolts on the base plate, all connected by the plate below.
    for bolt_index, (x, y) in enumerate(
        [(-0.17, -0.105), (-0.17, 0.105), (0.095, -0.105), (0.095, 0.105)]
    ):
        support.visual(
            Cylinder(radius=0.014, length=0.010),
            origin=Origin(xyz=(x, y, 0.050)),
            material=brushed_steel,
            name=f"anchor_bolt_{bolt_index}",
        )

    shoulder = model.part("shoulder_link")
    shoulder.visual(
        _link_plate_mesh(
            "shoulder_link_plate",
            length=SHOULDER_LEN,
            radius=0.062,
            thickness=0.034,
            start_hole_radius=0.049,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=safety_yellow,
        name="shoulder_plate",
    )
    shoulder.visual(
        Cylinder(radius=0.021, length=0.126),
        origin=Origin(xyz=(SHOULDER_LEN, 0.0, 0.094)),
        material=brushed_steel,
        name="elbow_pin",
    )
    shoulder.visual(
        Cylinder(radius=0.051, length=0.014),
        origin=Origin(xyz=(SHOULDER_LEN, 0.0, 0.160)),
        material=dark_steel,
        name="elbow_cap",
    )
    shoulder.visual(
        Box((0.275, 0.030, 0.020)),
        origin=Origin(xyz=(0.235, 0.0, 0.084)),
        material=dark_steel,
        name="top_rib",
    )

    forelink = model.part("forelink")
    forelink.visual(
        _link_plate_mesh(
            "forelink_plate",
            length=FORELINK_LEN,
            radius=0.052,
            thickness=0.030,
            start_hole_radius=0.031,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.114)),
        material=safety_yellow,
        name="forelink_plate",
    )
    forelink.visual(
        Cylinder(radius=0.018, length=0.112),
        origin=Origin(xyz=(FORELINK_LEN, 0.0, 0.145)),
        material=brushed_steel,
        name="wrist_pin",
    )
    forelink.visual(
        Cylinder(radius=0.043, length=0.012),
        origin=Origin(xyz=(FORELINK_LEN, 0.0, 0.203)),
        material=dark_steel,
        name="wrist_cap",
    )
    forelink.visual(
        Box((0.205, 0.024, 0.018)),
        origin=Origin(xyz=(0.190, 0.0, 0.137)),
        material=dark_steel,
        name="forelink_rib",
    )

    wrist = model.part("wrist_face")
    wrist.visual(
        _link_plate_mesh(
            "wrist_receiver_plate",
            length=WRIST_LEN,
            radius=0.042,
            thickness=0.028,
            start_hole_radius=0.028,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.176)),
        material=safety_yellow,
        name="receiver_plate",
    )
    wrist.visual(
        Box((0.092, 0.044, 0.030)),
        origin=Origin(xyz=(0.062, 0.0, 0.176)),
        material=safety_yellow,
        name="wrist_neck",
    )
    wrist.visual(
        Box((0.028, 0.128, 0.150)),
        origin=Origin(xyz=(0.116, 0.0, 0.176)),
        material=blue_face,
        name="tool_face",
    )
    wrist.visual(
        Cylinder(radius=0.040, length=0.016),
        origin=Origin(xyz=(0.136, 0.0, 0.176), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="tool_boss",
    )
    wrist.visual(
        Box((0.012, 0.070, 0.038)),
        origin=Origin(xyz=(0.1355, 0.0, 0.176)),
        material=black_rubber,
        name="face_pad",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=support,
        child=shoulder,
        origin=Origin(xyz=(SHOULDER_X, 0.0, SHOULDER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.6, lower=-0.55, upper=0.85),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=forelink,
        origin=Origin(xyz=(SHOULDER_LEN, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.8, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=forelink,
        child=wrist,
        origin=Origin(xyz=(FORELINK_LEN, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.2, lower=-1.35, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    shoulder = object_model.get_part("shoulder_link")
    forelink = object_model.get_part("forelink")
    wrist = object_model.get_part("wrist_face")
    shoulder_joint = object_model.get_articulation("shoulder")
    elbow_joint = object_model.get_articulation("elbow")
    wrist_joint = object_model.get_articulation("wrist")

    ctx.expect_origin_gap(
        forelink,
        shoulder,
        axis="x",
        min_gap=SHOULDER_LEN - 0.005,
        max_gap=SHOULDER_LEN + 0.005,
        name="elbow joint is carried at the end of the shoulder link",
    )
    ctx.expect_origin_gap(
        wrist,
        forelink,
        axis="x",
        min_gap=FORELINK_LEN - 0.005,
        max_gap=FORELINK_LEN + 0.005,
        name="wrist joint is carried at the end of the forelink",
    )
    ctx.allow_overlap(
        support,
        shoulder,
        elem_a="shoulder_pin",
        elem_b="shoulder_plate",
        reason=(
            "The vertical shoulder pin is intentionally captured through the "
            "bored receiver plate; the mesh collision proxy fills that visual bore."
        ),
    )
    ctx.expect_within(
        support,
        shoulder,
        axes="xy",
        inner_elem="shoulder_pin",
        outer_elem="shoulder_plate",
        margin=0.0,
        name="shoulder pin is centered inside the receiver plate footprint",
    )
    ctx.expect_overlap(
        support,
        shoulder,
        axes="z",
        elem_a="shoulder_pin",
        elem_b="shoulder_plate",
        min_overlap=0.025,
        name="shoulder pin passes through the receiver thickness",
    )

    ctx.allow_overlap(
        forelink,
        shoulder,
        elem_a="forelink_plate",
        elem_b="elbow_pin",
        reason=(
            "The elbow pin is fixed to the shoulder link and captured through "
            "the forelink bearing bore; the mesh collision proxy fills that bore."
        ),
    )
    ctx.expect_within(
        shoulder,
        forelink,
        axes="xy",
        inner_elem="elbow_pin",
        outer_elem="forelink_plate",
        margin=0.0,
        name="elbow pin is centered inside the forelink receiver footprint",
    )
    ctx.expect_overlap(
        shoulder,
        forelink,
        axes="z",
        elem_a="elbow_pin",
        elem_b="forelink_plate",
        min_overlap=0.025,
        name="elbow pin passes through the forelink receiver thickness",
    )

    ctx.allow_overlap(
        forelink,
        wrist,
        elem_a="wrist_pin",
        elem_b="receiver_plate",
        reason=(
            "The wrist pin is fixed to the forelink and captured through the "
            "wrist receiver bore; the mesh collision proxy fills that bore."
        ),
    )
    ctx.expect_within(
        forelink,
        wrist,
        axes="xy",
        inner_elem="wrist_pin",
        outer_elem="receiver_plate",
        margin=0.0,
        name="wrist pin is centered inside the wrist receiver footprint",
    )
    ctx.expect_overlap(
        forelink,
        wrist,
        axes="z",
        elem_a="wrist_pin",
        elem_b="receiver_plate",
        min_overlap=0.023,
        name="wrist pin passes through the receiver thickness",
    )

    # At the neutral pose the whole articulated chain sits to +X of the post tube.
    one_side_ok = True
    details: list[str] = []
    for moving_part in (shoulder, forelink, wrist):
        aabb = ctx.part_world_aabb(moving_part)
        if aabb is None:
            one_side_ok = False
            details.append(f"{moving_part.name}: no AABB")
            continue
        min_x = aabb[0][0]
        details.append(f"{moving_part.name}.min_x={min_x:.3f}")
        one_side_ok = one_side_ok and min_x > POST_RADIUS + 0.020
    ctx.check(
        "neutral chain projects entirely to one side of the support post",
        one_side_ok,
        details=", ".join(details),
    )

    rest_forelink = ctx.part_world_position(forelink)
    rest_wrist = ctx.part_world_position(wrist)
    with ctx.pose({shoulder_joint: 0.55, elbow_joint: 0.45, wrist_joint: -0.45}):
        posed_forelink = ctx.part_world_position(forelink)
        posed_wrist = ctx.part_world_position(wrist)
    ctx.check(
        "serial revolute shoulder and elbow swing the carried joints in plan",
        rest_forelink is not None
        and posed_forelink is not None
        and rest_wrist is not None
        and posed_wrist is not None
        and posed_forelink[1] > rest_forelink[1] + 0.20
        and posed_wrist[1] > rest_wrist[1] + 0.30,
        details=f"rest_forelink={rest_forelink}, posed_forelink={posed_forelink}, "
        f"rest_wrist={rest_wrist}, posed_wrist={posed_wrist}",
    )

    return ctx.report()


object_model = build_object_model()
