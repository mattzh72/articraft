from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(
    cx: float,
    cy: float,
    radius: float,
    *,
    segments: int = 32,
    clockwise: bool = False,
) -> list[tuple[float, float]]:
    angles = [2.0 * math.pi * i / segments for i in range(segments)]
    if clockwise:
        angles = list(reversed(angles))
    return [(cx + radius * math.cos(a), cy + radius * math.sin(a)) for a in angles]


def _translate_profile(
    profile: list[tuple[float, float]], dx: float, dy: float = 0.0
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _stadium_profile(
    pivot_length: float,
    width: float,
    *,
    segments: int = 18,
) -> list[tuple[float, float]]:
    """Flat link outline with semicircular ends centered on the two pivots."""
    radius = width / 2.0
    pts: list[tuple[float, float]] = []
    # Bottom edge, distal end arc, top edge, proximal end arc: counter-clockwise.
    pts.append((0.0, -radius))
    pts.append((pivot_length, -radius))
    for i in range(1, segments + 1):
        a = -math.pi / 2.0 + math.pi * i / segments
        pts.append((pivot_length + radius * math.cos(a), radius * math.sin(a)))
    pts.append((0.0, radius))
    for i in range(1, segments + 1):
        a = math.pi / 2.0 + math.pi * i / segments
        pts.append((radius * math.cos(a), radius * math.sin(a)))
    return pts


def _slot_profile(
    center_x: float,
    center_y: float,
    length: float,
    width: float,
    *,
    segments: int = 12,
) -> list[tuple[float, float]]:
    radius = width / 2.0
    half_straight = max(0.0, length / 2.0 - radius)
    x0 = center_x - half_straight
    x1 = center_x + half_straight
    pts: list[tuple[float, float]] = []
    pts.append((x0, center_y - radius))
    pts.append((x1, center_y - radius))
    for i in range(1, segments + 1):
        a = -math.pi / 2.0 + math.pi * i / segments
        pts.append((x1 + radius * math.cos(a), center_y + radius * math.sin(a)))
    pts.append((x0, center_y + radius))
    for i in range(1, segments + 1):
        a = math.pi / 2.0 + math.pi * i / segments
        pts.append((x0 + radius * math.cos(a), center_y + radius * math.sin(a)))
    return pts


def _link_geometry(
    pivot_length: float,
    width: float,
    thickness: float,
    *,
    bore_radius: float = 0.012,
    slot_width: float = 0.014,
) -> ExtrudeWithHolesGeometry:
    slot_length = max(0.04, pivot_length - 0.105)
    return ExtrudeWithHolesGeometry(
        _stadium_profile(pivot_length, width),
        [
            _circle_profile(0.0, 0.0, bore_radius),
            _circle_profile(pivot_length, 0.0, bore_radius),
            _slot_profile(
                pivot_length / 2.0,
                0.0,
                slot_length,
                slot_width,
            ),
        ],
        thickness,
        center=True,
    )


def _base_plate_geometry() -> ExtrudeWithHolesGeometry:
    outer = _translate_profile(rounded_rect_profile(0.220, 0.130, 0.018), -0.055)
    holes = []
    for x in (-0.135, 0.025):
        for y in (-0.043, 0.043):
            holes.append(_circle_profile(x, y, 0.006))
    return ExtrudeWithHolesGeometry(outer, holes, 0.012, center=True)


def _platform_geometry() -> ExtrudeWithHolesGeometry:
    outer = _translate_profile(rounded_rect_profile(0.115, 0.074, 0.014), 0.047)
    holes = [
        _circle_profile(0.0, 0.0, 0.012),
        _circle_profile(0.052, -0.020, 0.004),
        _circle_profile(0.052, 0.020, 0.004),
        _slot_profile(0.082, 0.0, 0.030, 0.008),
    ]
    return ExtrudeWithHolesGeometry(outer, holes, 0.007, center=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_fold_out_arm")

    dark_anodized = model.material("dark_anodized", rgba=(0.055, 0.060, 0.065, 1.0))
    graphite = model.material("graphite_plate", rgba=(0.16, 0.17, 0.18, 1.0))
    steel = model.material("brushed_steel", rgba=(0.72, 0.70, 0.66, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    base = model.part("mounting_plate")
    base.visual(
        mesh_from_geometry(_base_plate_geometry(), "mounting_plate"),
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
        material=graphite,
        name="broad_plate",
    )
    # Four low countersunk heads make the broad anchor plate read as bolted down.
    for i, (x, y) in enumerate(
        ((-0.135, -0.043), (-0.135, 0.043), (0.025, -0.043), (0.025, 0.043))
    ):
        base.visual(
            Cylinder(radius=0.009, length=0.0025),
            origin=Origin(xyz=(x, y, -0.0119)),
            material=steel,
            name=f"screw_head_{i}",
        )
        base.visual(
            Cylinder(radius=0.0024, length=0.0027),
            origin=Origin(xyz=(x, y, -0.0102)),
            material=rubber,
            name=f"screw_recess_{i}",
        )
    base.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=graphite,
        name="root_standoff",
    )
    base.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.0080)),
        material=steel,
        name="root_pin",
    )

    link0_len = 0.280
    link1_len = 0.250
    link2_len = 0.220
    link_thickness = 0.006

    link_0 = model.part("link_0")
    link_0.visual(
        mesh_from_geometry(
            _link_geometry(link0_len, 0.052, link_thickness),
            "link_0_web",
        ),
        material=dark_anodized,
        name="flat_link",
    )
    for x in (0.0, link0_len):
        link_0.visual(
            Cylinder(radius=0.019, length=0.003),
            origin=Origin(xyz=(x, 0.0, 0.0040)),
            material=dark_anodized,
            name=f"upper_knuckle_{int(x > 0.0)}",
        )
    link_0.visual(
        Cylinder(radius=0.0068, length=0.003),
        origin=Origin(xyz=(link0_len, 0.0, 0.0045)),
        material=steel,
        name="distal_pin",
    )

    link_1 = model.part("link_1")
    link_1.visual(
        mesh_from_geometry(
            _link_geometry(link1_len, 0.048, link_thickness),
            "link_1_web",
        ),
        material=dark_anodized,
        name="flat_link",
    )
    link_1.visual(
        Cylinder(radius=0.018, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0040)),
        material=dark_anodized,
        name="upper_knuckle",
    )
    link_1.visual(
        Cylinder(radius=0.018, length=0.003),
        origin=Origin(xyz=(link1_len, 0.0, -0.0040)),
        material=dark_anodized,
        name="lower_knuckle",
    )
    link_1.visual(
        Cylinder(radius=0.0068, length=0.003),
        origin=Origin(xyz=(link1_len, 0.0, -0.0045)),
        material=steel,
        name="distal_pin",
    )

    link_2 = model.part("link_2")
    link_2.visual(
        mesh_from_geometry(
            _link_geometry(link2_len, 0.046, link_thickness),
            "link_2_web",
        ),
        material=dark_anodized,
        name="flat_link",
    )
    for x in (0.0, link2_len):
        link_2.visual(
            Cylinder(radius=0.017, length=0.003),
            origin=Origin(xyz=(x, 0.0, 0.0040)),
            material=dark_anodized,
            name=f"upper_knuckle_{int(x > 0.0)}",
        )
    link_2.visual(
        Cylinder(radius=0.0068, length=0.003),
        origin=Origin(xyz=(link2_len, 0.0, 0.0045)),
        material=steel,
        name="distal_pin",
    )

    platform = model.part("platform_bracket")
    platform.visual(
        mesh_from_geometry(_platform_geometry(), "platform_bracket"),
        material=graphite,
        name="small_platform",
    )
    platform.visual(
        Box((0.080, 0.005, 0.018)),
        origin=Origin(xyz=(0.063, -0.0395, 0.006)),
        material=graphite,
        name="side_lip_0",
    )
    platform.visual(
        Box((0.080, 0.005, 0.018)),
        origin=Origin(xyz=(0.063, 0.0395, 0.006)),
        material=graphite,
        name="side_lip_1",
    )
    platform.visual(
        Cylinder(radius=0.017, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0040)),
        material=graphite,
        name="pivot_knuckle",
    )
    platform.visual(
        Box((0.046, 0.056, 0.002)),
        origin=Origin(xyz=(0.079, 0.0, 0.0045)),
        material=rubber,
        name="top_pad",
    )

    limits = MotionLimits(effort=12.0, velocity=2.0, lower=-2.35, upper=2.35)
    model.articulation(
        "root_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_0,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=limits,
    )
    model.articulation(
        "elbow_joint_0",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(link0_len, 0.0, 0.009)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=limits,
    )
    model.articulation(
        "elbow_joint_1",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(link1_len, 0.0, -0.009)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=limits,
    )
    model.articulation(
        "wrist_joint",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=platform,
        origin=Origin(xyz=(link2_len, 0.0, 0.009)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-2.1, upper=2.1),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    joints = [
        object_model.get_articulation("root_joint"),
        object_model.get_articulation("elbow_joint_0"),
        object_model.get_articulation("elbow_joint_1"),
        object_model.get_articulation("wrist_joint"),
    ]
    ctx.check(
        "four revolute joints",
        len(joints) == 4
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joints={[j.name for j in joints]}",
    )
    ctx.check(
        "all joints bend in the plate plane",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 0.0, 1.0) for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )
    ctx.check(
        "chain starts from grounded mounting plate",
        joints[0].parent == "mounting_plate"
        and joints[0].child == "link_0"
        and joints[1].parent == "link_0"
        and joints[1].child == "link_1"
        and joints[2].parent == "link_1"
        and joints[2].child == "link_2"
        and joints[3].parent == "link_2"
        and joints[3].child == "platform_bracket",
        details=", ".join(f"{j.parent}->{j.child}" for j in joints),
    )

    platform = object_model.get_part("platform_bracket")
    rest_pos = ctx.part_world_position(platform)
    with ctx.pose(
        {
            joints[0]: 0.55,
            joints[1]: -1.05,
            joints[2]: 0.85,
            joints[3]: -0.45,
        }
    ):
        folded_pos = ctx.part_world_position(platform)
    ctx.check(
        "folded pose stays planar and moves the platform",
        rest_pos is not None
        and folded_pos is not None
        and abs(folded_pos[2] - rest_pos[2]) < 0.001
        and abs(folded_pos[1] - rest_pos[1]) > 0.050,
        details=f"rest={rest_pos}, folded={folded_pos}",
    )

    return ctx.report()


object_model = build_object_model()
