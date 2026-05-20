from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


TOP_Z = 0.95
SECTION_HALF_WIDTH = 0.34
FRAME_RADIUS = 0.010
BAR_RADIUS = 0.0055
BAR_LENGTH = 2.0 * (SECTION_HALF_WIDTH + FRAME_RADIUS * 0.8)
HINGE_RADIUS = 0.0075
WING_HINGE_Y = 0.055
HINGE_BAR_LENGTH = 0.055
HINGE_SLEEVE_LENGTH = 0.095
PIVOT_BAR_LENGTH = 0.130
PIVOT_SLEEVE_LENGTH = 0.085

CENTER_FOOT_Y = 0.28
CENTER_FOOT_Z = 0.05

WING_OUT_Y = 0.54
WING_OUT_Z = -0.58
WING_TOP_CLEARANCE_Y = 0.026
WING_TOP_CLEARANCE_Z = -0.040

CENTER_BAR_TS = (0.20, 0.34, 0.48, 0.62, 0.76, 0.88)
WING_BAR_TS = (0.25, 0.45, 0.64, 0.82)
PIVOT_T = 0.88
LINK_T = 0.82


def _segment_origin(p0: tuple[float, float, float], p1: tuple[float, float, float]) -> tuple[Origin, float]:
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    center = ((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0, (p0[2] + p1[2]) / 2.0)
    ux = dx / length
    uy = dy / length
    uz = dz / length
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
    yaw = math.atan2(uy, ux)
    return Origin(xyz=center, rpy=(0.0, pitch, yaw)), length


def _add_rod(
    part,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    *,
    radius: float,
    material: str,
    name: str | None = None,
) -> None:
    origin, length = _segment_origin(p0, p1)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _lerp_point(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    t: float,
) -> tuple[float, float, float]:
    return (
        start[0] + (end[0] - start[0]) * t,
        start[1] + (end[1] - start[1]) * t,
        start[2] + (end[2] - start[2]) * t,
    )


def _add_cross_bar(
    part,
    center: tuple[float, float, float],
    *,
    radius: float,
    material: str,
    name: str,
    length: float = BAR_LENGTH,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _add_center_section(part, *, sign: float, material: str, prefix: str) -> tuple[float, float, float]:
    top_left = (-SECTION_HALF_WIDTH, 0.0, TOP_Z)
    top_right = (SECTION_HALF_WIDTH, 0.0, TOP_Z)
    foot_left = (-SECTION_HALF_WIDTH, sign * CENTER_FOOT_Y, CENTER_FOOT_Z)
    foot_right = (SECTION_HALF_WIDTH, sign * CENTER_FOOT_Y, CENTER_FOOT_Z)

    _add_rod(part, top_left, foot_left, radius=FRAME_RADIUS, material=material, name=f"{prefix}_rail_0")
    _add_rod(part, top_right, foot_right, radius=FRAME_RADIUS, material=material, name=f"{prefix}_rail_1")

    for index, t in enumerate(CENTER_BAR_TS):
        center = _lerp_point((0.0, 0.0, TOP_Z), (0.0, sign * CENTER_FOOT_Y, CENTER_FOOT_Z), t)
        name = f"{prefix}_bar_{index}"
        length = BAR_LENGTH
        if abs(t - PIVOT_T) < 1e-9:
            name = f"{prefix}_pivot_bar"
            length = PIVOT_BAR_LENGTH
        _add_cross_bar(part, center, radius=BAR_RADIUS, material=material, name=name, length=length)

    return _lerp_point((0.0, 0.0, TOP_Z), (0.0, sign * CENTER_FOOT_Y, CENTER_FOOT_Z), PIVOT_T)


def _add_wing_section(part, *, sign: float, material: str) -> tuple[float, float, float]:
    _add_cross_bar(
        part,
        (0.0, 0.0, 0.0),
        radius=HINGE_RADIUS,
        material=material,
        name="hinge_sleeve",
        length=HINGE_SLEEVE_LENGTH,
    )
    top_left = (-SECTION_HALF_WIDTH, sign * WING_TOP_CLEARANCE_Y, WING_TOP_CLEARANCE_Z)
    top_right = (SECTION_HALF_WIDTH, sign * WING_TOP_CLEARANCE_Y, WING_TOP_CLEARANCE_Z)
    edge_left = (-SECTION_HALF_WIDTH, sign * WING_OUT_Y, WING_OUT_Z)
    edge_right = (SECTION_HALF_WIDTH, sign * WING_OUT_Y, WING_OUT_Z)
    sleeve_half = HINGE_SLEEVE_LENGTH / 2.0

    _add_rod(
        part,
        (-sleeve_half, 0.0, 0.0),
        top_left,
        radius=FRAME_RADIUS * 0.82,
        material=material,
        name="hinge_brace_0",
    )
    _add_rod(
        part,
        (sleeve_half, 0.0, 0.0),
        top_right,
        radius=FRAME_RADIUS * 0.82,
        material=material,
        name="hinge_brace_1",
    )
    _add_rod(part, top_left, edge_left, radius=FRAME_RADIUS * 0.95, material=material, name="rail_0")
    _add_rod(part, top_right, edge_right, radius=FRAME_RADIUS * 0.95, material=material, name="rail_1")
    _add_cross_bar(part, (0.0, sign * WING_OUT_Y, WING_OUT_Z), radius=BAR_RADIUS, material=material, name="outer_rail")

    link_point = (0.0, 0.0, 0.0)
    for index, t in enumerate(WING_BAR_TS):
        center = _lerp_point((0.0, sign * WING_TOP_CLEARANCE_Y, WING_TOP_CLEARANCE_Z), (0.0, sign * WING_OUT_Y, WING_OUT_Z), t)
        name = f"bar_{index}"
        if abs(t - LINK_T) < 1e-9:
            name = "link_bar"
            link_point = center
        _add_cross_bar(part, center, radius=BAR_RADIUS, material=material, name=name)

    return link_point


def _add_spreader(part, *, sign: float, tip_point: tuple[float, float, float], material: str) -> None:
    hinge_radius = FRAME_RADIUS * 0.72
    rod_radius = FRAME_RADIUS * 0.62
    tip_length = math.sqrt(tip_point[1] * tip_point[1] + tip_point[2] * tip_point[2])
    start_scale = hinge_radius / tip_length if tip_length > 1e-9 else 0.0
    hinge_stub = (0.0, tip_point[1] * start_scale, tip_point[2] * start_scale)

    _add_cross_bar(
        part,
        (0.0, 0.0, 0.0),
        radius=hinge_radius,
        material=material,
        name="hinge_sleeve",
        length=PIVOT_SLEEVE_LENGTH,
    )
    _add_rod(part, hinge_stub, tip_point, radius=rod_radius, material=material, name="link")
    _add_cross_bar(part, tip_point, radius=FRAME_RADIUS * 0.64, material=material, name="tip")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="accordion_drying_rack")

    frame_finish = model.material("frame_finish", rgba=(0.79, 0.81, 0.84, 1.0))
    bar_finish = model.material("bar_finish", rgba=(0.90, 0.92, 0.94, 1.0))
    hinge_finish = model.material("hinge_finish", rgba=(0.42, 0.45, 0.48, 1.0))

    center_frame = model.part("center_frame")
    _add_cross_bar(center_frame, (0.0, 0.0, TOP_Z), radius=FRAME_RADIUS * 1.05, material=hinge_finish.name, name="ridge_bar")
    front_pivot = _add_center_section(center_frame, sign=1.0, material=frame_finish.name, prefix="front")
    rear_pivot = _add_center_section(center_frame, sign=-1.0, material=frame_finish.name, prefix="rear")
    hinge_half = HINGE_BAR_LENGTH / 2.0
    _add_cross_bar(
        center_frame,
        (0.0, WING_HINGE_Y, TOP_Z),
        radius=HINGE_RADIUS,
        material=hinge_finish.name,
        name="front_hinge_bar",
        length=HINGE_BAR_LENGTH,
    )
    _add_cross_bar(
        center_frame,
        (0.0, -WING_HINGE_Y, TOP_Z),
        radius=HINGE_RADIUS,
        material=hinge_finish.name,
        name="rear_hinge_bar",
        length=HINGE_BAR_LENGTH,
    )
    _add_rod(
        center_frame,
        (-hinge_half, WING_HINGE_Y, TOP_Z),
        (-hinge_half, 0.0, TOP_Z),
        radius=FRAME_RADIUS * 0.82,
        material=hinge_finish.name,
        name="front_hinge_brace_0",
    )
    _add_rod(
        center_frame,
        (hinge_half, WING_HINGE_Y, TOP_Z),
        (hinge_half, 0.0, TOP_Z),
        radius=FRAME_RADIUS * 0.82,
        material=hinge_finish.name,
        name="front_hinge_brace_1",
    )
    _add_rod(
        center_frame,
        (-hinge_half, -WING_HINGE_Y, TOP_Z),
        (-hinge_half, 0.0, TOP_Z),
        radius=FRAME_RADIUS * 0.82,
        material=hinge_finish.name,
        name="rear_hinge_brace_0",
    )
    _add_rod(
        center_frame,
        (hinge_half, -WING_HINGE_Y, TOP_Z),
        (hinge_half, 0.0, TOP_Z),
        radius=FRAME_RADIUS * 0.82,
        material=hinge_finish.name,
        name="rear_hinge_brace_1",
    )
    pivot_half = PIVOT_BAR_LENGTH / 2.0
    _add_rod(
        center_frame,
        (-pivot_half, front_pivot[1], front_pivot[2]),
        (-SECTION_HALF_WIDTH, front_pivot[1], front_pivot[2]),
        radius=FRAME_RADIUS * 0.68,
        material=hinge_finish.name,
        name="front_pivot_brace_0",
    )
    _add_rod(
        center_frame,
        (pivot_half, front_pivot[1], front_pivot[2]),
        (SECTION_HALF_WIDTH, front_pivot[1], front_pivot[2]),
        radius=FRAME_RADIUS * 0.68,
        material=hinge_finish.name,
        name="front_pivot_brace_1",
    )
    _add_rod(
        center_frame,
        (-pivot_half, rear_pivot[1], rear_pivot[2]),
        (-SECTION_HALF_WIDTH, rear_pivot[1], rear_pivot[2]),
        radius=FRAME_RADIUS * 0.68,
        material=hinge_finish.name,
        name="rear_pivot_brace_0",
    )
    _add_rod(
        center_frame,
        (pivot_half, rear_pivot[1], rear_pivot[2]),
        (SECTION_HALF_WIDTH, rear_pivot[1], rear_pivot[2]),
        radius=FRAME_RADIUS * 0.68,
        material=hinge_finish.name,
        name="rear_pivot_brace_1",
    )

    for name in ("front_bar_0", "front_bar_1", "front_bar_2", "front_bar_3", "front_bar_4", "front_pivot_bar"):
        center_frame.get_visual(name).material = bar_finish.name
    for name in ("rear_bar_0", "rear_bar_1", "rear_bar_2", "rear_bar_3", "rear_bar_4", "rear_pivot_bar"):
        center_frame.get_visual(name).material = bar_finish.name

    front_wing = model.part("front_wing")
    front_link_point = _add_wing_section(front_wing, sign=1.0, material=frame_finish.name)
    front_link_world = (0.0, WING_HINGE_Y + front_link_point[1], TOP_Z + front_link_point[2])
    front_wing.get_visual("outer_rail").material = bar_finish.name
    front_wing.get_visual("link_bar").material = bar_finish.name
    for name in ("bar_0", "bar_1", "bar_2"):
        front_wing.get_visual(name).material = bar_finish.name

    rear_wing = model.part("rear_wing")
    rear_link_point = _add_wing_section(rear_wing, sign=-1.0, material=frame_finish.name)
    rear_link_world = (0.0, -WING_HINGE_Y + rear_link_point[1], TOP_Z + rear_link_point[2])
    rear_wing.get_visual("outer_rail").material = bar_finish.name
    rear_wing.get_visual("link_bar").material = bar_finish.name
    for name in ("bar_0", "bar_1", "bar_2"):
        rear_wing.get_visual(name).material = bar_finish.name

    front_spreader = model.part("front_spreader")
    front_tip_world = (0.0, front_link_world[1] - 0.014, front_link_world[2] - 0.020)
    _add_spreader(
        front_spreader,
        sign=1.0,
        tip_point=(
            front_tip_world[0] - front_pivot[0],
            front_tip_world[1] - front_pivot[1],
            front_tip_world[2] - front_pivot[2],
        ),
        material=hinge_finish.name,
    )

    rear_spreader = model.part("rear_spreader")
    rear_tip_world = (0.0, rear_link_world[1] + 0.014, rear_link_world[2] - 0.020)
    _add_spreader(
        rear_spreader,
        sign=-1.0,
        tip_point=(
            rear_tip_world[0] - rear_pivot[0],
            rear_tip_world[1] - rear_pivot[1],
            rear_tip_world[2] - rear_pivot[2],
        ),
        material=hinge_finish.name,
    )

    wing_limits = MotionLimits(lower=0.0, upper=0.78, effort=8.0, velocity=1.6)
    spreader_limits = MotionLimits(lower=0.0, upper=0.78, effort=5.0, velocity=1.6)

    model.articulation(
        "front_wing_hinge",
        ArticulationType.REVOLUTE,
        parent=center_frame,
        child=front_wing,
        origin=Origin(xyz=(0.0, WING_HINGE_Y, TOP_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=wing_limits,
    )
    model.articulation(
        "rear_wing_hinge",
        ArticulationType.REVOLUTE,
        parent=center_frame,
        child=rear_wing,
        origin=Origin(xyz=(0.0, -WING_HINGE_Y, TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=wing_limits,
        mimic=Mimic(joint="front_wing_hinge"),
    )
    model.articulation(
        "front_spreader_hinge",
        ArticulationType.REVOLUTE,
        parent=center_frame,
        child=front_spreader,
        origin=Origin(xyz=front_pivot),
        axis=(1.0, 0.0, 0.0),
        motion_limits=spreader_limits,
        mimic=Mimic(joint="front_wing_hinge"),
    )
    model.articulation(
        "rear_spreader_hinge",
        ArticulationType.REVOLUTE,
        parent=center_frame,
        child=rear_spreader,
        origin=Origin(xyz=rear_pivot),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=spreader_limits,
        mimic=Mimic(joint="front_wing_hinge"),
    )

    return model


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return (
        (lower[0] + upper[0]) / 2.0,
        (lower[1] + upper[1]) / 2.0,
        (lower[2] + upper[2]) / 2.0,
    )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    center_frame = object_model.get_part("center_frame")
    front_wing = object_model.get_part("front_wing")
    rear_wing = object_model.get_part("rear_wing")
    front_spreader = object_model.get_part("front_spreader")
    rear_spreader = object_model.get_part("rear_spreader")
    front_wing_hinge = object_model.get_articulation("front_wing_hinge")

    ctx.allow_overlap(
        center_frame,
        front_wing,
        elem_a="front_hinge_bar",
        elem_b="hinge_sleeve",
        reason="The front wing rotates on the dedicated front hinge barrel at the top of the center frame.",
    )
    ctx.allow_overlap(
        center_frame,
        front_wing,
        elem_a="front_hinge_brace_0",
        elem_b="hinge_sleeve",
        reason="The front hinge brace is intentionally simplified to meet the front wing sleeve at the hinge knuckle.",
    )
    ctx.allow_overlap(
        center_frame,
        front_wing,
        elem_a="front_hinge_brace_1",
        elem_b="hinge_sleeve",
        reason="The opposite front hinge brace also meets the wing sleeve at the same simplified hinge knuckle.",
    )
    ctx.allow_overlap(
        center_frame,
        rear_wing,
        elem_a="rear_hinge_bar",
        elem_b="hinge_sleeve",
        reason="The rear wing rotates on the dedicated rear hinge barrel at the top of the center frame.",
    )
    ctx.allow_overlap(
        center_frame,
        rear_wing,
        elem_a="rear_hinge_brace_0",
        elem_b="hinge_sleeve",
        reason="The rear hinge brace is intentionally simplified to meet the rear wing sleeve at the hinge knuckle.",
    )
    ctx.allow_overlap(
        center_frame,
        rear_wing,
        elem_a="rear_hinge_brace_1",
        elem_b="hinge_sleeve",
        reason="The opposite rear hinge brace also meets the wing sleeve at the same simplified hinge knuckle.",
    )
    ctx.allow_overlap(
        center_frame,
        front_spreader,
        elem_a="front_pivot_bar",
        elem_b="hinge_sleeve",
        reason="The front spreader pivots around the lower front pivot bar near the feet.",
    )
    ctx.allow_overlap(
        center_frame,
        rear_spreader,
        elem_a="rear_pivot_bar",
        elem_b="hinge_sleeve",
        reason="The rear spreader pivots around the lower rear pivot bar near the feet.",
    )

    ctx.expect_gap(
        front_wing,
        center_frame,
        axis="y",
        positive_elem="outer_rail",
        negative_elem="ridge_bar",
        min_gap=0.42,
        max_gap=0.64,
        name="front wing deploys beyond the center ridge",
    )
    ctx.expect_gap(
        center_frame,
        rear_wing,
        axis="y",
        positive_elem="ridge_bar",
        negative_elem="outer_rail",
        min_gap=0.42,
        max_gap=0.64,
        name="rear wing deploys beyond the center ridge",
    )
    ctx.expect_overlap(
        front_wing,
        rear_wing,
        axes="x",
        elem_a="outer_rail",
        elem_b="outer_rail",
        min_overlap=0.66,
        name="all rack sections share the same drying width",
    )
    ctx.expect_gap(
        front_wing,
        front_spreader,
        axis="z",
        positive_elem="link_bar",
        negative_elem="tip",
        min_gap=0.0,
        max_gap=0.05,
        name="front spreader finishes just under the wing link bar",
    )
    ctx.expect_gap(
        rear_wing,
        rear_spreader,
        axis="z",
        positive_elem="link_bar",
        negative_elem="tip",
        min_gap=0.0,
        max_gap=0.05,
        name="rear spreader finishes just under the wing link bar",
    )

    front_rest = _aabb_center(ctx.part_element_world_aabb(front_wing, elem="outer_rail"))
    rear_rest = _aabb_center(ctx.part_element_world_aabb(rear_wing, elem="outer_rail"))
    front_spreader_rest = _aabb_center(ctx.part_element_world_aabb(front_spreader, elem="tip"))
    rear_spreader_rest = _aabb_center(ctx.part_element_world_aabb(rear_spreader, elem="tip"))

    limits = front_wing_hinge.motion_limits
    if limits is not None and limits.upper is not None:
        with ctx.pose({front_wing_hinge: limits.upper}):
            front_folded = _aabb_center(ctx.part_element_world_aabb(front_wing, elem="outer_rail"))
            rear_folded = _aabb_center(ctx.part_element_world_aabb(rear_wing, elem="outer_rail"))
            front_spreader_folded = _aabb_center(ctx.part_element_world_aabb(front_spreader, elem="tip"))
            rear_spreader_folded = _aabb_center(ctx.part_element_world_aabb(rear_spreader, elem="tip"))

            wings_fold_inward = (
                front_rest is not None
                and rear_rest is not None
                and front_folded is not None
                and rear_folded is not None
                and front_folded[1] < front_rest[1] - 0.30
                and rear_folded[1] > rear_rest[1] + 0.30
                and abs(front_folded[1] + rear_folded[1]) < 0.03
                and abs(front_folded[2] - rear_folded[2]) < 0.03
            )
            ctx.check(
                "wing sections collapse inward in matched alignment",
                wings_fold_inward,
                details=(
                    f"front_rest={front_rest}, rear_rest={rear_rest}, "
                    f"front_folded={front_folded}, rear_folded={rear_folded}"
                ),
            )

            spreaders_track = (
                front_spreader_rest is not None
                and rear_spreader_rest is not None
                and front_spreader_folded is not None
                and rear_spreader_folded is not None
                and front_spreader_folded[1] < front_spreader_rest[1] - 0.07
                and rear_spreader_folded[1] > rear_spreader_rest[1] + 0.07
                and front_spreader_folded[2] > front_spreader_rest[2] + 0.03
                and rear_spreader_folded[2] > rear_spreader_rest[2] + 0.03
            )
            ctx.check(
                "lower spreaders rotate with the folding wings",
                spreaders_track,
                details=(
                    f"front_rest={front_spreader_rest}, rear_rest={rear_spreader_rest}, "
                    f"front_folded={front_spreader_folded}, rear_folded={rear_spreader_folded}"
                ),
            )

    return ctx.report()


object_model = build_object_model()
