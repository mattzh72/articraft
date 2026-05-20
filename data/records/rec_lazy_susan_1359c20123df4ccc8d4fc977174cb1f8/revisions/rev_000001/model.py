from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


POST_RADIUS = 0.024
TRAY_RADIUS = 0.280
TRAY_RIM_THICKNESS = 0.014
TRAY_FLOOR_THICKNESS = 0.008
TRAY_RIM_HEIGHT = 0.044
HUB_INNER_RADIUS = 0.0255
HUB_OUTER_RADIUS = 0.072
HUB_BOTTOM = -0.030
HUB_TOP = 0.055
LOWER_TIER_Z = 0.140
UPPER_TIER_Z = 0.390


def _tray_lathe_mesh(name: str):
    """A connected annular tray with a raised outside rim and center clip hub."""
    profile = [
        (HUB_INNER_RADIUS, HUB_BOTTOM),
        (HUB_OUTER_RADIUS, HUB_BOTTOM),
        (HUB_OUTER_RADIUS, -TRAY_FLOOR_THICKNESS),
        (TRAY_RADIUS, -TRAY_FLOOR_THICKNESS),
        (TRAY_RADIUS, TRAY_RIM_HEIGHT),
        (TRAY_RADIUS - TRAY_RIM_THICKNESS, TRAY_RIM_HEIGHT),
        (TRAY_RADIUS - TRAY_RIM_THICKNESS, 0.004),
        (HUB_OUTER_RADIUS, 0.004),
        (HUB_OUTER_RADIUS, HUB_TOP),
        (HUB_INNER_RADIUS, HUB_TOP),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=96, closed=True), name)


def _add_bearing_pads(part, prefix: str, material) -> None:
    """Three tiny nylon pads touch the metal post and are buried into the tray hub."""
    pad_depth = 0.010
    pad_width = 0.010
    pad_height = 0.046
    pad_center_radius = POST_RADIUS + pad_depth / 2.0
    for i, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        part.visual(
            Box((pad_depth, pad_width, pad_height)),
            origin=Origin(
                xyz=(pad_center_radius * math.cos(angle), pad_center_radius * math.sin(angle), 0.012),
                rpy=(0.0, 0.0, angle),
            ),
            material=material,
            name=f"{prefix}_bearing_pad_{i}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_tier_pantry_lazy_susan")

    warm_white = model.material("warm_white_plastic", rgba=(0.94, 0.91, 0.84, 1.0))
    pale_grey = model.material("pale_grey_plastic", rgba=(0.82, 0.82, 0.78, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.07, 0.07, 0.065, 1.0))

    spindle = model.part("center_post")
    spindle.visual(
        Cylinder(radius=0.165, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=pale_grey,
        name="weighted_base",
    )
    spindle.visual(
        Cylinder(radius=POST_RADIUS, length=0.585),
        origin=Origin(xyz=(0.0, 0.0, 0.3175)),
        material=satin_steel,
        name="post_shaft",
    )
    spindle.visual(
        Cylinder(radius=0.052, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.612)),
        material=satin_steel,
        name="top_cap",
    )

    clip_radius = 0.052
    clip_thickness = 0.006
    clip_gap = 0.003
    for i, z in enumerate((LOWER_TIER_Z, UPPER_TIER_Z)):
        spindle.visual(
            Cylinder(radius=clip_radius, length=clip_thickness),
            origin=Origin(xyz=(0.0, 0.0, z + HUB_BOTTOM - clip_gap - clip_thickness / 2.0)),
            material=satin_steel,
            name=f"lower_clip_{i}",
        )
        spindle.visual(
            Cylinder(radius=clip_radius, length=clip_thickness),
            origin=Origin(xyz=(0.0, 0.0, z + HUB_TOP + clip_gap + clip_thickness / 2.0)),
            material=satin_steel,
            name=f"upper_clip_{i}",
        )

    for i, (x, y) in enumerate(((-0.105, -0.105), (0.105, -0.105), (0.105, 0.105), (-0.105, 0.105))):
        spindle.visual(
            Cylinder(radius=0.020, length=0.006),
            origin=Origin(xyz=(x, y, 0.003)),
            material=dark_rubber,
            name=f"foot_{i}",
        )

    lower_tray = model.part("lower_tray")
    lower_tray.visual(
        _tray_lathe_mesh("lower_annular_tray"),
        material=warm_white,
        name="lower_tray_body",
    )
    lower_tray.visual(
        Box((0.165, 0.016, 0.012)),
        origin=Origin(xyz=(0.155, 0.0, 0.005)),
        material=pale_grey,
        name="lower_index_rib",
    )
    _add_bearing_pads(lower_tray, "lower", pale_grey)

    upper_tray = model.part("upper_tray")
    upper_tray.visual(
        _tray_lathe_mesh("upper_annular_tray"),
        material=warm_white,
        name="upper_tray_body",
    )
    upper_tray.visual(
        Box((0.165, 0.016, 0.012)),
        origin=Origin(xyz=(0.0, 0.155, 0.005), rpy=(0.0, 0.0, math.pi / 2.0)),
        material=pale_grey,
        name="upper_index_rib",
    )
    _add_bearing_pads(upper_tray, "upper", pale_grey)

    model.articulation(
        "lower_spin",
        ArticulationType.CONTINUOUS,
        parent=spindle,
        child=lower_tray,
        origin=Origin(xyz=(0.0, 0.0, LOWER_TIER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )
    model.articulation(
        "upper_spin",
        ArticulationType.CONTINUOUS,
        parent=spindle,
        child=upper_tray,
        origin=Origin(xyz=(0.0, 0.0, UPPER_TIER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_tray = object_model.get_part("lower_tray")
    upper_tray = object_model.get_part("upper_tray")
    center_post = object_model.get_part("center_post")
    lower_spin = object_model.get_articulation("lower_spin")
    upper_spin = object_model.get_articulation("upper_spin")

    ctx.check(
        "two_independent_continuous_tray_joints",
        lower_spin.articulation_type == ArticulationType.CONTINUOUS
        and upper_spin.articulation_type == ArticulationType.CONTINUOUS
        and lower_spin.parent == "center_post"
        and upper_spin.parent == "center_post",
        details=f"lower={lower_spin}, upper={upper_spin}",
    )
    ctx.check(
        "shared_vertical_axis",
        lower_spin.axis == (0.0, 0.0, 1.0)
        and upper_spin.axis == (0.0, 0.0, 1.0)
        and abs(lower_spin.origin.xyz[0] - upper_spin.origin.xyz[0]) < 1e-6
        and abs(lower_spin.origin.xyz[1] - upper_spin.origin.xyz[1]) < 1e-6,
        details=f"lower_origin={lower_spin.origin}, upper_origin={upper_spin.origin}",
    )
    ctx.expect_origin_gap(
        upper_tray,
        lower_tray,
        axis="z",
        min_gap=0.20,
        max_gap=0.30,
        name="upper tier is stacked above lower tier",
    )
    ctx.expect_gap(
        upper_tray,
        lower_tray,
        axis="z",
        min_gap=0.12,
        name="tray bodies have pantry clearance between tiers",
    )
    ctx.expect_overlap(
        lower_tray,
        center_post,
        axes="xy",
        min_overlap=0.045,
        elem_a="lower_tray_body",
        elem_b="post_shaft",
        name="lower hub surrounds the shared center post",
    )
    ctx.expect_overlap(
        upper_tray,
        center_post,
        axes="xy",
        min_overlap=0.045,
        elem_a="upper_tray_body",
        elem_b="post_shaft",
        name="upper hub surrounds the shared center post",
    )
    for index, tray in enumerate((lower_tray, upper_tray)):
        tier_name = "lower" if index == 0 else "upper"
        ctx.expect_gap(
            tray,
            center_post,
            axis="z",
            min_gap=0.0015,
            max_gap=0.006,
            positive_elem=f"{tier_name}_tray_body",
            negative_elem=f"lower_clip_{index}",
            name=f"{tier_name} tray is retained above its lower clip",
        )
        ctx.expect_gap(
            center_post,
            tray,
            axis="z",
            min_gap=0.0015,
            max_gap=0.006,
            positive_elem=f"upper_clip_{index}",
            negative_elem=f"{tier_name}_tray_body",
            name=f"{tier_name} tray is retained below its upper clip",
        )

    def _aabb_center_xy(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((float(lo[0]) + float(hi[0])) * 0.5, (float(lo[1]) + float(hi[1])) * 0.5)

    lower_rest = _aabb_center_xy(ctx.part_element_world_aabb(lower_tray, elem="lower_index_rib"))
    upper_rest = _aabb_center_xy(ctx.part_element_world_aabb(upper_tray, elem="upper_index_rib"))
    with ctx.pose({lower_spin: math.pi / 2.0}):
        lower_rotated = _aabb_center_xy(ctx.part_element_world_aabb(lower_tray, elem="lower_index_rib"))
        upper_after_lower = _aabb_center_xy(ctx.part_element_world_aabb(upper_tray, elem="upper_index_rib"))
    with ctx.pose({upper_spin: -math.pi / 2.0}):
        upper_rotated = _aabb_center_xy(ctx.part_element_world_aabb(upper_tray, elem="upper_index_rib"))

    ctx.check(
        "lower tray rotates without carrying upper tray",
        lower_rest is not None
        and lower_rotated is not None
        and upper_rest is not None
        and upper_after_lower is not None
        and lower_rotated[1] > lower_rest[1] + 0.10
        and abs(upper_after_lower[0] - upper_rest[0]) < 0.002
        and abs(upper_after_lower[1] - upper_rest[1]) < 0.002,
        details=f"lower_rest={lower_rest}, lower_rotated={lower_rotated}, upper_rest={upper_rest}, upper_after_lower={upper_after_lower}",
    )
    ctx.check(
        "upper tray rotates independently",
        upper_rest is not None
        and upper_rotated is not None
        and upper_rotated[0] > upper_rest[0] + 0.10,
        details=f"upper_rest={upper_rest}, upper_rotated={upper_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
