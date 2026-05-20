from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_easel")

    ash_wood = model.material("ash_wood", rgba=(0.76, 0.63, 0.44, 1.0))
    walnut = model.material("walnut", rgba=(0.46, 0.31, 0.18, 1.0))
    graphite = model.material("graphite", rgba=(0.19, 0.20, 0.22, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    sleeve_bottom_z = 0.720
    sleeve_top_z = 1.500
    sleeve_height = sleeve_top_z - sleeve_bottom_z

    lower_frame = model.part("lower_frame")
    lower_frame.visual(
        Box((0.048, 0.084, 0.045)),
        origin=Origin(xyz=(-0.056, 0.0, 1.515)),
        material=graphite,
        name="crown_left",
    )
    lower_frame.visual(
        Box((0.048, 0.084, 0.045)),
        origin=Origin(xyz=(0.056, 0.0, 1.515)),
        material=graphite,
        name="crown_right",
    )
    lower_frame.visual(
        Box((0.112, 0.018, 0.032)),
        origin=Origin(xyz=(0.0, 0.051, 1.509)),
        material=graphite,
        name="crown_front",
    )
    lower_frame.visual(
        Box((0.112, 0.018, 0.045)),
        origin=Origin(xyz=(0.0, -0.043, 1.515)),
        material=graphite,
        name="crown_back",
    )
    lower_frame.visual(
        Box((0.078, 0.008, sleeve_height)),
        origin=Origin(xyz=(0.0, -0.018, sleeve_bottom_z + 0.5 * sleeve_height)),
        material=graphite,
        name="sleeve_back",
    )
    lower_frame.visual(
        Box((0.010, 0.032, sleeve_height)),
        origin=Origin(xyz=(-0.044, 0.0, sleeve_bottom_z + 0.5 * sleeve_height)),
        material=graphite,
        name="sleeve_left",
    )
    lower_frame.visual(
        Box((0.010, 0.032, sleeve_height)),
        origin=Origin(xyz=(0.044, 0.0, sleeve_bottom_z + 0.5 * sleeve_height)),
        material=graphite,
        name="sleeve_right",
    )
    lower_frame.visual(
        Box((0.090, 0.008, 0.080)),
        origin=Origin(xyz=(0.0, -0.016, 1.475)),
        material=graphite,
        name="crown_spine",
    )
    lower_frame.visual(
        Box((0.104, 0.036, 0.042)),
        origin=Origin(xyz=(0.0, -0.002, 0.721)),
        material=graphite,
        name="sleeve_base",
    )

    leg_joint_radius = 0.060
    leg_joint_z = 1.500
    leg_angle = 0.58
    leg_body_length = 1.700
    foot_length = 0.080
    leg_angles = (
        0.5 * math.pi,
        7.0 * math.pi / 6.0,
        11.0 * math.pi / 6.0,
    )

    for index, yaw in enumerate(leg_angles):
        leg = model.part(f"leg_{index}")
        sin_leg = math.sin(leg_angle)
        cos_leg = math.cos(leg_angle)

        leg.visual(
            Box((0.034, 0.022, leg_body_length)),
            origin=Origin(
                xyz=(
                    0.5 * leg_body_length * sin_leg,
                    0.0,
                    -0.5 * leg_body_length * cos_leg,
                ),
                rpy=(0.0, -leg_angle, 0.0),
            ),
            material=ash_wood,
            name="spar",
        )
        leg.visual(
            Box((0.040, 0.026, foot_length)),
            origin=Origin(
                xyz=(
                    (leg_body_length + 0.5 * foot_length) * sin_leg,
                    0.0,
                    -(leg_body_length + 0.5 * foot_length) * cos_leg,
                ),
                rpy=(0.0, -leg_angle, 0.0),
            ),
            material=rubber,
            name="foot",
        )

        model.articulation(
            f"lower_frame_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=lower_frame,
            child=leg,
            origin=Origin(
                xyz=(
                    leg_joint_radius * math.cos(yaw),
                    leg_joint_radius * math.sin(yaw),
                    leg_joint_z,
                ),
                rpy=(0.0, 0.0, yaw),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=40.0,
                velocity=1.6,
                lower=-0.48,
                upper=0.20,
            ),
        )

    mast = model.part("mast")
    mast.visual(
        Box((0.044, 0.024, 1.320)),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=walnut,
        name="rail",
    )
    mast.visual(
        Box((0.110, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, 0.008, 0.615)),
        material=walnut,
        name="top_cap",
    )
    mast.visual(
        Box((0.050, 0.018, 0.075)),
        origin=Origin(xyz=(0.0, 0.019, 0.5625)),
        material=graphite,
        name="top_clip",
    )

    model.articulation(
        "lower_frame_to_mast",
        ArticulationType.PRISMATIC,
        parent=lower_frame,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, sleeve_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.20,
            lower=0.0,
            upper=0.320,
        ),
    )

    shelf = model.part("shelf")
    shelf.visual(
        Box((0.300, 0.052, 0.016)),
        origin=Origin(xyz=(0.0, 0.040, 0.0)),
        material=ash_wood,
        name="board",
    )
    shelf.visual(
        Box((0.300, 0.010, 0.028)),
        origin=Origin(xyz=(0.0, 0.061, 0.022)),
        material=ash_wood,
        name="lip",
    )
    shelf.visual(
        Box((0.014, 0.028, 0.120)),
        origin=Origin(xyz=(-0.029, 0.002, 0.052)),
        material=graphite,
        name="guide_0",
    )
    shelf.visual(
        Box((0.014, 0.028, 0.120)),
        origin=Origin(xyz=(0.029, 0.002, 0.052)),
        material=graphite,
        name="guide_1",
    )
    shelf.visual(
        Box((0.072, 0.006, 0.030)),
        origin=Origin(xyz=(0.0, -0.015, 0.081)),
        material=graphite,
        name="bridge",
    )

    model.articulation(
        "mast_to_shelf",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=shelf,
        origin=Origin(xyz=(0.0, 0.0, -0.320)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.18,
            lower=0.0,
            upper=0.460,
        ),
    )

    return model


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[i] + high[i]) * 0.5 for i in range(3))


def _radial_xy(point: tuple[float, float, float] | None) -> float | None:
    if point is None:
        return None
    return math.hypot(point[0], point[1])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_frame = object_model.get_part("lower_frame")
    mast = object_model.get_part("mast")
    shelf = object_model.get_part("shelf")
    leg_0 = object_model.get_part("leg_0")

    mast_slide = object_model.get_articulation("lower_frame_to_mast")
    shelf_slide = object_model.get_articulation("mast_to_shelf")
    leg_0_hinge = object_model.get_articulation("lower_frame_to_leg_0")

    mast_limits = mast_slide.motion_limits
    shelf_limits = shelf_slide.motion_limits
    leg_limits = leg_0_hinge.motion_limits

    ctx.allow_overlap(
        leg_0,
        lower_frame,
        elem_a="spar",
        elem_b="crown_front",
        reason="The front leg pivots inside a simplified crown cheek that stands in for the easel hinge hardware.",
    )
    ctx.allow_overlap(
        "leg_1",
        lower_frame,
        elem_a="spar",
        elem_b="crown_left",
        reason="The rear leg root is intentionally simplified as a leg spar nested into the crown hinge cheek.",
    )
    ctx.allow_overlap(
        "leg_2",
        lower_frame,
        elem_a="spar",
        elem_b="crown_right",
        reason="The rear leg root is intentionally simplified as a leg spar nested into the crown hinge cheek.",
    )
    ctx.allow_overlap(
        "leg_1",
        lower_frame,
        elem_a="spar",
        elem_b="crown_back",
        reason="The rear crown crosspiece slightly interleaves the simplified hinge root so the folding leg reads as socketed into the crown.",
    )
    ctx.allow_overlap(
        "leg_2",
        lower_frame,
        elem_a="spar",
        elem_b="crown_back",
        reason="The rear crown crosspiece slightly interleaves the simplified hinge root so the folding leg reads as socketed into the crown.",
    )
    ctx.allow_overlap(
        "leg_1",
        lower_frame,
        elem_a="spar",
        elem_b="crown_spine",
        reason="The rear leg hinge root is simplified as a spar passing into the crown spine that ties the hinge cheeks back into the frame.",
    )
    ctx.allow_overlap(
        "leg_2",
        lower_frame,
        elem_a="spar",
        elem_b="crown_spine",
        reason="The rear leg hinge root is simplified as a spar passing into the crown spine that ties the hinge cheeks back into the frame.",
    )

    ctx.expect_within(
        mast,
        lower_frame,
        axes="xy",
        margin=0.012,
        name="mast stays centered in the lower frame",
    )
    ctx.expect_overlap(
        mast,
        lower_frame,
        axes="z",
        min_overlap=0.70,
        name="collapsed mast remains deeply inserted",
    )
    ctx.expect_contact(
        shelf,
        mast,
        contact_tol=0.001,
        name="shelf carriage bears on the mast",
    )
    ctx.expect_origin_distance(
        shelf,
        mast,
        axes="xy",
        max_dist=0.001,
        name="shelf stays centered on the mast",
    )

    mast_rest = ctx.part_world_position(mast)
    shelf_rest = ctx.part_world_position(shelf)
    leg_rest_center = _aabb_center(ctx.part_world_aabb(leg_0))
    leg_rest_aabb = ctx.part_world_aabb(leg_0)

    if mast_limits is not None and mast_limits.upper is not None:
        with ctx.pose({mast_slide: mast_limits.upper}):
            ctx.expect_overlap(
                mast,
                lower_frame,
                axes="z",
                min_overlap=0.40,
                name="extended mast still overlaps the frame",
            )
            mast_extended = ctx.part_world_position(mast)
        ctx.check(
            "mast extends upward",
            mast_rest is not None
            and mast_extended is not None
            and mast_extended[2] > mast_rest[2] + 0.25,
            details=f"rest={mast_rest}, extended={mast_extended}",
        )

    if shelf_limits is not None and shelf_limits.upper is not None:
        with ctx.pose({shelf_slide: shelf_limits.upper}):
            ctx.expect_contact(
                shelf,
                mast,
                contact_tol=0.001,
                name="raised shelf remains guided by the mast",
            )
            shelf_high = ctx.part_world_position(shelf)
            mast_aabb = ctx.part_world_aabb(mast)
            shelf_aabb = ctx.part_world_aabb(shelf)
        ctx.check(
            "shelf slides upward",
            shelf_rest is not None
            and shelf_high is not None
            and shelf_high[2] > shelf_rest[2] + 0.35,
            details=f"rest={shelf_rest}, raised={shelf_high}",
        )
        ctx.check(
            "shelf remains below the mast head",
            mast_aabb is not None
            and shelf_aabb is not None
            and mast_aabb[1][2] > shelf_aabb[1][2] + 0.18,
            details=f"mast={mast_aabb}, shelf={shelf_aabb}",
        )

    if leg_limits is not None and leg_limits.lower is not None:
        with ctx.pose({leg_0_hinge: leg_limits.lower}):
            leg_folded_center = _aabb_center(ctx.part_world_aabb(leg_0))
        ctx.check(
            "front leg folds inward toward the mast",
            _radial_xy(leg_rest_center) is not None
            and _radial_xy(leg_folded_center) is not None
            and _radial_xy(leg_folded_center) < _radial_xy(leg_rest_center) - 0.20,
            details=f"rest={leg_rest_center}, folded={leg_folded_center}",
        )

    ctx.check(
        "front leg reaches the ground plane",
        leg_rest_aabb is not None and leg_rest_aabb[0][2] < 0.03,
        details=f"aabb={leg_rest_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
