from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ground_spike_flood_light")

    powder_coat = model.material("powder_coat_black", rgba=(0.13, 0.14, 0.15, 1.0))
    housing_finish = model.material("charcoal_housing", rgba=(0.20, 0.21, 0.23, 1.0))
    glass = model.material("smoked_glass", rgba=(0.58, 0.68, 0.76, 0.35))

    spike_post = model.part("stake_post")
    spike_post.visual(
        mesh_from_geometry(
            ConeGeometry(radius=0.0075, height=0.090, radial_segments=28, closed=True),
            "ground_spike_tip",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=powder_coat,
        name="spike_tip",
    )
    spike_post.visual(
        Cylinder(radius=0.007, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, 0.1375)),
        material=powder_coat,
        name="spike_shank",
    )
    spike_post.visual(
        Cylinder(radius=0.015, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.204)),
        material=powder_coat,
        name="stake_collar",
    )
    spike_post.visual(
        Cylinder(radius=0.009, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.283)),
        material=powder_coat,
        name="upright_post",
    )
    spike_post.visual(
        Cylinder(radius=0.013, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.347)),
        material=powder_coat,
        name="post_cap",
    )

    pan_bracket = model.part("pan_bracket")
    pan_bracket.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=powder_coat,
        name="swivel_base",
    )
    pan_bracket.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=powder_coat,
        name="swivel_stem",
    )
    pan_bracket.visual(
        Box((0.010, 0.028, 0.014)),
        origin=Origin(xyz=(0.004, 0.0, 0.031)),
        material=powder_coat,
        name="center_block",
    )
    pan_bracket.visual(
        Box((0.014, 0.078, 0.012)),
        origin=Origin(xyz=(0.008, -0.0465, 0.021)),
        material=powder_coat,
        name="left_link",
    )
    pan_bracket.visual(
        Box((0.014, 0.078, 0.012)),
        origin=Origin(xyz=(0.008, 0.0465, 0.021)),
        material=powder_coat,
        name="right_link",
    )
    pan_bracket.visual(
        Box((0.032, 0.005, 0.060)),
        origin=Origin(xyz=(0.022, -0.0875, 0.050)),
        material=powder_coat,
        name="left_arm",
    )
    pan_bracket.visual(
        Box((0.032, 0.005, 0.060)),
        origin=Origin(xyz=(0.022, 0.0875, 0.050)),
        material=powder_coat,
        name="right_arm",
    )

    flood_head = model.part("flood_head")
    flood_head.visual(
        Box((0.004, 0.160, 0.100)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=housing_finish,
        name="back_plate",
    )
    flood_head.visual(
        Box((0.050, 0.006, 0.100)),
        origin=Origin(xyz=(0.029, -0.077, 0.0)),
        material=housing_finish,
        name="left_wall",
    )
    flood_head.visual(
        Box((0.050, 0.006, 0.100)),
        origin=Origin(xyz=(0.029, 0.077, 0.0)),
        material=housing_finish,
        name="right_wall",
    )
    flood_head.visual(
        Box((0.050, 0.148, 0.006)),
        origin=Origin(xyz=(0.029, 0.0, 0.047)),
        material=housing_finish,
        name="top_wall",
    )
    flood_head.visual(
        Box((0.050, 0.148, 0.006)),
        origin=Origin(xyz=(0.029, 0.0, -0.047)),
        material=housing_finish,
        name="bottom_wall",
    )
    flood_head.visual(
        Box((0.008, 0.010, 0.092)),
        origin=Origin(xyz=(0.055, -0.071, 0.0)),
        material=housing_finish,
        name="bezel_left",
    )
    flood_head.visual(
        Box((0.008, 0.010, 0.092)),
        origin=Origin(xyz=(0.055, 0.071, 0.0)),
        material=housing_finish,
        name="bezel_right",
    )
    flood_head.visual(
        Box((0.008, 0.132, 0.010)),
        origin=Origin(xyz=(0.055, 0.0, 0.041)),
        material=housing_finish,
        name="bezel_top",
    )
    flood_head.visual(
        Box((0.008, 0.132, 0.010)),
        origin=Origin(xyz=(0.055, 0.0, -0.041)),
        material=housing_finish,
        name="bezel_bottom",
    )
    flood_head.visual(
        Box((0.014, 0.152, 0.006)),
        origin=Origin(xyz=(0.050, 0.0, 0.053)),
        material=housing_finish,
        name="hood_lip",
    )
    for index, y_pos in enumerate((-0.054, -0.018, 0.018, 0.054), start=1):
        flood_head.visual(
            Box((0.004, 0.014, 0.078)),
            origin=Origin(xyz=(0.003, y_pos, 0.0)),
            material=housing_finish,
            name=f"heat_sink_fin_{index}",
        )
    flood_head.visual(
        Box((0.002, 0.132, 0.082)),
        origin=Origin(xyz=(0.052, 0.0, 0.0)),
        material=glass,
        name="lens",
    )
    flood_head.visual(
        Cylinder(radius=0.008, length=0.005),
        origin=Origin(xyz=(0.004, -0.0825, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=housing_finish,
        name="left_trunnion",
    )
    flood_head.visual(
        Cylinder(radius=0.008, length=0.005),
        origin=Origin(xyz=(0.004, 0.0825, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=housing_finish,
        name="right_trunnion",
    )

    model.articulation(
        "post_to_pan",
        ArticulationType.REVOLUTE,
        parent=spike_post,
        child=pan_bracket,
        origin=Origin(xyz=(0.0, 0.0, 0.351)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-1.8,
            upper=1.8,
        ),
    )
    model.articulation(
        "pan_to_head_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_bracket,
        child=flood_head,
        origin=Origin(xyz=(0.018, 0.0, 0.050)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.8,
            lower=-0.5,
            upper=1.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stake_post = object_model.get_part("stake_post")
    pan_bracket = object_model.get_part("pan_bracket")
    flood_head = object_model.get_part("flood_head")
    pan_joint = object_model.get_articulation("post_to_pan")
    tilt_joint = object_model.get_articulation("pan_to_head_tilt")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "all requested parts exist",
        all(part is not None for part in (stake_post, pan_bracket, flood_head)),
        details=f"parts={[part.name for part in (stake_post, pan_bracket, flood_head)]}",
    )
    ctx.check(
        "pan joint is vertical",
        tuple(pan_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={pan_joint.axis}",
    )
    ctx.check(
        "tilt joint is horizontal",
        tuple(tilt_joint.axis) == (0.0, -1.0, 0.0),
        details=f"axis={tilt_joint.axis}",
    )

    ctx.expect_contact(
        pan_bracket,
        stake_post,
        elem_a="swivel_base",
        elem_b="post_cap",
        name="pan bracket seats on the post cap",
    )
    ctx.expect_contact(
        flood_head,
        pan_bracket,
        elem_a="left_trunnion",
        elem_b="left_arm",
        name="left trunnion is supported by the yoke arm",
    )
    ctx.expect_contact(
        flood_head,
        pan_bracket,
        elem_a="right_trunnion",
        elem_b="right_arm",
        name="right trunnion is supported by the yoke arm",
    )
    ctx.expect_gap(
        flood_head,
        pan_bracket,
        axis="x",
        positive_elem="back_plate",
        negative_elem="center_block",
        min_gap=0.010,
        name="head housing sits forward of the pan knuckle block",
    )

    def elem_center(part_name: str, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))

    rest_lens = elem_center("flood_head", "lens")
    with ctx.pose({pan_joint: 1.0}):
        panned_lens = elem_center("flood_head", "lens")
    ctx.check(
        "pan joint swings the head sideways around the post",
        rest_lens is not None
        and panned_lens is not None
        and panned_lens[1] > rest_lens[1] + 0.05
        and panned_lens[0] < rest_lens[0] - 0.015,
        details=f"rest={rest_lens}, panned={panned_lens}",
    )

    with ctx.pose({tilt_joint: 0.8}):
        tilted_lens = elem_center("flood_head", "lens")
    ctx.check(
        "tilt joint raises the beam direction",
        rest_lens is not None
        and tilted_lens is not None
        and tilted_lens[2] > rest_lens[2] + 0.03
        and tilted_lens[0] < rest_lens[0] - 0.01,
        details=f"rest={rest_lens}, tilted={tilted_lens}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
