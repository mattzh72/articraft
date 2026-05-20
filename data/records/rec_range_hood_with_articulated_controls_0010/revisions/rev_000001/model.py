from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.82, 0.84, 0.86, 1.0))
    brushed_shadow = model.material("brushed_shadow", rgba=(0.62, 0.64, 0.66, 1.0))
    charcoal = model.material("charcoal", rgba=(0.14, 0.14, 0.15, 1.0))
    lamp_lens = model.material("lamp_lens", rgba=(0.92, 0.93, 0.88, 0.95))

    hood_body = model.part("hood_body")

    # Crisp front valance and lower capture canopy.
    hood_body.visual(
        Box((0.90, 0.024, 0.09)),
        origin=Origin(xyz=(0.0, -0.238, 0.045)),
        material=stainless,
        name="front_valance",
    )
    hood_body.visual(
        Box((0.82, 0.020, 0.080)),
        origin=Origin(xyz=(0.0, -0.218, 0.104)),
        material=brushed_shadow,
        name="inner_front_baffle",
    )
    hood_body.visual(
        Box((0.90, 0.45, 0.015)),
        origin=Origin(xyz=(0.0, -0.010, 0.154), rpy=(0.30, 0.0, 0.0)),
        material=stainless,
        name="sloped_canopy_top",
    )
    hood_body.visual(
        Box((0.012, 0.44, 0.07)),
        origin=Origin(xyz=(-0.444, -0.010, 0.100), rpy=(0.30, 0.0, 0.0)),
        material=stainless,
        name="left_side_panel",
    )
    hood_body.visual(
        Box((0.012, 0.44, 0.07)),
        origin=Origin(xyz=(0.444, -0.010, 0.100), rpy=(0.30, 0.0, 0.0)),
        material=stainless,
        name="right_side_panel",
    )
    hood_body.visual(
        Box((0.90, 0.016, 0.23)),
        origin=Origin(xyz=(0.0, 0.242, 0.115)),
        material=brushed_shadow,
        name="rear_panel",
    )

    # Underside frame and filters to keep the canopy readable as a vented shell.
    hood_body.visual(
        Box((0.03, 0.44, 0.03)),
        origin=Origin(xyz=(-0.435, -0.002, 0.015)),
        material=stainless,
        name="left_lower_rail",
    )
    hood_body.visual(
        Box((0.03, 0.44, 0.03)),
        origin=Origin(xyz=(0.435, -0.002, 0.015)),
        material=stainless,
        name="right_lower_rail",
    )
    hood_body.visual(
        Box((0.86, 0.03, 0.03)),
        origin=Origin(xyz=(0.0, 0.220, 0.015)),
        material=stainless,
        name="rear_lower_rail",
    )
    hood_body.visual(
        Box((0.84, 0.43, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=charcoal,
        name="filter_panel",
    )
    hood_body.visual(
        Box((0.07, 0.05, 0.008)),
        origin=Origin(xyz=(-0.18, -0.14, 0.008)),
        material=lamp_lens,
        name="left_task_light",
    )
    hood_body.visual(
        Box((0.07, 0.05, 0.008)),
        origin=Origin(xyz=(0.18, -0.14, 0.008)),
        material=lamp_lens,
        name="right_task_light",
    )

    # Short transition deck below the tall rectangular chimney cover.
    hood_body.visual(
        Box((0.34, 0.24, 0.018)),
        origin=Origin(xyz=(0.0, 0.130, 0.223)),
        material=stainless,
        name="transition_deck",
    )
    hood_body.visual(
        Box((0.24, 0.18, 0.128)),
        origin=Origin(xyz=(0.0, 0.160, 0.158)),
        material=brushed_shadow,
        name="duct_riser",
    )

    chimney_height = 0.68
    chimney_center_z = 0.232 + (chimney_height / 2.0)
    hood_body.visual(
        Box((0.32, 0.012, chimney_height)),
        origin=Origin(xyz=(0.0, 0.016, chimney_center_z)),
        material=stainless,
        name="chimney_front",
    )
    hood_body.visual(
        Box((0.32, 0.012, chimney_height)),
        origin=Origin(xyz=(0.0, 0.244, chimney_center_z)),
        material=brushed_shadow,
        name="chimney_back",
    )
    hood_body.visual(
        Box((0.012, 0.216, chimney_height)),
        origin=Origin(xyz=(-0.154, 0.130, chimney_center_z)),
        material=stainless,
        name="chimney_left",
    )
    hood_body.visual(
        Box((0.012, 0.216, chimney_height)),
        origin=Origin(xyz=(0.154, 0.130, chimney_center_z)),
        material=stainless,
        name="chimney_right",
    )

    hood_body.inertial = Inertial.from_geometry(
        Box((0.90, 0.50, 0.92)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
    )

    def add_knob(
        name: str,
        *,
        x_pos: float,
        z_pos: float,
    ) -> None:
        knob = model.part(name)
        skirt_length = 0.010
        body_length = 0.026
        total_length = skirt_length + body_length
        skirt_radius = 0.019
        body_radius = 0.017

        knob.visual(
            Cylinder(radius=skirt_radius, length=skirt_length),
            origin=Origin(xyz=(0.0, -(skirt_length / 2.0), 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=charcoal,
            name="knob_skirt",
        )
        knob.visual(
            Cylinder(radius=body_radius, length=body_length),
            origin=Origin(
                xyz=(0.0, -(skirt_length + (body_length / 2.0)), 0.0),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=charcoal,
            name="knob_body",
        )
        knob.visual(
            Box((0.004, 0.010, 0.0025)),
            origin=Origin(xyz=(0.0, -0.031, body_radius - 0.0015)),
            material=lamp_lens,
            name="indicator",
        )

        knob.inertial = Inertial.from_geometry(
            Cylinder(radius=skirt_radius, length=total_length),
            mass=0.15,
            origin=Origin(xyz=(0.0, -(total_length / 2.0), 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        )

        model.articulation(
            f"{name}_spin",
            ArticulationType.CONTINUOUS,
            parent=hood_body,
            child=knob,
            origin=Origin(xyz=(x_pos, -0.250, z_pos)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=8.0),
        )

    add_knob("upper_knob", x_pos=0.0, z_pos=0.070)
    add_knob("left_knob", x_pos=-0.052, z_pos=0.046)
    add_knob("right_knob", x_pos=0.052, z_pos=0.046)
    add_knob("lower_knob", x_pos=0.0, z_pos=0.022)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    hood_body = object_model.get_part("hood_body")
    upper_knob = object_model.get_part("upper_knob")
    left_knob = object_model.get_part("left_knob")
    right_knob = object_model.get_part("right_knob")
    lower_knob = object_model.get_part("lower_knob")

    upper_joint = object_model.get_articulation("upper_knob_spin")
    left_joint = object_model.get_articulation("left_knob_spin")
    right_joint = object_model.get_articulation("right_knob_spin")
    lower_joint = object_model.get_articulation("lower_knob_spin")

    front_valance = hood_body.get_visual("front_valance")
    chimney_front = hood_body.get_visual("chimney_front")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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

    for knob, joint in (
        (upper_knob, upper_joint),
        (left_knob, left_joint),
        (right_knob, right_joint),
        (lower_knob, lower_joint),
    ):
        ctx.expect_contact(
            knob,
            hood_body,
            elem_b=front_valance,
            name=f"{knob.name}_touches_valance",
        )
        ctx.check(
            f"{joint.name}_is_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"{joint.name} should be continuous, got {joint.articulation_type!r}",
        )
        ctx.check(
            f"{joint.name}_axis_is_front_to_back",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            details=f"{joint.name} axis should be (0, 1, 0), got {joint.axis!r}",
        )
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_has_unbounded_rotation",
            limits is not None and limits.lower is None and limits.upper is None,
            details=f"{joint.name} should have continuous limits, got {limits!r}",
        )

    ctx.expect_origin_gap(
        right_knob,
        left_knob,
        axis="x",
        min_gap=0.100,
        max_gap=0.106,
        name="left_right_knob_spacing",
    )
    ctx.expect_origin_gap(
        upper_knob,
        lower_knob,
        axis="z",
        min_gap=0.046,
        max_gap=0.050,
        name="upper_lower_knob_spacing",
    )
    ctx.expect_origin_distance(
        upper_knob,
        lower_knob,
        axes="x",
        max_dist=0.001,
        name="upper_and_lower_centered",
    )
    ctx.expect_origin_distance(
        left_knob,
        right_knob,
        axes="z",
        max_dist=0.001,
        name="left_and_right_level",
    )

    body_aabb = ctx.part_world_aabb(hood_body)
    if body_aabb is not None:
        (min_corner, max_corner) = body_aabb
        width = max_corner[0] - min_corner[0]
        depth = max_corner[1] - min_corner[1]
        height = max_corner[2] - min_corner[2]
        ctx.check(
            "range_hood_realistic_width",
            0.88 <= width <= 0.92,
            details=f"hood width should read as a 90 cm appliance, got {width:.3f} m",
        )
        ctx.check(
            "range_hood_realistic_depth",
            0.49 <= depth <= 0.51,
            details=f"hood depth should be about 50 cm, got {depth:.3f} m",
        )
        ctx.check(
            "range_hood_realistic_height",
            0.89 <= height <= 0.93,
            details=f"hood height should include a tall chimney cover, got {height:.3f} m",
        )
    else:
        ctx.fail("hood_body_has_world_aabb", "hood body AABB could not be resolved")

    chimney_aabb = ctx.part_element_world_aabb(hood_body, elem=chimney_front)
    if chimney_aabb is not None:
        (chimney_min, chimney_max) = chimney_aabb
        chimney_height = chimney_max[2] - chimney_min[2]
        ctx.check(
            "chimney_front_is_tall",
            chimney_height >= 0.66,
            details=f"chimney cover should be tall, got {chimney_height:.3f} m",
        )
    else:
        ctx.fail("chimney_front_has_world_aabb", "chimney front visual AABB could not be resolved")

    with ctx.pose(
        {
            upper_joint: 0.75,
            left_joint: 1.60,
            right_joint: 2.35,
            lower_joint: 3.10,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="turned_knobs_no_overlap")
        ctx.fail_if_isolated_parts(name="turned_knobs_no_floating")
        for knob in (upper_knob, left_knob, right_knob, lower_knob):
            ctx.expect_contact(
                knob,
                hood_body,
                elem_b=front_valance,
                name=f"{knob.name}_touches_valance_when_turned",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
