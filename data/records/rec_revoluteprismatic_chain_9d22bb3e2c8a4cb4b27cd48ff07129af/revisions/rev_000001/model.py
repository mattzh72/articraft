from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_arm")

    model.material("base_paint", color=(0.22, 0.24, 0.27, 1.0))
    model.material("arm_shell", color=(0.82, 0.84, 0.86, 1.0))
    model.material("dark_trim", color=(0.12, 0.13, 0.15, 1.0))
    model.material("slider_finish", color=(0.30, 0.33, 0.37, 1.0))

    # Broad grounded pedestal.
    base = model.part("base")
    base.visual(
        Box((0.36, 0.28, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material="base_paint",
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.095, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material="dark_trim",
        name="base_pedestal",
    )
    base.visual(
        Box((0.14, 0.18, 0.04)),
        origin=Origin(xyz=(-0.06, 0.0, 0.10)),
        material="base_paint",
        name="service_hump",
    )

    # Compact rigid arm body with a guided open nose channel.
    link = model.part("link")
    link.visual(
        Cylinder(radius=0.085, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material="dark_trim",
        name="turntable_disc",
    )
    link.visual(
        Box((0.10, 0.10, 0.08)),
        origin=Origin(xyz=(0.05, 0.0, 0.058)),
        material="arm_shell",
        name="shoulder_block",
    )
    link.visual(
        Box((0.22, 0.12, 0.10)),
        origin=Origin(xyz=(0.19, 0.0, 0.08)),
        material="arm_shell",
        name="main_body",
    )
    link.visual(
        Box((0.018, 0.08, 0.07)),
        origin=Origin(xyz=(0.299, 0.0, 0.075)),
        material="arm_shell",
        name="nose_rear_bridge",
    )
    link.visual(
        Box((0.20, 0.08, 0.018)),
        origin=Origin(xyz=(0.39, 0.0, 0.049)),
        material="arm_shell",
        name="nose_bottom_rail",
    )
    link.visual(
        Box((0.20, 0.08, 0.018)),
        origin=Origin(xyz=(0.39, 0.0, 0.101)),
        material="arm_shell",
        name="nose_top_cover",
    )
    link.visual(
        Box((0.20, 0.012, 0.052)),
        origin=Origin(xyz=(0.39, -0.034, 0.075)),
        material="dark_trim",
        name="nose_left_wall",
    )
    link.visual(
        Box((0.20, 0.012, 0.052)),
        origin=Origin(xyz=(0.39, 0.034, 0.075)),
        material="dark_trim",
        name="nose_right_wall",
    )

    # Smaller prismatic tip that runs inside the nose channel.
    slider = model.part("slider")
    slider.visual(
        Box((0.16, 0.056, 0.028)),
        origin=Origin(xyz=(0.08, 0.0, 0.0)),
        material="slider_finish",
        name="slider_beam",
    )
    slider.visual(
        Box((0.035, 0.038, 0.038)),
        origin=Origin(xyz=(0.1775, 0.0, 0.0)),
        material="dark_trim",
        name="slider_tip",
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link,
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.6,
            lower=-1.2,
            upper=1.2,
        ),
    )
    model.articulation(
        "nose_slide",
        ArticulationType.PRISMATIC,
        parent=link,
        child=slider,
        origin=Origin(xyz=(0.31, 0.0, 0.075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.25,
            lower=0.0,
            upper=0.12,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    link = object_model.get_part("link")
    slider = object_model.get_part("slider")
    yaw = object_model.get_articulation("base_yaw")
    slide = object_model.get_articulation("nose_slide")

    def aabb_size(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple(upper[i] - lower[i] for i in range(3))

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
        "parts_present",
        all(part is not None for part in (base, link, slider)),
        details="Expected base, link, and slider parts to exist.",
    )
    ctx.check(
        "joint_axes_match_mechanism",
        yaw.axis == (0.0, 0.0, 1.0) and slide.axis == (1.0, 0.0, 0.0),
        details=f"Unexpected axes: yaw={yaw.axis}, slide={slide.axis}",
    )

    ctx.expect_contact(
        link,
        base,
        elem_a="turntable_disc",
        elem_b="base_pedestal",
        name="turntable_seats_on_base",
    )
    ctx.expect_contact(
        slider,
        link,
        elem_a="slider_beam",
        elem_b="nose_left_wall",
        name="slider_guided_by_left_wall_closed",
    )
    ctx.expect_contact(
        slider,
        link,
        elem_a="slider_beam",
        elem_b="nose_right_wall",
        name="slider_guided_by_right_wall_closed",
    )

    body_size = aabb_size(ctx.part_element_world_aabb(link, elem="main_body"))
    slider_size = aabb_size(ctx.part_element_world_aabb(slider, elem="slider_beam"))
    ctx.check(
        "slider_clearly_smaller_than_link",
        body_size is not None
        and slider_size is not None
        and slider_size[0] < body_size[0] * 0.8
        and slider_size[1] < body_size[1] * 0.6
        and slider_size[2] < body_size[2] * 0.5,
        details=f"main_body={body_size}, slider_beam={slider_size}",
    )

    with ctx.pose({yaw: 0.0, slide: 0.0}):
        slider_home = ctx.part_world_position(slider)
    with ctx.pose({yaw: 0.8, slide: 0.0}):
        slider_swung = ctx.part_world_position(slider)
    ctx.check(
        "positive_yaw_swings_nose_toward_positive_y",
        slider_home is not None
        and slider_swung is not None
        and slider_swung[1] > slider_home[1] + 0.18,
        details=f"home={slider_home}, swung={slider_swung}",
    )

    with ctx.pose({yaw: 0.0, slide: 0.0}):
        slider_retracted = ctx.part_world_position(slider)
    with ctx.pose({yaw: 0.0, slide: 0.10}):
        slider_extended = ctx.part_world_position(slider)
        ctx.expect_contact(
            slider,
            link,
            elem_a="slider_beam",
            elem_b="nose_left_wall",
            name="slider_guided_by_left_wall_extended",
        )
        ctx.expect_contact(
            slider,
            link,
            elem_a="slider_beam",
            elem_b="nose_right_wall",
            name="slider_guided_by_right_wall_extended",
        )
    ctx.check(
        "positive_slide_extends_forward",
        slider_retracted is not None
        and slider_extended is not None
        and slider_extended[0] > slider_retracted[0] + 0.09,
        details=f"retracted={slider_retracted}, extended={slider_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
