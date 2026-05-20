from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


def add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material,
    *,
    name: str | None = None,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def add_cylinder(
    part,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material,
    *,
    name: str | None = None,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_mast")

    support_color = model.material("support_steel", rgba=(0.24, 0.27, 0.30, 1.0))
    mast_color = model.material("mast_aluminum", rgba=(0.70, 0.73, 0.76, 1.0))
    cartridge_color = model.material("cartridge_dark", rgba=(0.13, 0.14, 0.16, 1.0))

    support_frame = model.part("support_frame")
    add_box(support_frame, (0.28, 0.18, 0.03), (-0.04, -0.02, 0.015), support_color, name="support_frame_body")
    add_box(support_frame, (0.02, 0.22, 1.18), (-0.15, -0.02, 0.59), support_color)
    add_box(support_frame, (0.13, 0.10, 0.08), (-0.03, -0.01, 0.07), support_color)
    add_box(support_frame, (0.014, 0.05, 1.04), (-0.043, -0.015, 0.58), support_color)
    add_box(support_frame, (0.014, 0.05, 1.04), (0.043, -0.015, 0.58), support_color)
    add_box(support_frame, (0.19, 0.025, 0.05), (-0.045, -0.052, 0.15), support_color)
    add_box(support_frame, (0.19, 0.025, 0.045), (-0.045, -0.052, 0.56), support_color)
    add_box(support_frame, (0.19, 0.025, 0.05), (-0.045, -0.052, 1.00), support_color)

    mid_mast = model.part("mid_mast")
    add_box(mid_mast, (0.044, 0.024, 1.12), (0.0, -0.024, 0.56), mast_color, name="mid_mast_body")
    add_box(mid_mast, (0.014, 0.04, 0.92), (-0.029, -0.012, 0.56), mast_color)
    add_box(mid_mast, (0.014, 0.04, 0.92), (0.029, -0.012, 0.56), mast_color)
    add_box(mid_mast, (0.072, 0.024, 0.05), (0.0, -0.024, 0.075), mast_color)
    add_box(mid_mast, (0.072, 0.024, 0.05), (0.0, -0.024, 1.045), mast_color)

    top_mast = model.part("top_mast")
    add_box(top_mast, (0.032, 0.024, 0.92), (0.0, 0.0, 0.46), mast_color, name="top_mast_body")
    add_box(top_mast, (0.006, 0.02, 0.52), (-0.019, -0.004, 0.30), mast_color)
    add_box(top_mast, (0.006, 0.02, 0.52), (0.019, -0.004, 0.30), mast_color)
    add_box(top_mast, (0.09, 0.06, 0.05), (0.0, -0.005, 0.945), mast_color)
    add_cylinder(top_mast, 0.04, 0.06, (0.0, -0.005, 1.00), cartridge_color)

    rotary_stage = model.part("rotary_stage")
    add_cylinder(rotary_stage, 0.075, 0.018, (0.0, 0.0, 0.009), cartridge_color, name="rotary_stage_body")
    add_box(rotary_stage, (0.12, 0.08, 0.018), (0.03, 0.0, 0.027), cartridge_color)
    add_box(rotary_stage, (0.034, 0.044, 0.018), (-0.07, 0.0, 0.027), cartridge_color)
    add_cylinder(rotary_stage, 0.01, 0.04, (0.06, 0.0, 0.056), cartridge_color)

    model.articulation(
        "support_to_mid",
        ArticulationType.PRISMATIC,
        parent=support_frame,
        child=mid_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.40,
            lower=0.0,
            upper=0.38,
        ),
    )

    model.articulation(
        "mid_to_top",
        ArticulationType.PRISMATIC,
        parent=mid_mast,
        child=top_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=700.0,
            velocity=0.35,
            lower=0.0,
            upper=0.34,
        ),
    )

    model.articulation(
        "top_to_stage",
        ArticulationType.REVOLUTE,
        parent=top_mast,
        child=rotary_stage,
        origin=Origin(xyz=(0.0, 0.0, 1.03)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.8,
            lower=-0.9 * pi,
            upper=0.9 * pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_frame = object_model.get_part("support_frame")
    mid_mast = object_model.get_part("mid_mast")
    top_mast = object_model.get_part("top_mast")
    rotary_stage = object_model.get_part("rotary_stage")

    support_to_mid = object_model.get_articulation("support_to_mid")
    mid_to_top = object_model.get_articulation("mid_to_top")
    top_to_stage = object_model.get_articulation("top_to_stage")

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
        "support_to_mid_is_vertical_prismatic",
        support_to_mid.articulation_type == ArticulationType.PRISMATIC
        and tuple(support_to_mid.axis) == (0.0, 0.0, 1.0),
        details=f"type={support_to_mid.articulation_type}, axis={support_to_mid.axis}",
    )
    ctx.check(
        "mid_to_top_is_vertical_prismatic",
        mid_to_top.articulation_type == ArticulationType.PRISMATIC
        and tuple(mid_to_top.axis) == (0.0, 0.0, 1.0),
        details=f"type={mid_to_top.articulation_type}, axis={mid_to_top.axis}",
    )
    ctx.check(
        "top_to_stage_is_vertical_revolute",
        top_to_stage.articulation_type == ArticulationType.REVOLUTE
        and tuple(top_to_stage.axis) == (0.0, 0.0, 1.0),
        details=f"type={top_to_stage.articulation_type}, axis={top_to_stage.axis}",
    )

    ctx.expect_contact(
        mid_mast,
        support_frame,
        name="mid_mast_guides_contact_support_frame",
    )
    ctx.expect_contact(
        top_mast,
        mid_mast,
        name="top_mast_guides_contact_mid_mast",
    )
    ctx.expect_contact(
        rotary_stage,
        top_mast,
        name="rotary_stage_seats_on_cartridge",
    )

    with ctx.pose({support_to_mid: 0.38, mid_to_top: 0.34}):
        ctx.expect_contact(
            mid_mast,
            support_frame,
            name="mid_mast_stays_supported_at_full_extension",
        )
        ctx.expect_contact(
            top_mast,
            mid_mast,
            name="top_mast_stays_supported_at_full_extension",
        )

    mid_rest = ctx.part_world_position(mid_mast)
    top_rest = ctx.part_world_position(top_mast)
    if mid_rest is None or top_rest is None:
        ctx.fail("part_positions_available", "could not query rest-pose part positions")
    else:
        with ctx.pose({support_to_mid: 0.30, mid_to_top: 0.18, top_to_stage: 0.0}):
            mid_raised = ctx.part_world_position(mid_mast)
            top_raised = ctx.part_world_position(top_mast)
            stage_unturned = ctx.part_world_position(rotary_stage)
        with ctx.pose({support_to_mid: 0.30, mid_to_top: 0.18, top_to_stage: 1.1}):
            stage_turned = ctx.part_world_position(rotary_stage)

        if (
            mid_raised is None
            or top_raised is None
            or stage_unturned is None
            or stage_turned is None
        ):
            ctx.fail("posed_part_positions_available", "could not query posed part positions")
        else:
            ctx.check(
                "serial_prismatic_motion_raises_nested_members",
                abs((mid_raised[2] - mid_rest[2]) - 0.30) < 1e-5
                and abs((top_raised[2] - top_rest[2]) - 0.48) < 1e-5,
                details=(
                    f"mid_dz={mid_raised[2] - mid_rest[2]:.6f}, "
                    f"top_dz={top_raised[2] - top_rest[2]:.6f}"
                ),
            )
            ctx.check(
                "rotary_stage_spins_about_fixed_vertical_origin",
                all(abs(stage_turned[i] - stage_unturned[i]) < 1e-6 for i in range(3)),
                details=f"unturned={stage_unturned}, turned={stage_turned}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
