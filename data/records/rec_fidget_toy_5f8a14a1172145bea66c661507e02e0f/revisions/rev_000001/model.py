from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


def _circle_profile(radius: float, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _annulus_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    segments: int = 72,
):
    return ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments),
        [_circle_profile(inner_radius, segments)],
        thickness,
        center=True,
        cap=True,
        closed=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gyro_fidget_ring")

    outer_finish = model.material("outer_finish", rgba=(0.14, 0.15, 0.17, 1.0))
    gimbal_finish = model.material("gimbal_finish", rgba=(0.74, 0.76, 0.80, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.82, 0.84, 0.87, 1.0))
    flywheel_rim = model.material("flywheel_rim", rgba=(0.75, 0.58, 0.24, 1.0))
    flywheel_core = model.material("flywheel_core", rgba=(0.22, 0.23, 0.25, 1.0))

    outer_outer_radius = 0.0310
    outer_inner_radius = 0.0225
    outer_thickness = 0.0075
    outer_journal_size = (0.0060, 0.0028, 0.0058)
    outer_journal_center_y = outer_inner_radius - (outer_journal_size[1] * 0.5) + 0.0005

    gimbal_outer_radius = 0.0168
    gimbal_inner_radius = 0.0118
    gimbal_thickness = 0.0045
    gimbal_lug_size = (0.0056, 0.0024, 0.0048)
    gimbal_lug_center_y = gimbal_outer_radius + (gimbal_lug_size[1] * 0.5) - 0.0008
    gimbal_pin_radius = 0.0011
    gimbal_pin_length = outer_journal_center_y - (outer_journal_size[1] * 0.5) - (
        gimbal_lug_center_y + (gimbal_lug_size[1] * 0.5)
    )
    gimbal_pin_center_y = gimbal_lug_center_y + (gimbal_lug_size[1] * 0.5) + (gimbal_pin_length * 0.5)
    gimbal_cradle_size = (0.0038, 0.0048, 0.0048)
    gimbal_cradle_center_x = gimbal_inner_radius - (gimbal_cradle_size[0] * 0.5) + 0.0006

    disc_rim_outer_radius = 0.0102
    disc_rim_inner_radius = 0.0072
    disc_rim_thickness = 0.0040
    disc_web_outer_radius = 0.0076
    disc_web_inner_radius = 0.0020
    disc_web_thickness = 0.0020
    disc_hub_radius = 0.0027
    disc_hub_length = 0.0055
    disc_axle_radius = 0.0010
    disc_axle_length = (gimbal_cradle_center_x - (gimbal_cradle_size[0] * 0.5)) * 2.0

    outer_ring = model.part("outer_ring")
    outer_ring.visual(
        mesh_from_geometry(
            _annulus_mesh(
                outer_radius=outer_outer_radius,
                inner_radius=outer_inner_radius,
                thickness=outer_thickness,
            ),
            "outer_ring_band",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=outer_finish,
        name="band",
    )
    outer_ring.visual(
        Box(outer_journal_size),
        origin=Origin(xyz=(0.0, outer_journal_center_y, 0.0)),
        material=outer_finish,
        name="upper_journal",
    )
    outer_ring.visual(
        Box(outer_journal_size),
        origin=Origin(xyz=(0.0, -outer_journal_center_y, 0.0)),
        material=outer_finish,
        name="lower_journal",
    )

    gimbal_ring = model.part("gimbal_ring")
    gimbal_ring.visual(
        mesh_from_geometry(
            _annulus_mesh(
                outer_radius=gimbal_outer_radius,
                inner_radius=gimbal_inner_radius,
                thickness=gimbal_thickness,
            ),
            "gimbal_ring_band",
        ),
        material=gimbal_finish,
        name="band",
    )
    gimbal_ring.visual(
        Box(gimbal_lug_size),
        origin=Origin(xyz=(0.0, gimbal_lug_center_y, 0.0)),
        material=gimbal_finish,
        name="upper_lug",
    )
    gimbal_ring.visual(
        Box(gimbal_lug_size),
        origin=Origin(xyz=(0.0, -gimbal_lug_center_y, 0.0)),
        material=gimbal_finish,
        name="lower_lug",
    )
    gimbal_ring.visual(
        Cylinder(radius=gimbal_pin_radius, length=gimbal_pin_length),
        origin=Origin(xyz=(0.0, gimbal_pin_center_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="upper_pin",
    )
    gimbal_ring.visual(
        Cylinder(radius=gimbal_pin_radius, length=gimbal_pin_length),
        origin=Origin(xyz=(0.0, -gimbal_pin_center_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="lower_pin",
    )
    gimbal_ring.visual(
        Box(gimbal_cradle_size),
        origin=Origin(xyz=(gimbal_cradle_center_x, 0.0, 0.0)),
        material=gimbal_finish,
        name="right_cradle",
    )
    gimbal_ring.visual(
        Box(gimbal_cradle_size),
        origin=Origin(xyz=(-gimbal_cradle_center_x, 0.0, 0.0)),
        material=gimbal_finish,
        name="left_cradle",
    )

    flywheel = model.part("flywheel")
    flywheel.visual(
        mesh_from_geometry(
            _annulus_mesh(
                outer_radius=disc_rim_outer_radius,
                inner_radius=disc_rim_inner_radius,
                thickness=disc_rim_thickness,
            ),
            "flywheel_rim",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=flywheel_rim,
        name="rim",
    )
    flywheel.visual(
        mesh_from_geometry(
            _annulus_mesh(
                outer_radius=disc_web_outer_radius,
                inner_radius=disc_web_inner_radius,
                thickness=disc_web_thickness,
            ),
            "flywheel_web",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=flywheel_core,
        name="web",
    )
    flywheel.visual(
        Cylinder(radius=disc_hub_radius, length=disc_hub_length),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=flywheel_core,
        name="hub",
    )
    flywheel.visual(
        Cylinder(radius=disc_axle_radius, length=disc_axle_length),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=axle_steel,
        name="axle",
    )

    model.articulation(
        "outer_to_gimbal",
        ArticulationType.CONTINUOUS,
        parent=outer_ring,
        child=gimbal_ring,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=18.0),
    )
    model.articulation(
        "gimbal_to_flywheel",
        ArticulationType.CONTINUOUS,
        parent=gimbal_ring,
        child=flywheel,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_ring = object_model.get_part("outer_ring")
    gimbal_ring = object_model.get_part("gimbal_ring")
    flywheel = object_model.get_part("flywheel")
    outer_to_gimbal = object_model.get_articulation("outer_to_gimbal")
    gimbal_to_flywheel = object_model.get_articulation("gimbal_to_flywheel")

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

    ctx.expect_origin_distance(
        gimbal_ring,
        outer_ring,
        axes="xyz",
        max_dist=1e-6,
        name="gimbal_centered_in_outer_ring",
    )
    ctx.expect_origin_distance(
        flywheel,
        gimbal_ring,
        axes="xyz",
        max_dist=1e-6,
        name="flywheel_centered_in_gimbal",
    )

    ctx.check(
        "outer_to_gimbal_axis_is_y",
        tuple(outer_to_gimbal.axis) == (0.0, 1.0, 0.0),
        f"expected (0, 1, 0), got {outer_to_gimbal.axis}",
    )
    ctx.check(
        "gimbal_to_flywheel_axis_is_x",
        tuple(gimbal_to_flywheel.axis) == (1.0, 0.0, 0.0),
        f"expected (1, 0, 0), got {gimbal_to_flywheel.axis}",
    )
    ctx.check(
        "continuous_gyro_joints",
        (
            outer_to_gimbal.articulation_type == ArticulationType.CONTINUOUS
            and gimbal_to_flywheel.articulation_type == ArticulationType.CONTINUOUS
            and outer_to_gimbal.motion_limits is not None
            and gimbal_to_flywheel.motion_limits is not None
            and outer_to_gimbal.motion_limits.lower is None
            and outer_to_gimbal.motion_limits.upper is None
            and gimbal_to_flywheel.motion_limits.lower is None
            and gimbal_to_flywheel.motion_limits.upper is None
        ),
        "gyro ring and flywheel should spin continuously without hard stops",
    )

    ctx.expect_contact(
        gimbal_ring,
        outer_ring,
        elem_a="upper_pin",
        elem_b="upper_journal",
        name="upper_gimbal_pin_seated",
    )
    ctx.expect_contact(
        gimbal_ring,
        outer_ring,
        elem_a="lower_pin",
        elem_b="lower_journal",
        name="lower_gimbal_pin_seated",
    )
    ctx.expect_contact(
        flywheel,
        gimbal_ring,
        elem_a="axle",
        elem_b="left_cradle",
        name="left_flywheel_bearing_seated",
    )
    ctx.expect_contact(
        flywheel,
        gimbal_ring,
        elem_a="axle",
        elem_b="right_cradle",
        name="right_flywheel_bearing_seated",
    )

    with ctx.pose({outer_to_gimbal: math.pi / 2.0}):
        ctx.expect_contact(
            gimbal_ring,
            outer_ring,
            elem_a="upper_pin",
            elem_b="upper_journal",
            name="upper_gimbal_pin_contact_at_quarter_turn",
        )
        ctx.expect_contact(
            gimbal_ring,
            outer_ring,
            elem_a="lower_pin",
            elem_b="lower_journal",
            name="lower_gimbal_pin_contact_at_quarter_turn",
        )

    with ctx.pose({gimbal_to_flywheel: math.pi / 2.0}):
        ctx.expect_contact(
            flywheel,
            gimbal_ring,
            elem_a="axle",
            elem_b="left_cradle",
            name="left_flywheel_bearing_contact_at_spin",
        )
        ctx.expect_contact(
            flywheel,
            gimbal_ring,
            elem_a="axle",
            elem_b="right_cradle",
            name="right_flywheel_bearing_contact_at_spin",
        )

    with ctx.pose({outer_to_gimbal: 1.0, gimbal_to_flywheel: 0.8}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_in_offset_gyro_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
