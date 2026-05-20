from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _side_shell_mesh():
    profile = [
        (-0.082, 0.004),
        (-0.075, 0.000),
        (-0.020, 0.000),
        (0.040, 0.002),
        (0.070, 0.004),
        (0.085, 0.007),
        (0.083, 0.014),
        (0.066, 0.021),
        (0.034, 0.025),
        (-0.024, 0.026),
        (-0.060, 0.023),
        (-0.078, 0.017),
    ]
    geom = ExtrudeGeometry.from_z0(profile, 0.0038)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, "utility_knife_side_shell")


def _blade_mesh():
    profile = [
        (0.000, 0.000),
        (0.043, 0.000),
        (0.052, 0.004),
        (0.052, 0.012),
        (0.037, 0.016),
        (0.010, 0.016),
        (0.000, 0.012),
    ]
    geom = ExtrudeGeometry.from_z0(profile, 0.0008)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, "utility_knife_blade")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_utility_knife")

    body_yellow = model.material("body_yellow", rgba=(0.90, 0.76, 0.13, 1.0))
    body_charcoal = model.material("body_charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.65, 0.67, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.78, 0.79, 0.80, 1.0))

    side_shell = _side_shell_mesh()
    blade_mesh = _blade_mesh()

    handle = model.part("handle")
    handle.visual(
        side_shell,
        origin=Origin(xyz=(0.0, 0.016, 0.0)),
        material=body_yellow,
        name="left_shell",
    )
    handle.visual(
        side_shell,
        origin=Origin(xyz=(0.0, -0.0122, 0.0)),
        material=body_yellow,
        name="right_shell",
    )
    handle.visual(
        Box((0.162, 0.032, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=body_charcoal,
        name="bottom_spine",
    )
    handle.visual(
        Box((0.014, 0.032, 0.018)),
        origin=Origin(xyz=(-0.076, 0.0, 0.009)),
        material=body_yellow,
        name="rear_cap",
    )
    handle.visual(
        Box((0.022, 0.032, 0.005)),
        origin=Origin(xyz=(0.074, 0.0, 0.0025)),
        material=body_charcoal,
        name="nose_bridge",
    )
    handle.visual(
        Box((0.164, 0.003, 0.009)),
        origin=Origin(xyz=(0.0, 0.0090, 0.0105)),
        material=steel,
        name="left_carrier_guide",
    )
    handle.visual(
        Box((0.164, 0.003, 0.009)),
        origin=Origin(xyz=(0.0, -0.0090, 0.0105)),
        material=steel,
        name="right_carrier_guide",
    )
    handle.visual(
        Box((0.028, 0.003, 0.010)),
        origin=Origin(xyz=(0.092, 0.0110, 0.013)),
        material=steel,
        name="left_guard_guide",
    )
    handle.visual(
        Box((0.028, 0.003, 0.010)),
        origin=Origin(xyz=(0.092, -0.0110, 0.013)),
        material=steel,
        name="right_guard_guide",
    )
    handle.visual(
        Box((0.068, 0.0016, 0.013)),
        origin=Origin(xyz=(-0.006, 0.0156, 0.0105)),
        material=grip_black,
        name="left_grip_strip",
    )
    handle.visual(
        Box((0.068, 0.0016, 0.013)),
        origin=Origin(xyz=(-0.006, -0.0156, 0.0105)),
        material=grip_black,
        name="right_grip_strip",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.170, 0.032, 0.026)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )

    blade_carrier = model.part("blade_carrier")
    blade_carrier.visual(
        Box((0.074, 0.015, 0.009)),
        origin=Origin(xyz=(0.0, 0.0, 0.0105)),
        material=steel,
        name="carrier_body",
    )
    blade_carrier.visual(
        Box((0.018, 0.015, 0.004)),
        origin=Origin(xyz=(0.028, 0.0, 0.006)),
        material=steel,
        name="blade_clamp",
    )
    blade_carrier.visual(
        Box((0.020, 0.011, 0.005)),
        origin=Origin(xyz=(0.004, 0.0, 0.0175)),
        material=body_charcoal,
        name="thumb_button",
    )
    blade_carrier.visual(
        blade_mesh,
        origin=Origin(xyz=(0.028, 0.0004, 0.005)),
        material=blade_steel,
        name="blade",
    )
    blade_carrier.inertial = Inertial.from_geometry(
        Box((0.082, 0.015, 0.022)),
        mass=0.06,
        origin=Origin(xyz=(0.004, 0.0, 0.011)),
    )

    model.articulation(
        "handle_to_blade_carrier",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=blade_carrier,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.20,
            lower=0.0,
            upper=0.055,
        ),
    )

    nose_guard = model.part("nose_guard")
    nose_guard.visual(
        Box((0.020, 0.004, 0.016)),
        origin=Origin(xyz=(0.010, 0.0145, 0.014)),
        material=body_yellow,
        name="left_guard_cheek",
    )
    nose_guard.visual(
        Box((0.020, 0.004, 0.016)),
        origin=Origin(xyz=(0.010, -0.0145, 0.014)),
        material=body_yellow,
        name="right_guard_cheek",
    )
    nose_guard.visual(
        Box((0.020, 0.033, 0.005)),
        origin=Origin(xyz=(0.010, 0.0, 0.0245)),
        material=body_yellow,
        name="guard_bridge",
    )
    nose_guard.inertial = Inertial.from_geometry(
        Box((0.020, 0.033, 0.024)),
        mass=0.025,
        origin=Origin(xyz=(0.010, 0.0, 0.015)),
    )

    model.articulation(
        "handle_to_nose_guard",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=nose_guard,
        origin=Origin(xyz=(0.086, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.12,
            lower=-0.014,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle = object_model.get_part("handle")
    blade_carrier = object_model.get_part("blade_carrier")
    nose_guard = object_model.get_part("nose_guard")
    carrier_slide = object_model.get_articulation("handle_to_blade_carrier")
    guard_slide = object_model.get_articulation("handle_to_nose_guard")

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
        "blade carrier joint is prismatic on handle axis",
        carrier_slide.articulation_type == ArticulationType.PRISMATIC
        and carrier_slide.axis == (1.0, 0.0, 0.0),
        details=f"type={carrier_slide.articulation_type}, axis={carrier_slide.axis}",
    )
    ctx.check(
        "nose guard joint is prismatic on handle axis",
        guard_slide.articulation_type == ArticulationType.PRISMATIC
        and guard_slide.axis == (1.0, 0.0, 0.0),
        details=f"type={guard_slide.articulation_type}, axis={guard_slide.axis}",
    )
    ctx.expect_contact(blade_carrier, handle)
    ctx.expect_contact(nose_guard, handle)
    ctx.expect_overlap(blade_carrier, handle, axes="yz", min_overlap=0.012)
    ctx.expect_overlap(nose_guard, handle, axes="yz", min_overlap=0.015)

    carrier_rest = ctx.part_world_position(blade_carrier)
    guard_rest = ctx.part_world_position(nose_guard)
    with ctx.pose({carrier_slide: 0.055, guard_slide: -0.014}):
        carrier_open = ctx.part_world_position(blade_carrier)
        guard_retracted = ctx.part_world_position(nose_guard)
        ctx.check(
            "blade carrier translates forward",
            carrier_rest is not None
            and carrier_open is not None
            and carrier_open[0] > carrier_rest[0] + 0.045,
            details=f"rest={carrier_rest}, open={carrier_open}",
        )
        ctx.check(
            "nose guard retracts rearward",
            guard_rest is not None
            and guard_retracted is not None
            and guard_retracted[0] < guard_rest[0] - 0.010,
            details=f"rest={guard_rest}, retracted={guard_retracted}",
        )
        ctx.expect_contact(blade_carrier, handle)
        ctx.expect_contact(nose_guard, handle)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
