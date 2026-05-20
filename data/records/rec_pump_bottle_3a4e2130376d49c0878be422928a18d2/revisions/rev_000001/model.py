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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _rounded_section(
    width: float,
    depth: float,
    z: float,
    *,
    radius: float,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z)
        for x, y in rounded_rect_profile(
            width,
            depth,
            radius,
            corner_segments=corner_segments,
        )
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cosmetic_pump_bottle")

    body_shell = model.material("body_shell", rgba=(0.93, 0.90, 0.88, 1.0))
    collar_finish = model.material("collar_finish", rgba=(0.96, 0.96, 0.97, 1.0))
    actuator_finish = model.material("actuator_finish", rgba=(0.98, 0.98, 0.99, 1.0))
    accent_finish = model.material("accent_finish", rgba=(0.86, 0.84, 0.83, 1.0))
    clear_cap = model.material("clear_cap", rgba=(0.86, 0.94, 0.98, 0.28))

    body = model.part("bottle_body")
    body_mesh = section_loft(
        [
            _rounded_section(0.062, 0.062, 0.000, radius=0.008),
            _rounded_section(0.062, 0.062, 0.092, radius=0.008),
            _rounded_section(0.058, 0.058, 0.104, radius=0.008),
            _rounded_section(0.046, 0.046, 0.118, radius=0.006),
        ]
    )
    body.visual(
        mesh_from_geometry(body_mesh, "pump_bottle_body_shell"),
        material=body_shell,
        name="body_shell",
    )
    body.visual(
        Box((0.050, 0.050, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=accent_finish,
        name="base_pad",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.062, 0.062, 0.122)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
    )

    collar = model.part("collar")
    collar.visual(
        Box((0.040, 0.040, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=collar_finish,
        name="collar_flange",
    )
    collar.visual(
        Box((0.034, 0.009, 0.012)),
        origin=Origin(xyz=(0.0, 0.0125, 0.006)),
        material=collar_finish,
        name="front_guide_wall",
    )
    collar.visual(
        Box((0.034, 0.009, 0.012)),
        origin=Origin(xyz=(0.0, -0.0125, 0.006)),
        material=collar_finish,
        name="rear_guide_wall",
    )
    collar.visual(
        Box((0.009, 0.016, 0.012)),
        origin=Origin(xyz=(-0.0125, 0.0, 0.006)),
        material=collar_finish,
        name="left_guide_wall",
    )
    collar.visual(
        Box((0.009, 0.016, 0.012)),
        origin=Origin(xyz=(0.0125, 0.0, 0.006)),
        material=collar_finish,
        name="right_guide_wall",
    )
    collar.visual(
        Box((0.028, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, -0.018, 0.011)),
        material=collar_finish,
        name="hinge_boss",
    )
    collar.visual(
        Cylinder(radius=0.0033, length=0.008),
        origin=Origin(xyz=(-0.010, -0.020, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=collar_finish,
        name="left_hinge_knuckle",
    )
    collar.visual(
        Cylinder(radius=0.0033, length=0.008),
        origin=Origin(xyz=(0.010, -0.020, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=collar_finish,
        name="right_hinge_knuckle",
    )
    collar.inertial = Inertial.from_geometry(
        Box((0.040, 0.040, 0.022)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )

    actuator = model.part("actuator")
    actuator.visual(
        Box((0.016, 0.016, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=actuator_finish,
        name="slide_plunger",
    )
    actuator.visual(
        Cylinder(radius=0.0036, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.025), rpy=(0.0, 0.0, 0.0)),
        material=actuator_finish,
        name="actuator_stem",
    )
    actuator.visual(
        Box((0.030, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.004, 0.036)),
        material=actuator_finish,
        name="actuator_head",
    )
    actuator.visual(
        Box((0.022, 0.014, 0.004)),
        origin=Origin(xyz=(0.0, 0.002, 0.043)),
        material=actuator_finish,
        name="finger_pad",
    )
    actuator.visual(
        Cylinder(radius=0.0032, length=0.014),
        origin=Origin(xyz=(0.0, 0.016, 0.037), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=actuator_finish,
        name="nozzle",
    )
    actuator.inertial = Inertial.from_geometry(
        Box((0.030, 0.022, 0.047)),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.004, 0.0235)),
    )

    overcap = model.part("overcap")
    cap_width = 0.058
    cap_depth = 0.058
    cap_height = 0.058
    cap_wall = 0.0022
    cap_bottom = -0.003
    overcap.visual(
        Box((cap_width, cap_depth, cap_wall)),
        origin=Origin(
            xyz=(
                0.0,
                cap_depth * 0.5,
                cap_bottom + cap_height - cap_wall * 0.5,
            )
        ),
        material=clear_cap,
        name="cap_top",
    )
    overcap.visual(
        Box((cap_width, cap_wall, cap_height)),
        origin=Origin(xyz=(0.0, cap_wall * 0.5, cap_bottom + cap_height * 0.5)),
        material=clear_cap,
        name="rear_wall",
    )
    overcap.visual(
        Box((cap_width, cap_wall, cap_height)),
        origin=Origin(
            xyz=(
                0.0,
                cap_depth - cap_wall * 0.5,
                cap_bottom + cap_height * 0.5,
            )
        ),
        material=clear_cap,
        name="front_wall",
    )
    overcap.visual(
        Box((cap_wall, cap_depth - 2.0 * cap_wall, cap_height)),
        origin=Origin(
            xyz=(
                -cap_width * 0.5 + cap_wall * 0.5,
                cap_depth * 0.5,
                cap_bottom + cap_height * 0.5,
            )
        ),
        material=clear_cap,
        name="left_wall",
    )
    overcap.visual(
        Box((cap_wall, cap_depth - 2.0 * cap_wall, cap_height)),
        origin=Origin(
            xyz=(
                cap_width * 0.5 - cap_wall * 0.5,
                cap_depth * 0.5,
                cap_bottom + cap_height * 0.5,
            )
        ),
        material=clear_cap,
        name="right_wall",
    )
    overcap.visual(
        Cylinder(radius=0.0033, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clear_cap,
        name="cap_barrel",
    )
    overcap.inertial = Inertial.from_geometry(
        Box((cap_width, cap_depth, cap_height)),
        mass=0.025,
        origin=Origin(xyz=(0.0, cap_depth * 0.5, cap_bottom + cap_height * 0.5)),
    )

    model.articulation(
        "body_to_collar",
        ArticulationType.FIXED,
        parent=body,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
    )
    model.articulation(
        "collar_to_actuator",
        ArticulationType.PRISMATIC,
        parent=collar,
        child=actuator,
        origin=Origin(),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.06,
            lower=0.0,
            upper=0.004,
        ),
    )
    model.articulation(
        "collar_to_overcap",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=overcap,
        origin=Origin(xyz=(0.0, -0.020, 0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=2.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("bottle_body")
    collar = object_model.get_part("collar")
    actuator = object_model.get_part("actuator")
    overcap = object_model.get_part("overcap")
    pump_stroke = object_model.get_articulation("collar_to_actuator")
    cap_hinge = object_model.get_articulation("collar_to_overcap")

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

    pump_axis = tuple(round(value, 3) for value in pump_stroke.axis)
    hinge_axis = tuple(round(value, 3) for value in cap_hinge.axis)
    ctx.check(
        "pump_stroke_is_vertical",
        pump_axis == (0.0, 0.0, -1.0),
        details=f"expected vertical downward pump axis, got {pump_stroke.axis}",
    )
    ctx.check(
        "overcap_hinge_is_rear_cross_pin",
        hinge_axis == (1.0, 0.0, 0.0),
        details=f"expected hinge axis along x, got {cap_hinge.axis}",
    )

    ctx.expect_contact(collar, body, name="collar_seats_on_body")
    ctx.expect_overlap(collar, body, axes="xy", min_overlap=0.030, name="collar_is_centered_on_body")
    ctx.expect_contact(actuator, collar, name="actuator_is_guided_by_collar")
    ctx.expect_contact(overcap, collar, name="overcap_remains_clipped_to_hinge")
    ctx.expect_within(collar, overcap, axes="xy", margin=0.003, name="closed_cap_covers_collar")
    ctx.expect_within(actuator, overcap, axes="xy", margin=0.003, name="closed_cap_covers_actuator")
    ctx.expect_gap(
        actuator,
        collar,
        axis="z",
        positive_elem="actuator_head",
        negative_elem="collar_flange",
        min_gap=0.026,
        max_gap=0.029,
        name="actuator_head_sits_above_collar",
    )

    with ctx.pose({pump_stroke: pump_stroke.motion_limits.upper}):
        ctx.expect_gap(
            actuator,
            collar,
            axis="z",
            positive_elem="actuator_head",
            negative_elem="collar_flange",
            min_gap=0.022,
            max_gap=0.025,
            name="pump_stroke_moves_head_downward",
        )
        ctx.expect_contact(actuator, collar, name="actuator_stays_guided_at_full_stroke")

    with ctx.pose({cap_hinge: cap_hinge.motion_limits.upper}):
        ctx.expect_contact(overcap, collar, name="open_cap_stays_attached_at_hinge")
        cap_top_aabb = ctx.part_element_world_aabb(overcap, elem="cap_top")
        collar_aabb = ctx.part_world_aabb(collar)
        opened_ok = (
            cap_top_aabb is not None
            and collar_aabb is not None
            and cap_top_aabb[1][1] < collar_aabb[0][1]
        )
        cap_top_y = None if cap_top_aabb is None else cap_top_aabb[1][1]
        collar_min_y = None if collar_aabb is None else collar_aabb[0][1]
        ctx.check(
            "opened_cap_flips_behind_collar",
            opened_ok,
            details=f"cap top max_y={cap_top_y}, collar min_y={collar_min_y}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
