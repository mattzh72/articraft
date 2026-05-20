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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _regular_polygon_profile(radius: float, sides: int, *, rotation: float = 0.0) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(rotation + (2.0 * math.pi * index / sides)),
            radius * math.sin(rotation + (2.0 * math.pi * index / sides)),
        )
        for index in range(sides)
    ]


def _circle_profile(radius: float, *, segments: int = 48) -> list[tuple[float, float]]:
    return _regular_polygon_profile(radius, segments)


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return (
        0.5 * (lower[0] + upper[0]),
        0.5 * (lower[1] + upper[1]),
        0.5 * (lower[2] + upper[2]),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_lobe_fidget_spinner")

    body_black = model.material("body_black", rgba=(0.11, 0.11, 0.12, 1.0))
    brass_weight = model.material("brass_weight", rgba=(0.72, 0.56, 0.23, 1.0))
    steel = model.material("steel", rgba=(0.77, 0.79, 0.82, 1.0))

    body_thickness = 0.0080
    arm_thickness = 0.0062
    weight_thickness = 0.0072
    hub_across_flats = 0.0230
    hub_outer_radius = hub_across_flats / math.sqrt(3.0)
    spindle_radius = 0.0032
    cap_radius = 0.0110
    cap_thickness = 0.0024
    cap_gap = 0.00035
    cap_center_z = (body_thickness * 0.5) + cap_gap + (cap_thickness * 0.5)
    spindle_length = (2.0 * cap_center_z) + cap_thickness
    arm_length = 0.0300
    arm_width = 0.0135
    arm_center_radius = 0.0175
    weight_center_radius = 0.0260
    weight_radius = 0.0118
    lobe_shell_radius = 0.0152

    grip_core = model.part("grip_core")
    grip_core.visual(
        Cylinder(radius=spindle_radius, length=spindle_length),
        material=steel,
        name="axle_spindle",
    )
    grip_core.visual(
        Cylinder(radius=cap_radius, length=cap_thickness),
        origin=Origin(xyz=(0.0, 0.0, cap_center_z)),
        material=steel,
        name="top_cap_plate",
    )
    grip_core.visual(
        Cylinder(radius=cap_radius, length=cap_thickness),
        origin=Origin(xyz=(0.0, 0.0, -cap_center_z)),
        material=steel,
        name="bottom_cap_plate",
    )
    grip_core.inertial = Inertial.from_geometry(
        Box((cap_radius * 2.0, cap_radius * 2.0, spindle_length)),
        mass=0.018,
        origin=Origin(),
    )

    spinner_body = model.part("spinner_body")
    hub_ring_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _regular_polygon_profile(hub_outer_radius, 6, rotation=math.pi / 6.0),
            [_circle_profile(spindle_radius, segments=56)],
            height=body_thickness,
            center=True,
        ),
        "spinner_hub_ring",
    )
    arm_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(
                arm_length,
                arm_width,
                radius=arm_width * 0.5,
                corner_segments=8,
            ),
            height=arm_thickness,
            center=True,
        ),
        "spinner_lobe_arm",
    )

    spinner_body.visual(hub_ring_mesh, material=body_black, name="hub_ring")
    for index in range(3):
        angle = (2.0 * math.pi * index) / 3.0
        ux = math.cos(angle)
        uy = math.sin(angle)
        spinner_body.visual(
            arm_mesh,
            origin=Origin(
                xyz=(arm_center_radius * ux, arm_center_radius * uy, 0.0),
                rpy=(0.0, 0.0, angle),
            ),
            material=body_black,
            name=f"lobe_arm_{index}",
        )
        spinner_body.visual(
            Cylinder(radius=lobe_shell_radius, length=body_thickness),
            origin=Origin(xyz=(weight_center_radius * ux, weight_center_radius * uy, 0.0)),
            material=body_black,
            name=f"lobe_shell_{index}",
        )
        spinner_body.visual(
            Cylinder(radius=weight_radius, length=weight_thickness),
            origin=Origin(xyz=(weight_center_radius * ux, weight_center_radius * uy, 0.0)),
            material=brass_weight,
            name=f"weight_pad_{index}",
        )
    spinner_body.inertial = Inertial.from_geometry(
        Box((0.076, 0.076, body_thickness)),
        mass=0.074,
        origin=Origin(),
    )

    model.articulation(
        "spinner_spin",
        ArticulationType.CONTINUOUS,
        parent=grip_core,
        child=spinner_body,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=40.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    grip_core = object_model.get_part("grip_core")
    spinner_body = object_model.get_part("spinner_body")
    spin_joint = object_model.get_articulation("spinner_spin")

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
    ctx.allow_overlap(
        grip_core,
        spinner_body,
        elem_a="axle_spindle",
        elem_b="hub_ring",
        reason="The spinner body uses a simplified solid bearing hub proxy around the hidden axle spindle.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_origin_distance(
        spinner_body,
        grip_core,
        axes="xy",
        min_dist=0.0,
        max_dist=0.0001,
        name="spinner body stays centered on the cap axle",
    )
    ctx.expect_gap(
        grip_core,
        spinner_body,
        axis="z",
        positive_elem="top_cap_plate",
        negative_elem="hub_ring",
        min_gap=0.0002,
        max_gap=0.0008,
        name="top cap clears the rotating hub face",
    )
    ctx.expect_gap(
        spinner_body,
        grip_core,
        axis="z",
        positive_elem="hub_ring",
        negative_elem="bottom_cap_plate",
        min_gap=0.0002,
        max_gap=0.0008,
        name="bottom cap clears the rotating hub face",
    )
    ctx.expect_overlap(
        spinner_body,
        grip_core,
        axes="xy",
        elem_a="hub_ring",
        elem_b="top_cap_plate",
        min_overlap=0.020,
        name="cap plates sit over the central hub bearing area",
    )
    ctx.expect_within(
        grip_core,
        spinner_body,
        axes="xy",
        inner_elem="axle_spindle",
        outer_elem="hub_ring",
        margin=0.010,
        name="axle spindle remains centered within the hub envelope",
    )

    rest_weight_center = _aabb_center(ctx.part_element_world_aabb(spinner_body, elem="weight_pad_0"))
    with ctx.pose({spin_joint: 1.0}):
        spun_weight_center = _aabb_center(ctx.part_element_world_aabb(spinner_body, elem="weight_pad_0"))
    ctx.check(
        "continuous axle rotates a lobe around the hub center",
        rest_weight_center is not None
        and spun_weight_center is not None
        and spun_weight_center[1] > rest_weight_center[1] + 0.015
        and spun_weight_center[0] < rest_weight_center[0] - 0.008,
        details=f"rest={rest_weight_center}, spun={spun_weight_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
