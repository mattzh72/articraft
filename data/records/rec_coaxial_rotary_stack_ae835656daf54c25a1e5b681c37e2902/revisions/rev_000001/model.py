from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SHAFT_RADIUS = 0.0105
BORE_RADIUS = 0.0130
SHAFT_LENGTH = 0.340
STAGE_ZS = (-0.055, -0.145, -0.245)


def _support_frame_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(0.220, 0.080, 0.012, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
    )

    hang_lug = cq.Workplane("XY").box(0.056, 0.020, 0.040).translate((0.0, 0.0, 0.032))
    lug_hole = cq.Workplane("XZ").center(0.0, 0.036).circle(0.009).extrude(0.030, both=True)

    return plate.union(hang_lug).cut(lug_hole)


def _stage_shell_shape(
    *,
    outer_radius: float,
    ring_width: float,
    ring_height: float,
    top_drop: float,
    hub_outer_radius: float,
    hub_height: float,
    spoke_width: float,
    spoke_height: float,
    spoke_count: int = 3,
) -> cq.Workplane:
    rim_inner_radius = outer_radius - ring_width

    rim = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(rim_inner_radius)
        .extrude(ring_height)
        .translate((0.0, 0.0, -top_drop - ring_height))
    )

    hub = (
        cq.Workplane("XY")
        .circle(hub_outer_radius)
        .circle(BORE_RADIUS)
        .extrude(hub_height)
        .translate((0.0, 0.0, -hub_height))
    )

    shell = rim.union(hub)

    spoke_span = rim_inner_radius - hub_outer_radius + 0.010
    spoke_mid = 0.5 * (rim_inner_radius + hub_outer_radius)
    spoke_center_z = -0.5 * (top_drop + hub_height)

    for index in range(spoke_count):
        angle_deg = 360.0 * index / spoke_count
        spoke = (
            cq.Workplane("XY")
            .box(spoke_span, spoke_width, spoke_height)
            .translate((spoke_mid, 0.0, spoke_center_z))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        shell = shell.union(spoke)

    return shell


def _add_stage(
    model: ArticulatedObject,
    support,
    *,
    part_name: str,
    joint_name: str,
    mesh_name: str,
    stage_z: float,
    outer_radius: float,
    ring_width: float,
    ring_height: float,
    top_drop: float,
    hub_outer_radius: float,
    hub_height: float,
    spoke_width: float,
    spoke_height: float,
    collar_radius: float,
    pointer_color: str,
) -> None:
    stage = model.part(part_name)
    stage.visual(
        mesh_from_cadquery(
            _stage_shell_shape(
                outer_radius=outer_radius,
                ring_width=ring_width,
                ring_height=ring_height,
                top_drop=top_drop,
                hub_outer_radius=hub_outer_radius,
                hub_height=hub_height,
                spoke_width=spoke_width,
                spoke_height=spoke_height,
            ),
            mesh_name,
        ),
        material="stage_shell",
        name="shell",
    )

    pad_height = 0.004
    pad_size = (0.018, 0.010, pad_height)
    pad_radius = min(collar_radius - 0.007, hub_outer_radius - 0.001 + pad_size[0] * 0.5)
    for index in range(3):
        angle = 2.0 * pi * index / 3.0
        stage.visual(
            Box(pad_size),
            origin=Origin(
                xyz=(pad_radius * cos(angle), pad_radius * sin(angle), -0.5 * pad_height),
                rpy=(0.0, 0.0, angle),
            ),
            material="stage_shell",
            name=f"pad_{index + 1}",
        )

    pointer_length = 0.022
    stage.visual(
        Box((pointer_length, ring_width * 0.9, ring_height * 0.7)),
        origin=Origin(
            xyz=(
                outer_radius + 0.5 * pointer_length - 0.004,
                0.0,
                -top_drop - 0.5 * ring_height,
            )
        ),
        material=pointer_color,
        name="pointer",
    )

    model.articulation(
        joint_name,
        ArticulationType.REVOLUTE,
        parent=support,
        child=stage,
        origin=Origin(xyz=(0.0, 0.0, stage_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=-0.9 * pi,
            upper=0.9 * pi,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_coaxial_stage_stack")

    model.material("support_frame", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("shaft_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("stage_shell", rgba=(0.62, 0.65, 0.69, 1.0))
    model.material("pointer_amber", rgba=(0.88, 0.57, 0.14, 1.0))
    model.material("pointer_blue", rgba=(0.17, 0.36, 0.67, 1.0))
    model.material("pointer_red", rgba=(0.72, 0.22, 0.20, 1.0))

    support = model.part("support")
    support.visual(
        mesh_from_cadquery(_support_frame_shape(), "support_frame"),
        material="support_frame",
        name="frame",
    )
    support.visual(
        Cylinder(radius=0.025, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material="shaft_steel",
        name="top_boss",
    )
    support.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -0.5 * SHAFT_LENGTH + 0.001)),
        material="shaft_steel",
        name="center_shaft",
    )

    collar_specs = (
        ("collar_1", STAGE_ZS[0] + 0.004, 0.028),
        ("collar_2", STAGE_ZS[1] + 0.004, 0.031),
        ("collar_3", STAGE_ZS[2] + 0.004, 0.035),
    )
    for collar_name, collar_center_z, collar_radius in collar_specs:
        support.visual(
            Cylinder(radius=collar_radius, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, collar_center_z)),
            material="shaft_steel",
            name=collar_name,
        )

    _add_stage(
        model,
        support,
        part_name="stage_upper",
        joint_name="support_to_stage_upper",
        mesh_name="stage_upper_shell",
        stage_z=STAGE_ZS[0],
        outer_radius=0.056,
        ring_width=0.013,
        ring_height=0.016,
        top_drop=0.026,
        hub_outer_radius=0.022,
        hub_height=0.014,
        spoke_width=0.010,
        spoke_height=0.024,
        collar_radius=0.028,
        pointer_color="pointer_amber",
    )
    _add_stage(
        model,
        support,
        part_name="stage_middle",
        joint_name="support_to_stage_middle",
        mesh_name="stage_middle_shell",
        stage_z=STAGE_ZS[1],
        outer_radius=0.084,
        ring_width=0.015,
        ring_height=0.018,
        top_drop=0.028,
        hub_outer_radius=0.024,
        hub_height=0.014,
        spoke_width=0.011,
        spoke_height=0.026,
        collar_radius=0.031,
        pointer_color="pointer_blue",
    )
    _add_stage(
        model,
        support,
        part_name="stage_lower",
        joint_name="support_to_stage_lower",
        mesh_name="stage_lower_shell",
        stage_z=STAGE_ZS[2],
        outer_radius=0.114,
        ring_width=0.018,
        ring_height=0.020,
        top_drop=0.030,
        hub_outer_radius=0.026,
        hub_height=0.014,
        spoke_width=0.012,
        spoke_height=0.028,
        collar_radius=0.035,
        pointer_color="pointer_red",
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    stage_upper = object_model.get_part("stage_upper")
    stage_middle = object_model.get_part("stage_middle")
    stage_lower = object_model.get_part("stage_lower")

    upper_joint = object_model.get_articulation("support_to_stage_upper")
    middle_joint = object_model.get_articulation("support_to_stage_middle")
    lower_joint = object_model.get_articulation("support_to_stage_lower")

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

    ctx.expect_contact(support, stage_upper, name="upper stage hangs from the support collars")
    ctx.expect_contact(support, stage_middle, name="middle stage hangs from the support collars")
    ctx.expect_contact(support, stage_lower, name="lower stage hangs from the support collars")

    ctx.expect_gap(
        stage_upper,
        stage_middle,
        axis="z",
        min_gap=0.040,
        name="middle stage clears the upper stage vertically",
    )
    ctx.expect_gap(
        stage_middle,
        stage_lower,
        axis="z",
        min_gap=0.040,
        name="lower stage clears the middle stage vertically",
    )

    joint_axes = [upper_joint.axis, middle_joint.axis, lower_joint.axis]
    joint_origins = [upper_joint.origin.xyz, middle_joint.origin.xyz, lower_joint.origin.xyz]
    ctx.check(
        "all stage joints share one vertical coaxial axis",
        all(axis == (0.0, 0.0, 1.0) for axis in joint_axes)
        and all(abs(origin[0]) < 1e-9 and abs(origin[1]) < 1e-9 for origin in joint_origins)
        and joint_origins[0][2] > joint_origins[1][2] > joint_origins[2][2],
        details=f"axes={joint_axes}, origins={joint_origins}",
    )

    def pointer_center(part_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_name, elem="pointer")
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple(0.5 * (lo + hi) for lo, hi in zip(mins, maxs))

    upper_pointer_rest = pointer_center("stage_upper")
    middle_pointer_rest = pointer_center("stage_middle")
    lower_pointer_rest = pointer_center("stage_lower")

    with ctx.pose({upper_joint: 1.0}):
        upper_pointer_turned = pointer_center("stage_upper")
        middle_pointer_still = pointer_center("stage_middle")

    with ctx.pose({middle_joint: -1.0}):
        middle_pointer_turned = pointer_center("stage_middle")
        lower_pointer_still = pointer_center("stage_lower")

    with ctx.pose({lower_joint: 0.8}):
        lower_pointer_turned = pointer_center("stage_lower")
        upper_pointer_still = pointer_center("stage_upper")

    ctx.check(
        "upper stage rotates without dragging the middle stage",
        upper_pointer_rest is not None
        and upper_pointer_turned is not None
        and middle_pointer_rest is not None
        and middle_pointer_still is not None
        and abs(upper_pointer_turned[1] - upper_pointer_rest[1]) > 0.030
        and abs(middle_pointer_still[1] - middle_pointer_rest[1]) < 1e-6,
        details=(
            f"upper_rest={upper_pointer_rest}, upper_turned={upper_pointer_turned}, "
            f"middle_rest={middle_pointer_rest}, middle_still={middle_pointer_still}"
        ),
    )
    ctx.check(
        "middle stage rotates without dragging the lower stage",
        middle_pointer_rest is not None
        and middle_pointer_turned is not None
        and lower_pointer_rest is not None
        and lower_pointer_still is not None
        and abs(middle_pointer_turned[1] - middle_pointer_rest[1]) > 0.045
        and abs(lower_pointer_still[1] - lower_pointer_rest[1]) < 1e-6,
        details=(
            f"middle_rest={middle_pointer_rest}, middle_turned={middle_pointer_turned}, "
            f"lower_rest={lower_pointer_rest}, lower_still={lower_pointer_still}"
        ),
    )
    ctx.check(
        "lower stage rotates independently of the upper stage",
        lower_pointer_rest is not None
        and lower_pointer_turned is not None
        and upper_pointer_rest is not None
        and upper_pointer_still is not None
        and abs(lower_pointer_turned[1] - lower_pointer_rest[1]) > 0.040
        and abs(upper_pointer_still[1] - upper_pointer_rest[1]) < 1e-6,
        details=(
            f"lower_rest={lower_pointer_rest}, lower_turned={lower_pointer_turned}, "
            f"upper_rest={upper_pointer_rest}, upper_still={upper_pointer_still}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
