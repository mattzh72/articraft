from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _hollow_cylinder_mesh(name: str, outer_radius: float, inner_radius: float, height: float):
    """Closed annular sleeve centered on the local origin and aligned to +Z."""
    half_height = height / 2.0
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=((outer_radius, -half_height), (outer_radius, half_height)),
            inner_profile=((inner_radius, -half_height), (inner_radius, half_height)),
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_arm_turnstile_gate")

    dark_paint = model.material("dark_powder_coat", color=(0.08, 0.09, 0.10, 1.0))
    base_rubber = model.material("dark_floor_plate", color=(0.03, 0.035, 0.04, 1.0))
    galvanized = model.material("galvanized_steel", color=(0.58, 0.62, 0.64, 1.0))
    brushed = model.material("brushed_hub", color=(0.74, 0.76, 0.75, 1.0))
    warning_yellow = model.material("yellow_safety_caps", color=(1.0, 0.72, 0.08, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((1.85, 1.90, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=base_rubber,
        name="floor_plate",
    )

    # Paired side guard frames create the passage and visibly bracket the rotor.
    for x in (-0.76, 0.76):
        for y in (-0.86, 0.86):
            frame.visual(
                Box((0.08, 0.08, 1.55)),
                origin=Origin(xyz=(x, y, 0.855)),
                material=dark_paint,
                name=f"side_post_{x:+.2f}_{y:+.2f}",
            )

    for y, suffix in ((-0.86, "0"), (0.86, "1")):
        for z, rail in ((0.42, "lower"), (0.98, "middle"), (1.52, "upper")):
            frame.visual(
                Box((1.60, 0.07, 0.06)),
                origin=Origin(xyz=(0.0, y, z)),
                material=dark_paint,
                name=f"side_support_{suffix}_{rail}_rail",
            )

    frame.visual(
        Box((0.14, 1.80, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 1.64)),
        material=dark_paint,
        name="top_bridge",
    )
    frame.visual(
        Cylinder(radius=0.050, length=1.58),
        origin=Origin(xyz=(0.0, 0.0, 0.86)),
        material=galvanized,
        name="central_shaft",
    )
    frame.visual(
        Cylinder(radius=0.105, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, 0.84)),
        material=galvanized,
        name="lower_bearing",
    )
    frame.visual(
        Cylinder(radius=0.105, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, 1.26)),
        material=galvanized,
        name="upper_bearing",
    )

    rotor = model.part("rotor")
    rotor.visual(
        _hollow_cylinder_mesh("rotor_hub_sleeve", outer_radius=0.13, inner_radius=0.075, height=0.30),
        material=brushed,
        name="hub_sleeve",
    )
    rotor.visual(
        _hollow_cylinder_mesh("rotor_lower_flange", outer_radius=0.155, inner_radius=0.075, height=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.145)),
        material=brushed,
        name="lower_flange",
    )
    rotor.visual(
        _hollow_cylinder_mesh("rotor_upper_flange", outer_radius=0.155, inner_radius=0.075, height=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=brushed,
        name="upper_flange",
    )

    arm_start_radius = 0.095
    arm_length = 0.60
    arm_center_radius = arm_start_radius + arm_length / 2.0
    arm_tip_radius = arm_start_radius + arm_length
    for idx, theta in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        c = math.cos(theta)
        s = math.sin(theta)
        rotor.visual(
            Cylinder(radius=0.035, length=arm_length),
            origin=Origin(
                xyz=(arm_center_radius * c, arm_center_radius * s, 0.0),
                rpy=(0.0, math.pi / 2.0, theta),
            ),
            material=brushed,
            name=f"arm_{idx}",
        )
        rotor.visual(
            Sphere(radius=0.052),
            origin=Origin(xyz=(arm_tip_radius * c, arm_tip_radius * s, 0.0)),
            material=warning_yellow,
            name=f"arm_tip_{idx}",
        )

    model.articulation(
        "frame_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 1.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("frame_to_rotor")

    ctx.check(
        "hub rotates continuously about the vertical axis",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        margin=0.0,
        name="three arm rotor fits between the paired side supports",
    )
    ctx.expect_overlap(
        rotor,
        frame,
        axes="z",
        elem_a="hub_sleeve",
        elem_b="central_shaft",
        min_overlap=0.25,
        name="hub sleeve surrounds the supported central shaft height",
    )
    ctx.expect_within(
        frame,
        rotor,
        axes="xy",
        inner_elem="central_shaft",
        outer_elem="hub_sleeve",
        margin=0.0,
        name="central shaft is contained within the hub sleeve bore footprint",
    )

    def _aabb_center_x_y(aabb):
        lo, hi = aabb
        return ((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0)

    rest_tip = ctx.part_element_world_aabb(rotor, elem="arm_tip_0")
    with ctx.pose({spin: 2.0 * math.pi / 3.0}):
        turned_tip = ctx.part_element_world_aabb(rotor, elem="arm_tip_0")
        ctx.expect_within(
            rotor,
            frame,
            axes="xy",
            margin=0.0,
            name="rotated arms remain inside the fixed side frames",
        )

    rest_xy = _aabb_center_x_y(rest_tip) if rest_tip is not None else None
    turned_xy = _aabb_center_x_y(turned_tip) if turned_tip is not None else None
    ctx.check(
        "continuous joint carries an arm tip around the center",
        rest_xy is not None
        and turned_xy is not None
        and rest_xy[0] > 0.60
        and abs(rest_xy[1]) < 0.08
        and turned_xy[0] < -0.25
        and turned_xy[1] > 0.50,
        details=f"rest_tip={rest_xy}, turned_tip={turned_xy}",
    )

    return ctx.report()


object_model = build_object_model()
