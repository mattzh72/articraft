from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_mesh(outer_radius: float, inner_radius: float, height: float, name: str):
    """Centered vertical annular cylinder used for real clearance around the spindle."""
    ring = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, -height / 2.0))
    )
    return mesh_from_cadquery(ring, name, tolerance=0.0015, angular_tolerance=0.08)


def _radial_origin(radius: float, z: float, angle: float) -> Origin:
    return Origin(
        xyz=(radius * math.cos(angle), radius * math.sin(angle), z),
        rpy=(0.0, math.pi / 2.0, angle),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_turnstile_gate")

    galvanized = model.material("hot_dip_galvanized", rgba=(0.55, 0.58, 0.56, 1.0))
    dark_steel = model.material("dark_powder_coated_steel", rgba=(0.08, 0.09, 0.09, 1.0))
    safety_yellow = model.material("service_safety_yellow", rgba=(0.95, 0.72, 0.10, 1.0))
    black_rubber = model.material("replaceable_black_rubber", rgba=(0.015, 0.015, 0.012, 1.0))
    bronze = model.material("bronze_bearing_liner", rgba=(0.70, 0.43, 0.16, 1.0))
    zinc = model.material("zinc_fasteners", rgba=(0.76, 0.76, 0.70, 1.0))

    frame = model.part("frame")

    # Heavy service base: a single skid plate ties every static post, rail, and
    # bearing support together so the assembly reads as one bolted workhorse.
    frame.visual(
        Box((2.30, 1.40, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_steel,
        name="skid_plate",
    )
    frame.visual(
        Box((0.50, 0.50, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=galvanized,
        name="central_pedestal",
    )
    frame.visual(
        Cylinder(radius=0.27, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.187)),
        material=galvanized,
        name="lower_mounting_flange",
    )

    # A hollow central post and two cartridge housings leave real clearance for
    # the rotating spindle instead of hiding it inside a solid cylinder.
    frame.visual(
        _annular_mesh(0.135, 0.070, 0.46, "hollow_central_post"),
        origin=Origin(xyz=(0.0, 0.0, 0.435)),
        material=galvanized,
        name="hollow_central_post",
    )
    frame.visual(
        _annular_mesh(0.165, 0.090, 0.24, "lower_bearing_cartridge"),
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
        material=dark_steel,
        name="lower_bearing_cartridge",
    )
    frame.visual(
        _annular_mesh(0.155, 0.090, 0.22, "upper_bearing_cartridge"),
        origin=Origin(xyz=(0.0, 0.0, 1.205)),
        material=dark_steel,
        name="upper_bearing_cartridge",
    )
    frame.visual(
        _annular_mesh(0.092, 0.043, 0.205, "lower_bronze_liner"),
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
        material=bronze,
        name="lower_bronze_liner",
    )
    frame.visual(
        _annular_mesh(0.092, 0.043, 0.190, "upper_bronze_liner"),
        origin=Origin(xyz=(0.0, 0.0, 1.205)),
        material=bronze,
        name="upper_bronze_liner",
    )

    # Removable maintenance covers sit proud of the post with visible fasteners.
    for side, y in (("front", -0.141), ("rear", 0.141)):
        frame.visual(
            Box((0.22, 0.018, 0.30)),
            origin=Origin(xyz=(0.0, y, 0.49)),
            material=safety_yellow,
            name=f"{side}_service_panel",
        )
        for ix, x in enumerate((-0.075, 0.075)):
            for iz, z in enumerate((0.39, 0.59)):
                frame.visual(
                    Cylinder(radius=0.013, length=0.010),
                    origin=Origin(
                        xyz=(x, y - math.copysign(0.010, y), z),
                        rpy=(-math.pi / 2.0, 0.0, 0.0),
                    ),
                    material=zinc,
                    name=f"{side}_panel_bolt_{ix}_{iz}",
                )
        frame.visual(
            Box((0.12, 0.020, 0.018)),
            origin=Origin(xyz=(0.0, y - math.copysign(0.018, y), 0.49)),
            material=dark_steel,
            name=f"{side}_pull_handle",
        )

    # Overhead gantry supports the upper bearing without putting posts in the
    # sweep of the radial arms.
    for x in (-1.05, 1.05):
        for y in (-0.58, 0.58):
            frame.visual(
                Cylinder(radius=0.042, length=1.30),
                origin=Origin(xyz=(x, y, 0.70)),
                material=galvanized,
                name=f"gantry_post_{x:+.0f}_{y:+.0f}",
            )
            frame.visual(
                Cylinder(radius=0.022, length=0.035),
                origin=Origin(xyz=(x, y, 0.075)),
                material=zinc,
                name=f"base_anchor_{x:+.0f}_{y:+.0f}",
            )

    frame.visual(
        Box((2.22, 0.085, 0.075)),
        origin=Origin(xyz=(0.0, -0.58, 1.33)),
        material=galvanized,
        name="front_top_crossbeam",
    )
    frame.visual(
        Box((2.22, 0.085, 0.075)),
        origin=Origin(xyz=(0.0, 0.58, 1.33)),
        material=galvanized,
        name="rear_top_crossbeam",
    )
    frame.visual(
        Box((0.085, 1.24, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 1.33)),
        material=galvanized,
        name="bearing_bridge",
    )
    for angle in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
        frame.visual(
            Box((0.32, 0.060, 0.075)),
            origin=Origin(
                xyz=(0.16 * math.cos(angle), 0.16 * math.sin(angle), 1.285),
                rpy=(0.0, 0.0, angle),
            ),
            material=galvanized,
            name=f"bearing_gusset_{int(round(angle * 10))}",
        )

    # Guard rails are connected through the base posts and kept above/below the
    # rotor arm plane to provide a field-friendly passage with generous clearance.
    for y in (-0.58, 0.58):
        for z, label in ((0.44, "lower"), (1.04, "upper")):
            frame.visual(
                Cylinder(radius=0.030, length=2.06),
                origin=Origin(xyz=(0.0, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=galvanized,
                name=f"{label}_guide_rail_{'front' if y < 0 else 'rear'}",
            )
        frame.visual(
            Box((2.16, 0.025, 0.10)),
            origin=Origin(xyz=(0.0, y, 0.22)),
            material=black_rubber,
            name=f"replaceable_kick_wear_strip_{'front' if y < 0 else 'rear'}",
        )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.045, length=1.00),
        origin=Origin(xyz=(0.0, 0.0, 0.72)),
        material=dark_steel,
        name="spindle",
    )
    rotor.visual(
        Cylinder(radius=0.175, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.82)),
        material=galvanized,
        name="bearing_core",
    )
    rotor.visual(
        Cylinder(radius=0.118, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.705)),
        material=dark_steel,
        name="lower_shaft_collar",
    )
    rotor.visual(
        Cylinder(radius=0.118, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 1.025)),
        material=dark_steel,
        name="upper_shaft_collar",
    )

    arm_length = 0.82
    arm_start_radius = 0.135
    arm_center_radius = arm_start_radius + arm_length / 2.0
    sleeve_radius = arm_start_radius + arm_length - 0.055
    for idx, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        rotor.visual(
            Cylinder(radius=0.036, length=arm_length),
            origin=_radial_origin(arm_center_radius, 0.82, angle),
            material=galvanized,
            name=f"arm_{idx}",
        )
        rotor.visual(
            Cylinder(radius=0.050, length=0.130),
            origin=_radial_origin(sleeve_radius, 0.82, angle),
            material=black_rubber,
            name=f"wear_sleeve_{idx}",
        )
        rotor.visual(
            Box((0.105, 0.028, 0.095)),
            origin=Origin(
                xyz=(0.235 * math.cos(angle), 0.235 * math.sin(angle), 0.82),
                rpy=(0.0, 0.0, angle),
            ),
            material=safety_yellow,
            name=f"split_clamp_{idx}",
        )

    model.articulation(
        "frame_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0),
        motion_properties=MotionProperties(damping=0.8, friction=0.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    joint = object_model.get_articulation("frame_to_rotor")

    ctx.allow_overlap(
        frame,
        rotor,
        elem_a="lower_bronze_liner",
        elem_b="spindle",
        reason=(
            "The serviceable bronze liner is intentionally modeled as a light "
            "interference fit around the spindle so the rotating core is "
            "captured and supported rather than floating in an artificial gap."
        ),
    )
    ctx.allow_overlap(
        frame,
        rotor,
        elem_a="upper_bronze_liner",
        elem_b="spindle",
        reason=(
            "The upper bearing liner lightly captures the spindle as a modeled "
            "bushing fit; the overlap is local to the replaceable wear liner."
        ),
    )

    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        inner_elem="spindle",
        outer_elem="lower_bearing_cartridge",
        margin=0.0,
        name="spindle centered in lower bearing housing footprint",
    )
    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        inner_elem="spindle",
        outer_elem="upper_bearing_cartridge",
        margin=0.0,
        name="spindle centered in upper bearing housing footprint",
    )
    ctx.expect_overlap(
        rotor,
        frame,
        axes="z",
        elem_a="spindle",
        elem_b="lower_bearing_cartridge",
        min_overlap=0.20,
        name="spindle retained through lower bearing",
    )
    ctx.expect_overlap(
        rotor,
        frame,
        axes="z",
        elem_a="spindle",
        elem_b="upper_bearing_cartridge",
        min_overlap=0.10,
        name="spindle retained through upper bearing",
    )

    rest_arm_aabb = ctx.part_element_world_aabb(rotor, elem="arm_0")
    with ctx.pose({joint: math.pi / 2.0}):
        turned_arm_aabb = ctx.part_element_world_aabb(rotor, elem="arm_0")

    if rest_arm_aabb is not None and turned_arm_aabb is not None:
        rest_min, rest_max = rest_arm_aabb
        turned_min, turned_max = turned_arm_aabb
        rest_center_x = (rest_min[0] + rest_max[0]) / 2.0
        turned_center_y = (turned_min[1] + turned_max[1]) / 2.0
        ctx.check(
            "radial arm rotates about spindle axis",
            rest_center_x > 0.45 and turned_center_y > 0.45,
            details=f"rest_center_x={rest_center_x:.3f}, turned_center_y={turned_center_y:.3f}",
        )
    else:
        ctx.fail("radial arm rotates about spindle axis", "arm_0 AABB unavailable")

    return ctx.report()


object_model = build_object_model()
