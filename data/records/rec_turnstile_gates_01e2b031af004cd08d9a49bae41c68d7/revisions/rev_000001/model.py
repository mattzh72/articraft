from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _bearing_ring_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
):
    """A simple hollow bearing collar mesh with a true center bore."""
    outer = cq.Workplane("XY").cylinder(height, outer_radius)
    bore = cq.Workplane("XY").cylinder(height * 1.35, inner_radius)
    return mesh_from_cadquery(outer.cut(bore), name, tolerance=0.0015)


def _add_horizontal_cylinder(
    part,
    *,
    name: str,
    length: float,
    radius: float,
    center: tuple[float, float, float],
    yaw: float,
    material,
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(0.0, math.pi / 2.0, yaw)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_turnstile_gate")

    frame_paint = model.material("hammered_graphite_paint", rgba=(0.11, 0.12, 0.12, 1.0))
    dark_paint = model.material("matte_black_wear_paint", rgba=(0.015, 0.017, 0.016, 1.0))
    safety_yellow = model.material("painted_safety_yellow", rgba=(0.95, 0.68, 0.05, 1.0))
    zinc = model.material("zinc_plated_fasteners", rgba=(0.72, 0.72, 0.66, 1.0))
    rubber = model.material("black_molded_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    bearing_blue = model.material("blue_grey_bearing_housing", rgba=(0.18, 0.24, 0.29, 1.0))

    frame = model.part("frame")

    # Heavy floor skid and raised anti-slip pads.
    frame.visual(
        Box((1.85, 1.42, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=frame_paint,
        name="floor_skid",
    )
    for y in (-0.46, 0.46):
        frame.visual(
            Box((1.38, 0.11, 0.012)),
            origin=Origin(xyz=(0.0, y, 0.066)),
            material=dark_paint,
            name=f"anti_slip_tread_{0 if y < 0 else 1}",
        )

    # Four perimeter posts and guard rails keep the turnstile practical and tied
    # to a fixed utility frame without entering the rotor sweep.
    post_xy = [(-0.82, -0.66), (-0.82, 0.66), (0.82, -0.66), (0.82, 0.66)]
    for i, (x, y) in enumerate(post_xy):
        frame.visual(
            Cylinder(radius=0.045, length=1.18),
            origin=Origin(xyz=(x, y, 0.64)),
            material=frame_paint,
            name=f"corner_post_{i}",
        )
        frame.visual(
            Box((0.18, 0.18, 0.025)),
            origin=Origin(xyz=(x, y, 0.075)),
            material=frame_paint,
            name=f"post_foot_{i}",
        )

    for y in (-0.66, 0.66):
        for z, label in ((0.45, "lower"), (0.98, "upper")):
            _add_horizontal_cylinder(
                frame,
                name=f"{label}_guard_rail_{0 if y < 0 else 1}",
                length=1.64,
                radius=0.032,
                center=(0.0, y, z),
                yaw=0.0,
                material=frame_paint,
            )
    for x in (-0.82, 0.82):
        _add_horizontal_cylinder(
            frame,
            name=f"top_end_rail_{0 if x < 0 else 1}",
            length=1.32,
            radius=0.030,
            center=(x, 0.0, 1.20),
            yaw=math.pi / 2.0,
            material=frame_paint,
        )

    # Forked yoke around the upper bearing leaves a real central clearance for
    # the spindle while visibly tying the collar back to the fixed frame.
    for y in (-0.18, 0.18):
        frame.visual(
            Box((1.68, 0.09, 0.09)),
            origin=Origin(xyz=(0.0, y, 1.05)),
            material=frame_paint,
            name=f"upper_yoke_beam_{0 if y < 0 else 1}",
        )
    for x in (-0.82, 0.82):
        frame.visual(
            Box((0.09, 1.32, 0.075)),
            origin=Origin(xyz=(x, 0.0, 1.05)),
            material=frame_paint,
            name=f"yoke_side_tie_{0 if x < 0 else 1}",
        )

    # True hollow bearing collars: the rotating post passes through their bores
    # rather than through a solid proxy.
    frame.visual(
        _bearing_ring_mesh(
            "lower_bearing_pedestal",
            outer_radius=0.185,
            inner_radius=0.078,
            height=0.12,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=bearing_blue,
        name="lower_bearing_pedestal",
    )
    frame.visual(
        _bearing_ring_mesh(
            "lower_bearing_collar",
            outer_radius=0.155,
            inner_radius=0.100,
            height=0.12,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        material=bearing_blue,
        name="lower_bearing_collar",
    )
    frame.visual(
        _bearing_ring_mesh(
            "upper_bearing_collar",
            outer_radius=0.160,
            inner_radius=0.100,
            height=0.14,
        ),
        origin=Origin(xyz=(0.0, 0.0, 1.05)),
        material=bearing_blue,
        name="upper_bearing_collar",
    )

    # Serviceable fasteners and washers on the base and bearing housings.
    for i, (x, y) in enumerate([(-0.72, -0.52), (-0.72, 0.52), (0.72, -0.52), (0.72, 0.52)]):
        frame.visual(
            Cylinder(radius=0.030, length=0.008),
            origin=Origin(xyz=(x, y, 0.064)),
            material=dark_paint,
            name=f"anchor_washer_{i}",
        )
        frame.visual(
            Cylinder(radius=0.018, length=0.018),
            origin=Origin(xyz=(x, y, 0.073)),
            material=zinc,
            name=f"anchor_bolt_{i}",
        )

    for z, radius, label in ((0.303, 0.122, "lower"), (1.123, 0.126, "upper")):
        for i in range(8):
            a = i * math.tau / 8.0
            frame.visual(
                Cylinder(radius=0.010, length=0.014),
                origin=Origin(xyz=(math.cos(a) * radius, math.sin(a) * radius, z)),
                material=zinc,
                name=f"{label}_bearing_cap_screw_{i}",
            )

    # Replace the idealized clearance gap with bronze bearing shoes that touch
    # the rotor's inner race at four serviceable cardinal pads.
    for z, label in ((0.24, "lower"), (1.05, "upper")):
        for sign, cue in ((-1.0, "negative"), (1.0, "positive")):
            frame.visual(
                Box((0.040, 0.075, 0.105)),
                origin=Origin(xyz=(sign * 0.100, 0.0, z)),
                material=zinc,
                name=f"{label}_{cue}_x_bearing_shoe",
            )
            frame.visual(
                Box((0.075, 0.040, 0.105)),
                origin=Origin(xyz=(0.0, sign * 0.100, z)),
                material=zinc,
                name=f"{label}_{cue}_y_bearing_shoe",
            )

    # Low welded gusset blocks around the lower bearing housing.
    for i in range(4):
        a = i * math.pi / 2.0
        frame.visual(
            Box((0.16, 0.055, 0.075)),
            origin=Origin(
                xyz=(math.cos(a) * 0.170, math.sin(a) * 0.170, 0.105),
                rpy=(0.0, 0.0, a),
            ),
            material=frame_paint,
            name=f"lower_gusset_{i}",
        )

    rotor = model.part("rotor")
    # Main spindle and thick welded hub stack.
    rotor.visual(
        Cylinder(radius=0.046, length=1.04),
        origin=Origin(xyz=(0.0, 0.0, 0.65)),
        material=dark_paint,
        name="central_spindle",
    )
    for z, label in ((0.24, "lower"), (1.05, "upper")):
        rotor.visual(
            Cylinder(radius=0.080, length=0.105),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=dark_paint,
            name=f"{label}_bearing_inner_race",
        )
    rotor.visual(
        Cylinder(radius=0.132, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.73)),
        material=safety_yellow,
        name="arm_hub_drum",
    )
    for z, label in ((0.625, "lower"), (0.835, "upper")):
        rotor.visual(
            Cylinder(radius=0.165, length=0.035),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=safety_yellow,
            name=f"{label}_hub_flange",
        )
    rotor.visual(
        Cylinder(radius=0.060, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 1.185)),
        material=zinc,
        name="top_retainer_cap",
    )
    rotor.visual(
        Cylinder(radius=0.058, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=zinc,
        name="bottom_retainer_cap",
    )

    # Three rugged radial gate arms with clamp pads, rubber strike sleeves, and
    # end knobs. Each arm penetrates the hub locally as a welded/captured tube.
    arm_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for i, a in enumerate(arm_angles):
        c = math.cos(a)
        s = math.sin(a)
        _add_horizontal_cylinder(
            rotor,
            name=f"arm_{i}_tube",
            length=0.69,
            radius=0.035,
            center=(c * 0.405, s * 0.405, 0.735),
            yaw=a,
            material=safety_yellow,
        )
        _add_horizontal_cylinder(
            rotor,
            name=f"arm_{i}_rubber_sleeve",
            length=0.100,
            radius=0.041,
            center=(c * 0.665, s * 0.665, 0.735),
            yaw=a,
            material=rubber,
        )
        rotor.visual(
            Sphere(radius=0.050),
            origin=Origin(xyz=(c * 0.760, s * 0.760, 0.735)),
            material=rubber,
            name=f"arm_{i}_end_knob",
        )
        rotor.visual(
            Box((0.31, 0.105, 0.035)),
            origin=Origin(xyz=(c * 0.165, s * 0.165, 0.665), rpy=(0.0, 0.0, a)),
            material=safety_yellow,
            name=f"arm_{i}_lower_clamp_pad",
        )
        rotor.visual(
            Box((0.31, 0.105, 0.035)),
            origin=Origin(xyz=(c * 0.165, s * 0.165, 0.805), rpy=(0.0, 0.0, a)),
            material=safety_yellow,
            name=f"arm_{i}_upper_clamp_pad",
        )
        for j, dz in enumerate((-0.048, 0.048)):
            rotor.visual(
                Cylinder(radius=0.013, length=0.016),
                origin=Origin(
                    xyz=(c * 0.170, s * 0.170, 0.735 + dz),
                    rpy=(0.0, math.pi / 2.0, a),
                ),
                material=zinc,
                name=f"arm_{i}_hub_bolt_{j}",
            )

    rotor_joint = model.articulation(
        "frame_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.2),
        motion_properties=MotionProperties(damping=0.35, friction=0.18),
    )
    rotor_joint.meta["description"] = "Vertical bearing-supported spindle rotation for the radial turnstile arms."

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    rotor_joint = object_model.get_articulation("frame_to_rotor")

    ctx.expect_origin_distance(
        frame,
        rotor,
        axes="xy",
        max_dist=0.001,
        name="rotor axis remains concentric with fixed frame bearings",
    )
    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        inner_elem="central_spindle",
        outer_elem="upper_bearing_collar",
        margin=0.0,
        name="spindle is captured inside upper bearing collar footprint",
    )
    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        inner_elem="central_spindle",
        outer_elem="lower_bearing_collar",
        margin=0.0,
        name="spindle is captured inside lower bearing collar footprint",
    )
    ctx.expect_overlap(
        rotor,
        frame,
        axes="z",
        elem_a="central_spindle",
        elem_b="upper_bearing_collar",
        min_overlap=0.10,
        name="upper collar surrounds a real length of spindle",
    )
    ctx.expect_overlap(
        rotor,
        frame,
        axes="z",
        elem_a="central_spindle",
        elem_b="lower_bearing_collar",
        min_overlap=0.10,
        name="lower collar surrounds a real length of spindle",
    )

    arm_before = ctx.part_element_world_aabb(rotor, elem="arm_0_tube")
    with ctx.pose({rotor_joint: math.pi / 2.0}):
        arm_after = ctx.part_element_world_aabb(rotor, elem="arm_0_tube")
    ctx.check(
        "radial arm rotates about vertical spindle",
        arm_before is not None
        and arm_after is not None
        and arm_before[1][0] > 0.70
        and arm_after[1][1] > 0.70,
        details=f"before={arm_before}, after={arm_after}",
    )

    return ctx.report()


object_model = build_object_model()
