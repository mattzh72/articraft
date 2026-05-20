from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _rpy_for_cylinder_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[float, float, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    radial = math.hypot(dx, dy)
    return (0.0, math.atan2(radial, dz), math.atan2(dy, dx))


def _cylinder_between(part, name, start, end, radius, material) -> None:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=(
                (start[0] + end[0]) / 2.0,
                (start[1] + end[1]) / 2.0,
                (start[2] + end[2]) / 2.0,
            ),
            rpy=_rpy_for_cylinder_between(start, end),
        ),
        material=material,
        name=name,
    )


def _radial_point(radius: float, angle: float, z: float) -> tuple[float, float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle), z)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_turnstile_gate")

    galvanized = model.material("galvanized_steel", rgba=(0.50, 0.53, 0.52, 1.0))
    dark_steel = model.material("dark_burnished_steel", rgba=(0.17, 0.18, 0.18, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(1.0, 0.72, 0.03, 1.0))
    hazard_black = model.material("hazard_black", rgba=(0.02, 0.02, 0.018, 1.0))
    lockout_red = model.material("lockout_red", rgba=(0.86, 0.03, 0.02, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.013, 0.012, 1.0))
    bolt_finish = model.material("zinc_bolt_heads", rgba=(0.78, 0.78, 0.74, 1.0))

    frame = model.part("frame")

    # Heavy floor plate and fixed spindle support.
    frame.visual(
        Box((2.70, 2.70, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_steel,
        name="floor_plate",
    )
    frame.visual(
        Cylinder(radius=0.130, length=0.720),
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        material=galvanized,
        name="central_post",
    )
    frame.visual(
        Cylinder(radius=0.260, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=dark_steel,
        name="base_flange",
    )
    frame.visual(
        Cylinder(radius=0.205, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.735)),
        material=dark_steel,
        name="lower_bearing_plate",
    )

    lower_bearing = mesh_from_geometry(
        TorusGeometry(radius=0.165, tube=0.030, radial_segments=24, tubular_segments=40),
        "lower_bearing_ring",
    )
    frame.visual(
        lower_bearing,
        origin=Origin(xyz=(0.0, 0.0, 0.780)),
        material=galvanized,
        name="lower_bearing_ring",
    )

    upper_bearing = mesh_from_geometry(
        TorusGeometry(radius=0.165, tube=0.030, radial_segments=24, tubular_segments=40),
        "upper_bearing_ring",
    )
    frame.visual(
        upper_bearing,
        origin=Origin(xyz=(0.0, 0.0, 1.485)),
        material=galvanized,
        name="upper_bearing_ring",
    )

    # Perimeter guard cage: all posts overlap the floor plate and rails overlap posts.
    for sx in (-1.20, 1.20):
        for sy in (-1.20, 1.20):
            frame.visual(
                Box((0.085, 0.085, 1.560)),
                origin=Origin(xyz=(sx, sy, 0.815)),
                material=safety_yellow,
                name=f"guard_post_{sx:+.0f}_{sy:+.0f}".replace("+", "p").replace("-", "m"),
            )
            frame.visual(
                Box((0.210, 0.210, 0.025)),
                origin=Origin(xyz=(sx, sy, 0.073)),
                material=dark_steel,
                name=f"post_foot_{sx:+.0f}_{sy:+.0f}".replace("+", "p").replace("-", "m"),
            )

    for y in (-1.20, 1.20):
        for z, thick, mat, suffix in (
            (1.555, 0.070, safety_yellow, "top"),
            (0.880, 0.055, safety_yellow, "middle"),
            (0.225, 0.050, hazard_black, "kick"),
        ):
            frame.visual(
                Box((2.485, 0.075, thick)),
                origin=Origin(xyz=(0.0, y, z)),
                material=mat,
                name=f"{suffix}_rail_y_{'p' if y > 0 else 'm'}",
            )
    for x in (-1.20, 1.20):
        for z, thick, mat, suffix in (
            (1.555, 0.070, safety_yellow, "top"),
            (0.880, 0.055, safety_yellow, "middle"),
            (0.225, 0.050, hazard_black, "kick"),
        ):
            frame.visual(
                Box((0.075, 2.485, thick)),
                origin=Origin(xyz=(x, 0.0, z)),
                material=mat,
                name=f"{suffix}_rail_x_{'p' if x > 0 else 'm'}",
            )

    # Overhead bearing support load path: four diagonal struts tie the top
    # bearing ring back to the guard frame, keeping the spindle supported above.
    for i, angle in enumerate((math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)):
        _cylinder_between(
            frame,
            f"upper_bearing_strut_{i}",
            _radial_point(0.170, angle, 1.485),
            _radial_point(1.700, angle, 1.555),
            0.026,
            galvanized,
        )

    # Bronze bearing shoes are modeled as replaceable solid pads inside the
    # bearing rings.  They deliberately contact the rotating spindle shaft.
    for z, prefix in ((0.780, "lower"), (1.485, "upper")):
        for i, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
            frame.visual(
                Box((0.096, 0.034, 0.046)),
                origin=Origin(xyz=_radial_point(0.092, angle, z), rpy=(0.0, 0.0, angle)),
                material=bolt_finish,
                name=f"{prefix}_bearing_shoe_{i}",
            )

    # Stiff base gussets and bolted flange logic at the fixed central post.
    for i, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        frame.visual(
            Box((0.360, 0.045, 0.170)),
            origin=Origin(xyz=_radial_point(0.225, angle, 0.185), rpy=(0.0, 0.0, angle)),
            material=dark_steel,
            name=f"post_gusset_{i}",
        )
        frame.visual(
            Cylinder(radius=0.023, length=0.014),
            origin=Origin(xyz=_radial_point(0.285, angle, 0.067)),
            material=bolt_finish,
            name=f"flange_bolt_{i}",
        )

    # Guard-cage anchor bolts are slightly seated into each foot plate.
    bolt_index = 0
    for sx in (-1.20, 1.20):
        for sy in (-1.20, 1.20):
            for ox in (-0.060, 0.060):
                for oy in (-0.060, 0.060):
                    frame.visual(
                        Cylinder(radius=0.015, length=0.012),
                        origin=Origin(xyz=(sx + ox, sy + oy, 0.079)),
                        material=bolt_finish,
                        name=f"anchor_bolt_{bolt_index}",
                    )
                    bolt_index += 1

    # Service lockout bracket mounted from the right guard rail, above the arm
    # sweep, with a fork gap for the hinged red lockout pawl.
    frame.visual(
        Box((0.620, 0.080, 0.075)),
        origin=Origin(xyz=(0.885, -0.355, 1.345)),
        material=dark_steel,
        name="lockout_support",
    )
    frame.visual(
        Box((0.080, 0.080, 0.760)),
        origin=Origin(xyz=(1.205, -0.355, 1.220)),
        material=dark_steel,
        name="lockout_upright",
    )
    frame.visual(
        Box((0.045, 0.026, 0.150)),
        origin=Origin(xyz=(0.535, -0.395, 1.345)),
        material=dark_steel,
        name="lockout_cheek_0",
    )
    frame.visual(
        Box((0.045, 0.026, 0.150)),
        origin=Origin(xyz=(0.535, -0.315, 1.345)),
        material=dark_steel,
        name="lockout_cheek_1",
    )
    frame.visual(
        Box((0.040, 0.026, 0.075)),
        origin=Origin(xyz=(0.575, -0.395, 1.345)),
        material=dark_steel,
        name="lockout_lug_0",
    )
    frame.visual(
        Box((0.040, 0.026, 0.075)),
        origin=Origin(xyz=(0.575, -0.315, 1.345)),
        material=dark_steel,
        name="lockout_lug_1",
    )
    frame.visual(
        Cylinder(radius=0.025, length=0.110),
        origin=Origin(xyz=(0.535, -0.355, 1.345), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bolt_finish,
        name="lockout_pivot_pin",
    )

    # Rubber over-travel bumpers and their supported stop brackets.  The bumpers
    # sit above the arm sweep and catch a small dog on the rotating hub.
    for i, angle in enumerate((-2.09, 2.09)):
        p = _radial_point(0.455, angle, 1.330)
        frame.visual(
            Box((0.120, 0.070, 0.080)),
            origin=Origin(xyz=p, rpy=(0.0, 0.0, angle + math.pi / 2.0)),
            material=rubber,
            name=f"overtravel_bumper_{i}",
        )
        _cylinder_between(
            frame,
            f"stop_brace_{i}",
            _radial_point(0.170, angle, 1.485),
            _radial_point(0.455, angle, 1.330),
            0.024,
            dark_steel,
        )

    rotor = model.part("rotor")
    rotor_origin_z = 0.800
    rotor.visual(
        Cylinder(radius=0.048, length=0.760),
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        material=dark_steel,
        name="spindle_shaft",
    )
    rotor.visual(
        Cylinder(radius=0.160, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=galvanized,
        name="bearing_hub",
    )
    rotor.visual(
        Cylinder(radius=0.215, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=dark_steel,
        name="lower_hub_flange",
    )
    rotor.visual(
        Cylinder(radius=0.215, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.455)),
        material=dark_steel,
        name="upper_hub_flange",
    )
    rotor.visual(
        Cylinder(radius=0.260, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.540)),
        material=dark_steel,
        name="ratchet_index_plate",
    )
    rotor.visual(
        Box((0.190, 0.070, 0.060)),
        origin=Origin(xyz=(0.345, 0.0, 0.540)),
        material=dark_steel,
        name="stop_dog",
    )

    arm_z = 0.300
    for i, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        mid = _radial_point(0.615, angle, arm_z)
        rotor.visual(
            Cylinder(radius=0.041, length=0.900),
            origin=Origin(xyz=mid, rpy=(0.0, math.pi / 2.0, angle)),
            material=galvanized,
            name=f"radial_arm_{i}",
        )
        rotor.visual(
            Sphere(radius=0.055),
            origin=Origin(xyz=_radial_point(1.070, angle, arm_z)),
            material=safety_yellow,
            name=f"arm_end_cap_{i}",
        )
        rotor.visual(
            Box((0.180, 0.080, 0.070)),
            origin=Origin(xyz=_radial_point(0.245, angle, arm_z), rpy=(0.0, 0.0, angle)),
            material=dark_steel,
            name=f"arm_clamp_{i}",
        )
        for j, z0 in enumerate((0.155, 0.445)):
            _cylinder_between(
                rotor,
                f"arm_gusset_{i}_{j}",
                _radial_point(0.105, angle, z0),
                _radial_point(0.465, angle, arm_z),
                0.018,
                dark_steel,
            )
        rotor.visual(
            Cylinder(radius=0.016, length=0.020),
            origin=Origin(xyz=_radial_point(0.295, angle + 0.115, arm_z + 0.042)),
            material=bolt_finish,
            name=f"clamp_bolt_a_{i}",
        )
        rotor.visual(
            Cylinder(radius=0.016, length=0.020),
            origin=Origin(xyz=_radial_point(0.295, angle - 0.115, arm_z + 0.042)),
            material=bolt_finish,
            name=f"clamp_bolt_b_{i}",
        )
        # Three indexing teeth read as ratchet/lockout engagement features.
        rotor.visual(
            Box((0.100, 0.045, 0.050)),
            origin=Origin(xyz=_radial_point(0.292, angle + 0.34, 0.540), rpy=(0.0, 0.0, angle + 0.34)),
            material=dark_steel,
            name=f"ratchet_tooth_{i}",
        )

    lockout_pawl = model.part("lockout_pawl")
    lockout_pawl.visual(
        Cylinder(radius=0.032, length=0.054),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=lockout_red,
        name="pivot_boss",
    )
    lockout_pawl.visual(
        Box((0.305, 0.020, 0.040)),
        origin=Origin(xyz=(-0.1825, 0.0, 0.0)),
        material=lockout_red,
        name="pawl_bar",
    )
    lockout_pawl.visual(
        Box((0.095, 0.032, 0.070)),
        origin=Origin(xyz=(-0.360, 0.0, -0.010)),
        material=dark_steel,
        name="hardened_tooth",
    )
    lockout_pawl.visual(
        Box((0.055, 0.030, 0.180)),
        origin=Origin(xyz=(-0.050, 0.0, 0.080)),
        material=lockout_red,
        name="manual_handle",
    )

    model.articulation(
        "frame_to_rotor",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, rotor_origin_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=450.0, velocity=1.2, lower=-2.09, upper=2.09),
        motion_properties=MotionProperties(damping=4.0, friction=1.5),
    )
    model.articulation(
        "frame_to_lockout",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lockout_pawl,
        origin=Origin(xyz=(0.535, -0.355, 1.345)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.0, lower=0.0, upper=0.80),
        motion_properties=MotionProperties(damping=0.8, friction=0.4),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    lockout_pawl = object_model.get_part("lockout_pawl")
    rotor_joint = object_model.get_articulation("frame_to_rotor")
    lockout_joint = object_model.get_articulation("frame_to_lockout")

    ctx.allow_overlap(
        frame,
        lockout_pawl,
        elem_a="lockout_pivot_pin",
        elem_b="pivot_boss",
        reason=(
            "The red lockout pawl is captured on the fixed hinge pin; the "
            "solid pin/boss proxy intentionally represents the supported pivot."
        ),
    )
    ctx.expect_overlap(
        frame,
        lockout_pawl,
        axes="xyz",
        elem_a="lockout_pivot_pin",
        elem_b="pivot_boss",
        min_overlap=0.040,
        name="lockout pawl is captured on the hinge pin",
    )

    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        inner_elem="spindle_shaft",
        outer_elem="lower_bearing_ring",
        margin=0.0,
        name="spindle is centered inside lower bearing core",
    )
    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        inner_elem="spindle_shaft",
        outer_elem="upper_bearing_ring",
        margin=0.0,
        name="spindle is centered inside upper bearing core",
    )
    ctx.expect_overlap(
        frame,
        rotor,
        axes="z",
        elem_a="lower_bearing_shoe_0",
        elem_b="spindle_shaft",
        min_overlap=0.030,
        name="lower bearing shoe supports spindle height",
    )
    ctx.expect_overlap(
        frame,
        rotor,
        axes="z",
        elem_a="upper_bearing_shoe_0",
        elem_b="spindle_shaft",
        min_overlap=0.030,
        name="upper bearing shoe supports spindle height",
    )

    rest_pos = ctx.part_world_position(rotor)
    with ctx.pose({rotor_joint: 1.20}):
        turned_pos = ctx.part_world_position(rotor)
        ctx.expect_within(
            rotor,
            frame,
            axes="xy",
            inner_elem="spindle_shaft",
            outer_elem="upper_bearing_ring",
            margin=0.0,
            name="spindle stays centered while rotor turns",
        )
    ctx.check(
        "rotor turns about fixed vertical spindle axis",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) < 0.001
        and abs(rest_pos[1] - turned_pos[1]) < 0.001
        and abs(rest_pos[2] - turned_pos[2]) < 0.001,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    closed_tooth_aabb = ctx.part_element_world_aabb(lockout_pawl, elem="hardened_tooth")
    closed_tooth_z = (
        None
        if closed_tooth_aabb is None
        else (closed_tooth_aabb[0][2] + closed_tooth_aabb[1][2]) / 2.0
    )
    with ctx.pose({lockout_joint: 0.65}):
        raised_tooth_aabb = ctx.part_element_world_aabb(lockout_pawl, elem="hardened_tooth")
        raised_tooth_z = (
            None
            if raised_tooth_aabb is None
            else (raised_tooth_aabb[0][2] + raised_tooth_aabb[1][2]) / 2.0
        )
    ctx.check(
        "lockout pawl uses a bounded hinged release motion",
        closed_tooth_z is not None and raised_tooth_z is not None and raised_tooth_z > closed_tooth_z + 0.050,
        details=f"closed_tooth_z={closed_tooth_z}, raised_tooth_z={raised_tooth_z}",
    )

    return ctx.report()


object_model = build_object_model()
