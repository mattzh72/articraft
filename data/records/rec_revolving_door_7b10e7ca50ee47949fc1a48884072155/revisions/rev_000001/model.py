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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_box_visual(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    *,
    material,
    name: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_chord_panel(
    part,
    *,
    radius: float,
    start_deg: float,
    end_deg: float,
    thickness: float,
    height: float,
    z_center: float,
    material,
    name: str,
) -> None:
    start = math.radians(start_deg)
    end = math.radians(end_deg)
    mid = 0.5 * (start + end)
    span = abs(end - start)
    chord_length = 2.0 * radius * math.sin(span * 0.5)
    center_radius = radius * math.cos(span * 0.5)
    center = (
        center_radius * math.cos(mid),
        center_radius * math.sin(mid),
        z_center,
    )
    _add_box_visual(
        part,
        (chord_length, thickness, height),
        center,
        rpy=(0.0, 0.0, mid + math.pi * 0.5),
        material=material,
        name=name,
    )


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="security_airlock_revolving_door")

    aluminum = model.material("aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    threshold = model.material("threshold", rgba=(0.36, 0.37, 0.39, 1.0))
    floor_stone = model.material("floor_stone", rgba=(0.64, 0.65, 0.67, 1.0))
    glass = model.material("glass", rgba=(0.70, 0.86, 0.94, 0.24))
    gate_glass = model.material("gate_glass", rgba=(0.75, 0.90, 0.98, 0.28))

    vestibule = model.part("vestibule")
    vestibule.inertial = Inertial.from_geometry(
        Box((3.40, 2.70, 2.62)),
        mass=950.0,
        origin=Origin(xyz=(0.0, 0.0, 1.31)),
    )

    floor_top = 0.06
    canopy_bottom = 2.48
    canopy_center = 2.55
    wall_height = canopy_bottom - floor_top
    wall_center_z = floor_top + 0.5 * wall_height
    front_y = 1.18
    rear_y = -1.18
    side_x = 1.69

    _add_box_visual(
        vestibule,
        (3.40, 2.70, 0.06),
        (0.0, 0.0, 0.03),
        material=floor_stone,
        name="floor_slab",
    )
    _add_box_visual(
        vestibule,
        (3.40, 2.70, 0.14),
        (0.0, 0.0, canopy_center),
        material=dark_metal,
        name="ceiling_canopy",
    )

    for name, xyz in (
        ("front_left_corner_post", (-1.64, 1.13, wall_center_z)),
        ("front_right_corner_post", (1.64, 1.13, wall_center_z)),
        ("rear_left_corner_post", (-1.64, -1.13, wall_center_z)),
        ("rear_right_corner_post", (1.64, -1.13, wall_center_z)),
    ):
        _add_box_visual(
            vestibule,
            (0.10, 0.10, wall_height),
            xyz,
            material=aluminum,
            name=name,
        )

    for name, x in (("left_side_glass", -side_x), ("right_side_glass", side_x)):
        _add_box_visual(
            vestibule,
            (0.022, 2.42, wall_height),
            (x, -0.03, wall_center_z),
            material=glass,
            name=name,
        )

    _add_box_visual(
        vestibule,
        (1.00, 0.022, wall_height),
        (-1.11, rear_y, wall_center_z),
        material=glass,
        name="rear_left_glass",
    )
    _add_box_visual(
        vestibule,
        (0.92, 0.022, wall_height),
        (-1.15, front_y, wall_center_z),
        material=glass,
        name="front_left_glass",
    )
    _add_box_visual(
        vestibule,
        (0.92, 0.022, wall_height),
        (1.15, front_y, wall_center_z),
        material=glass,
        name="front_right_glass",
    )

    _add_box_visual(
        vestibule,
        (0.08, 0.10, 2.12),
        (-0.64, rear_y, 1.12),
        material=aluminum,
        name="main_exit_left_jamb",
    )
    vestibule.visual(
        Box((0.18, 0.10, 2.12)),
        origin=Origin(xyz=(0.69, rear_y, 1.12)),
        material=aluminum,
        name="bypass_center_mullion",
    )
    _add_box_visual(
        vestibule,
        (2.14, 0.08, 0.30),
        (0.45, rear_y, 2.33),
        material=aluminum,
        name="rear_header",
    )
    vestibule.visual(
        Box((0.05, 0.12, 1.08)),
        origin=Origin(xyz=(1.545, rear_y, 0.60)),
        material=dark_metal,
        name="bypass_hinge_post",
    )
    _add_box_visual(
        vestibule,
        (0.05, 0.08, 1.04),
        (1.545, rear_y, 1.66),
        material=aluminum,
        name="bypass_hinge_jamb",
    )
    _add_box_visual(
        vestibule,
        (0.11, 0.022, wall_height),
        (1.635, rear_y, wall_center_z),
        material=glass,
        name="rear_right_sidelight",
    )

    _add_box_visual(
        vestibule,
        (0.10, 0.18, 0.10),
        (-0.24, 0.0, 2.43),
        material=dark_metal,
        name="drum_drive_housing_left",
    )
    _add_box_visual(
        vestibule,
        (0.10, 0.18, 0.10),
        (0.24, 0.0, 2.43),
        material=dark_metal,
        name="drum_drive_housing_right",
    )
    center_floor_bearing = LatheGeometry.from_shell_profiles(
        [(0.09, 0.06), (0.09, 0.08)],
        [(0.065, 0.06), (0.065, 0.08)],
        segments=36,
        start_cap="flat",
        end_cap="flat",
    )
    vestibule.visual(
        mesh_from_geometry(center_floor_bearing, "center_floor_bearing_mesh"),
        material=threshold,
        name="center_floor_bearing",
    )

    drum_radius = 0.97
    drum_glass_height = 2.28
    drum_glass_center_z = 1.24
    for idx, (a0, a1) in enumerate(
        (
            (-60.0, -30.0),
            (-30.0, 0.0),
            (0.0, 30.0),
            (30.0, 60.0),
            (120.0, 150.0),
            (150.0, 180.0),
            (180.0, 210.0),
            (210.0, 240.0),
        )
    ):
        if idx == 2:
            start = math.radians(a0)
            end = math.radians(a1)
            mid = 0.5 * (start + end)
            span = abs(end - start)
            chord_length = 2.0 * drum_radius * math.sin(span * 0.5)
            center_radius = drum_radius * math.cos(span * 0.5)
            vestibule.visual(
                Box((chord_length, 0.018, drum_glass_height)),
                origin=Origin(
                    xyz=(
                        center_radius * math.cos(mid),
                        center_radius * math.sin(mid),
                        drum_glass_center_z,
                    ),
                    rpy=(0.0, 0.0, mid + math.pi * 0.5),
                ),
                material=glass,
                name="drum_glass_panel_2",
            )
        else:
            _add_chord_panel(
                vestibule,
                radius=drum_radius,
                start_deg=a0,
                end_deg=a1,
                thickness=0.018,
                height=drum_glass_height,
                z_center=drum_glass_center_z,
                material=glass,
                name=f"drum_glass_panel_{idx}",
            )
        _add_chord_panel(
            vestibule,
            radius=drum_radius,
            start_deg=a0,
            end_deg=a1,
            thickness=0.10,
            height=0.08,
            z_center=0.10,
            material=threshold,
            name=f"drum_threshold_segment_{idx}",
        )
        _add_chord_panel(
            vestibule,
            radius=drum_radius,
            start_deg=a0,
            end_deg=a1,
            thickness=0.10,
            height=0.12,
            z_center=2.42,
            material=dark_metal,
            name=f"drum_header_segment_{idx}",
        )

    rotor = model.part("rotor")
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.88, length=2.42),
        mass=120.0,
        origin=Origin(xyz=(0.0, 0.0, 1.27)),
    )
    rotor.visual(
        Cylinder(radius=0.06, length=2.42),
        origin=Origin(xyz=(0.0, 0.0, 1.27)),
        material=dark_metal,
        name="central_shaft",
    )
    rotor.visual(
        Cylinder(radius=0.12, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=threshold,
        name="lower_hub",
    )
    rotor.visual(
        Cylinder(radius=0.18, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 2.40)),
        material=dark_metal,
        name="upper_hub",
    )

    wing_length = 0.84
    glass_span = 0.73
    for idx, angle_deg in enumerate((90.0, -30.0, -150.0)):
        angle = math.radians(angle_deg)
        c = math.cos(angle)
        s = math.sin(angle)
        glass_center_r = 0.5 * glass_span + 0.075
        rail_center_r = 0.5 * wing_length
        rotor.visual(
            Box((glass_span, 0.024, 2.02)),
            origin=Origin(
                xyz=(glass_center_r * c, glass_center_r * s, 1.18),
                rpy=(0.0, 0.0, angle),
            ),
            material=gate_glass,
            name=f"wing_{idx}_glass",
        )
        rotor.visual(
            Box((0.08, 0.08, 2.08)),
            origin=Origin(xyz=(0.10 * c, 0.10 * s, 1.18), rpy=(0.0, 0.0, angle)),
            material=aluminum,
            name=f"wing_{idx}_inner_stile",
        )
        if idx == 0:
            rotor.visual(
                Box((0.07, 0.08, 2.08)),
                origin=Origin(xyz=(wing_length * c, wing_length * s, 1.18), rpy=(0.0, 0.0, angle)),
                material=aluminum,
                name="wing_0_outer_stile",
            )
        else:
            rotor.visual(
                Box((0.07, 0.08, 2.08)),
                origin=Origin(xyz=(wing_length * c, wing_length * s, 1.18), rpy=(0.0, 0.0, angle)),
                material=aluminum,
                name=f"wing_{idx}_outer_stile",
            )
        rotor.visual(
            Box((wing_length + 0.04, 0.08, 0.06)),
            origin=Origin(xyz=(rail_center_r * c, rail_center_r * s, 0.16), rpy=(0.0, 0.0, angle)),
            material=threshold,
            name=f"wing_{idx}_bottom_rail",
        )
        rotor.visual(
            Box((wing_length + 0.04, 0.08, 0.06)),
            origin=Origin(xyz=(rail_center_r * c, rail_center_r * s, 2.20), rpy=(0.0, 0.0, angle)),
            material=dark_metal,
            name=f"wing_{idx}_top_rail",
        )

    gate = model.part("bypass_gate")
    gate.inertial = Inertial.from_geometry(
        Box((0.74, 0.08, 0.92)),
        mass=18.0,
        origin=Origin(xyz=(-0.37, 0.0, 0.56)),
    )
    gate.visual(
        Box((0.04, 0.06, 0.92)),
        origin=Origin(xyz=(-0.02, 0.0, 0.56)),
        material=dark_metal,
        name="gate_hinge_rail",
    )
    _add_box_visual(
        gate,
        (0.04, 0.06, 0.12),
        (-0.02, 0.0, 0.18),
        material=dark_metal,
        name="gate_hinge_knuckle_lower",
    )
    _add_box_visual(
        gate,
        (0.04, 0.06, 0.12),
        (-0.02, 0.0, 0.78),
        material=dark_metal,
        name="gate_hinge_knuckle_upper",
    )
    _add_box_visual(
        gate,
        (0.74, 0.05, 0.04),
        (-0.37, 0.0, 0.14),
        material=threshold,
        name="gate_bottom_rail",
    )
    _add_box_visual(
        gate,
        (0.74, 0.05, 0.04),
        (-0.37, 0.0, 0.94),
        material=dark_metal,
        name="gate_top_rail",
    )
    _add_box_visual(
        gate,
        (0.04, 0.05, 0.92),
        (-0.72, 0.0, 0.56),
        material=dark_metal,
        name="gate_latch_rail",
    )
    gate.visual(
        Box((0.66, 0.016, 0.76)),
        origin=Origin(xyz=(-0.37, 0.0, 0.54)),
        material=gate_glass,
        name="gate_leaf_glass",
    )

    model.articulation(
        "drum_rotation",
        ArticulationType.CONTINUOUS,
        parent=vestibule,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.9),
    )
    model.articulation(
        "bypass_gate_hinge",
        ArticulationType.REVOLUTE,
        parent=vestibule,
        child=gate,
        origin=Origin(xyz=(1.52, rear_y, floor_top)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.4,
            lower=0.0,
            upper=1.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    vestibule = object_model.get_part("vestibule")
    rotor = object_model.get_part("rotor")
    gate = object_model.get_part("bypass_gate")
    drum_joint = object_model.get_articulation("drum_rotation")
    gate_joint = object_model.get_articulation("bypass_gate_hinge")

    ctx.expect_contact(
        gate,
        vestibule,
        elem_a="gate_hinge_rail",
        elem_b="bypass_hinge_post",
        name="bypass gate is mounted against hinge post",
    )

    closed_latch = _aabb_center(ctx.part_element_world_aabb(gate, elem="gate_latch_rail"))
    with ctx.pose({gate_joint: gate_joint.motion_limits.upper}):
        ctx.expect_gap(
            gate,
            vestibule,
            axis="x",
            positive_elem="gate_leaf_glass",
            negative_elem="bypass_center_mullion",
            min_gap=0.24,
            name="open bypass gate clears the center mullion",
        )
        open_latch = _aabb_center(ctx.part_element_world_aabb(gate, elem="gate_latch_rail"))

    ctx.check(
        "positive gate motion swings leaf into the bypass lane",
        closed_latch is not None
        and open_latch is not None
        and open_latch[1] > closed_latch[1] + 0.40,
        details=f"closed_latch={closed_latch}, open_latch={open_latch}",
    )

    closed_wing = _aabb_center(ctx.part_element_world_aabb(rotor, elem="wing_0_outer_stile"))
    with ctx.pose({drum_joint: 1.0}):
        rotated_wing = _aabb_center(ctx.part_element_world_aabb(rotor, elem="wing_0_outer_stile"))

    same_radius = False
    positive_rotation = False
    if closed_wing is not None and rotated_wing is not None:
        closed_radius = math.hypot(closed_wing[0], closed_wing[1])
        rotated_radius = math.hypot(rotated_wing[0], rotated_wing[1])
        same_radius = abs(closed_radius - rotated_radius) <= 0.03
        closed_angle = math.atan2(closed_wing[1], closed_wing[0])
        rotated_angle = math.atan2(rotated_wing[1], rotated_wing[0])
        delta = rotated_angle - closed_angle
        while delta <= -math.pi:
            delta += 2.0 * math.pi
        while delta > math.pi:
            delta -= 2.0 * math.pi
        positive_rotation = 0.8 <= delta <= 1.2
    ctx.check(
        "drum rotates continuously about the central shaft",
        same_radius and positive_rotation,
        details=f"closed_wing={closed_wing}, rotated_wing={rotated_wing}",
    )

    ctx.check(
        "drum wing stays within the cylindrical drum footprint",
        closed_wing is not None and math.hypot(closed_wing[0], closed_wing[1]) <= 0.90,
        details=f"closed_wing={closed_wing}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
