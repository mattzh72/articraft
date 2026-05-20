from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _cylinder_between(part, start, end, radius, *, material, name, segments: int = 24) -> None:
    """Add a round steel member whose local cylinder axis runs from start to end."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-9:
        return
    ux, uy, uz = dx / length, dy / length, dz / length
    yaw = math.atan2(uy, ux)
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _add_torus(part, radius, tube, y, *, material, name) -> None:
    geom = TorusGeometry(radius, tube, radial_segments=28, tubular_segments=128)
    geom.rotate_x(math.pi / 2.0).translate(0.0, y, 0.0)
    part.visual(mesh_from_geometry(geom, name), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="landmark_observation_wheel")

    concrete = model.material("warm_concrete", rgba=(0.56, 0.54, 0.50, 1.0))
    steel = model.material("painted_white_steel", rgba=(0.88, 0.90, 0.88, 1.0))
    dark_steel = model.material("dark_galvanized_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    bearing_blue = model.material("bearing_blue", rgba=(0.06, 0.18, 0.34, 1.0))
    glass = model.material("blue_green_glass", rgba=(0.24, 0.62, 0.80, 0.62))
    cabin_paint = model.material("cabin_ivory", rgba=(0.92, 0.86, 0.72, 1.0))
    cabin_trim = model.material("cabin_red_trim", rgba=(0.65, 0.08, 0.07, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    deck = model.material("decking", rgba=(0.37, 0.31, 0.24, 1.0))
    lamp = model.material("warm_lamp_glass", rgba=(1.0, 0.78, 0.35, 1.0))

    foundation = model.part("foundation")
    foundation.visual(
        Box((86.0, 36.0, 0.8)),
        origin=Origin(xyz=(0.0, 0.0, 0.4)),
        material=concrete,
        name="mat_slab",
    )
    foundation.visual(
        Box((30.0, 14.0, 0.72)),
        origin=Origin(xyz=(0.0, 0.0, 1.16)),
        material=deck,
        name="boarding_deck",
    )
    foundation.visual(
        Box((8.0, 5.0, 3.2)),
        origin=Origin(xyz=(-13.0, -3.5, 3.1)),
        material=cabin_paint,
        name="operator_booth",
    )
    foundation.visual(
        Box((8.4, 5.4, 0.35)),
        origin=Origin(xyz=(-13.0, -3.5, 4.875)),
        material=cabin_trim,
        name="booth_roof",
    )
    for y in (-6.8, 6.8):
        foundation.visual(
            Box((30.0, 0.18, 1.2)),
            origin=Origin(xyz=(0.0, y, 2.25)),
            material=dark_steel,
            name=f"platform_rail_{'rear' if y < 0 else 'front'}",
        )
        for x in (-14.0, -8.0, -2.0, 4.0, 10.0, 14.0):
            foundation.visual(
                Cylinder(radius=0.09, length=1.6),
                origin=Origin(xyz=(x, y, 2.25)),
                material=dark_steel,
                name=f"rail_post_{x}_{y}",
            )
    for x in (-13.5, -4.5, 4.5, 13.5):
        for y in (-6.2, 6.2):
            _cylinder_between(
                foundation,
                (x, y, 0.8),
                (x, y, 1.52),
                0.18,
                material=dark_steel,
                name=f"deck_pier_{x}_{y}",
            )

    # Four splayed A-frame legs, deep side bracing, and visible bearing housings.
    for y in (-8.4, 8.4):
        _cylinder_between(
            foundation,
            (-34.0, y, 0.8),
            (0.0, y, 70.0),
            1.25,
            material=steel,
            name=f"main_leg_neg_{y}",
        )
        _cylinder_between(
            foundation,
            (34.0, y, 0.8),
            (0.0, y, 70.0),
            1.25,
            material=steel,
            name=f"main_leg_pos_{y}",
        )
        _cylinder_between(
            foundation,
            (-26.0, y, 16.0),
            (21.0, y, 58.0),
            0.36,
            material=dark_steel,
            name=f"x_brace_a_{y}",
        )
        _cylinder_between(
            foundation,
            (26.0, y, 16.0),
            (-21.0, y, 58.0),
            0.36,
            material=dark_steel,
            name=f"x_brace_b_{y}",
        )
        _cylinder_between(
            foundation,
            (-34.0, y, 0.8),
            (34.0, y, 0.8),
            0.32,
            material=dark_steel,
            name=f"base_tie_{y}",
        )
        foundation.visual(
            Cylinder(radius=2.9, length=1.8),
            origin=Origin(xyz=(0.0, y, 70.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bearing_blue,
            name=f"bearing_housing_{y}",
        )
    _cylinder_between(
        foundation,
        (0.0, -8.4, 70.0),
        (0.0, 8.4, 70.0),
        0.72,
        material=dark_steel,
        name="static_axle",
    )
    for anchor_x in (-38.0, 38.0):
        for anchor_y in (-12.0, 12.0):
            _cylinder_between(
                foundation,
                (anchor_x, anchor_y, 1.0),
                (0.0, math.copysign(8.4, anchor_y), 70.0),
                0.18,
                material=dark_steel,
                name=f"stay_cable_{anchor_x}_{anchor_y}",
            )

    wheel = model.part("wheel")
    for y in (-3.4, 3.4):
        _add_torus(wheel, 60.0, 0.62, y, material=steel, name=f"outer_rim_{y}")
        _add_torus(wheel, 54.0, 0.34, y, material=steel, name=f"inner_rim_{y}")

    wheel.visual(
        Cylinder(radius=2.45, length=8.0),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_blue,
        name="rotating_hub",
    )
    wheel.visual(
        Cylinder(radius=1.35, length=10.2),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hub_shaft",
    )

    spoke_count = 24
    for i in range(spoke_count):
        a = 2.0 * math.pi * i / spoke_count
        x_outer = 60.0 * math.cos(a)
        z_outer = 60.0 * math.sin(a)
        x_inner = 54.0 * math.cos(a + math.pi / spoke_count)
        z_inner = 54.0 * math.sin(a + math.pi / spoke_count)
        for y in (-3.4, 3.4):
            x_spoke_root = 2.25 * math.cos(a)
            z_spoke_root = 2.25 * math.sin(a)
            _cylinder_between(
                wheel,
                (x_spoke_root, y, z_spoke_root),
                (x_outer, y, z_outer),
                0.20 if i % 2 else 0.26,
                material=steel,
                name=f"spoke_{i}_{y}",
            )
            _cylinder_between(
                wheel,
                (x_inner, y, z_inner),
                (x_outer, y, z_outer),
                0.16,
                material=dark_steel,
                name=f"rim_truss_{i}_{y}",
            )
        if i % 2 == 0:
            _cylinder_between(
                wheel,
                (x_outer, -3.6, z_outer),
                (x_outer, 3.6, z_outer),
                0.20,
                material=steel,
                name=f"rim_cross_tie_{i}",
            )
        if i % 3 == 0:
            lamp_y = -3.4 if (i // 3) % 2 == 0 else 3.4
            wheel.visual(
                Sphere(radius=0.58),
                origin=Origin(xyz=(60.9 * math.cos(a), lamp_y, 60.9 * math.sin(a))),
                material=lamp,
                name=f"rim_lamp_{i}",
            )

    gondola_count = 12
    gondola_radius = 53.0
    for i in range(gondola_count):
        a = -math.pi / 2.0 + 2.0 * math.pi * i / gondola_count
        px = gondola_radius * math.cos(a)
        pz = gondola_radius * math.sin(a)
        _cylinder_between(
            wheel,
            (px, -3.25, pz),
            (px, 3.25, pz),
            0.10,
            material=dark_steel,
            name=f"hanger_pin_{i}",
        )
        wheel.visual(
            Sphere(radius=0.48),
            origin=Origin(xyz=(px, -3.45, pz)),
            material=dark_steel,
            name=f"pin_end_{i}_0",
        )
        wheel.visual(
            Sphere(radius=0.48),
            origin=Origin(xyz=(px, 3.45, pz)),
            material=dark_steel,
            name=f"pin_end_{i}_1",
        )

    wheel_joint = model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=foundation,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 70.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=250000.0, velocity=0.08),
        motion_properties=MotionProperties(damping=6.0, friction=1.0),
    )

    for i in range(gondola_count):
        gondola = model.part(f"gondola_{i}")
        # The gondola part frame is the hanging pivot.  The cabin is built below it.
        gondola.visual(
            Cylinder(radius=0.24, length=5.4),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="pivot_pin",
        )
        for y in (-2.0, 2.0):
            _cylinder_between(
                gondola,
                (0.0, y, -0.24),
                (-1.9, y, -3.0),
                0.12,
                material=dark_steel,
                name=f"hanger_arm_a_{y}",
            )
            _cylinder_between(
                gondola,
                (0.0, y, -0.24),
                (1.9, y, -3.0),
                0.12,
                material=dark_steel,
                name=f"hanger_arm_b_{y}",
            )
        gondola.visual(
            Box((5.1, 4.5, 2.85)),
            origin=Origin(xyz=(0.0, 0.0, -4.35)),
            material=cabin_paint,
            name="body",
        )
        gondola.visual(
            Box((5.4, 4.8, 0.42)),
            origin=Origin(xyz=(0.0, 0.0, -2.74)),
            material=cabin_trim,
            name="roof_band",
        )
        gondola.visual(
            Box((5.3, 4.7, 0.35)),
            origin=Origin(xyz=(0.0, 0.0, -5.95)),
            material=cabin_trim,
            name="floor_band",
        )
        gondola.visual(
            Box((4.5, 0.08, 1.35)),
            origin=Origin(xyz=(0.0, -2.22, -4.2)),
            material=glass,
            name="rear_windows",
        )
        gondola.visual(
            Box((4.5, 0.08, 1.35)),
            origin=Origin(xyz=(0.0, 2.22, -4.2)),
            material=glass,
            name="front_windows",
        )
        gondola.visual(
            Box((0.08, 3.5, 1.25)),
            origin=Origin(xyz=(-2.52, 0.0, -4.2)),
            material=glass,
            name="side_window_0",
        )
        gondola.visual(
            Box((0.08, 3.5, 1.25)),
            origin=Origin(xyz=(2.52, 0.0, -4.2)),
            material=glass,
            name="side_window_1",
        )
        gondola.visual(
            Box((3.8, 0.55, 0.38)),
            origin=Origin(xyz=(0.0, 0.0, -5.2)),
            material=deck,
            name="bench",
        )
        gondola.visual(
            Cylinder(radius=0.28, length=4.0),
            origin=Origin(xyz=(0.0, 0.0, -2.45), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name="roof_gutter",
        )

        a = -math.pi / 2.0 + 2.0 * math.pi * i / gondola_count
        px = gondola_radius * math.cos(a)
        pz = gondola_radius * math.sin(a)
        model.articulation(
            f"gondola_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=wheel,
            child=gondola,
            origin=Origin(xyz=(px, 0.0, pz)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3000.0, velocity=0.25, lower=-math.pi, upper=math.pi),
            motion_properties=MotionProperties(damping=2.0, friction=0.05),
            mimic=Mimic(joint=wheel_joint.name, multiplier=-1.0, offset=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    foundation = object_model.get_part("foundation")
    wheel = object_model.get_part("wheel")
    wheel_spin = object_model.get_articulation("wheel_spin")

    ctx.expect_origin_gap(
        wheel,
        foundation,
        axis="z",
        min_gap=69.5,
        max_gap=70.5,
        name="wheel hub is mounted high above the foundation",
    )

    wheel_aabb = ctx.part_world_aabb(wheel)
    ctx.check(
        "wheel has landmark scale",
        wheel_aabb is not None
        and (wheel_aabb[1][0] - wheel_aabb[0][0]) > 118.0
        and (wheel_aabb[1][2] - wheel_aabb[0][2]) > 118.0,
        details=f"wheel_aabb={wheel_aabb}",
    )

    ctx.allow_overlap(
        foundation,
        wheel,
        elem_a="static_axle",
        elem_b="hub_shaft",
        reason="The simplified fixed axle is intentionally coaxial inside the rotating hub shaft.",
    )
    ctx.expect_overlap(
        foundation,
        wheel,
        axes="y",
        elem_a="static_axle",
        elem_b="hub_shaft",
        min_overlap=9.5,
        name="fixed axle passes through hub shaft",
    )
    ctx.expect_overlap(
        foundation,
        wheel,
        axes="xz",
        elem_a="static_axle",
        elem_b="hub_shaft",
        min_overlap=1.3,
        name="fixed axle is coaxial with hub shaft",
    )
    ctx.allow_overlap(
        foundation,
        wheel,
        elem_a="static_axle",
        elem_b="rotating_hub",
        reason="The fixed axle also passes through the larger rotating bearing hub.",
    )
    ctx.expect_overlap(
        foundation,
        wheel,
        axes="y",
        elem_a="static_axle",
        elem_b="rotating_hub",
        min_overlap=7.5,
        name="fixed axle passes through rotating hub",
    )

    for i in range(12):
        gondola = object_model.get_part(f"gondola_{i}")
        ctx.allow_overlap(
            gondola,
            wheel,
            elem_a="pivot_pin",
            elem_b=f"hanger_pin_{i}",
            reason="The gondola pivot pin is intentionally captured through the wheel hanger barrel.",
        )
        ctx.expect_overlap(
            gondola,
            wheel,
            axes="y",
            elem_a="pivot_pin",
            elem_b=f"hanger_pin_{i}",
            min_overlap=4.8,
            name=f"gondola_{i} pivot spans hanger width",
        )
        ctx.expect_overlap(
            gondola,
            wheel,
            axes="xz",
            elem_a="pivot_pin",
            elem_b=f"hanger_pin_{i}",
            min_overlap=0.18,
            name=f"gondola_{i} pivot is coaxial with hanger",
        )

    def _body_hangs_below_pivot(gondola_name: str) -> bool:
        gondola = object_model.get_part(gondola_name)
        pivot = ctx.part_world_position(gondola)
        body_box = ctx.part_element_world_aabb(gondola, elem="body")
        return pivot is not None and body_box is not None and body_box[1][2] < pivot[2] - 1.0

    ctx.check(
        "bottom gondola hangs below its pivot at rest",
        _body_hangs_below_pivot("gondola_0"),
        details="The cabin body should remain suspended below the hinge pin.",
    )
    rest_pos = ctx.part_world_position(object_model.get_part("gondola_0"))
    with ctx.pose({wheel_spin: 0.65}):
        moved_pos = ctx.part_world_position(object_model.get_part("gondola_0"))
        ctx.check(
            "wheel rotation carries gondolas around the hub",
            rest_pos is not None
            and moved_pos is not None
            and abs(moved_pos[0] - rest_pos[0]) > 20.0,
            details=f"rest={rest_pos}, moved={moved_pos}",
        )
        ctx.check(
            "mimic pivot keeps rotated gondola upright",
            _body_hangs_below_pivot("gondola_0"),
            details="The passive gondola pivot should counter-rotate as the wheel turns.",
        )

    return ctx.report()


object_model = build_object_model()
