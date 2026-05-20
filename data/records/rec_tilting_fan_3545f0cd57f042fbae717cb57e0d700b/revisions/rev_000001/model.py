from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


FAN_RADIUS = 0.165
HEAD_CENTER_Z = 0.45
TILT_UPPER = math.radians(35.0)
TILT_LOWER = math.radians(-22.0)


def _guard_cage_geometry() -> MeshGeometry:
    """Build a connected, open radial grille in the fan-head local frame.

    The mesh is a thin polar lattice rather than separate wires.  All rings and
    spokes share vertices, which keeps the guard as one supported part while
    preserving visible openings for the fan.
    """

    cage = MeshGeometry()
    vertices: dict[tuple[int, int, int], int] = {}
    radial_edges = (0.0, 0.030, 0.042, 0.049, 0.080, 0.089, 0.121, 0.130, 0.158, 0.171)
    ring_bands = ((0.0, 0.030), (0.042, 0.049), (0.080, 0.089), (0.121, 0.130), (0.158, 0.171))
    segment_count = 72
    angles = [2.0 * math.pi * i / segment_count for i in range(segment_count + 1)]

    def vertex(x: float, radius: float, angle: float) -> int:
        key = (round(x * 1_000_000), round(radius * 1_000_000), round(angle * 1_000_000))
        if key not in vertices:
            vertices[key] = cage.add_vertex(x, radius * math.cos(angle), radius * math.sin(angle))
        return vertices[key]

    def quad(a: int, b: int, c: int, d: int) -> None:
        cage.add_face(a, b, c)
        cage.add_face(a, c, d)

    def add_cell(x0: float, x1: float, r0: float, r1: float, a0: float, a1: float) -> None:
        v000 = vertex(x0, r0, a0)
        v001 = vertex(x0, r0, a1)
        v011 = vertex(x0, r1, a1)
        v010 = vertex(x0, r1, a0)
        v100 = vertex(x1, r0, a0)
        v101 = vertex(x1, r0, a1)
        v111 = vertex(x1, r1, a1)
        v110 = vertex(x1, r1, a0)
        quad(v000, v001, v011, v010)
        quad(v100, v110, v111, v101)
        quad(v000, v100, v101, v001)
        quad(v010, v011, v111, v110)
        quad(v000, v010, v110, v100)
        quad(v001, v101, v111, v011)

    def add_guard_face(x_center: float, thickness: float, spoke_stride: int, phase: int = 0) -> None:
        x0 = x_center - thickness / 2.0
        x1 = x_center + thickness / 2.0
        for ai in range(segment_count):
            a0 = angles[ai]
            a1 = angles[ai + 1]
            is_spoke = (ai - phase) % spoke_stride == 0
            for r0, r1 in zip(radial_edges[:-1], radial_edges[1:]):
                midpoint = (r0 + r1) * 0.5
                is_ring = any(lo <= midpoint <= hi for lo, hi in ring_bands)
                if is_ring or is_spoke:
                    add_cell(x0, x1, r0, r1, a0, a1)

    front_x = -0.064
    rear_x = 0.064
    face_thickness = 0.004
    add_guard_face(front_x, face_thickness, spoke_stride=4)
    add_guard_face(rear_x, face_thickness, spoke_stride=6, phase=1)

    # A thin perimeter band joins the two guard faces into a cage with real depth.
    for ai in range(segment_count):
        add_cell(
            front_x + face_thickness / 2.0,
            rear_x - face_thickness / 2.0,
            0.158,
            0.171,
            angles[ai],
            angles[ai + 1],
        )

    return cage


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilting_desk_fan")

    base_plastic = model.material("satin_dark_plastic", rgba=(0.055, 0.060, 0.065, 1.0))
    guard_metal = model.material("brushed_guard_metal", rgba=(0.78, 0.80, 0.78, 1.0))
    rubber = model.material("rubber_feet", rgba=(0.015, 0.015, 0.016, 1.0))
    blade_plastic = model.material("translucent_blue_blades", rgba=(0.20, 0.45, 0.78, 0.86))
    motor_black = model.material("motor_black", rgba=(0.025, 0.027, 0.030, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.46, 0.30, 0.045)),
        origin=Origin(xyz=(0.015, 0.0, 0.0225)),
        material=base_plastic,
        name="weighted_foot",
    )
    base.visual(
        Cylinder(radius=0.070, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=base_plastic,
        name="pedestal_boss",
    )
    base.visual(
        Cylinder(radius=0.038, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.104)),
        material=base_plastic,
        name="neck_post",
    )
    base.visual(
        Box((0.110, 0.440, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.134)),
        material=base_plastic,
        name="tilt_yoke",
    )
    for suffix, y in (("0", 0.205), ("1", -0.205)):
        base.visual(
            Box((0.065, 0.030, 0.385)),
            origin=Origin(xyz=(0.0, y, 0.3005)),
            material=base_plastic,
            name=f"side_bracket_{suffix}",
        )
    for suffix, y in (("0", 0.193), ("1", -0.193)):
        base.visual(
            Cylinder(radius=0.038, length=0.006),
            origin=Origin(xyz=(0.0, y, HEAD_CENTER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=guard_metal,
            name=f"pivot_bushing_{suffix}",
        )
    # Four small rubber pads make the base read as a desk appliance sitting on a surface.
    for i, (x, y) in enumerate(((-0.16, -0.105), (-0.16, 0.105), (0.18, -0.105), (0.18, 0.105))):
        base.visual(
            Cylinder(radius=0.018, length=0.008),
            origin=Origin(xyz=(x, y, 0.004)),
            material=rubber,
            name=f"foot_pad_{i}",
        )

    head = model.part("head")
    head.visual(
        mesh_from_geometry(_guard_cage_geometry(), "guard_cage"),
        material=guard_metal,
        name="guard_cage",
    )
    head.visual(
        Cylinder(radius=0.057, length=0.072),
        origin=Origin(xyz=(0.086, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=motor_black,
        name="motor_shell",
    )
    head.visual(
        Cylinder(radius=0.034, length=0.022),
        origin=Origin(xyz=(0.041, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=motor_black,
        name="motor_nose",
    )
    head.visual(
        Cylinder(radius=0.006, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=guard_metal,
        name="rotor_axle",
    )
    head.visual(
        Cylinder(radius=0.026, length=0.008),
        origin=Origin(xyz=(-0.066, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=guard_metal,
        name="front_badge",
    )
    for suffix, y in (("0", 0.168), ("1", -0.168)):
        head.visual(
            Cylinder(radius=0.018, length=0.036),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=guard_metal,
            name=f"pivot_pin_{suffix}",
        )
    for suffix, y in (("0", 0.187), ("1", -0.187)):
        head.visual(
            Cylinder(radius=0.034, length=0.006),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=guard_metal,
            name=f"pivot_washer_{suffix}",
        )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.124,
                0.034,
                5,
                thickness=0.017,
                blade_pitch_deg=31.0,
                blade_sweep_deg=27.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=15.0, camber=0.14, tip_clearance=0.004),
                hub=FanRotorHub(style="spinner", rear_collar_height=0.010, rear_collar_radius=0.030),
            ),
            "rotor_blades",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_plastic,
        name="rotor_blades",
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, HEAD_CENTER_Z)),
        # The fan face points along local -X. Positive pitch raises that front edge.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=1.2, lower=TILT_LOWER, upper=TILT_UPPER),
    )

    model.articulation(
        "head_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=50.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("base_to_head")
    spin = object_model.get_articulation("head_to_rotor")

    ctx.allow_overlap(
        head,
        rotor,
        elem_a="rotor_axle",
        elem_b="rotor_blades",
        reason="The stationary axle is intentionally modeled passing through the rotor hub bearing.",
    )

    ctx.check(
        "paired_yoke_present",
        base is not None
        and base.get_visual("side_bracket_0") is not None
        and base.get_visual("side_bracket_1") is not None,
    )
    ctx.check("guarded_head_present", head is not None and head.get_visual("guard_cage") is not None)
    ctx.check("rotor_present", rotor is not None and rotor.get_visual("rotor_blades") is not None)
    ctx.check(
        "head_tilt_is_limited_revolute",
        tilt is not None
        and tilt.articulation_type == ArticulationType.REVOLUTE
        and tilt.motion_limits is not None
        and tilt.motion_limits.lower < 0.0
        and tilt.motion_limits.upper > 0.0,
    )
    ctx.check(
        "rotor_spin_is_continuous",
        spin is not None and spin.articulation_type == ArticulationType.CONTINUOUS,
    )

    if base is not None and head is not None and rotor is not None:
        ctx.expect_within(
            head,
            base,
            axes="y",
            inner_elem="guard_cage",
            outer_elem="tilt_yoke",
            margin=0.0,
            name="guard cage sits between the side brackets",
        )
        ctx.expect_within(
            rotor,
            head,
            axes="yz",
            inner_elem="rotor_blades",
            outer_elem="guard_cage",
            margin=0.004,
            name="rotor is contained inside the guard",
        )
        ctx.expect_contact(
            head,
            base,
            elem_a="pivot_washer_0",
            elem_b="pivot_bushing_0",
            contact_tol=0.0005,
            name="one side bracket bears on the head pivot washer",
        )
        ctx.expect_contact(
            head,
            base,
            elem_a="pivot_washer_1",
            elem_b="pivot_bushing_1",
            contact_tol=0.0005,
            name="opposite side bracket bears on the head pivot washer",
        )
        ctx.expect_gap(
            rotor,
            head,
            axis="x",
            positive_elem="rotor_blades",
            negative_elem="front_badge",
            min_gap=0.020,
            name="rotor clears the front grille badge",
        )
        ctx.expect_within(
            head,
            rotor,
            axes="yz",
            inner_elem="rotor_axle",
            outer_elem="rotor_blades",
            margin=0.0,
            name="axle runs through the rotor hub center",
        )
        ctx.expect_overlap(
            head,
            rotor,
            axes="x",
            elem_a="rotor_axle",
            elem_b="rotor_blades",
            min_overlap=0.015,
            name="rotor hub is retained on the axle",
        )

    if head is not None and tilt is not None:
        rest_aabb = ctx.part_element_world_aabb(head, elem="front_badge")
        with ctx.pose({tilt: TILT_UPPER}):
            raised_aabb = ctx.part_element_world_aabb(head, elem="front_badge")
        rest_center_z = None if rest_aabb is None else (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5
        raised_center_z = None if raised_aabb is None else (raised_aabb[0][2] + raised_aabb[1][2]) * 0.5
        ctx.check(
            "positive_tilt_raises_front",
            rest_center_z is not None and raised_center_z is not None and raised_center_z > rest_center_z + 0.030,
            details=f"rest_z={rest_center_z}, raised_z={raised_center_z}",
        )

    return ctx.report()


object_model = build_object_model()
