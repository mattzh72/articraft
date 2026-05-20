from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
    wire_from_points,
)


PIVOT_HEIGHT = 0.58
GUARD_CENTER_X = 0.08
GUARD_RADIUS = 0.245
ROTOR_RADIUS = 0.205
ROTOR_THICKNESS = 0.026
TILT_UPPER = math.radians(35.0)
TILT_LOWER = math.radians(-20.0)


def _circle_points(radius: float, x: float, *, segments: int = 48) -> list[tuple[float, float, float]]:
    return [
        (x, radius * math.cos(i * 2.0 * math.pi / segments), radius * math.sin(i * 2.0 * math.pi / segments))
        for i in range(segments)
    ]


def _make_straight_tube(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    name: str,
):
    return mesh_from_geometry(
        wire_from_points(
            [start, end],
            radius=radius,
            radial_segments=14,
            cap_ends=True,
            closed_path=False,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilting_desk_fan")

    ivory = model.material("warm_ivory_plastic", rgba=(0.86, 0.82, 0.72, 1.0))
    dark = model.material("dark_motor_housing", rgba=(0.08, 0.09, 0.10, 1.0))
    guard_metal = model.material("brushed_guard_wire", rgba=(0.62, 0.64, 0.66, 1.0))
    blade_plastic = model.material("smoky_blue_blades", rgba=(0.18, 0.34, 0.55, 0.82))
    rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))

    base = model.part("base")
    base_plate = ExtrudeGeometry(rounded_rect_profile(0.40, 0.30, 0.075, corner_segments=10), 0.040)
    base.visual(
        mesh_from_geometry(base_plate, "base_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=ivory,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.035, length=0.39),
        origin=Origin(xyz=(-0.08, 0.0, 0.235)),
        material=ivory,
        name="pedestal_post",
    )
    base.visual(
        Cylinder(radius=0.052, length=0.026),
        origin=Origin(xyz=(-0.08, 0.0, 0.432)),
        material=ivory,
        name="fork_collar",
    )
    for idx, y in enumerate((0.315, -0.315)):
        # Tall side brackets straddle the head and place their bores on the tilt axis.
        base.visual(
            Box((0.075, 0.032, 0.24)),
            origin=Origin(xyz=(0.0, y, PIVOT_HEIGHT)),
            material=ivory,
            name=f"side_bracket_{idx}",
        )
        base.visual(
            Cylinder(radius=0.038, length=0.020),
            origin=Origin(xyz=(0.0, y * 0.954, PIVOT_HEIGHT), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=f"pivot_bushing_{idx}",
        )
        base.visual(
            _make_straight_tube(
                (-0.08, 0.0, 0.420),
                (-0.002, y, PIVOT_HEIGHT - 0.075),
                radius=0.018,
                name=f"fork_arm_{idx}",
            ),
            material=ivory,
            name=f"fork_arm_{idx}",
        )
    for idx, (x, y) in enumerate(((0.125, 0.095), (0.125, -0.095), (-0.125, 0.095), (-0.125, -0.095))):
        base.visual(
            Cylinder(radius=0.018, length=0.008),
            origin=Origin(xyz=(x, y, 0.004)),
            material=rubber,
            name=f"rubber_foot_{idx}",
        )

    head = model.part("head")
    front_x = GUARD_CENTER_X + 0.058
    rear_x = GUARD_CENTER_X - 0.058
    for name, x, tube in (
        ("front_ring", front_x, 0.0065),
        ("rear_ring", rear_x, 0.0058),
        ("front_mid_ring", front_x + 0.002, 0.0038),
        ("rear_mid_ring", rear_x - 0.002, 0.0034),
        ("front_hub_guard", front_x + 0.004, 0.0048),
        ("rear_hub_guard", rear_x - 0.004, 0.0045),
    ):
        ring_radius = {
            "front_ring": GUARD_RADIUS,
            "rear_ring": GUARD_RADIUS,
            "front_mid_ring": 0.150,
            "rear_mid_ring": 0.150,
            "front_hub_guard": 0.055,
            "rear_hub_guard": 0.055,
        }[name]
        head.visual(
            mesh_from_geometry(TorusGeometry(ring_radius, tube, radial_segments=18, tubular_segments=80), name),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=guard_metal,
            name=name,
        )

    for i in range(12):
        angle = i * 2.0 * math.pi / 12.0
        y_outer = GUARD_RADIUS * math.cos(angle)
        z_outer = GUARD_RADIUS * math.sin(angle)
        y_mid = 0.150 * math.cos(angle)
        z_mid = 0.150 * math.sin(angle)
        y_inner = 0.055 * math.cos(angle)
        z_inner = 0.055 * math.sin(angle)
        head.visual(
            _make_straight_tube(
                (front_x + 0.004, y_inner, z_inner),
                (front_x + 0.001, y_outer, z_outer),
                radius=0.0035,
                name=f"front_spoke_{i}",
            ),
            material=guard_metal,
            name=f"front_spoke_{i}",
        )
        if i % 2 == 0:
            head.visual(
                _make_straight_tube(
                    (rear_x - 0.004, y_inner, z_inner),
                    (rear_x - 0.001, y_outer, z_outer),
                    radius=0.0032,
                    name=f"rear_spoke_{i}",
                ),
                material=guard_metal,
                name=f"rear_spoke_{i}",
            )
        head.visual(
            _make_straight_tube(
                (rear_x, y_outer, z_outer),
                (front_x, y_outer, z_outer),
                radius=0.0038,
                name=f"cage_depth_wire_{i}",
            ),
            material=guard_metal,
            name=f"cage_depth_wire_{i}",
        )

    # Motor pod, rear cap, front bearing bridge, and side trunnions rotate as one tilting head.
    head.visual(
        Cylinder(radius=0.078, length=0.115),
        origin=Origin(xyz=(-0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="motor_pod",
    )
    head.visual(
        Cylinder(radius=0.055, length=0.030),
        origin=Origin(xyz=(GUARD_CENTER_X - 0.043, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="bearing_nose",
    )
    head.visual(
        Cylinder(radius=0.014, length=0.055),
        origin=Origin(xyz=(GUARD_CENTER_X - 0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="axle_stub",
    )
    head.visual(
        Cylinder(radius=0.029, length=0.190),
        origin=Origin(xyz=(0.0, 0.171, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="side_ear_0",
    )
    head.visual(
        Cylinder(radius=0.029, length=0.190),
        origin=Origin(xyz=(0.0, -0.171, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="side_ear_1",
    )
    head.visual(
        Cylinder(radius=0.026, length=0.041),
        origin=Origin(xyz=(0.0, 0.2705, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="trunnion_0",
    )
    head.visual(
        Cylinder(radius=0.026, length=0.041),
        origin=Origin(xyz=(0.0, -0.2705, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="trunnion_1",
    )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                ROTOR_RADIUS,
                0.052,
                5,
                thickness=ROTOR_THICKNESS,
                blade_pitch_deg=31.0,
                blade_sweep_deg=24.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.12, tip_clearance=0.010),
                hub=FanRotorHub(style="spinner"),
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
        origin=Origin(xyz=(0.0, 0.0, PIVOT_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=TILT_LOWER, upper=TILT_UPPER),
    )
    model.articulation(
        "head_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(xyz=(GUARD_CENTER_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=45.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("base_to_head")
    spin = object_model.get_articulation("head_to_rotor")

    ctx.check("head tilt is revolute", tilt.articulation_type == ArticulationType.REVOLUTE, details=str(tilt.articulation_type))
    ctx.check("rotor spin is continuous", spin.articulation_type == ArticulationType.CONTINUOUS, details=str(spin.articulation_type))
    ctx.check("tilt axis is horizontal", tuple(round(v, 3) for v in tilt.axis) == (0.0, -1.0, 0.0), details=str(tilt.axis))
    ctx.check("rotor axle axis is front to back", tuple(round(v, 3) for v in spin.axis) == (1.0, 0.0, 0.0), details=str(spin.axis))

    ctx.allow_overlap(
        head,
        rotor,
        elem_a="axle_stub",
        elem_b="rotor_blades",
        reason="The fixed motor axle is intentionally inserted into the rotating hub to show the captured spin bearing.",
    )
    ctx.allow_overlap(
        base,
        head,
        elem_a="pivot_bushing_0",
        elem_b="trunnion_0",
        reason="The side trunnion is intentionally seated in the yoke bushing with a tiny hidden bearing interference.",
    )
    ctx.allow_overlap(
        base,
        head,
        elem_a="pivot_bushing_1",
        elem_b="trunnion_1",
        reason="The side trunnion is intentionally seated in the yoke bushing with a tiny hidden bearing interference.",
    )

    ctx.expect_overlap(
        head,
        rotor,
        axes="x",
        elem_a="axle_stub",
        elem_b="rotor_blades",
        min_overlap=0.010,
        name="motor axle remains inserted in rotor hub",
    )
    ctx.expect_within(
        head,
        rotor,
        axes="yz",
        inner_elem="axle_stub",
        outer_elem="rotor_blades",
        margin=0.0,
        name="motor axle is centered in rotor envelope",
    )
    ctx.expect_gap(
        base,
        head,
        axis="y",
        positive_elem="pivot_bushing_0",
        negative_elem="trunnion_0",
        max_gap=0.002,
        max_penetration=0.002,
        name="positive side pivot is seated in bracket",
    )
    ctx.expect_gap(
        head,
        base,
        axis="y",
        positive_elem="trunnion_1",
        negative_elem="pivot_bushing_1",
        max_gap=0.002,
        max_penetration=0.002,
        name="negative side pivot is seated in bracket",
    )
    ctx.expect_gap(
        head,
        rotor,
        axis="x",
        positive_elem="front_ring",
        negative_elem="rotor_blades",
        min_gap=0.020,
        name="front guard clears spinning blades",
    )
    ctx.expect_gap(
        rotor,
        head,
        axis="x",
        positive_elem="rotor_blades",
        negative_elem="rear_ring",
        min_gap=0.020,
        name="rear guard clears spinning blades",
    )
    ctx.expect_within(
        rotor,
        head,
        axes="yz",
        inner_elem="rotor_blades",
        outer_elem="front_ring",
        margin=0.0,
        name="rotor disk sits inside the circular guard diameter",
    )

    rotor_box = ctx.part_element_world_aabb(rotor, elem="rotor_blades")
    base_box = ctx.part_element_world_aabb(base, elem="base_plate")
    if rotor_box is not None and base_box is not None:
        rotor_mins, rotor_maxs = rotor_box
        base_mins, base_maxs = base_box
        rotor_diameter = max(float(rotor_maxs[1] - rotor_mins[1]), float(rotor_maxs[2] - rotor_mins[2]))
        base_width = float(base_maxs[1] - base_mins[1])
        ctx.check(
            "rotor is large relative to fixed base",
            rotor_diameter > base_width * 1.25,
            details=f"rotor_diameter={rotor_diameter:.3f}, base_width={base_width:.3f}",
        )
    else:
        ctx.fail("rotor and base dimensions are measurable", "Missing rotor or base AABB.")

    rest_pos = ctx.part_world_position(rotor)
    with ctx.pose({tilt: TILT_UPPER}):
        raised_pos = ctx.part_world_position(rotor)
    ctx.check(
        "positive tilt raises the fan head",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.035,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
