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
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _radial_rod_origin(
    radius_mid: float, angle: float, x: float, z_offset: float = 0.0
) -> Origin:
    """Cylinder origin for a rod lying in the local YZ guard plane."""
    return Origin(
        xyz=(x, radius_mid * math.sin(angle), z_offset + radius_mid * math.cos(angle)),
        rpy=(-angle, 0.0, 0.0),
    )


def _y_cylinder_origin(x: float, y: float, z: float) -> Origin:
    """Cylinder origin with its local Z axis rotated onto world/local +Y."""
    return Origin(xyz=(x, y, z), rpy=(-math.pi / 2.0, 0.0, 0.0))


def _x_cylinder_origin(x: float, y: float, z: float) -> Origin:
    """Cylinder origin with its local Z axis rotated onto world/local +X."""
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_tilting_utility_fan")

    painted = model.material("hammered_safety_yellow", rgba=(0.92, 0.66, 0.16, 1.0))
    dark = model.material("matte_black_guard", rgba=(0.015, 0.017, 0.016, 1.0))
    blade_mat = model.material("dark_molded_blades", rgba=(0.05, 0.055, 0.06, 1.0))
    rubber = model.material("black_rubber", rgba=(0.006, 0.006, 0.005, 1.0))
    steel = model.material("zinc_fasteners", rgba=(0.58, 0.58, 0.55, 1.0))
    warning = model.material("white_marking", rgba=(0.92, 0.91, 0.84, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.82, 0.58, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=painted,
        name="base_pan",
    )
    base.visual(
        Box((0.70, 0.44, 0.025)),
        origin=Origin(xyz=(-0.015, 0.0, 0.092)),
        material=painted,
        name="raised_deck",
    )

    for i, (x, y) in enumerate(
        ((0.32, 0.22), (0.32, -0.22), (-0.32, 0.22), (-0.32, -0.22))
    ):
        base.visual(
            Cylinder(radius=0.055, length=0.026),
            origin=Origin(xyz=(x, y, 0.010)),
            material=rubber,
            name=f"rubber_foot_{i}",
        )
        base.visual(
            Cylinder(radius=0.018, length=0.014),
            origin=Origin(xyz=(x, y, 0.105)),
            material=steel,
            name=f"deck_bolt_{i}",
        )

    # A thick boxed pedestal and wide saddle keep the utility yoke serviceable.
    base.visual(
        Box((0.18, 0.16, 0.58)),
        origin=Origin(xyz=(-0.16, 0.0, 0.365)),
        material=painted,
        name="pedestal_post",
    )
    base.visual(
        Box((0.20, 0.22, 0.06)),
        origin=Origin(xyz=(-0.165, 0.0, 0.555)),
        material=painted,
        name="post_cap",
    )
    base.visual(
        Box((0.36, 0.92, 0.08)),
        origin=Origin(xyz=(-0.035, 0.0, 0.610)),
        material=painted,
        name="yoke_crossbeam",
    )
    for side, y in enumerate((-0.435, 0.435)):
        side_name = f"side_{side}"
        base.visual(
            Box((0.12, 0.060, 0.55)),
            origin=Origin(xyz=(0.0, y, 0.915)),
            material=painted,
            name=f"{side_name}_upright",
        )
        base.visual(
            Box((0.24, 0.070, 0.040)),
            origin=Origin(xyz=(-0.105, y, 0.715)),
            material=painted,
            name=f"{side_name}_lower_gusset",
        )
        base.visual(
            Box((0.24, 0.070, 0.040)),
            origin=Origin(xyz=(-0.08, y, 0.840), rpy=(0.0, -0.42, 0.0)),
            material=painted,
            name=f"{side_name}_diagonal_gusset",
        )
        base.visual(
            Cylinder(radius=0.078, length=0.075),
            origin=_y_cylinder_origin(0.0, y, 1.000),
            material=painted,
            name=f"bearing_block_{side}",
        )
        base.visual(
            Cylinder(radius=0.028, length=0.016),
            origin=_y_cylinder_origin(0.0, y + (0.045 if y > 0 else -0.045), 1.000),
            material=steel,
            name=f"pivot_washer_{side}",
        )
        for j, z in enumerate((0.79, 1.115)):
            base.visual(
                Cylinder(radius=0.016, length=0.014),
                origin=_y_cylinder_origin(0.04, y + (0.037 if y > 0 else -0.037), z),
                material=steel,
                name=f"{side_name}_bracket_bolt_{j}",
            )

    # Service speed dial on the front of the molded base.
    base.visual(
        Box((0.050, 0.18, 0.12)),
        origin=Origin(xyz=(0.360, 0.0, 0.160)),
        material=dark,
        name="control_pod",
    )
    for j, y in enumerate((-0.060, 0.060)):
        base.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=_x_cylinder_origin(0.387, y, 0.185),
            material=steel,
            name=f"control_screw_{j}",
        )

    fan_head = model.part("fan_head")

    # Stationary motor, side trunnions, and cage are authored in the head frame;
    # the frame itself is the tilt axis carried by the yellow side yoke.
    fan_head.visual(
        Cylinder(radius=0.110, length=0.190),
        origin=_x_cylinder_origin(-0.125, 0.0, 0.0),
        material=painted,
        name="motor_shell",
    )
    fan_head.visual(
        Cylinder(radius=0.130, length=0.045),
        origin=_x_cylinder_origin(-0.035, 0.0, 0.0),
        material=painted,
        name="motor_flange",
    )
    fan_head.visual(
        Cylinder(radius=0.035, length=0.024),
        origin=_x_cylinder_origin(-0.032, 0.0, 0.0),
        material=steel,
        name="bearing_sleeve",
    )
    fan_head.visual(
        Cylinder(radius=0.040, length=0.085),
        origin=_x_cylinder_origin(-0.240, 0.0, 0.0),
        material=dark,
        name="rear_cord_gland",
    )
    for side, y in enumerate((-0.220, 0.220)):
        fan_head.visual(
            Box((0.070, 0.300, 0.060)),
            origin=Origin(xyz=(-0.075, y, 0.0)),
            material=painted,
            name=f"trunnion_web_{side}",
        )
    for side, y in enumerate((-0.368, 0.368)):
        fan_head.visual(
            Box((0.090, 0.050, 0.160)),
            origin=Origin(xyz=(0.0, y * 0.88, 0.0)),
            material=painted,
            name=f"trunnion_lug_{side}",
        )
        fan_head.visual(
            Cylinder(radius=0.038, length=0.074),
            origin=_y_cylinder_origin(0.0, y, 0.0),
            material=steel,
            name=f"head_trunnion_{side}",
        )
        fan_head.visual(
            Cylinder(radius=0.054, length=0.018),
            origin=_y_cylinder_origin(0.0, y * 1.06, 0.0),
            material=dark,
            name=f"clamp_knob_{side}",
        )

    # Deep wire cage: two heavy outer hoops, intermediate hoops, radial spokes,
    # and standoff tubes between front and rear guards.
    for x, tag in ((0.135, "front"), (-0.135, "rear")):
        fan_head.visual(
            mesh_from_geometry(TorusGeometry(radius=0.322, tube=0.012), f"{tag}_outer_hoop"),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"{tag}_outer_hoop",
        )
        fan_head.visual(
            mesh_from_geometry(TorusGeometry(radius=0.225, tube=0.0065), f"{tag}_middle_hoop"),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"{tag}_middle_hoop",
        )
        fan_head.visual(
            mesh_from_geometry(TorusGeometry(radius=0.125, tube=0.006), f"{tag}_inner_hoop"),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"{tag}_inner_hoop",
        )
        for k in range(16):
            angle = 2.0 * math.pi * k / 16.0
            fan_head.visual(
                Cylinder(radius=0.0048, length=0.270),
                origin=_radial_rod_origin(0.194, angle, x),
                material=dark,
                name=f"{tag}_spoke_{k}",
            )

    for k in range(8):
        angle = 2.0 * math.pi * k / 8.0 + math.pi / 8.0
        y = 0.322 * math.sin(angle)
        z = 0.322 * math.cos(angle)
        fan_head.visual(
            Cylinder(radius=0.0065, length=0.310),
            origin=_x_cylinder_origin(0.0, y, z),
            material=dark,
            name=f"cage_standoff_{k}",
        )

    fan_head.visual(
        Cylinder(radius=0.066, length=0.018),
        origin=_x_cylinder_origin(0.148, 0.0, 0.0),
        material=dark,
        name="front_hub_guard",
    )
    fan_head.visual(
        Cylinder(radius=0.078, length=0.022),
        origin=_x_cylinder_origin(-0.148, 0.0, 0.0),
        material=dark,
        name="rear_hub_guard",
    )
    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.028, length=0.070),
        origin=_x_cylinder_origin(0.010, 0.0, 0.0),
        material=steel,
        name="shaft_stub",
    )
    rotor.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.276,
                0.070,
                5,
                thickness=0.044,
                blade_pitch_deg=34.0,
                blade_sweep_deg=32.0,
                blade=FanRotorBlade(shape="broad", tip_pitch_deg=15.0, camber=0.18),
                hub=FanRotorHub(
                    style="spinner",
                    rear_collar_height=0.020,
                    rear_collar_radius=0.055,
                    bore_diameter=0.014,
                ),
            ),
            "broad_utility_rotor",
        ),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_mat,
        name="rotor_blades",
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.060,
                0.034,
                body_style="faceted",
                grip=KnobGrip(style="ribbed", count=16, depth=0.0022, width=0.0020),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "speed_knob",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="knob_cap",
    )
    speed_knob.visual(
        Box((0.010, 0.004, 0.003)),
        origin=Origin(xyz=(0.035, 0.0, 0.024)),
        material=warning,
        name="pointer_mark",
    )

    model.articulation(
        "tilt_axis",
        ArticulationType.REVOLUTE,
        parent=base,
        child=fan_head,
        origin=Origin(xyz=(0.0, 0.0, 1.000)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=85.0, velocity=0.9, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=fan_head,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=45.0),
    )
    model.articulation(
        "speed_knob_turn",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_knob,
        origin=Origin(xyz=(0.385, 0.0, 0.160)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=3.0, lower=0.0, upper=4.7),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    fan_head = object_model.get_part("fan_head")
    rotor = object_model.get_part("rotor")
    speed_knob = object_model.get_part("speed_knob")
    tilt_axis = object_model.get_articulation("tilt_axis")

    for side in (0, 1):
        ctx.allow_overlap(
            base,
            fan_head,
            elem_a=f"bearing_block_{side}",
            elem_b=f"head_trunnion_{side}",
            reason=(
                "The head trunnion is intentionally captured in the solid bearing "
                "boss proxy at the exposed tilt pivot."
            ),
        )
        ctx.expect_overlap(
            base,
            fan_head,
            axes="xz",
            elem_a=f"bearing_block_{side}",
            elem_b=f"head_trunnion_{side}",
            min_overlap=0.040,
            name=f"tilt pivot {side} shares the bearing axis",
        )

    ctx.expect_gap(
        fan_head,
        base,
        axis="y",
        positive_elem="head_trunnion_0",
        negative_elem="bearing_block_0",
        max_penetration=0.010,
        name="negative side trunnion is locally seated",
    )
    ctx.expect_gap(
        base,
        fan_head,
        axis="y",
        positive_elem="bearing_block_1",
        negative_elem="head_trunnion_1",
        max_penetration=0.010,
        name="positive side trunnion is locally seated",
    )

    ctx.expect_within(
        rotor,
        fan_head,
        axes="yz",
        inner_elem="rotor_blades",
        margin=0.020,
        name="rotor stays inside the guarded cage diameter",
    )
    ctx.allow_overlap(
        fan_head,
        rotor,
        elem_a="bearing_sleeve",
        elem_b="shaft_stub",
        reason="The rotor shaft is intentionally captured inside the motor bearing sleeve.",
    )
    ctx.allow_overlap(
        fan_head,
        rotor,
        elem_a="motor_flange",
        elem_b="shaft_stub",
        reason="The steel shaft passes through the simplified solid motor-flange bore proxy.",
    )
    ctx.expect_within(
        rotor,
        fan_head,
        axes="yz",
        inner_elem="shaft_stub",
        outer_elem="bearing_sleeve",
        margin=0.008,
        name="rotor shaft is centered in the bearing sleeve",
    )
    ctx.expect_overlap(
        rotor,
        fan_head,
        axes="x",
        elem_a="shaft_stub",
        elem_b="bearing_sleeve",
        min_overlap=0.003,
        name="rotor shaft remains inserted in the bearing sleeve",
    )
    ctx.expect_gap(
        fan_head,
        rotor,
        axis="x",
        positive_elem="front_outer_hoop",
        negative_elem="rotor_blades",
        min_gap=0.040,
        name="front guard clears the spinning rotor",
    )
    ctx.expect_gap(
        rotor,
        fan_head,
        axis="x",
        positive_elem="rotor_blades",
        negative_elem="rear_outer_hoop",
        min_gap=0.040,
        name="rear guard clears the spinning rotor",
    )
    ctx.expect_contact(
        speed_knob,
        base,
        elem_a="knob_cap",
        elem_b="control_pod",
        contact_tol=0.002,
        name="speed knob sits on the front control pod",
    )

    rest_front = ctx.part_element_world_aabb(fan_head, elem="front_outer_hoop")
    with ctx.pose({tilt_axis: 0.55}):
        tilted_front = ctx.part_element_world_aabb(fan_head, elem="front_outer_hoop")
    rest_center_z = None
    tilted_center_z = None
    if rest_front is not None and tilted_front is not None:
        rest_center_z = 0.5 * (rest_front[0][2] + rest_front[1][2])
        tilted_center_z = 0.5 * (tilted_front[0][2] + tilted_front[1][2])
    ctx.check(
        "tilt joint visibly pitches the guarded head",
        rest_center_z is not None
        and tilted_center_z is not None
        and abs(tilted_center_z - rest_center_z) > 0.050,
        details=f"rest_z={rest_center_z}, tilted_z={tilted_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
