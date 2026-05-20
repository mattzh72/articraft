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
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


BB_SPACING = 0.68
BB_OUTER_RADIUS = 0.03
BB_INNER_RADIUS = 0.018
BB_SHELL_LENGTH = 0.08
SPINDLE_RADIUS = 0.0125
SPINDLE_LENGTH = 0.178
CRANK_ARM_LENGTH = 0.17
CRANK_ARM_WIDTH = 0.022
CRANK_ARM_THICKNESS = 0.018
ARM_SIDE_OFFSET = 0.086
PEDAL_BOSS_RADIUS = 0.012
PEDAL_BOSS_LENGTH = 0.03
PEDAL_BOSS_CENTER = 0.105
PEDAL_JOINT_OFFSET = 0.12
CHAINRING_OUTER_RADIUS = 0.1015
CHAINRING_INNER_RADIUS = 0.088
CHAINRING_THICKNESS = 0.004
CHAINRING_Y = 0.05
CHAIN_RADIUS = 0.004
CHAIN_Y = CHAINRING_Y + CHAINRING_THICKNESS / 2.0 + CHAIN_RADIUS
CHAIN_WRAP_RADIUS = CHAINRING_OUTER_RADIUS + CHAIN_RADIUS + 0.0005


def _tube_shell_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    mesh_name: str,
):
    half = length / 2.0
    shell = LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=48,
    )
    shell.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(shell, mesh_name)


def _timing_chain_mesh(mesh_name: str):
    front_x = 0.0
    rear_x = -BB_SPACING
    wrap_radius = CHAIN_WRAP_RADIUS
    arc_samples = 19

    points: list[tuple[float, float, float]] = []
    points.append((front_x, CHAIN_Y, wrap_radius))
    points.append(((front_x + rear_x) * 0.5, CHAIN_Y, wrap_radius))
    points.append((rear_x, CHAIN_Y, wrap_radius))

    for i in range(1, arc_samples):
        angle = math.pi / 2.0 + math.pi * i / arc_samples
        points.append(
            (
                rear_x + wrap_radius * math.cos(angle),
                CHAIN_Y,
                wrap_radius * math.sin(angle),
            )
        )

    points.append((rear_x, CHAIN_Y, -wrap_radius))
    points.append(((front_x + rear_x) * 0.5, CHAIN_Y, -wrap_radius))
    points.append((front_x, CHAIN_Y, -wrap_radius))

    for i in range(1, arc_samples):
        angle = -math.pi / 2.0 + math.pi * i / arc_samples
        points.append(
            (
                front_x + wrap_radius * math.cos(angle),
                CHAIN_Y,
                wrap_radius * math.sin(angle),
            )
        )

    chain_geom = tube_from_spline_points(
        points,
        radius=CHAIN_RADIUS,
        samples_per_segment=8,
        closed_spline=True,
        radial_segments=16,
        cap_ends=False,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(chain_geom, mesh_name)


def _crank_arm_mesh(mesh_name: str):
    half = CRANK_ARM_LENGTH / 2.0

    def section(width: float, thickness: float, radius: float, z: float):
        return [(x, y, z) for x, y in rounded_rect_profile(width, thickness, radius)]

    arm_geom = section_loft(
        [
            section(0.028, 0.018, 0.0055, -half),
            section(0.018, 0.014, 0.0035, 0.0),
            section(0.03, 0.017, 0.0055, half),
        ]
    )
    return mesh_from_geometry(arm_geom, mesh_name)


def _add_crankset(
    part,
    *,
    spindle_name: str,
    left_arm_name: str,
    right_arm_name: str,
    left_pedal_boss_name: str,
    right_pedal_boss_name: str,
    chainring_name: str,
    spider_boss_name: str,
    spider_vertical_name: str,
    spider_horizontal_name: str,
    metal,
    pedal_metal,
) -> None:
    part.visual(
        Cylinder(radius=SPINDLE_RADIUS, length=SPINDLE_LENGTH),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name=spindle_name,
    )
    for cap_name, y_pos in (
        (f"{spindle_name}_left_bearing_cap", BB_SHELL_LENGTH / 2.0 + 0.006),
        (f"{spindle_name}_right_bearing_cap", -(BB_SHELL_LENGTH / 2.0 + 0.006)),
    ):
        part.visual(
            Cylinder(radius=0.022, length=0.012),
            origin=Origin(
                xyz=(0.0, y_pos, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=pedal_metal,
            name=cap_name,
        )

    arm_z = {
        "left": -CRANK_ARM_LENGTH / 2.0,
        "right": CRANK_ARM_LENGTH / 2.0,
    }

    for side, side_sign in (("left", 1.0), ("right", -1.0)):
        arm_name = left_arm_name if side == "left" else right_arm_name
        pedal_boss_name = left_pedal_boss_name if side == "left" else right_pedal_boss_name
        part.visual(
            _crank_arm_mesh(f"{arm_name}_mesh"),
            origin=Origin(xyz=(0.0, side_sign * ARM_SIDE_OFFSET, arm_z[side])),
            material=metal,
            name=arm_name,
        )
        part.visual(
            Cylinder(radius=PEDAL_BOSS_RADIUS, length=PEDAL_BOSS_LENGTH),
            origin=Origin(
                xyz=(0.0, side_sign * PEDAL_BOSS_CENTER, arm_z[side] * 2.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=pedal_metal,
            name=pedal_boss_name,
        )

    chainring_mesh = _tube_shell_mesh(
        outer_radius=CHAINRING_OUTER_RADIUS,
        inner_radius=CHAINRING_INNER_RADIUS,
        length=CHAINRING_THICKNESS,
        mesh_name=f"{chainring_name}_mesh",
    )
    part.visual(
        chainring_mesh,
        origin=Origin(xyz=(0.0, CHAINRING_Y, 0.0)),
        material=metal,
        name=chainring_name,
    )
    part.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(
            xyz=(0.0, CHAINRING_Y, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal,
        name=spider_boss_name,
    )
    for feature_name, size, xyz in (
        (
            spider_vertical_name,
            (0.014, 0.008, 0.18),
            (0.0, CHAINRING_Y, 0.0),
        ),
        (
            spider_horizontal_name,
            (0.18, 0.008, 0.014),
            (0.0, CHAINRING_Y, 0.0),
        ),
    ):
        part.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=metal,
            name=feature_name,
        )


def _add_pedal(part, *, side_sign: float, metal, body_color, reflector_color) -> None:
    part.visual(
        Cylinder(radius=0.0045, length=0.05),
        origin=Origin(
            xyz=(0.0, side_sign * 0.025, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal,
        name="pedal_axle",
    )
    part.visual(
        Box((0.095, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, side_sign * 0.056, 0.0)),
        material=body_color,
        name="pedal_body",
    )
    part.visual(
        Box((0.016, 0.004, 0.028)),
        origin=Origin(xyz=(0.032, side_sign * 0.056, 0.022)),
        material=reflector_color,
        name="pedal_reflector",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tandem_stoker_crankset")

    frame_paint = model.material("frame_paint", rgba=(0.18, 0.18, 0.2, 1.0))
    forged_metal = model.material("forged_metal", rgba=(0.72, 0.74, 0.78, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.28, 0.3, 0.32, 1.0))
    pedal_black = model.material("pedal_black", rgba=(0.08, 0.08, 0.09, 1.0))
    reflector_amber = model.material("reflector_amber", rgba=(0.85, 0.42, 0.08, 1.0))

    frame_bridge = model.part("frame_bridge")
    front_bb_shell_mesh = _tube_shell_mesh(
        outer_radius=BB_OUTER_RADIUS,
        inner_radius=BB_INNER_RADIUS,
        length=BB_SHELL_LENGTH,
        mesh_name="front_bb_shell_mesh",
    )
    rear_bb_shell_mesh = _tube_shell_mesh(
        outer_radius=BB_OUTER_RADIUS,
        inner_radius=BB_INNER_RADIUS,
        length=BB_SHELL_LENGTH,
        mesh_name="rear_bb_shell_mesh",
    )
    frame_bridge.visual(
        front_bb_shell_mesh,
        origin=Origin(xyz=(BB_SPACING / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="front_bb_shell",
    )
    frame_bridge.visual(
        rear_bb_shell_mesh,
        origin=Origin(xyz=(-BB_SPACING / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="rear_bb_shell",
    )
    frame_bridge.visual(
        Box((BB_SPACING + 0.04, 0.042, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=frame_paint,
        name="spine_tube",
    )
    frame_bridge.visual(
        Box((0.04, 0.03, 0.08)),
        origin=Origin(xyz=(BB_SPACING / 2.0, 0.0, 0.07)),
        material=frame_paint,
        name="front_web",
    )
    frame_bridge.visual(
        Box((0.04, 0.03, 0.08)),
        origin=Origin(xyz=(-BB_SPACING / 2.0, 0.0, 0.07)),
        material=frame_paint,
        name="rear_web",
    )
    frame_bridge.inertial = Inertial.from_geometry(
        Box((BB_SPACING + 0.08, 0.08, 0.16)),
        mass=3.5,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
    )

    front_crankset = model.part("front_crankset")
    _add_crankset(
        front_crankset,
        spindle_name="front_spindle",
        left_arm_name="front_left_arm",
        right_arm_name="front_right_arm",
        left_pedal_boss_name="front_left_pedal_boss",
        right_pedal_boss_name="front_right_pedal_boss",
        chainring_name="front_chainring",
        spider_boss_name="front_spider_boss",
        spider_vertical_name="front_spider_vertical",
        spider_horizontal_name="front_spider_horizontal",
        metal=forged_metal,
        pedal_metal=dark_metal,
    )
    front_crankset.inertial = Inertial.from_geometry(
        Box((0.24, 0.24, 0.36)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    rear_crankset = model.part("rear_crankset")
    _add_crankset(
        rear_crankset,
        spindle_name="rear_spindle",
        left_arm_name="rear_left_arm",
        right_arm_name="rear_right_arm",
        left_pedal_boss_name="rear_left_pedal_boss",
        right_pedal_boss_name="rear_right_pedal_boss",
        chainring_name="rear_chainring",
        spider_boss_name="rear_spider_boss",
        spider_vertical_name="rear_spider_vertical",
        spider_horizontal_name="rear_spider_horizontal",
        metal=forged_metal,
        pedal_metal=dark_metal,
    )
    rear_crankset.inertial = Inertial.from_geometry(
        Box((0.24, 0.24, 0.36)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    timing_chain = model.part("timing_chain")
    timing_chain.visual(
        _timing_chain_mesh("timing_chain_mesh"),
        material=dark_metal,
        name="timing_loop",
    )
    for lug_name, x_pos, z_pos in (
        ("front_top_engagement", 0.0, CHAIN_WRAP_RADIUS + 0.0005),
        ("front_bottom_engagement", 0.0, -(CHAIN_WRAP_RADIUS + 0.0005)),
        ("rear_top_engagement", -BB_SPACING, CHAIN_WRAP_RADIUS + 0.0005),
        ("rear_bottom_engagement", -BB_SPACING, -(CHAIN_WRAP_RADIUS + 0.0005)),
    ):
        timing_chain.visual(
            Box((0.01, 0.008, 0.01)),
            origin=Origin(xyz=(x_pos, CHAIN_Y - 0.00001, z_pos)),
            material=dark_metal,
            name=lug_name,
        )
    timing_chain.inertial = Inertial.from_geometry(
        Box((BB_SPACING + 0.22, 0.02, 0.24)),
        mass=0.35,
        origin=Origin(xyz=(-BB_SPACING / 2.0, CHAIN_Y, 0.0)),
    )

    pedal_specs = (
        ("front_left_pedal", 1.0),
        ("front_right_pedal", -1.0),
        ("rear_left_pedal", 1.0),
        ("rear_right_pedal", -1.0),
    )
    for name, side_sign in pedal_specs:
        pedal = model.part(name)
        _add_pedal(
            pedal,
            side_sign=side_sign,
            metal=dark_metal,
            body_color=pedal_black,
            reflector_color=reflector_amber,
        )
        pedal.inertial = Inertial.from_geometry(
            Box((0.11, 0.07, 0.06)),
            mass=0.18,
            origin=Origin(xyz=(0.0, side_sign * 0.04, 0.0)),
        )

    model.articulation(
        "frame_to_front_crankset",
        ArticulationType.CONTINUOUS,
        parent=frame_bridge,
        child=front_crankset,
        origin=Origin(xyz=(BB_SPACING / 2.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=18.0),
    )
    model.articulation(
        "frame_to_rear_crankset",
        ArticulationType.CONTINUOUS,
        parent=frame_bridge,
        child=rear_crankset,
        origin=Origin(xyz=(-BB_SPACING / 2.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=18.0),
    )
    model.articulation(
        "front_crankset_to_timing_chain",
        ArticulationType.FIXED,
        parent=front_crankset,
        child=timing_chain,
        origin=Origin(),
    )

    pedal_joint_data = (
        (
            "front_crankset_to_front_left_pedal",
            front_crankset,
            "front_left_pedal",
            1.0,
            -CRANK_ARM_LENGTH,
        ),
        (
            "front_crankset_to_front_right_pedal",
            front_crankset,
            "front_right_pedal",
            -1.0,
            CRANK_ARM_LENGTH,
        ),
        (
            "rear_crankset_to_rear_left_pedal",
            rear_crankset,
            "rear_left_pedal",
            1.0,
            -CRANK_ARM_LENGTH,
        ),
        (
            "rear_crankset_to_rear_right_pedal",
            rear_crankset,
            "rear_right_pedal",
            -1.0,
            CRANK_ARM_LENGTH,
        ),
    )
    for joint_name, parent, child_name, side_sign, z_pos in pedal_joint_data:
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=parent,
            child=child_name,
            origin=Origin(xyz=(0.0, side_sign * PEDAL_JOINT_OFFSET, z_pos)),
            axis=(0.0, side_sign, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=25.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    frame_bridge = object_model.get_part("frame_bridge")
    front_crankset = object_model.get_part("front_crankset")
    rear_crankset = object_model.get_part("rear_crankset")
    timing_chain = object_model.get_part("timing_chain")
    front_left_pedal = object_model.get_part("front_left_pedal")
    front_right_pedal = object_model.get_part("front_right_pedal")
    rear_left_pedal = object_model.get_part("rear_left_pedal")
    rear_right_pedal = object_model.get_part("rear_right_pedal")

    front_crank_joint = object_model.get_articulation("frame_to_front_crankset")
    rear_crank_joint = object_model.get_articulation("frame_to_rear_crankset")
    front_left_pedal_joint = object_model.get_articulation("front_crankset_to_front_left_pedal")
    front_right_pedal_joint = object_model.get_articulation("front_crankset_to_front_right_pedal")

    ctx.expect_within(
        front_crankset,
        frame_bridge,
        axes="xz",
        inner_elem="front_spindle",
        outer_elem="front_bb_shell",
        margin=0.001,
        name="front spindle stays inside front bottom bracket shell",
    )
    ctx.expect_overlap(
        front_crankset,
        frame_bridge,
        axes="y",
        elem_a="front_spindle",
        elem_b="front_bb_shell",
        min_overlap=0.06,
        name="front spindle remains inserted through front bottom bracket shell",
    )
    ctx.expect_within(
        rear_crankset,
        frame_bridge,
        axes="xz",
        inner_elem="rear_spindle",
        outer_elem="rear_bb_shell",
        margin=0.001,
        name="rear spindle stays inside rear bottom bracket shell",
    )
    ctx.expect_overlap(
        rear_crankset,
        frame_bridge,
        axes="y",
        elem_a="rear_spindle",
        elem_b="rear_bb_shell",
        min_overlap=0.06,
        name="rear spindle remains inserted through rear bottom bracket shell",
    )

    ctx.expect_contact(
        timing_chain,
        front_crankset,
        elem_b="front_chainring",
        contact_tol=0.002,
        name="timing chain bears against the front timing ring",
    )
    ctx.expect_contact(
        timing_chain,
        rear_crankset,
        elem_b="rear_chainring",
        contact_tol=0.002,
        name="timing chain bears against the rear timing ring",
    )

    ctx.expect_contact(
        front_left_pedal,
        front_crankset,
        elem_a="pedal_axle",
        elem_b="front_left_pedal_boss",
        contact_tol=0.001,
        name="front left pedal spindle seats against its crank boss",
    )
    ctx.expect_contact(
        front_right_pedal,
        front_crankset,
        elem_a="pedal_axle",
        elem_b="front_right_pedal_boss",
        contact_tol=0.001,
        name="front right pedal spindle seats against its crank boss",
    )
    ctx.expect_contact(
        rear_left_pedal,
        rear_crankset,
        elem_a="pedal_axle",
        elem_b="rear_left_pedal_boss",
        contact_tol=0.001,
        name="rear left pedal spindle seats against its crank boss",
    )
    ctx.expect_contact(
        rear_right_pedal,
        rear_crankset,
        elem_a="pedal_axle",
        elem_b="rear_right_pedal_boss",
        contact_tol=0.001,
        name="rear right pedal spindle seats against its crank boss",
    )

    front_left_rest = ctx.part_world_position(front_left_pedal)
    rear_left_rest = ctx.part_world_position(rear_left_pedal)
    with ctx.pose({front_crank_joint: math.pi / 2.0, rear_crank_joint: math.pi / 2.0}):
        front_left_quarter = ctx.part_world_position(front_left_pedal)
        rear_left_quarter = ctx.part_world_position(rear_left_pedal)

    ctx.check(
        "front crankset rotation carries the left pedal forward",
        front_left_rest is not None
        and front_left_quarter is not None
        and front_left_quarter[0] > front_left_rest[0] + 0.14
        and front_left_quarter[2] > front_left_rest[2] + 0.14,
        details=f"rest={front_left_rest}, quarter_turn={front_left_quarter}",
    )
    ctx.check(
        "rear crankset rotation carries the left pedal forward",
        rear_left_rest is not None
        and rear_left_quarter is not None
        and rear_left_quarter[0] > rear_left_rest[0] + 0.14
        and rear_left_quarter[2] > rear_left_rest[2] + 0.14,
        details=f"rest={rear_left_rest}, quarter_turn={rear_left_quarter}",
    )

    left_reflector_rest = ctx.part_element_world_aabb(front_left_pedal, elem="pedal_reflector")
    right_reflector_rest = ctx.part_element_world_aabb(front_right_pedal, elem="pedal_reflector")
    with ctx.pose(
        {
            front_left_pedal_joint: math.pi,
            front_right_pedal_joint: math.pi,
        }
    ):
        left_reflector_flipped = ctx.part_element_world_aabb(front_left_pedal, elem="pedal_reflector")
        right_reflector_flipped = ctx.part_element_world_aabb(front_right_pedal, elem="pedal_reflector")

    def reflector_center_z(aabb):
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) * 0.5

    left_z0 = reflector_center_z(left_reflector_rest)
    left_z1 = reflector_center_z(left_reflector_flipped)
    right_z0 = reflector_center_z(right_reflector_rest)
    right_z1 = reflector_center_z(right_reflector_flipped)

    ctx.check(
        "left pedal spins about its axle",
        left_z0 is not None and left_z1 is not None and left_z1 < left_z0 - 0.03,
        details=f"rest_z={left_z0}, flipped_z={left_z1}",
    )
    ctx.check(
        "right pedal spins about its axle",
        right_z0 is not None and right_z1 is not None and right_z1 < right_z0 - 0.03,
        details=f"rest_z={right_z0}, flipped_z={right_z1}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
