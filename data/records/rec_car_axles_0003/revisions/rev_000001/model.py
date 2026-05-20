from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    kwargs = {"name": name} if name else {}
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        **kwargs,
    )


def _build_subframe(part, *, subframe_paint, machined_steel) -> None:
    part.visual(
        Box((0.72, 0.18, 0.11)),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=subframe_paint,
        name="center_crossmember",
    )
    part.visual(
        Box((0.62, 0.08, 0.07)),
        origin=Origin(xyz=(0.0, 0.22, 0.10)),
        material=subframe_paint,
        name="front_tie_bar",
    )
    part.visual(
        Box((0.62, 0.08, 0.07)),
        origin=Origin(xyz=(0.0, -0.22, 0.10)),
        material=subframe_paint,
        name="rear_tie_bar",
    )
    part.visual(
        Box((0.42, 0.12, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.37)),
        material=subframe_paint,
        name="upper_bridge",
    )
    part.visual(
        Box((0.18, 0.10, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        material=subframe_paint,
        name="center_riser",
    )

    for side_sign in (-1.0, 1.0):
        side_prefix = "left" if side_sign > 0.0 else "right"
        rail_x = 0.31 * side_sign
        tower_x = 0.37 * side_sign
        part.visual(
            Box((0.14, 0.48, 0.11)),
            origin=Origin(xyz=(rail_x, 0.0, 0.075)),
            material=subframe_paint,
            name=f"{side_prefix}_lower_rail",
        )
        part.visual(
            Box((0.12, 0.26, 0.24)),
            origin=Origin(xyz=(tower_x, 0.0, 0.27)),
            material=subframe_paint,
            name=f"{side_prefix}_upper_tower",
        )
        part.visual(
            Box((0.08, 0.18, 0.10)),
            origin=Origin(xyz=(tower_x - 0.06 * side_sign, 0.0, 0.16)),
            material=subframe_paint,
            name=f"{side_prefix}_tower_gusset",
        )
        _add_member(
            part,
            (tower_x, 0.08, 0.19),
            (rail_x, 0.15, 0.08),
            radius=0.018,
            material=subframe_paint,
        )
        _add_member(
            part,
            (tower_x, -0.08, 0.19),
            (rail_x, -0.15, 0.08),
            radius=0.018,
            material=subframe_paint,
        )

        mount_specs = [
            (f"{side_prefix}_upper_front_mount", (0.37 * side_sign, 0.078, 0.39), 0.026, 0.070),
            (f"{side_prefix}_upper_rear_mount", (0.37 * side_sign, -0.078, 0.39), 0.026, 0.070),
            (f"{side_prefix}_lower_front_mount", (0.31 * side_sign, 0.118, 0.11), 0.032, 0.082),
            (f"{side_prefix}_lower_rear_mount", (0.31 * side_sign, -0.118, 0.11), 0.032, 0.082),
        ]
        for name, center, radius, length in mount_specs:
            part.visual(
                Cylinder(radius=radius, length=length),
                origin=Origin(xyz=center, rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=machined_steel,
                name=name,
            )


def _build_wishbone_arm(
    part,
    *,
    side_sign: float,
    upper: bool,
    forged_steel,
    machined_steel,
) -> None:
    if upper:
        front_inner = (0.0, 0.078, 0.0)
        rear_inner = (0.0, -0.078, 0.0)
        outboard = (0.21 * side_sign, 0.02, -0.06)
        leg_radius = 0.013
        brace_a = (0.075 * side_sign, 0.050, -0.020)
        brace_b = (0.075 * side_sign, -0.040, -0.020)
        inner_ball_radius = 0.021
        outer_ball_radius = 0.019
        body_box = Box((0.068, 0.060, 0.020))
        body_origin = Origin(xyz=(0.090 * side_sign, 0.0, -0.026))
        body_name = "upper_web"
        root_bridge = Box((0.024, 0.126, 0.024))
        root_bridge_origin = Origin(xyz=(0.0, 0.0, -0.003))
    else:
        front_inner = (0.0, 0.118, 0.0)
        rear_inner = (0.0, -0.118, 0.0)
        outboard = (0.27 * side_sign, 0.02, -0.02)
        leg_radius = 0.019
        brace_a = (0.105 * side_sign, 0.060, -0.006)
        brace_b = (0.105 * side_sign, -0.055, -0.006)
        inner_ball_radius = 0.026
        outer_ball_radius = 0.023
        body_box = Box((0.088, 0.072, 0.024))
        body_origin = Origin(xyz=(0.125 * side_sign, 0.0, -0.012))
        body_name = "lower_web"
        root_bridge = Box((0.030, 0.186, 0.030))
        root_bridge_origin = Origin(xyz=(0.0, 0.0, -0.004))

    _add_member(part, front_inner, outboard, radius=leg_radius, material=forged_steel)
    _add_member(part, rear_inner, outboard, radius=leg_radius, material=forged_steel)
    _add_member(
        part,
        brace_a,
        brace_b,
        radius=leg_radius * 0.9,
        material=forged_steel,
        name=body_name,
    )
    _add_member(
        part,
        (brace_a[0], 0.040, brace_a[2]),
        (brace_b[0], -0.040, brace_b[2]),
        radius=leg_radius * 0.55,
        material=forged_steel,
    )
    part.visual(body_box, origin=body_origin, material=forged_steel)
    part.visual(root_bridge, origin=root_bridge_origin, material=forged_steel, name="inner_bridge")

    inner_length = 0.032 if upper else 0.036
    outer_length = 0.030 if upper else 0.034
    part.visual(
        Sphere(radius=inner_ball_radius),
        origin=Origin(xyz=front_inner),
        material=machined_steel,
        name="front_inner_ball",
    )
    part.visual(
        Sphere(radius=inner_ball_radius),
        origin=Origin(xyz=rear_inner),
        material=machined_steel,
        name="rear_inner_ball",
    )
    part.visual(
        Sphere(radius=outer_ball_radius),
        origin=Origin(xyz=outboard),
        material=machined_steel,
        name="outboard_ball",
    )
    part.visual(
        Cylinder(radius=inner_ball_radius * 0.95, length=inner_length),
        origin=Origin(xyz=front_inner, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=forged_steel,
    )
    part.visual(
        Cylinder(radius=inner_ball_radius * 0.95, length=inner_length),
        origin=Origin(xyz=rear_inner, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=forged_steel,
    )
    part.visual(
        Cylinder(radius=outer_ball_radius * 0.92, length=outer_length),
        origin=Origin(xyz=outboard, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=forged_steel,
    )


def _build_knuckle(part, *, side_sign: float, forged_steel, machined_steel) -> None:
    part.visual(
        Cylinder(radius=0.024, length=0.22),
        origin=Origin(),
        material=forged_steel,
        name="upright_body",
    )
    part.visual(
        Sphere(radius=0.033),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=forged_steel,
        name="upper_socket",
    )
    part.visual(
        Sphere(radius=0.036),
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
        material=forged_steel,
        name="lower_socket",
    )
    part.visual(
        Box((0.080, 0.050, 0.085)),
        origin=Origin(xyz=(0.040 * side_sign, 0.0, -0.012)),
        material=forged_steel,
        name="spindle_bridge",
    )
    part.visual(
        Sphere(radius=0.044),
        origin=Origin(xyz=(0.045 * side_sign, 0.0, -0.014)),
        material=forged_steel,
    )
    _add_member(
        part,
        (0.0, 0.0, 0.075),
        (0.050 * side_sign, 0.0, -0.014),
        radius=0.017,
        material=forged_steel,
    )
    _add_member(
        part,
        (0.0, 0.0, -0.080),
        (0.050 * side_sign, 0.0, -0.014),
        radius=0.019,
        material=forged_steel,
    )
    part.visual(
        Cylinder(radius=0.024, length=0.16),
        origin=Origin(xyz=(0.110 * side_sign, 0.0, -0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="spindle",
    )
    part.visual(
        Box((0.090, 0.060, 0.018)),
        origin=Origin(xyz=(0.030 * side_sign, -0.140, -0.055)),
        material=forged_steel,
        name="steering_arm",
    )
    _add_member(
        part,
        (0.018 * side_sign, -0.030, -0.032),
        (0.030 * side_sign, -0.112, -0.055),
        radius=0.012,
        material=forged_steel,
    )


def _build_hub(part, *, side_sign: float, machined_steel, rotor_steel, fastener_steel) -> None:
    part.visual(
        Cylinder(radius=0.046, length=0.102),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="hub_barrel",
    )
    part.visual(
        Cylinder(radius=0.145, length=0.012),
        origin=Origin(xyz=(-0.040 * side_sign, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rotor_steel,
        name="rotor_flange",
    )
    part.visual(
        Cylinder(radius=0.056, length=0.052),
        origin=Origin(xyz=(-0.018 * side_sign, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="rotor_hat",
    )
    part.visual(
        Cylinder(radius=0.075, length=0.018),
        origin=Origin(xyz=(0.046 * side_sign, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="wheel_face",
    )
    part.visual(
        Cylinder(radius=0.030, length=0.028),
        origin=Origin(xyz=(0.060 * side_sign, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="pilot_register",
    )
    for index in range(5):
        angle = 2.0 * math.pi * index / 5.0
        part.visual(
            Cylinder(radius=0.008, length=0.028),
            origin=Origin(
                xyz=(0.062 * side_sign, 0.052 * math.cos(angle), 0.052 * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=fastener_steel,
            name=f"lug_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_wishbone_front_axle", assets=ASSETS)

    subframe_paint = model.material("subframe_paint", rgba=(0.18, 0.19, 0.20, 1.0))
    forged_steel = model.material("forged_steel", rgba=(0.34, 0.35, 0.37, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    rotor_steel = model.material("rotor_steel", rgba=(0.56, 0.57, 0.59, 1.0))
    fastener_steel = model.material("fastener_steel", rgba=(0.80, 0.81, 0.83, 1.0))

    subframe = model.part("subframe")
    subframe.inertial = Inertial.from_geometry(
        Box((0.88, 0.56, 0.46)),
        mass=62.0,
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
    )
    _build_subframe(subframe, subframe_paint=subframe_paint, machined_steel=machined_steel)

    left_upper_arm = model.part("left_upper_arm")
    left_upper_arm.inertial = Inertial.from_geometry(
        Box((0.30, 0.18, 0.10)),
        mass=4.2,
        origin=Origin(xyz=(0.11, 0.0, -0.03)),
    )
    _build_wishbone_arm(
        left_upper_arm,
        side_sign=1.0,
        upper=True,
        forged_steel=forged_steel,
        machined_steel=machined_steel,
    )

    right_upper_arm = model.part("right_upper_arm")
    right_upper_arm.inertial = Inertial.from_geometry(
        Box((0.30, 0.18, 0.10)),
        mass=4.2,
        origin=Origin(xyz=(-0.11, 0.0, -0.03)),
    )
    _build_wishbone_arm(
        right_upper_arm,
        side_sign=-1.0,
        upper=True,
        forged_steel=forged_steel,
        machined_steel=machined_steel,
    )

    left_lower_arm = model.part("left_lower_arm")
    left_lower_arm.inertial = Inertial.from_geometry(
        Box((0.36, 0.24, 0.10)),
        mass=6.8,
        origin=Origin(xyz=(0.13, 0.0, -0.01)),
    )
    _build_wishbone_arm(
        left_lower_arm,
        side_sign=1.0,
        upper=False,
        forged_steel=forged_steel,
        machined_steel=machined_steel,
    )

    right_lower_arm = model.part("right_lower_arm")
    right_lower_arm.inertial = Inertial.from_geometry(
        Box((0.36, 0.24, 0.10)),
        mass=6.8,
        origin=Origin(xyz=(-0.13, 0.0, -0.01)),
    )
    _build_wishbone_arm(
        right_lower_arm,
        side_sign=-1.0,
        upper=False,
        forged_steel=forged_steel,
        machined_steel=machined_steel,
    )

    left_knuckle = model.part("left_knuckle")
    left_knuckle.inertial = Inertial.from_geometry(
        Box((0.24, 0.20, 0.30)),
        mass=8.0,
        origin=Origin(xyz=(0.06, -0.03, 0.0)),
    )
    _build_knuckle(
        left_knuckle,
        side_sign=1.0,
        forged_steel=forged_steel,
        machined_steel=machined_steel,
    )

    right_knuckle = model.part("right_knuckle")
    right_knuckle.inertial = Inertial.from_geometry(
        Box((0.24, 0.20, 0.30)),
        mass=8.0,
        origin=Origin(xyz=(-0.06, -0.03, 0.0)),
    )
    _build_knuckle(
        right_knuckle,
        side_sign=-1.0,
        forged_steel=forged_steel,
        machined_steel=machined_steel,
    )

    left_hub = model.part("left_hub")
    left_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.145, length=0.11),
        mass=6.2,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _build_hub(
        left_hub,
        side_sign=1.0,
        machined_steel=machined_steel,
        rotor_steel=rotor_steel,
        fastener_steel=fastener_steel,
    )

    right_hub = model.part("right_hub")
    right_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.145, length=0.11),
        mass=6.2,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _build_hub(
        right_hub,
        side_sign=-1.0,
        machined_steel=machined_steel,
        rotor_steel=rotor_steel,
        fastener_steel=fastener_steel,
    )

    model.articulation(
        "left_upper_arm_mount",
        ArticulationType.FIXED,
        parent="subframe",
        child="left_upper_arm",
        origin=Origin(xyz=(0.37, 0.0, 0.39)),
    )
    model.articulation(
        "right_upper_arm_mount",
        ArticulationType.FIXED,
        parent="subframe",
        child="right_upper_arm",
        origin=Origin(xyz=(-0.37, 0.0, 0.39)),
    )
    model.articulation(
        "left_lower_arm_mount",
        ArticulationType.FIXED,
        parent="subframe",
        child="left_lower_arm",
        origin=Origin(xyz=(0.31, 0.0, 0.11)),
    )
    model.articulation(
        "right_lower_arm_mount",
        ArticulationType.FIXED,
        parent="subframe",
        child="right_lower_arm",
        origin=Origin(xyz=(-0.31, 0.0, 0.11)),
    )
    model.articulation(
        "left_steer",
        ArticulationType.REVOLUTE,
        parent="subframe",
        child="left_knuckle",
        origin=Origin(xyz=(0.58, 0.02, 0.21)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.0, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "right_steer",
        ArticulationType.REVOLUTE,
        parent="subframe",
        child="right_knuckle",
        origin=Origin(xyz=(-0.58, 0.02, 0.21)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.0, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "left_hub_spin",
        ArticulationType.CONTINUOUS,
        parent="left_knuckle",
        child="left_hub",
        origin=Origin(xyz=(0.12, 0.0, -0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=22.0),
    )
    model.articulation(
        "right_hub_spin",
        ArticulationType.CONTINUOUS,
        parent="right_knuckle",
        child="right_hub",
        origin=Origin(xyz=(-0.12, 0.0, -0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=22.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    subframe = object_model.get_part("subframe")
    left_upper_arm = object_model.get_part("left_upper_arm")
    right_upper_arm = object_model.get_part("right_upper_arm")
    left_lower_arm = object_model.get_part("left_lower_arm")
    right_lower_arm = object_model.get_part("right_lower_arm")
    left_knuckle = object_model.get_part("left_knuckle")
    right_knuckle = object_model.get_part("right_knuckle")
    left_hub = object_model.get_part("left_hub")
    right_hub = object_model.get_part("right_hub")

    left_steer = object_model.get_articulation("left_steer")
    right_steer = object_model.get_articulation("right_steer")
    left_hub_spin = object_model.get_articulation("left_hub_spin")
    right_hub_spin = object_model.get_articulation("right_hub_spin")

    left_upper_front_mount = subframe.get_visual("left_upper_front_mount")
    left_upper_rear_mount = subframe.get_visual("left_upper_rear_mount")
    right_upper_front_mount = subframe.get_visual("right_upper_front_mount")
    right_upper_rear_mount = subframe.get_visual("right_upper_rear_mount")
    left_lower_front_mount = subframe.get_visual("left_lower_front_mount")
    left_lower_rear_mount = subframe.get_visual("left_lower_rear_mount")
    right_lower_front_mount = subframe.get_visual("right_lower_front_mount")
    right_lower_rear_mount = subframe.get_visual("right_lower_rear_mount")

    left_upper_front_ball = left_upper_arm.get_visual("front_inner_ball")
    left_upper_rear_ball = left_upper_arm.get_visual("rear_inner_ball")
    right_upper_front_ball = right_upper_arm.get_visual("front_inner_ball")
    right_upper_rear_ball = right_upper_arm.get_visual("rear_inner_ball")
    left_lower_front_ball = left_lower_arm.get_visual("front_inner_ball")
    left_lower_rear_ball = left_lower_arm.get_visual("rear_inner_ball")
    right_lower_front_ball = right_lower_arm.get_visual("front_inner_ball")
    right_lower_rear_ball = right_lower_arm.get_visual("rear_inner_ball")
    left_upper_out_ball = left_upper_arm.get_visual("outboard_ball")
    right_upper_out_ball = right_upper_arm.get_visual("outboard_ball")
    left_lower_out_ball = left_lower_arm.get_visual("outboard_ball")
    right_lower_out_ball = right_lower_arm.get_visual("outboard_ball")

    left_upper_socket = left_knuckle.get_visual("upper_socket")
    right_upper_socket = right_knuckle.get_visual("upper_socket")
    left_lower_socket = left_knuckle.get_visual("lower_socket")
    right_lower_socket = right_knuckle.get_visual("lower_socket")
    left_spindle = left_knuckle.get_visual("spindle")
    right_spindle = right_knuckle.get_visual("spindle")
    left_steering_arm = left_knuckle.get_visual("steering_arm")
    right_steering_arm = right_knuckle.get_visual("steering_arm")

    left_hub_barrel = left_hub.get_visual("hub_barrel")
    right_hub_barrel = right_hub.get_visual("hub_barrel")
    left_rotor_flange = left_hub.get_visual("rotor_flange")
    right_rotor_flange = right_hub.get_visual("rotor_flange")
    left_wheel_face = left_hub.get_visual("wheel_face")
    right_wheel_face = right_hub.get_visual("wheel_face")
    left_lug_0 = left_hub.get_visual("lug_0")
    left_lug_2 = left_hub.get_visual("lug_2")
    right_lug_0 = right_hub.get_visual("lug_0")
    right_lug_2 = right_hub.get_visual("lug_2")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # The steering axis is a virtual kingpin line between upper and lower ball joints,
    # so the parent-side origin is intentionally not seated on a literal subframe bracket.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.15)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()

    for part_a, part_b, reason in (
        (left_upper_arm, left_knuckle, "upper ball joint nests inside the left forged socket"),
        (right_upper_arm, right_knuckle, "upper ball joint nests inside the right forged socket"),
        (left_lower_arm, left_knuckle, "lower ball joint nests inside the left forged socket"),
        (right_lower_arm, right_knuckle, "lower ball joint nests inside the right forged socket"),
        (left_hub, left_knuckle, "left hub bearing and rotor hat nest over the spindle"),
        (right_hub, right_knuckle, "right hub bearing and rotor hat nest over the spindle"),
    ):
        ctx.allow_overlap(part_a, part_b, reason=reason)

    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
    )
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_within(
        left_upper_arm,
        subframe,
        axes="xz",
        inner_elem=left_upper_front_ball,
        outer_elem=left_upper_front_mount,
        name="left_upper_front_ball_seated_in_subframe",
    )
    ctx.expect_within(
        left_upper_arm,
        subframe,
        axes="xz",
        inner_elem=left_upper_rear_ball,
        outer_elem=left_upper_rear_mount,
        name="left_upper_rear_ball_seated_in_subframe",
    )
    ctx.expect_within(
        right_upper_arm,
        subframe,
        axes="xz",
        inner_elem=right_upper_front_ball,
        outer_elem=right_upper_front_mount,
        name="right_upper_front_ball_seated_in_subframe",
    )
    ctx.expect_within(
        right_upper_arm,
        subframe,
        axes="xz",
        inner_elem=right_upper_rear_ball,
        outer_elem=right_upper_rear_mount,
        name="right_upper_rear_ball_seated_in_subframe",
    )
    ctx.expect_within(
        left_lower_arm,
        subframe,
        axes="xz",
        inner_elem=left_lower_front_ball,
        outer_elem=left_lower_front_mount,
        name="left_lower_front_ball_seated_in_subframe",
    )
    ctx.expect_within(
        left_lower_arm,
        subframe,
        axes="xz",
        inner_elem=left_lower_rear_ball,
        outer_elem=left_lower_rear_mount,
        name="left_lower_rear_ball_seated_in_subframe",
    )
    ctx.expect_within(
        right_lower_arm,
        subframe,
        axes="xz",
        inner_elem=right_lower_front_ball,
        outer_elem=right_lower_front_mount,
        name="right_lower_front_ball_seated_in_subframe",
    )
    ctx.expect_within(
        right_lower_arm,
        subframe,
        axes="xz",
        inner_elem=right_lower_rear_ball,
        outer_elem=right_lower_rear_mount,
        name="right_lower_rear_ball_seated_in_subframe",
    )

    ctx.expect_within(
        left_upper_arm,
        left_knuckle,
        axes="yz",
        inner_elem=left_upper_out_ball,
        outer_elem=left_upper_socket,
        name="left_upper_ball_within_knuckle_socket",
    )
    ctx.expect_within(
        left_lower_arm,
        left_knuckle,
        axes="yz",
        inner_elem=left_lower_out_ball,
        outer_elem=left_lower_socket,
        name="left_lower_ball_within_knuckle_socket",
    )
    ctx.expect_within(
        right_upper_arm,
        right_knuckle,
        axes="yz",
        inner_elem=right_upper_out_ball,
        outer_elem=right_upper_socket,
        name="right_upper_ball_within_knuckle_socket",
    )
    ctx.expect_within(
        right_lower_arm,
        right_knuckle,
        axes="yz",
        inner_elem=right_lower_out_ball,
        outer_elem=right_lower_socket,
        name="right_lower_ball_within_knuckle_socket",
    )

    ctx.expect_gap(
        left_upper_arm,
        left_lower_arm,
        axis="z",
        min_gap=0.15,
        name="left_upper_arm_clears_lower_arm_vertically",
    )
    ctx.expect_gap(
        right_upper_arm,
        right_lower_arm,
        axis="z",
        min_gap=0.15,
        name="right_upper_arm_clears_lower_arm_vertically",
    )

    ctx.expect_within(
        left_knuckle,
        left_hub,
        axes="yz",
        inner_elem=left_spindle,
        outer_elem=left_hub_barrel,
        name="left_hub_bearing_wraps_spindle",
    )
    ctx.expect_within(
        right_knuckle,
        right_hub,
        axes="yz",
        inner_elem=right_spindle,
        outer_elem=right_hub_barrel,
        name="right_hub_bearing_wraps_spindle",
    )
    ctx.expect_within(
        left_hub,
        left_hub,
        axes="yz",
        inner_elem=left_hub_barrel,
        outer_elem=left_rotor_flange,
        name="left_rotor_flange_reads_larger_than_hub_barrel",
    )
    ctx.expect_within(
        right_hub,
        right_hub,
        axes="yz",
        inner_elem=right_hub_barrel,
        outer_elem=right_rotor_flange,
        name="right_rotor_flange_reads_larger_than_hub_barrel",
    )
    ctx.expect_gap(
        left_hub,
        left_knuckle,
        axis="y",
        min_gap=0.015,
        positive_elem=left_wheel_face,
        negative_elem=left_steering_arm,
        name="left_steering_arm_trails_wheel_face",
    )
    ctx.expect_gap(
        right_hub,
        right_knuckle,
        axis="y",
        min_gap=0.015,
        positive_elem=right_wheel_face,
        negative_elem=right_steering_arm,
        name="right_steering_arm_trails_wheel_face",
    )
    ctx.expect_within(
        left_hub,
        left_hub,
        axes="yz",
        inner_elem=left_lug_0,
        outer_elem=left_wheel_face,
        name="left_lug_bolt_pattern_on_hub_face_a",
    )
    ctx.expect_within(
        left_hub,
        left_hub,
        axes="yz",
        inner_elem=left_lug_2,
        outer_elem=left_wheel_face,
        name="left_lug_bolt_pattern_on_hub_face_b",
    )
    ctx.expect_within(
        right_hub,
        right_hub,
        axes="yz",
        inner_elem=right_lug_0,
        outer_elem=right_wheel_face,
        name="right_lug_bolt_pattern_on_hub_face_a",
    )
    ctx.expect_within(
        right_hub,
        right_hub,
        axes="yz",
        inner_elem=right_lug_2,
        outer_elem=right_wheel_face,
        name="right_lug_bolt_pattern_on_hub_face_b",
    )

    with ctx.pose({left_steer: 0.42, right_steer: -0.42}):
        ctx.expect_within(
            left_upper_arm,
            left_knuckle,
            axes="yz",
            inner_elem=left_upper_out_ball,
            outer_elem=left_upper_socket,
            name="left_upper_ball_stays_nested_at_full_steer",
        )
        ctx.expect_within(
            left_lower_arm,
            left_knuckle,
            axes="yz",
            inner_elem=left_lower_out_ball,
            outer_elem=left_lower_socket,
            name="left_lower_ball_stays_nested_at_full_steer",
        )
        ctx.expect_within(
            right_upper_arm,
            right_knuckle,
            axes="yz",
            inner_elem=right_upper_out_ball,
            outer_elem=right_upper_socket,
            name="right_upper_ball_stays_nested_at_full_steer",
        )
        ctx.expect_within(
            right_lower_arm,
            right_knuckle,
            axes="yz",
            inner_elem=right_lower_out_ball,
            outer_elem=right_lower_socket,
            name="right_lower_ball_stays_nested_at_full_steer",
        )
        ctx.expect_within(
            left_knuckle,
            left_hub,
            axes="yz",
            inner_elem=left_spindle,
            outer_elem=left_hub_barrel,
            name="left_hub_stays_on_spindle_when_steered",
        )
        ctx.expect_within(
            right_knuckle,
            right_hub,
            axes="yz",
            inner_elem=right_spindle,
            outer_elem=right_hub_barrel,
            name="right_hub_stays_on_spindle_when_steered",
        )

    with ctx.pose({left_hub_spin: 1.30, right_hub_spin: -0.85}):
        ctx.expect_within(
            left_knuckle,
            left_hub,
            axes="yz",
            inner_elem=left_spindle,
            outer_elem=left_hub_barrel,
            name="left_hub_bearing_remains_centered_while_spinning",
        )
        ctx.expect_within(
            right_knuckle,
            right_hub,
            axes="yz",
            inner_elem=right_spindle,
            outer_elem=right_hub_barrel,
            name="right_hub_bearing_remains_centered_while_spinning",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
