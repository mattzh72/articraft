from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

try:
    os.getcwd()
except FileNotFoundError:
    os.chdir("/")

ASSETS = AssetContext.from_script(__file__)


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
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


def _add_member(part, a, b, radius: float, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="immersion_blender", assets=ASSETS)

    body_plastic = model.material("body_plastic", rgba=(0.18, 0.19, 0.21, 1.0))
    soft_grip = model.material("soft_grip", rgba=(0.09, 0.10, 0.11, 1.0))
    switch_grey = model.material("switch_grey", rgba=(0.42, 0.45, 0.48, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.63, 0.65, 0.68, 1.0))
    accent_grey = model.material("accent_grey", rgba=(0.58, 0.60, 0.63, 1.0))
    dark_stainless = model.material("dark_stainless", rgba=(0.50, 0.52, 0.55, 1.0))

    guard_ring_mesh = _save_mesh(
        "blender_guard_ring.obj",
        TorusGeometry(radius=0.015, tube=0.0018, radial_segments=14, tubular_segments=48),
    )

    motor_body = model.part("motor_body")
    motor_body.visual(
        Cylinder(radius=0.031, length=0.112),
        origin=Origin(xyz=(0.0, -0.001, 0.182), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_plastic,
        name="top_motor",
    )
    motor_body.visual(
        Box((0.068, 0.082, 0.074)),
        origin=Origin(xyz=(0.0, 0.006, 0.148)),
        material=body_plastic,
        name="main_housing",
    )
    motor_body.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.0, -0.057, 0.181), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_plastic,
        name="rear_cap",
    )
    motor_body.visual(
        Box((0.046, 0.040, 0.122)),
        origin=Origin(xyz=(0.0, -0.002, 0.070)),
        material=soft_grip,
        name="grip_shell",
    )
    motor_body.visual(
        Box((0.034, 0.020, 0.084)),
        origin=Origin(xyz=(0.0, -0.023, 0.078)),
        material=soft_grip,
        name="grip_back_pad",
    )
    motor_body.visual(
        Box((0.046, 0.032, 0.042)),
        origin=Origin(xyz=(0.0, 0.039, 0.108)),
        material=body_plastic,
        name="front_socket_block",
    )
    motor_body.visual(
        Cylinder(radius=0.022, length=0.028),
        origin=Origin(xyz=(0.0, 0.059, 0.101), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_plastic,
        name="socket_housing",
    )
    motor_body.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.0, 0.052, 0.087), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent_grey,
        name="collar_seat",
    )
    motor_body.visual(
        Box((0.026, 0.004, 0.032)),
        origin=Origin(xyz=(0.0, -0.060, 0.177)),
        material=accent_grey,
        name="switch_track",
    )
    motor_body.visual(
        Box((0.012, 0.014, 0.012)),
        origin=Origin(xyz=(-0.020, 0.060, 0.100)),
        material=accent_grey,
        name="left_button_seat",
    )
    motor_body.visual(
        Box((0.012, 0.014, 0.012)),
        origin=Origin(xyz=(0.020, 0.060, 0.100)),
        material=accent_grey,
        name="right_button_seat",
    )
    motor_body.inertial = Inertial.from_geometry(
        Box((0.090, 0.180, 0.215)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.000, 0.108)),
    )

    trigger = model.part("trigger")
    trigger.visual(
        Box((0.030, 0.015, 0.054)),
        origin=Origin(xyz=(0.0, 0.0075, -0.028)),
        material=switch_grey,
        name="trigger_paddle",
    )
    trigger.visual(
        Box((0.024, 0.015, 0.024)),
        origin=Origin(xyz=(0.0, 0.0075, -0.066)),
        material=switch_grey,
        name="trigger_tip",
    )
    trigger.inertial = Inertial.from_geometry(
        Box((0.032, 0.018, 0.082)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0075, -0.040)),
    )

    selector = model.part("selector")
    selector.visual(
        Box((0.024, 0.004, 0.014)),
        origin=Origin(),
        material=switch_grey,
        name="selector_thumb",
    )
    selector.inertial = Inertial.from_geometry(
        Box((0.024, 0.004, 0.014)),
        mass=0.01,
        origin=Origin(),
    )

    lock_collar = model.part("lock_collar")
    lock_collar.visual(
        Cylinder(radius=0.0185, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=accent_grey,
        name="twist_collar",
    )
    lock_collar.visual(
        Box((0.008, 0.010, 0.006)),
        origin=Origin(xyz=(-0.017, 0.0, -0.008)),
        material=accent_grey,
        name="collar_tab_left",
    )
    lock_collar.visual(
        Box((0.008, 0.010, 0.006)),
        origin=Origin(xyz=(0.017, 0.0, -0.008)),
        material=accent_grey,
        name="collar_tab_right",
    )
    lock_collar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0185, length=0.016),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
    )

    shaft_assembly = model.part("shaft_assembly")
    shaft_assembly.visual(
        Cylinder(radius=0.0115, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=brushed_steel,
        name="coupler_stub",
    )
    shaft_assembly.visual(
        Cylinder(radius=0.010, length=0.186),
        origin=Origin(xyz=(0.0, 0.0, -0.121)),
        material=satin_steel,
        name="shaft_tube",
    )
    shaft_assembly.visual(
        Cylinder(radius=0.017, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.215)),
        material=brushed_steel,
        name="guard_collar",
    )
    shaft_assembly.visual(
        guard_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.243)),
        material=dark_stainless,
        name="guard_ring",
    )
    _add_member(
        shaft_assembly,
        (0.012, 0.0, -0.216),
        (0.014, 0.0, -0.243),
        0.0025,
        brushed_steel,
        "guard_strut_x_pos",
    )
    _add_member(
        shaft_assembly,
        (-0.012, 0.0, -0.216),
        (-0.014, 0.0, -0.243),
        0.0025,
        brushed_steel,
        "guard_strut_x_neg",
    )
    _add_member(
        shaft_assembly,
        (0.0, 0.012, -0.216),
        (0.0, 0.014, -0.243),
        0.0025,
        brushed_steel,
        "guard_strut_y_pos",
    )
    _add_member(
        shaft_assembly,
        (0.0, -0.012, -0.216),
        (0.0, -0.014, -0.243),
        0.0025,
        brushed_steel,
        "guard_strut_y_neg",
    )
    _add_member(
        shaft_assembly,
        (-0.014, 0.0, -0.243),
        (0.014, 0.0, -0.243),
        0.0025,
        brushed_steel,
        "guard_cross_x",
    )
    _add_member(
        shaft_assembly,
        (0.0, -0.014, -0.243),
        (0.0, 0.014, -0.243),
        0.0025,
        brushed_steel,
        "guard_cross_y",
    )
    shaft_assembly.visual(
        Cylinder(radius=0.0045, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.228)),
        material=brushed_steel,
        name="blade_spindle",
    )
    shaft_assembly.visual(
        Box((0.028, 0.004, 0.0022)),
        origin=Origin(xyz=(0.0, 0.0, -0.235), rpy=(0.0, 0.0, math.radians(24.0))),
        material=dark_stainless,
        name="blade_left",
    )
    shaft_assembly.visual(
        Box((0.028, 0.004, 0.0022)),
        origin=Origin(xyz=(0.0, 0.0, -0.235), rpy=(0.0, 0.0, math.radians(114.0))),
        material=dark_stainless,
        name="blade_right",
    )
    shaft_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.260),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, -0.134)),
    )

    left_button = model.part("left_button")
    left_button.visual(
        Box((0.010, 0.014, 0.010)),
        origin=Origin(xyz=(-0.005, 0.0, 0.0)),
        material=switch_grey,
        name="left_button_cap",
    )
    left_button.inertial = Inertial.from_geometry(
        Box((0.010, 0.014, 0.010)),
        mass=0.005,
        origin=Origin(xyz=(-0.005, 0.0, 0.0)),
    )

    right_button = model.part("right_button")
    right_button.visual(
        Box((0.010, 0.014, 0.010)),
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
        material=switch_grey,
        name="right_button_cap",
    )
    right_button.inertial = Inertial.from_geometry(
        Box((0.010, 0.014, 0.010)),
        mass=0.005,
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
    )

    model.articulation(
        "trigger_pull",
        ArticulationType.PRISMATIC,
        parent=motor_body,
        child=trigger,
        origin=Origin(xyz=(0.0, 0.004, 0.119)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.006),
    )
    model.articulation(
        "selector_slide",
        ArticulationType.PRISMATIC,
        parent=motor_body,
        child=selector,
        origin=Origin(xyz=(0.0, -0.062, 0.168)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.08, lower=0.0, upper=0.014),
    )
    model.articulation(
        "collar_twist",
        ArticulationType.REVOLUTE,
        parent=motor_body,
        child=lock_collar,
        origin=Origin(xyz=(0.0, 0.060, 0.091)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=0.35),
    )
    model.articulation(
        "shaft_mount",
        ArticulationType.FIXED,
        parent=lock_collar,
        child=shaft_assembly,
        origin=Origin(),
    )
    model.articulation(
        "left_button_press",
        ArticulationType.PRISMATIC,
        parent=motor_body,
        child=left_button,
        origin=Origin(xyz=(-0.026, 0.060, 0.100)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.04, lower=0.0, upper=0.0025),
    )
    model.articulation(
        "right_button_press",
        ArticulationType.PRISMATIC,
        parent=motor_body,
        child=right_button,
        origin=Origin(xyz=(0.026, 0.060, 0.100)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.04, lower=0.0, upper=0.0025),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    motor_body = object_model.get_part("motor_body")
    trigger = object_model.get_part("trigger")
    selector = object_model.get_part("selector")
    lock_collar = object_model.get_part("lock_collar")
    shaft_assembly = object_model.get_part("shaft_assembly")
    left_button = object_model.get_part("left_button")
    right_button = object_model.get_part("right_button")

    trigger_pull = object_model.get_articulation("trigger_pull")
    selector_slide = object_model.get_articulation("selector_slide")
    collar_twist = object_model.get_articulation("collar_twist")
    left_button_press = object_model.get_articulation("left_button_press")
    right_button_press = object_model.get_articulation("right_button_press")

    grip_shell = motor_body.get_visual("grip_shell")
    trigger_paddle = trigger.get_visual("trigger_paddle")
    switch_track = motor_body.get_visual("switch_track")
    selector_thumb = selector.get_visual("selector_thumb")
    socket_housing = motor_body.get_visual("socket_housing")
    twist_collar = lock_collar.get_visual("twist_collar")
    coupler_stub = shaft_assembly.get_visual("coupler_stub")
    main_housing = motor_body.get_visual("main_housing")
    guard_collar = shaft_assembly.get_visual("guard_collar")
    guard_ring = shaft_assembly.get_visual("guard_ring")
    shaft_tube = shaft_assembly.get_visual("shaft_tube")
    blade_left = shaft_assembly.get_visual("blade_left")
    blade_right = shaft_assembly.get_visual("blade_right")
    left_button_seat = motor_body.get_visual("left_button_seat")
    right_button_seat = motor_body.get_visual("right_button_seat")
    left_button_cap = left_button.get_visual("left_button_cap")
    right_button_cap = right_button.get_visual("right_button_cap")

    ctx.allow_overlap(
        trigger,
        motor_body,
        reason="the trigger nests into a front finger recess in the pistol grip while remaining externally accessible",
    )
    ctx.allow_overlap(
        selector,
        motor_body,
        reason="the speed selector rides in a shallow back-face track rather than floating above the housing",
    )
    ctx.allow_overlap(
        lock_collar,
        motor_body,
        reason="the twist-lock collar sleeves into the front socket for a seated detachable coupling",
    )
    ctx.allow_overlap(
        left_button,
        motor_body,
        reason="left release button depresses into a recessed latch pocket in the collar housing",
    )
    ctx.allow_overlap(
        right_button,
        motor_body,
        reason="right release button depresses into a recessed latch pocket in the collar housing",
    )
    ctx.allow_overlap(
        lock_collar,
        shaft_assembly,
        reason="shaft coupler nests into the twist-lock collar as a detachable keyed connection",
    )
    ctx.allow_overlap(
        shaft_assembly,
        motor_body,
        reason="the detachable shaft's upper coupler seats slightly into the motor socket when locked",
    )

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_overlap(
        trigger,
        motor_body,
        axes="xz",
        min_overlap=0.020,
        elem_a=trigger_paddle,
        elem_b=grip_shell,
    )
    ctx.expect_overlap(
        selector,
        motor_body,
        axes="xz",
        min_overlap=0.012,
        elem_a=selector_thumb,
        elem_b=switch_track,
    )
    ctx.expect_gap(
        motor_body,
        selector,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0025,
        positive_elem=switch_track,
        negative_elem=selector_thumb,
    )
    ctx.expect_gap(
        motor_body,
        selector,
        axis="y",
        min_gap=0.020,
        positive_elem=main_housing,
        negative_elem=selector_thumb,
    )

    ctx.expect_overlap(
        lock_collar,
        motor_body,
        axes="xy",
        min_overlap=0.020,
        elem_a=twist_collar,
        elem_b=socket_housing,
    )
    ctx.expect_gap(
        motor_body,
        lock_collar,
        axis="z",
        max_gap=0.001,
        max_penetration=0.013,
        positive_elem=socket_housing,
        negative_elem=twist_collar,
    )
    ctx.expect_overlap(
        shaft_assembly,
        lock_collar,
        axes="xy",
        min_overlap=0.020,
        elem_a=coupler_stub,
        elem_b=twist_collar,
    )
    ctx.expect_gap(
        lock_collar,
        shaft_assembly,
        axis="z",
        max_gap=0.001,
        max_penetration=0.017,
        positive_elem=twist_collar,
        negative_elem=coupler_stub,
    )

    ctx.expect_gap(
        motor_body,
        shaft_assembly,
        axis="z",
        min_gap=0.26,
        positive_elem=main_housing,
        negative_elem=guard_ring,
    )
    ctx.expect_overlap(
        shaft_assembly,
        shaft_assembly,
        axes="xy",
        min_overlap=0.020,
        elem_a=guard_collar,
        elem_b=guard_ring,
    )
    ctx.expect_gap(
        shaft_assembly,
        shaft_assembly,
        axis="z",
        min_gap=0.025,
        positive_elem=shaft_tube,
        negative_elem=guard_ring,
    )
    ctx.expect_within(
        shaft_assembly,
        shaft_assembly,
        axes="xy",
        inner_elem=blade_left,
        outer_elem=guard_collar,
    )
    ctx.expect_within(
        shaft_assembly,
        shaft_assembly,
        axes="xy",
        inner_elem=blade_right,
        outer_elem=guard_collar,
    )

    ctx.expect_overlap(
        left_button,
        motor_body,
        axes="yz",
        min_overlap=0.010,
        elem_a=left_button_cap,
        elem_b=left_button_seat,
    )
    ctx.expect_gap(
        motor_body,
        left_button,
        axis="x",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem=left_button_seat,
        negative_elem=left_button_cap,
    )
    ctx.expect_overlap(
        right_button,
        motor_body,
        axes="yz",
        min_overlap=0.010,
        elem_a=right_button_cap,
        elem_b=right_button_seat,
    )
    ctx.expect_gap(
        right_button,
        motor_body,
        axis="x",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem=right_button_cap,
        negative_elem=right_button_seat,
    )
    ctx.expect_gap(
        lock_collar,
        left_button,
        axis="x",
        min_gap=0.006,
        max_gap=0.009,
        positive_elem=twist_collar,
        negative_elem=left_button_cap,
    )
    ctx.expect_gap(
        right_button,
        lock_collar,
        axis="x",
        min_gap=0.006,
        max_gap=0.009,
        positive_elem=right_button_cap,
        negative_elem=twist_collar,
    )

    with ctx.pose({trigger_pull: 0.006}):
        ctx.expect_overlap(
            trigger,
            motor_body,
            axes="xz",
            min_overlap=0.020,
            elem_a=trigger_paddle,
            elem_b=grip_shell,
        )
        ctx.expect_gap(
            motor_body,
            trigger,
            axis="y",
            max_gap=0.001,
            max_penetration=0.04,
            positive_elem=grip_shell,
            negative_elem=trigger_paddle,
        )

    with ctx.pose({selector_slide: 0.014}):
        ctx.expect_overlap(
            selector,
            motor_body,
            axes="xz",
            min_overlap=0.012,
            elem_a=selector_thumb,
            elem_b=switch_track,
        )
        ctx.expect_gap(
            motor_body,
            selector,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0025,
            positive_elem=switch_track,
            negative_elem=selector_thumb,
        )
        ctx.expect_gap(
            motor_body,
            selector,
            axis="y",
            min_gap=0.020,
            positive_elem=main_housing,
            negative_elem=selector_thumb,
        )

    with ctx.pose({collar_twist: 0.28}):
        ctx.expect_overlap(
            lock_collar,
            motor_body,
            axes="xy",
            min_overlap=0.020,
            elem_a=twist_collar,
            elem_b=socket_housing,
        )
        ctx.expect_gap(
            motor_body,
            lock_collar,
            axis="z",
            max_gap=0.001,
            max_penetration=0.013,
            positive_elem=socket_housing,
            negative_elem=twist_collar,
        )

    with ctx.pose({left_button_press: 0.0025, right_button_press: 0.0025}):
        ctx.expect_gap(
            motor_body,
            left_button,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0026,
            positive_elem=left_button_seat,
            negative_elem=left_button_cap,
        )
        ctx.expect_gap(
            right_button,
            motor_body,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0026,
            positive_elem=right_button_cap,
            negative_elem=right_button_seat,
        )
        ctx.expect_gap(
            lock_collar,
            left_button,
            axis="x",
            min_gap=0.004,
            max_gap=0.006,
            positive_elem=twist_collar,
            negative_elem=left_button_cap,
        )
        ctx.expect_gap(
            right_button,
            lock_collar,
            axis="x",
            min_gap=0.004,
            max_gap=0.006,
            positive_elem=right_button_cap,
            negative_elem=twist_collar,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
