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
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _cylinder_origin(a: tuple[float, float, float], b: tuple[float, float, float]) -> Origin:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    horiz = math.hypot(dx, dy)
    pitch = math.atan2(horiz, dz)
    yaw = math.atan2(dy, dx)
    return Origin(xyz=_midpoint(a, b), rpy=(0.0, pitch, yaw))


def _box_beam_origin(a: tuple[float, float, float], b: tuple[float, float, float]) -> Origin:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    horiz = math.hypot(dx, dy)
    pitch = -math.atan2(dz, horiz)
    yaw = math.atan2(dy, dx)
    return Origin(xyz=_midpoint(a, b), rpy=(0.0, pitch, yaw))


def _add_tube(
    part,
    name: str,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
):
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=_cylinder_origin(a, b),
        material=material,
        name=name,
    )


def _add_beam(
    part,
    name: str,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    width: float,
    height: float,
    material,
):
    part.visual(
        Box((_distance(a, b), width, height)),
        origin=_box_beam_origin(a, b),
        material=material,
        name=name,
    )


def _wheel_visual(part, *, rubber, rim_steel, hub_steel) -> None:
    spin_rpy = (math.pi / 2.0, 0.0, 0.0)
    part.visual(
        Cylinder(radius=0.20, length=0.09),
        origin=Origin(rpy=spin_rpy),
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.165, length=0.068),
        origin=Origin(rpy=spin_rpy),
        material=rubber,
        name="tire_sidewall",
    )
    part.visual(
        Cylinder(radius=0.13, length=0.055),
        origin=Origin(rpy=spin_rpy),
        material=rim_steel,
        name="rim_band",
    )
    part.visual(
        Cylinder(radius=0.072, length=0.078),
        origin=Origin(rpy=spin_rpy),
        material=hub_steel,
        name="hub_shell",
    )
    part.visual(
        Cylinder(radius=0.026, length=0.116),
        origin=Origin(rpy=spin_rpy),
        material=hub_steel,
        name="axle_sleeve",
    )
    part.visual(
        Cylinder(radius=0.056, length=0.018),
        origin=Origin(xyz=(0.0, 0.049, 0.0), rpy=spin_rpy),
        material=rim_steel,
        name="left_hub_flange",
    )
    part.visual(
        Cylinder(radius=0.056, length=0.018),
        origin=Origin(xyz=(0.0, -0.049, 0.0), rpy=spin_rpy),
        material=rim_steel,
        name="right_hub_flange",
    )
    for index, angle in enumerate((0.0, math.pi / 3.0, 2.0 * math.pi / 3.0), start=1):
        part.visual(
            Box((0.205, 0.010, 0.016)),
            origin=Origin(rpy=(0.0, angle, 0.0)),
            material=rim_steel,
            name=f"spoke_bar_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_utility_wheelbarrow", assets=ASSETS)

    tray_paint = model.material("tray_paint", rgba=(0.31, 0.39, 0.14, 1.0))
    frame_paint = model.material("frame_paint", rgba=(0.17, 0.18, 0.17, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.06, 0.06, 0.06, 1.0))
    rim_steel = model.material("rim_steel", rgba=(0.64, 0.66, 0.69, 1.0))
    hub_steel = model.material("hub_steel", rgba=(0.29, 0.30, 0.33, 1.0))
    fastener_zinc = model.material("fastener_zinc", rgba=(0.78, 0.80, 0.82, 1.0))
    molded_grip = model.material("molded_grip", rgba=(0.12, 0.12, 0.12, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.55, 0.68, 0.66)),
        mass=18.0,
        origin=Origin(xyz=(0.78, 0.0, 0.30)),
    )

    _add_tube(frame, "left_handle_rail", (0.46, 0.30, 0.42), (1.45, 0.25, 0.58), radius=0.018, material=frame_paint)
    _add_tube(frame, "right_handle_rail", (0.46, -0.30, 0.42), (1.45, -0.25, 0.58), radius=0.018, material=frame_paint)
    _add_tube(frame, "left_rear_leg", (1.03, 0.255, 0.50), (1.18, 0.20, 0.05), radius=0.017, material=frame_paint)
    _add_tube(frame, "right_rear_leg", (1.03, -0.255, 0.50), (1.18, -0.20, 0.05), radius=0.017, material=frame_paint)
    _add_beam(frame, "left_tray_support", (0.47, 0.245, 0.31), (0.96, 0.245, 0.34), width=0.026, height=0.040, material=frame_paint)
    _add_beam(frame, "right_tray_support", (0.47, -0.245, 0.31), (0.96, -0.245, 0.34), width=0.026, height=0.040, material=frame_paint)
    _add_tube(frame, "front_crossmember", (0.47, 0.245, 0.31), (0.47, -0.245, 0.31), radius=0.014, material=frame_paint)
    _add_tube(frame, "mid_crossbrace", (0.72, 0.245, 0.33), (0.72, -0.245, 0.33), radius=0.013, material=frame_paint)
    _add_tube(frame, "rear_crossbrace", (1.10, 0.235, 0.30), (1.10, -0.235, 0.30), radius=0.014, material=frame_paint)
    _add_tube(frame, "left_hinge_bridge", (0.47, 0.245, 0.31), (0.31, 0.287, 0.38), radius=0.014, material=frame_paint)
    _add_tube(frame, "right_hinge_bridge", (0.47, -0.245, 0.31), (0.31, -0.287, 0.38), radius=0.014, material=frame_paint)
    _add_tube(frame, "left_outrigger", (0.47, 0.245, 0.31), (0.46, 0.30, 0.42), radius=0.016, material=frame_paint)
    _add_tube(frame, "right_outrigger", (0.47, -0.245, 0.31), (0.46, -0.30, 0.42), radius=0.016, material=frame_paint)
    _add_tube(frame, "left_fork_brace", (0.47, 0.245, 0.29), (0.25, 0.067, 0.36), radius=0.013, material=frame_paint)
    _add_tube(frame, "right_fork_brace", (0.47, -0.245, 0.29), (0.25, -0.067, 0.36), radius=0.013, material=frame_paint)

    frame.visual(
        Box((0.10, 0.14, 0.06)),
        origin=Origin(xyz=(0.21, 0.0, 0.44)),
        material=frame_paint,
        name="fork_crown",
    )
    _add_beam(frame, "left_fork_arm", (0.24, 0.067, 0.40), (0.15, 0.067, 0.245), width=0.018, height=0.042, material=frame_paint)
    _add_beam(frame, "right_fork_arm", (0.24, -0.067, 0.40), (0.15, -0.067, 0.245), width=0.018, height=0.042, material=frame_paint)
    frame.visual(
        Box((0.030, 0.018, 0.050)),
        origin=Origin(xyz=(0.15, 0.067, 0.220)),
        material=frame_paint,
        name="left_dropout",
    )
    frame.visual(
        Box((0.030, 0.018, 0.050)),
        origin=Origin(xyz=(0.15, -0.067, 0.220)),
        material=frame_paint,
        name="right_dropout",
    )
    frame.visual(
        Box((0.050, 0.050, 0.080)),
        origin=Origin(xyz=(0.47, 0.245, 0.31)),
        material=frame_paint,
        name="left_support_node",
    )
    frame.visual(
        Box((0.050, 0.050, 0.080)),
        origin=Origin(xyz=(0.47, -0.245, 0.31)),
        material=frame_paint,
        name="right_support_node",
    )
    frame.visual(
        Box((0.080, 0.014, 0.080)),
        origin=Origin(xyz=(0.31, 0.287, 0.38)),
        material=frame_paint,
        name="left_hinge_plate",
    )
    frame.visual(
        Box((0.080, 0.014, 0.080)),
        origin=Origin(xyz=(0.31, -0.287, 0.38)),
        material=frame_paint,
        name="right_hinge_plate",
    )
    frame.visual(
        Box((0.14, 0.05, 0.06)),
        origin=Origin(xyz=(1.18, 0.20, 0.03)),
        material=frame_paint,
        name="left_rear_skid",
    )
    frame.visual(
        Box((0.14, 0.05, 0.06)),
        origin=Origin(xyz=(1.18, -0.20, 0.03)),
        material=frame_paint,
        name="right_rear_skid",
    )
    frame.visual(
        Box((0.14, 0.045, 0.022)),
        origin=Origin(xyz=(0.84, 0.210, 0.383)),
        material=frame_paint,
        name="rear_rest_pad_left",
    )
    frame.visual(
        Box((0.14, 0.045, 0.022)),
        origin=Origin(xyz=(0.84, -0.210, 0.383)),
        material=frame_paint,
        name="rear_rest_pad_right",
    )
    frame.visual(
        Box((0.08, 0.03, 0.046)),
        origin=Origin(xyz=(0.84, 0.242, 0.359)),
        material=frame_paint,
        name="left_pad_post",
    )
    frame.visual(
        Box((0.08, 0.03, 0.046)),
        origin=Origin(xyz=(0.84, -0.242, 0.359)),
        material=frame_paint,
        name="right_pad_post",
    )
    frame.visual(
        Box((0.06, 0.05, 0.06)),
        origin=Origin(xyz=(1.03, 0.255, 0.50)),
        material=frame_paint,
        name="left_leg_socket",
    )
    frame.visual(
        Box((0.06, 0.05, 0.06)),
        origin=Origin(xyz=(1.03, -0.255, 0.50)),
        material=frame_paint,
        name="right_leg_socket",
    )
    _add_tube(frame, "left_grip_core", (1.31, 0.25, 0.56), (1.45, 0.25, 0.58), radius=0.022, material=molded_grip)
    _add_tube(frame, "right_grip_core", (1.31, -0.25, 0.56), (1.45, -0.25, 0.58), radius=0.022, material=molded_grip)
    for side_sign, side_name in ((1.0, "left"), (-1.0, "right")):
        y_axle = 0.072 * side_sign
        y_pivot = 0.297 * side_sign
        frame.visual(
            Cylinder(radius=0.036, length=0.010),
            origin=Origin(xyz=(0.15, y_axle, 0.20), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=fastener_zinc,
            name=f"{side_name}_axle_washer",
        )
        frame.visual(
            Cylinder(radius=0.018, length=0.014),
            origin=Origin(xyz=(0.15, (0.081 if side_sign > 0 else -0.081), 0.20), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hub_steel,
            name=f"{side_name}_axle_bolt_head",
        )
        frame.visual(
            Cylinder(radius=0.028, length=0.006),
            origin=Origin(xyz=(0.30, y_pivot, 0.38), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=fastener_zinc,
            name=f"{side_name}_pivot_washer",
        )
        frame.visual(
            Cylinder(radius=0.014, length=0.020),
            origin=Origin(xyz=(0.30, (0.307 if side_sign > 0 else -0.307), 0.38), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hub_steel,
            name=f"{side_name}_pivot_bolt_head",
        )

    tray = model.part("tray")
    tray.inertial = Inertial.from_geometry(
        Box((0.86, 0.56, 0.26)),
        mass=9.5,
        origin=Origin(xyz=(0.36, 0.0, 0.08)),
    )
    tray.visual(
        Box((0.72, 0.46, 0.012)),
        origin=Origin(xyz=(0.36, 0.0, 0.020)),
        material=tray_paint,
        name="tray_floor",
    )
    tray.visual(
        Box((0.68, 0.012, 0.18)),
        origin=Origin(xyz=(0.39, 0.224, 0.104)),
        material=tray_paint,
        name="left_side_panel",
    )
    tray.visual(
        Box((0.68, 0.012, 0.18)),
        origin=Origin(xyz=(0.39, -0.224, 0.104)),
        material=tray_paint,
        name="right_side_panel",
    )
    tray.visual(
        Box((0.05, 0.26, 0.04)),
        origin=Origin(xyz=(0.10, 0.0, 0.046)),
        material=tray_paint,
        name="front_panel",
    )
    tray.visual(
        Box((0.030, 0.40, 0.15)),
        origin=Origin(xyz=(0.705, 0.0, 0.089)),
        material=tray_paint,
        name="rear_panel",
    )
    _add_tube(tray, "left_rim_rail", (0.04, 0.229, 0.194), (0.71, 0.229, 0.194), radius=0.009, material=hub_steel)
    _add_tube(tray, "right_rim_rail", (0.04, -0.229, 0.194), (0.71, -0.229, 0.194), radius=0.009, material=hub_steel)
    _add_tube(tray, "front_rim_rail", (0.10, 0.224, 0.066), (0.10, -0.224, 0.066), radius=0.009, material=hub_steel)
    tray_shell_lower = [
        (0.10, -0.205, 0.028),
        (0.62, -0.215, 0.028),
        (0.71, -0.145, 0.040),
        (0.71, 0.145, 0.040),
        (0.62, 0.215, 0.028),
        (0.10, 0.205, 0.028),
        (0.08, 0.100, 0.050),
        (0.08, -0.100, 0.050),
    ]
    tray_shell_upper = [
        (0.08, -0.238, 0.180),
        (0.66, -0.250, 0.180),
        (0.77, -0.162, 0.168),
        (0.77, 0.162, 0.168),
        (0.66, 0.250, 0.180),
        (0.08, 0.238, 0.180),
        (0.06, 0.118, 0.162),
        (0.06, -0.118, 0.162),
    ]
    tray.visual(
        mesh_from_geometry(
            LoftGeometry([tray_shell_lower, tray_shell_upper], cap=False, closed=True),
            ASSETS.mesh_path("wheelbarrow_tray_shell.obj"),
        ),
        material=tray_paint,
        name="tray_shell",
    )
    tray.visual(
        Box((0.090, 0.014, 0.090)),
        origin=Origin(xyz=(0.055, 0.235, 0.050)),
        material=hub_steel,
        name="left_pivot_plate",
    )
    tray.visual(
        Box((0.090, 0.014, 0.090)),
        origin=Origin(xyz=(0.055, -0.235, 0.050)),
        material=hub_steel,
        name="right_pivot_plate",
    )
    tray.visual(
        Cylinder(radius=0.024, length=0.012),
        origin=Origin(xyz=(0.0, 0.242, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_steel,
        name="left_pivot_boss",
    )
    tray.visual(
        Cylinder(radius=0.024, length=0.012),
        origin=Origin(xyz=(0.0, -0.242, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_steel,
        name="right_pivot_boss",
    )
    tray.visual(
        Box((0.12, 0.12, 0.004)),
        origin=Origin(xyz=(0.13, 0.0, 0.026)),
        material=fastener_zinc,
        name="front_wear_strip",
    )
    for side in ("left", "right"):
        mount_y = 0.229 if side == "left" else -0.229
        for idx, x_pos in enumerate((0.24, 0.48, 0.72), start=1):
            tray.visual(
                Cylinder(radius=0.006, length=0.014),
                origin=Origin(xyz=(x_pos - 0.02, mount_y, 0.14 - 0.03 * idx), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=fastener_zinc,
                name=f"{side}_tray_fastener_{idx}",
            )

    wheel = model.part("front_wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.20, length=0.09),
        mass=4.2,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _wheel_visual(wheel, rubber=rubber_black, rim_steel=rim_steel, hub_steel=hub_steel)

    model.articulation(
        "frame_to_tray",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=tray,
        origin=Origin(xyz=(0.30, 0.0, 0.38)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.4, lower=0.0, upper=0.50),
    )
    model.articulation(
        "frame_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.15, 0.0, 0.20)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    tray = object_model.get_part("tray")
    wheel = object_model.get_part("front_wheel")

    tray_dump = object_model.get_articulation("frame_to_tray")
    wheel_spin = object_model.get_articulation("frame_to_front_wheel")

    rear_rest_pad_left = frame.get_visual("rear_rest_pad_left")
    rear_rest_pad_right = frame.get_visual("rear_rest_pad_right")
    left_fork_arm = frame.get_visual("left_fork_arm")
    right_fork_arm = frame.get_visual("right_fork_arm")
    left_dropout = frame.get_visual("left_dropout")
    right_dropout = frame.get_visual("right_dropout")
    left_axle_washer = frame.get_visual("left_axle_washer")
    right_axle_washer = frame.get_visual("right_axle_washer")
    left_hinge_plate = frame.get_visual("left_hinge_plate")
    right_hinge_plate = frame.get_visual("right_hinge_plate")

    tray_floor = tray.get_visual("tray_floor")
    left_side_panel = tray.get_visual("left_side_panel")
    right_side_panel = tray.get_visual("right_side_panel")
    front_panel = tray.get_visual("front_panel")
    tray_rear_panel = tray.get_visual("rear_panel")
    left_pivot_boss = tray.get_visual("left_pivot_boss")
    right_pivot_boss = tray.get_visual("right_pivot_boss")

    wheel_tire = wheel.get_visual("tire")
    axle_sleeve = wheel.get_visual("axle_sleeve")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        frame,
        wheel,
        elem_a=left_dropout,
        elem_b=axle_sleeve,
        reason="Wheel axle sleeve seats inside the left dropout slot.",
    )
    ctx.allow_overlap(
        frame,
        wheel,
        elem_a=right_dropout,
        elem_b=axle_sleeve,
        reason="Wheel axle sleeve seats inside the right dropout slot.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "wheel_spin_axis_is_lateral",
        wheel_spin.axis == (0.0, 1.0, 0.0),
        details=f"expected (0,1,0), got {wheel_spin.axis}",
    )
    ctx.check(
        "tray_dump_limits_realistic",
        tray_dump.motion_limits is not None
        and tray_dump.motion_limits.lower == 0.0
        and tray_dump.motion_limits.upper is not None
        and 0.45 <= tray_dump.motion_limits.upper <= 0.55,
        details="utility wheelbarrow tray should dump through a substantial but braced tilt range",
    )

    ctx.expect_gap(
        tray,
        frame,
        axis="z",
        positive_elem=tray_floor,
        negative_elem=rear_rest_pad_left,
        min_gap=0.0,
        max_gap=0.001,
        name="left_pad_supports_tray_floor",
    )
    ctx.expect_gap(
        tray,
        frame,
        axis="z",
        positive_elem=tray_floor,
        negative_elem=rear_rest_pad_right,
        min_gap=0.0,
        max_gap=0.001,
        name="right_pad_supports_tray_floor",
    )
    ctx.expect_overlap(frame, tray, axes="xy", elem_a=rear_rest_pad_left, elem_b=tray_floor, min_overlap=0.02, name="left_pad_under_floor")
    ctx.expect_overlap(frame, tray, axes="xy", elem_a=rear_rest_pad_right, elem_b=tray_floor, min_overlap=0.02, name="right_pad_under_floor")
    ctx.expect_gap(
        tray,
        frame,
        axis="z",
        positive_elem=tray_floor,
        negative_elem=rear_rest_pad_left,
        min_gap=0.0,
        max_gap=0.001,
        name="tray_floor_seated_on_left_pad",
    )
    ctx.expect_gap(
        frame,
        wheel,
        axis="y",
        positive_elem=left_fork_arm,
        negative_elem=wheel_tire,
        min_gap=0.010,
        max_gap=0.016,
        name="left_fork_wheel_clearance",
    )
    ctx.expect_gap(
        wheel,
        frame,
        axis="y",
        positive_elem=wheel_tire,
        negative_elem=right_fork_arm,
        min_gap=0.008,
        max_gap=0.016,
        name="right_fork_wheel_clearance",
    )
    ctx.expect_gap(
        frame,
        tray,
        axis="y",
        positive_elem=left_hinge_plate,
        negative_elem=left_pivot_boss,
        min_gap=0.030,
        max_gap=0.0345,
        name="left_hinge_clearance",
    )
    ctx.expect_gap(
        tray,
        frame,
        axis="y",
        positive_elem=right_pivot_boss,
        negative_elem=right_hinge_plate,
        min_gap=0.030,
        max_gap=0.0345,
        name="right_hinge_clearance",
    )
    ctx.expect_overlap(frame, tray, axes="y", elem_a=rear_rest_pad_left, elem_b=left_side_panel, min_overlap=0.012, name="left_pad_tucks_under_sidewall")
    ctx.expect_overlap(frame, tray, axes="y", elem_a=rear_rest_pad_right, elem_b=right_side_panel, min_overlap=0.012, name="right_pad_tucks_under_sidewall")
    ctx.expect_gap(
        tray,
        frame,
        axis="x",
        positive_elem=front_panel,
        negative_elem=left_hinge_plate,
        min_gap=-0.25,
        max_gap=0.15,
        name="front_panel_near_hinge_region",
    )
    ctx.expect_gap(
        frame,
        wheel,
        axis="y",
        positive_elem=left_axle_washer,
        negative_elem=axle_sleeve,
        min_gap=0.008,
        max_gap=0.0105,
        name="left_axle_washer_outboard_gap",
    )
    ctx.expect_gap(
        wheel,
        frame,
        axis="y",
        positive_elem=axle_sleeve,
        negative_elem=right_axle_washer,
        min_gap=0.008,
        max_gap=0.0105,
        name="right_axle_washer_outboard_gap",
    )

    frame_aabb = ctx.part_world_aabb(frame)
    tray_aabb = ctx.part_world_aabb(tray)
    wheel_aabb = ctx.part_world_aabb(wheel)
    if frame_aabb is not None and tray_aabb is not None and wheel_aabb is not None:
        overall_min = (
            min(frame_aabb[0][0], tray_aabb[0][0], wheel_aabb[0][0]),
            min(frame_aabb[0][1], tray_aabb[0][1], wheel_aabb[0][1]),
            min(frame_aabb[0][2], tray_aabb[0][2], wheel_aabb[0][2]),
        )
        overall_max = (
            max(frame_aabb[1][0], tray_aabb[1][0], wheel_aabb[1][0]),
            max(frame_aabb[1][1], tray_aabb[1][1], wheel_aabb[1][1]),
            max(frame_aabb[1][2], tray_aabb[1][2], wheel_aabb[1][2]),
        )
        ctx.check(
            "overall_length_realistic",
            1.35 <= overall_max[0] - overall_min[0] <= 1.65,
            details=f"length was {overall_max[0] - overall_min[0]:.3f} m",
        )
        ctx.check(
            "overall_width_realistic",
            0.55 <= overall_max[1] - overall_min[1] <= 0.80,
            details=f"width was {overall_max[1] - overall_min[1]:.3f} m",
        )
        ctx.check(
            "overall_height_realistic",
            0.55 <= overall_max[2] - overall_min[2] <= 0.95,
            details=f"height was {overall_max[2] - overall_min[2]:.3f} m",
        )

    tray_rear_rest = ctx.part_element_world_aabb(tray, elem=tray_rear_panel)
    limits = tray_dump.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({tray_dump: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="tray_lower_no_overlaps")
            ctx.fail_if_isolated_parts(name="tray_lower_no_floating")
        with ctx.pose({tray_dump: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="tray_upper_no_overlaps")
            ctx.fail_if_isolated_parts(name="tray_upper_no_floating")
            ctx.expect_gap(
                tray,
                wheel,
                axis="z",
                positive_elem=tray_rear_panel,
                negative_elem=wheel_tire,
                min_gap=0.30,
                name="dump_pose_rear_panel_above_wheel",
            )
            tray_rear_dump = ctx.part_element_world_aabb(tray, elem=tray_rear_panel)
            if tray_rear_rest is not None and tray_rear_dump is not None:
                ctx.check(
                    "tray_dump_lifts_rear_wall",
                    tray_rear_dump[1][2] > tray_rear_rest[1][2] + 0.28,
                    details="rear wall should rise significantly in dump pose",
                )

    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="articulated_clearance_sweep")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
