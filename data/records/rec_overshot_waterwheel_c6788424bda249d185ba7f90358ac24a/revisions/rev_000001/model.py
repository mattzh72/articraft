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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


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


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        (min_x + max_x) * 0.5,
        (min_y + max_y) * 0.5,
        (min_z + max_z) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overshot_waterwheel")

    weathered_oak = model.material("weathered_oak", rgba=(0.46, 0.34, 0.22, 1.0))
    wet_timber = model.material("wet_timber", rgba=(0.34, 0.24, 0.16, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.25, 0.25, 0.27, 1.0))
    gate_steel = model.material("gate_steel", rgba=(0.59, 0.62, 0.64, 1.0))
    stone = model.material("stone", rgba=(0.55, 0.54, 0.51, 1.0))

    wheel_radius = 0.88
    rim_tube = 0.04
    axle_height = 1.20
    wheel_width = 0.24
    rim_side_x = wheel_width * 0.5 - 0.01

    frame = model.part("frame")
    frame.visual(
        Box((0.16, 0.58, 0.14)),
        origin=Origin(xyz=(-0.34, 0.0, 0.07)),
        material=stone,
        name="left_sill",
    )
    frame.visual(
        Box((0.16, 0.58, 0.14)),
        origin=Origin(xyz=(0.34, 0.0, 0.07)),
        material=stone,
        name="right_sill",
    )

    for side_name, x_side in (("left", -0.34), ("right", 0.34)):
        if side_name == "left":
            front_cheek_name = "left_bearing_front_cheek"
            rear_cheek_name = "left_bearing_rear_cheek"
        else:
            front_cheek_name = "right_bearing_front_cheek"
            rear_cheek_name = "right_bearing_rear_cheek"
        frame.visual(
            Box((0.08, 0.08, 2.34)),
            origin=Origin(xyz=(x_side, 0.20, 1.31)),
            material=weathered_oak,
            name=f"{side_name}_front_post",
        )
        frame.visual(
            Box((0.08, 0.08, 2.34)),
            origin=Origin(xyz=(x_side, -0.20, 1.31)),
            material=weathered_oak,
            name=f"{side_name}_rear_post",
        )
        frame.visual(
            Box((0.10, 0.36, 0.08)),
            origin=Origin(xyz=(x_side, 0.0, axle_height - 0.08)),
            material=weathered_oak,
            name=f"{side_name}_bearing_pedestal",
        )
        frame.visual(
            Box((0.10, 0.36, 0.08)),
            origin=Origin(xyz=(x_side, 0.0, axle_height + 0.27)),
            material=weathered_oak,
            name=f"{side_name}_bearing_cap",
        )
        frame.visual(
            Box((0.10, 0.03, 0.16)),
            origin=Origin(xyz=(x_side, 0.050, axle_height + 0.07)),
            material=dark_iron,
            name=front_cheek_name,
        )
        frame.visual(
            Box((0.10, 0.03, 0.16)),
            origin=Origin(xyz=(x_side, -0.050, axle_height + 0.07)),
            material=dark_iron,
            name=rear_cheek_name,
        )
        _add_member(
            frame,
            (x_side, 0.20, 0.20),
            (x_side, 0.02, axle_height + 0.32),
            0.016,
            weathered_oak,
            name=f"{side_name}_front_brace",
        )
        _add_member(
            frame,
            (x_side, -0.20, 0.20),
            (x_side, -0.02, axle_height + 0.32),
            0.016,
            weathered_oak,
            name=f"{side_name}_rear_brace",
        )

    frame.visual(
        Box((0.84, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, -0.16, 2.52)),
        material=weathered_oak,
        name="head_beam",
    )
    frame.visual(
        Box((0.84, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, 0.18, 2.52)),
        material=weathered_oak,
        name="front_tie_beam",
    )
    frame.visual(
        Box((0.84, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, -0.16, 0.54)),
        material=weathered_oak,
        name="mid_tie_beam",
    )
    _add_member(frame, (-0.34, -0.20, 2.38), (0.34, -0.20, 2.38), 0.016, dark_iron)
    _add_member(frame, (-0.34, 0.20, 2.32), (0.34, 0.20, 2.32), 0.016, dark_iron)
    frame.inertial = Inertial.from_geometry(
        Box((0.92, 0.64, 2.70)),
        mass=44.0,
        origin=Origin(xyz=(0.0, 0.0, 1.35)),
    )

    wheel = model.part("wheel")
    rim_mesh = mesh_from_geometry(
        TorusGeometry(wheel_radius, rim_tube, radial_segments=18, tubular_segments=48),
        "waterwheel_rim_ring",
    )
    wheel.visual(
        Cylinder(radius=0.035, length=0.76),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="shaft",
    )
    wheel.visual(
        Cylinder(radius=0.10, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=0.15, length=0.03),
        origin=Origin(xyz=(-0.065, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="left_hub_flange",
    )
    wheel.visual(
        Cylinder(radius=0.15, length=0.03),
        origin=Origin(xyz=(0.065, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="right_hub_flange",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(-rim_side_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wet_timber,
        name="left_rim",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(rim_side_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wet_timber,
        name="right_rim",
    )

    spoke_angles = tuple(2.0 * math.pi * index / 8.0 for index in range(8))
    for x_side in (-rim_side_x + 0.01, rim_side_x - 0.01):
        for angle in spoke_angles:
            inner_point = (x_side, 0.10 * math.sin(angle), 0.10 * math.cos(angle))
            outer_point = (x_side, 0.84 * math.sin(angle), 0.84 * math.cos(angle))
            _add_member(wheel, inner_point, outer_point, 0.018, wet_timber)

    bucket_count = 12
    for index in range(bucket_count):
        angle = 2.0 * math.pi * index / bucket_count
        bucket_name = "bucket_ref" if index == 0 else f"bucket_{index}"
        wheel.visual(
            Box((wheel_width, 0.032, 0.18)),
            origin=Origin(
                xyz=(0.0, 0.82 * math.sin(angle), 0.82 * math.cos(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=wet_timber,
            name=bucket_name,
        )
        wheel.visual(
            Box((wheel_width, 0.10, 0.040)),
            origin=Origin(
                xyz=(0.0, 0.75 * math.sin(angle), 0.75 * math.cos(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=weathered_oak,
            name=f"bucket_floor_{index}",
        )

    wheel.inertial = Inertial.from_geometry(
        Box((0.80, 1.92, 1.92)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    inlet_box = model.part("inlet_box")
    inlet_box.visual(
        Box((0.42, 0.24, 0.018)),
        origin=Origin(xyz=(0.0, -0.07, 0.10)),
        material=weathered_oak,
        name="box_floor",
    )
    inlet_box.visual(
        Box((0.018, 0.24, 0.18)),
        origin=Origin(xyz=(-0.201, -0.07, 0.19)),
        material=weathered_oak,
        name="left_wall",
    )
    inlet_box.visual(
        Box((0.018, 0.24, 0.18)),
        origin=Origin(xyz=(0.201, -0.07, 0.19)),
        material=weathered_oak,
        name="right_wall",
    )
    inlet_box.visual(
        Box((0.384, 0.018, 0.16)),
        origin=Origin(xyz=(0.0, -0.181, 0.18)),
        material=weathered_oak,
        name="back_wall",
    )
    inlet_box.visual(
        Box((0.384, 0.018, 0.07)),
        origin=Origin(xyz=(0.0, 0.041, 0.235)),
        material=weathered_oak,
        name="front_header",
    )
    inlet_box.visual(
        Box((0.018, 0.024, 0.16)),
        origin=Origin(xyz=(-0.164, -0.020, 0.189)),
        material=dark_iron,
        name="left_guide",
    )
    inlet_box.visual(
        Box((0.018, 0.024, 0.16)),
        origin=Origin(xyz=(0.164, -0.020, 0.189)),
        material=dark_iron,
        name="right_guide",
    )
    inlet_box.visual(
        Box((0.33, 0.010, 0.16)),
        origin=Origin(xyz=(0.0, -0.031, 0.189)),
        material=dark_iron,
        name="guide_backer",
    )
    inlet_box.visual(
        Box((0.04, 0.14, 0.06)),
        origin=Origin(xyz=(-0.14, -0.11, 0.25)),
        material=dark_iron,
        name="left_hanger_lug",
    )
    inlet_box.visual(
        Box((0.04, 0.14, 0.06)),
        origin=Origin(xyz=(0.14, -0.11, 0.25)),
        material=dark_iron,
        name="right_hanger_lug",
    )
    inlet_box.visual(
        Cylinder(radius=0.012, length=0.14),
        origin=Origin(xyz=(-0.14, -0.11, 0.35)),
        material=dark_iron,
        name="hanger_left",
    )
    inlet_box.visual(
        Cylinder(radius=0.012, length=0.14),
        origin=Origin(xyz=(0.14, -0.11, 0.35)),
        material=dark_iron,
        name="hanger_right",
    )
    inlet_box.inertial = Inertial.from_geometry(
        Box((0.46, 0.28, 0.56)),
        mass=5.5,
        origin=Origin(xyz=(0.0, -0.07, 0.28)),
    )

    regulator = model.part("regulator_plate")
    regulator.visual(
        Box((0.31, 0.012, 0.10)),
        material=gate_steel,
        name="plate_panel",
    )
    regulator.visual(
        Cylinder(radius=0.006, length=0.030),
        origin=Origin(xyz=(0.0, 0.021, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="handle_stem",
    )
    regulator.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(0.0, 0.042, 0.0)),
        material=dark_iron,
        name="handle_knob",
    )
    regulator.inertial = Inertial.from_geometry(
        Box((0.33, 0.05, 0.12)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, axle_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=2.5),
    )
    model.articulation(
        "frame_to_inlet_box",
        ArticulationType.FIXED,
        parent=frame,
        child=inlet_box,
        origin=Origin(xyz=(0.0, -0.05, 2.06)),
    )
    model.articulation(
        "inlet_box_to_regulator",
        ArticulationType.PRISMATIC,
        parent=inlet_box,
        child=regulator,
        origin=Origin(xyz=(0.0, -0.020, 0.16)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.15,
            lower=0.0,
            upper=0.06,
        ),
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
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    inlet_box = object_model.get_part("inlet_box")
    regulator = object_model.get_part("regulator_plate")

    wheel_joint = object_model.get_articulation("frame_to_wheel")
    regulator_joint = object_model.get_articulation("inlet_box_to_regulator")

    ctx.check(
        "wheel articulation is a continuous horizontal shaft rotation",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(value, 6) for value in wheel_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
    )
    ctx.check(
        "regulator articulation is a short upward slide",
        regulator_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(value, 6) for value in regulator_joint.axis) == (0.0, 0.0, 1.0)
        and regulator_joint.motion_limits is not None
        and regulator_joint.motion_limits.upper is not None
        and 0.04 <= regulator_joint.motion_limits.upper <= 0.08,
        details=f"type={regulator_joint.articulation_type}, axis={regulator_joint.axis}, limits={regulator_joint.motion_limits}",
    )

    ctx.expect_contact(
        wheel,
        frame,
        elem_a="shaft",
        elem_b="left_bearing_front_cheek",
        name="left bearing cheek seats the shaft",
    )
    ctx.expect_contact(
        wheel,
        frame,
        elem_a="shaft",
        elem_b="right_bearing_front_cheek",
        name="right bearing cheek seats the shaft",
    )
    ctx.expect_contact(
        inlet_box,
        frame,
        elem_a="hanger_left",
        elem_b="head_beam",
        name="left hanger suspends the inlet box from the head beam",
    )
    ctx.expect_contact(
        inlet_box,
        frame,
        elem_a="hanger_right",
        elem_b="head_beam",
        name="right hanger suspends the inlet box from the head beam",
    )
    ctx.expect_contact(
        regulator,
        inlet_box,
        elem_a="plate_panel",
        elem_b="guide_backer",
        name="regulator plate bears against the inlet guide backer",
    )
    ctx.expect_within(
        regulator,
        inlet_box,
        axes="x",
        inner_elem="plate_panel",
        outer_elem="guide_backer",
        margin=0.012,
        name="regulator plate stays laterally between the short guides",
    )
    ctx.expect_overlap(
        regulator,
        inlet_box,
        axes="z",
        elem_a="plate_panel",
        elem_b="guide_backer",
        min_overlap=0.09,
        name="regulator plate remains retained vertically in the guide backer",
    )

    bucket_rest_center = _aabb_center(ctx.part_element_world_aabb(wheel, elem="bucket_ref"))
    regulator_rest_pos = ctx.part_world_position(regulator)
    upper = regulator_joint.motion_limits.upper if regulator_joint.motion_limits is not None else 0.0
    with ctx.pose({wheel_joint: math.pi / 6.0, regulator_joint: upper}):
        ctx.expect_contact(
            wheel,
            frame,
            elem_a="shaft",
            elem_b="left_bearing_front_cheek",
            name="wheel stays seated in the left bearing while turning",
        )
        ctx.expect_overlap(
            regulator,
            inlet_box,
            axes="z",
            elem_a="plate_panel",
            elem_b="guide_backer",
            min_overlap=0.09,
            name="opened regulator still retains overlap with the guide backer",
        )
        bucket_turn_center = _aabb_center(ctx.part_element_world_aabb(wheel, elem="bucket_ref"))
        regulator_open_pos = ctx.part_world_position(regulator)

    ctx.check(
        "wheel rotation moves a bucket around the shaft",
        bucket_rest_center is not None
        and bucket_turn_center is not None
        and (
            abs(bucket_turn_center[1] - bucket_rest_center[1]) > 0.30
            or abs(bucket_turn_center[2] - bucket_rest_center[2]) > 0.12
        ),
        details=f"rest={bucket_rest_center}, turned={bucket_turn_center}",
    )
    ctx.check(
        "regulator plate rises when opened",
        regulator_rest_pos is not None
        and regulator_open_pos is not None
        and regulator_open_pos[2] > regulator_rest_pos[2] + 0.05,
        details=f"rest={regulator_rest_pos}, open={regulator_open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
