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
    TestContext,
    TestReport,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_z_member(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_box_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    size_x: float,
    size_y: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Box((size_x, size_y, _distance(a, b))),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_z_member(a, b)),
        material=material,
        name=name,
    )


def _spreader_open_angle(
    mount_point: tuple[float, float, float], center_point: tuple[float, float, float]
) -> float:
    dy = center_point[1] - mount_point[1]
    dz = center_point[2] - mount_point[2]
    return math.atan2(dy, -dz)


def _add_spreader_visuals(
    part,
    *,
    length: float,
    mount_eye_x: float,
    free_tab_x: float,
    strap_material,
    hardware_material,
) -> None:
    part.visual(
        Box((0.010, 0.005, length)),
        origin=Origin(xyz=(0.0, 0.0, -0.5 * length)),
        material=strap_material,
        name="strap",
    )
    part.visual(
        Cylinder(radius=0.008, length=0.008),
        origin=Origin(
            xyz=(mount_eye_x, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=hardware_material,
        name="mount_eye",
    )
    part.visual(
        Box((0.006, 0.012, 0.028)),
        origin=Origin(xyz=(free_tab_x, 0.0, -length + 0.014)),
        material=strap_material,
        name="free_tab",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_aframe_step_ladder")

    rail_finish = model.material("rail_finish", rgba=(0.78, 0.80, 0.82, 1.0))
    polymer_cap = model.material("polymer_cap", rgba=(0.20, 0.22, 0.24, 1.0))
    rubber_foot = model.material("rubber_foot", rgba=(0.10, 0.11, 0.12, 1.0))
    hardware = model.material("hardware", rgba=(0.56, 0.59, 0.62, 1.0))
    brace_finish = model.material("brace_finish", rgba=(0.64, 0.67, 0.70, 1.0))

    front_frame = model.part("front_frame")
    rear_frame = model.part("rear_frame")
    left_spreader_front = model.part("left_spreader_front")
    left_spreader_rear = model.part("left_spreader_rear")
    right_spreader_front = model.part("right_spreader_front")
    right_spreader_rear = model.part("right_spreader_rear")

    front_top_left = (-0.220, -0.035, -0.030)
    front_top_right = (0.220, -0.035, -0.030)
    front_bottom_left = (-0.275, -0.190, -0.950)
    front_bottom_right = (0.275, -0.190, -0.950)
    rear_top_left = (-0.205, 0.030, -0.030)
    rear_top_right = (0.205, 0.030, -0.030)
    rear_bottom_left = (-0.290, 0.340, -0.930)
    rear_bottom_right = (0.290, 0.340, -0.930)

    left_front_mount = (-0.289, -0.105, -0.500)
    right_front_mount = (0.289, -0.105, -0.500)
    left_rear_mount = (-0.276, 0.135, -0.490)
    right_rear_mount = (0.276, 0.135, -0.490)
    left_front_stop = (-0.289, 0.020, -0.720)
    right_front_stop = (0.289, 0.020, -0.720)
    left_rear_stop = (-0.276, 0.020, -0.720)
    right_rear_stop = (0.276, 0.020, -0.720)

    _add_box_member(
        front_frame,
        front_top_left,
        front_bottom_left,
        size_x=0.048,
        size_y=0.028,
        material=rail_finish,
        name="left_front_rail",
    )
    _add_box_member(
        front_frame,
        front_top_right,
        front_bottom_right,
        size_x=0.048,
        size_y=0.028,
        material=rail_finish,
        name="right_front_rail",
    )
    front_frame.visual(
        Box((0.360, 0.180, 0.040)),
        origin=Origin(xyz=(0.0, -0.040, -0.020)),
        material=polymer_cap,
        name="top_shell",
    )
    front_frame.visual(
        Box((0.360, 0.140, 0.010)),
        origin=Origin(xyz=(0.0, -0.102, 0.005)),
        material=polymer_cap,
        name="drip_cap",
    )
    front_frame.visual(
        Box((0.320, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, -0.160, -0.018)),
        material=polymer_cap,
        name="front_drip_lip",
    )
    front_frame.visual(
        Box((0.014, 0.106, 0.018)),
        origin=Origin(xyz=(-0.153, -0.103, -0.018)),
        material=polymer_cap,
        name="left_drip_lip",
    )
    front_frame.visual(
        Box((0.014, 0.106, 0.018)),
        origin=Origin(xyz=(0.153, -0.103, -0.018)),
        material=polymer_cap,
        name="right_drip_lip",
    )
    for index, z in enumerate((-0.235, -0.445, -0.655), start=1):
        y = -0.073 - (index - 1) * 0.035
        front_frame.visual(
            Box((0.485, 0.115, 0.024)),
            origin=Origin(xyz=(0.0, y, z)),
            material=rail_finish,
            name=f"tread_{index}",
        )
        front_frame.visual(
            Box((0.485, 0.018, 0.016)),
            origin=Origin(xyz=(0.0, y - 0.048, z + 0.004)),
            material=polymer_cap,
            name=f"tread_nosing_{index}",
        )
    front_frame.visual(
        Cylinder(radius=0.020, length=0.024),
        origin=Origin(
            xyz=(-0.246, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=hardware,
        name="left_hinge_sleeve",
    )
    front_frame.visual(
        Cylinder(radius=0.020, length=0.024),
        origin=Origin(
            xyz=(0.246, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=hardware,
        name="right_hinge_sleeve",
    )
    front_frame.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(
            xyz=(-0.259, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=polymer_cap,
        name="left_pivot_cover",
    )
    front_frame.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(
            xyz=(0.259, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=polymer_cap,
        name="right_pivot_cover",
    )
    front_frame.visual(
        Box((0.070, 0.104, 0.080)),
        origin=Origin(xyz=(-0.215, -0.072, -0.020)),
        material=polymer_cap,
        name="left_top_bridge",
    )
    front_frame.visual(
        Box((0.070, 0.104, 0.080)),
        origin=Origin(xyz=(0.215, -0.072, -0.020)),
        material=polymer_cap,
        name="right_top_bridge",
    )
    front_frame.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(
            xyz=(-0.275, left_front_mount[1], left_front_mount[2]),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=hardware,
        name="left_front_spreader_boss",
    )
    front_frame.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(
            xyz=(0.275, right_front_mount[1], right_front_mount[2]),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=hardware,
        name="right_front_spreader_boss",
    )
    front_frame.visual(
        Box((0.026, 0.020, 0.078)),
        origin=Origin(xyz=(-0.262, left_front_mount[1], left_front_mount[2])),
        material=polymer_cap,
        name="left_front_mount_bracket",
    )
    front_frame.visual(
        Box((0.026, 0.020, 0.078)),
        origin=Origin(xyz=(0.262, right_front_mount[1], right_front_mount[2])),
        material=polymer_cap,
        name="right_front_mount_bracket",
    )
    front_frame.visual(
        Box((0.070, 0.085, 0.030)),
        origin=Origin(xyz=(-0.275, -0.190, -0.965)),
        material=rubber_foot,
        name="left_front_foot",
    )
    front_frame.visual(
        Box((0.070, 0.085, 0.030)),
        origin=Origin(xyz=(0.275, -0.190, -0.965)),
        material=rubber_foot,
        name="right_front_foot",
    )
    front_frame.inertial = Inertial.from_geometry(
        Box((0.62, 0.28, 1.02)),
        mass=7.4,
        origin=Origin(xyz=(0.0, -0.10, -0.47)),
    )

    _add_box_member(
        rear_frame,
        rear_top_left,
        rear_bottom_left,
        size_x=0.044,
        size_y=0.026,
        material=rail_finish,
        name="left_rear_rail",
    )
    _add_box_member(
        rear_frame,
        rear_top_right,
        rear_bottom_right,
        size_x=0.044,
        size_y=0.026,
        material=rail_finish,
        name="right_rear_rail",
    )
    rear_frame.visual(
        Box((0.400, 0.034, 0.024)),
        origin=Origin(xyz=(0.0, 0.074, -0.013)),
        material=polymer_cap,
        name="rear_top_cap",
    )
    rear_frame.visual(
        Box((0.028, 0.060, 0.060)),
        origin=Origin(xyz=(-0.202, 0.050, -0.030)),
        material=polymer_cap,
        name="left_cap_web",
    )
    rear_frame.visual(
        Box((0.028, 0.060, 0.060)),
        origin=Origin(xyz=(0.202, 0.050, -0.030)),
        material=polymer_cap,
        name="right_cap_web",
    )
    rear_frame.visual(
        Box((0.455, 0.045, 0.020)),
        origin=Origin(xyz=(0.0, 0.095, -0.145)),
        material=rail_finish,
        name="rear_upper_tie",
    )
    rear_frame.visual(
        Box((0.520, 0.035, 0.020)),
        origin=Origin(xyz=(0.0, 0.150, -0.315)),
        material=rail_finish,
        name="rear_mid_tie",
    )
    rear_frame.visual(
        Box((0.570, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, 0.245, -0.605)),
        material=rail_finish,
        name="rear_lower_tie",
    )
    rear_frame.visual(
        Cylinder(radius=0.020, length=0.024),
        origin=Origin(
            xyz=(-0.222, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=hardware,
        name="left_pivot_knuckle",
    )
    rear_frame.visual(
        Cylinder(radius=0.020, length=0.024),
        origin=Origin(
            xyz=(0.222, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=hardware,
        name="right_pivot_knuckle",
    )
    rear_frame.visual(
        Box((0.022, 0.028, 0.060)),
        origin=Origin(xyz=(-0.212, 0.040, -0.030)),
        material=polymer_cap,
        name="left_pivot_bridge",
    )
    rear_frame.visual(
        Box((0.022, 0.028, 0.060)),
        origin=Origin(xyz=(0.212, 0.040, -0.030)),
        material=polymer_cap,
        name="right_pivot_bridge",
    )
    rear_frame.visual(
        Box((0.022, 0.012, 0.024)),
        origin=Origin(xyz=(-0.212, 0.020, -0.010)),
        material=polymer_cap,
        name="left_pivot_saddle",
    )
    rear_frame.visual(
        Box((0.022, 0.012, 0.024)),
        origin=Origin(xyz=(0.212, 0.020, -0.010)),
        material=polymer_cap,
        name="right_pivot_saddle",
    )
    rear_frame.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(
            xyz=(-0.262, left_rear_mount[1], left_rear_mount[2]),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=hardware,
        name="left_rear_spreader_boss",
    )
    rear_frame.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(
            xyz=(0.262, right_rear_mount[1], right_rear_mount[2]),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=hardware,
        name="right_rear_spreader_boss",
    )
    rear_frame.visual(
        Box((0.024, 0.020, 0.076)),
        origin=Origin(xyz=(-0.255, left_rear_mount[1], left_rear_mount[2])),
        material=polymer_cap,
        name="left_rear_mount_bracket",
    )
    rear_frame.visual(
        Box((0.024, 0.020, 0.076)),
        origin=Origin(xyz=(0.255, right_rear_mount[1], right_rear_mount[2])),
        material=polymer_cap,
        name="right_rear_mount_bracket",
    )
    rear_frame.visual(
        Box((0.020, 0.024, 0.080)),
        origin=Origin(xyz=(-0.244, left_rear_mount[1], left_rear_mount[2])),
        material=polymer_cap,
        name="left_rear_mount_rib",
    )
    rear_frame.visual(
        Box((0.020, 0.024, 0.080)),
        origin=Origin(xyz=(0.244, right_rear_mount[1], right_rear_mount[2])),
        material=polymer_cap,
        name="right_rear_mount_rib",
    )
    rear_frame.visual(
        Box((0.020, 0.060, 0.120)),
        origin=Origin(xyz=(-0.248, left_rear_mount[1] + 0.020, left_rear_mount[2] - 0.010)),
        material=polymer_cap,
        name="left_rear_mount_web",
    )
    rear_frame.visual(
        Box((0.020, 0.060, 0.120)),
        origin=Origin(xyz=(0.248, right_rear_mount[1] + 0.020, right_rear_mount[2] - 0.010)),
        material=polymer_cap,
        name="right_rear_mount_web",
    )
    rear_frame.visual(
        Box((0.070, 0.090, 0.030)),
        origin=Origin(xyz=(-0.290, 0.340, -0.945)),
        material=rubber_foot,
        name="left_rear_foot",
    )
    rear_frame.visual(
        Box((0.070, 0.090, 0.030)),
        origin=Origin(xyz=(0.290, 0.340, -0.945)),
        material=rubber_foot,
        name="right_rear_foot",
    )
    rear_frame.inertial = Inertial.from_geometry(
        Box((0.66, 0.40, 0.98)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.18, -0.46)),
    )

    left_front_length = _distance(left_front_mount, left_front_stop)
    left_rear_length = _distance(left_rear_mount, left_rear_stop)
    right_front_length = _distance(right_front_mount, right_front_stop)
    right_rear_length = _distance(right_rear_mount, right_rear_stop)

    _add_spreader_visuals(
        left_spreader_front,
        length=left_front_length,
        mount_eye_x=0.006,
        free_tab_x=0.004,
        strap_material=brace_finish,
        hardware_material=hardware,
    )
    left_spreader_front.inertial = Inertial.from_geometry(
        Box((0.03, 0.02, left_front_length)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, -0.5 * left_front_length)),
    )
    _add_spreader_visuals(
        left_spreader_rear,
        length=left_rear_length,
        mount_eye_x=0.006,
        free_tab_x=-0.003,
        strap_material=brace_finish,
        hardware_material=hardware,
    )
    left_spreader_rear.inertial = Inertial.from_geometry(
        Box((0.03, 0.02, left_rear_length)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, -0.5 * left_rear_length)),
    )
    _add_spreader_visuals(
        right_spreader_front,
        length=right_front_length,
        mount_eye_x=-0.006,
        free_tab_x=-0.004,
        strap_material=brace_finish,
        hardware_material=hardware,
    )
    right_spreader_front.inertial = Inertial.from_geometry(
        Box((0.03, 0.02, right_front_length)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, -0.5 * right_front_length)),
    )
    _add_spreader_visuals(
        right_spreader_rear,
        length=right_rear_length,
        mount_eye_x=-0.006,
        free_tab_x=0.003,
        strap_material=brace_finish,
        hardware_material=hardware,
    )
    right_spreader_rear.inertial = Inertial.from_geometry(
        Box((0.03, 0.02, right_rear_length)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, -0.5 * right_rear_length)),
    )

    model.articulation(
        "rear_leg_pivot",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_frame,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=-0.50,
            upper=0.08,
        ),
    )
    model.articulation(
        "left_front_spreader_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=left_spreader_front,
        origin=Origin(
            xyz=left_front_mount,
            rpy=(_spreader_open_angle(left_front_mount, left_front_stop), 0.0, 0.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=3.0,
            lower=-0.10,
            upper=2.70,
        ),
    )
    model.articulation(
        "left_rear_spreader_hinge",
        ArticulationType.REVOLUTE,
        parent=rear_frame,
        child=left_spreader_rear,
        origin=Origin(
            xyz=left_rear_mount,
            rpy=(_spreader_open_angle(left_rear_mount, left_rear_stop), 0.0, 0.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=3.0,
            lower=-2.70,
            upper=0.10,
        ),
    )
    model.articulation(
        "right_front_spreader_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=right_spreader_front,
        origin=Origin(
            xyz=right_front_mount,
            rpy=(_spreader_open_angle(right_front_mount, right_front_stop), 0.0, 0.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=3.0,
            lower=-0.10,
            upper=2.70,
        ),
    )
    model.articulation(
        "right_rear_spreader_hinge",
        ArticulationType.REVOLUTE,
        parent=rear_frame,
        child=right_spreader_rear,
        origin=Origin(
            xyz=right_rear_mount,
            rpy=(_spreader_open_angle(right_rear_mount, right_rear_stop), 0.0, 0.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=3.0,
            lower=-2.70,
            upper=0.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    rear_frame = object_model.get_part("rear_frame")
    left_spreader_front = object_model.get_part("left_spreader_front")
    left_spreader_rear = object_model.get_part("left_spreader_rear")
    right_spreader_front = object_model.get_part("right_spreader_front")
    right_spreader_rear = object_model.get_part("right_spreader_rear")

    rear_leg_pivot = object_model.get_articulation("rear_leg_pivot")
    left_front_spreader_hinge = object_model.get_articulation("left_front_spreader_hinge")
    left_rear_spreader_hinge = object_model.get_articulation("left_rear_spreader_hinge")
    right_front_spreader_hinge = object_model.get_articulation("right_front_spreader_hinge")
    right_rear_spreader_hinge = object_model.get_articulation("right_rear_spreader_hinge")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        front_frame,
        rear_frame,
        elem_a="left_hinge_sleeve",
        elem_b="left_pivot_knuckle",
        name="left_top_pivot_is_seated",
    )
    ctx.expect_contact(
        front_frame,
        rear_frame,
        elem_a="right_hinge_sleeve",
        elem_b="right_pivot_knuckle",
        name="right_top_pivot_is_seated",
    )
    ctx.expect_contact(
        left_spreader_front,
        front_frame,
        name="left_front_spreader_is_mounted",
    )
    ctx.expect_contact(
        left_spreader_rear,
        rear_frame,
        name="left_rear_spreader_is_mounted",
    )
    ctx.expect_contact(
        right_spreader_front,
        front_frame,
        name="right_front_spreader_is_mounted",
    )
    ctx.expect_contact(
        right_spreader_rear,
        rear_frame,
        name="right_rear_spreader_is_mounted",
    )
    ctx.expect_contact(
        left_spreader_front,
        left_spreader_rear,
        name="left_spreaders_close_the_open_stop",
    )
    ctx.expect_contact(
        right_spreader_front,
        right_spreader_rear,
        name="right_spreaders_close_the_open_stop",
    )
    ctx.expect_gap(
        rear_frame,
        front_frame,
        axis="y",
        positive_elem="left_rear_foot",
        negative_elem="left_front_foot",
        min_gap=0.32,
        name="open_pose_has_stable_a_frame_stance",
    )
    ctx.expect_overlap(
        front_frame,
        rear_frame,
        axes="yz",
        min_overlap=0.035,
        elem_a="left_hinge_sleeve",
        elem_b="left_pivot_knuckle",
        name="pivot_axis_stays_aligned",
    )

    closed_pose = {
        rear_leg_pivot: -0.34,
        left_front_spreader_hinge: 2.48,
        left_rear_spreader_hinge: -2.45,
        right_front_spreader_hinge: 2.48,
        right_rear_spreader_hinge: -2.45,
    }
    with ctx.pose(closed_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="closed_pose_clears_without_interpenetration")
        ctx.expect_gap(
            rear_frame,
            front_frame,
            axis="y",
            positive_elem="left_rear_foot",
            negative_elem="left_front_foot",
            min_gap=0.0,
            max_gap=0.12,
            name="closed_pose_packs_tightly_without_crossing_feet",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
