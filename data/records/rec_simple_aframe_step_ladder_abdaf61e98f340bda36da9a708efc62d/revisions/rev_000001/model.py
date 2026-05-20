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


OPEN_REAR_ANGLE = 0.0
CLOSED_REAR_ANGLE = 0.42
CLOSED_UPPER_BRACE_ANGLE = 0.80


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_segment(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _add_box_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    size: tuple[float, float],
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Box((size[0], size[1], _distance(a, b))),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_segment(a, b)),
        material=material,
        name=name,
    )


def _build_upper_spreader(part, *, brace_material, pin_material) -> None:
    part.visual(
        Cylinder(radius=0.007, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_material,
        name="root_knuckle",
    )
    _add_box_member(
        part,
        (0.002, 0.0, -0.002),
        (0.276, 0.0, -0.160),
        size=(0.014, 0.006),
        material=brace_material,
        name="brace_bar",
    )
    part.visual(
        Box((0.024, 0.012, 0.028)),
        origin=Origin(xyz=(0.282, 0.0, -0.166)),
        material=pin_material,
        name="tip_shoe",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.32, 0.03, 0.20)),
        mass=0.22,
        origin=Origin(xyz=(0.15, 0.0, -0.09)),
    )


def _build_lower_spreader(
    part, *, brace_material, pin_material, pad_material
) -> None:
    part.visual(
        Cylinder(radius=0.007, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_material,
        name="root_knuckle",
    )
    _add_box_member(
        part,
        (0.0, 0.0, 0.0),
        (0.165, 0.0, -0.103),
        size=(0.015, 0.010),
        material=brace_material,
        name="brace_bar",
    )
    part.visual(
        Box((0.022, 0.014, 0.024)),
        origin=Origin(xyz=(0.161, 0.0, -0.101)),
        material=pin_material,
        name="tip_shoe",
    )
    part.visual(
        Box((0.014, 0.010, 0.030)),
        origin=Origin(xyz=(0.173, 0.0, -0.112)),
        material=pad_material,
        name="tip_pad",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.20, 0.03, 0.12)),
        mass=0.16,
        origin=Origin(xyz=(0.10, 0.0, -0.05)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_apartment_step_ladder")

    aluminum = model.material("aluminum", rgba=(0.76, 0.79, 0.82, 1.0))
    steel = model.material("steel", rgba=(0.45, 0.47, 0.50, 1.0))
    charcoal = model.material("charcoal", rgba=(0.21, 0.22, 0.24, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.11, 0.12, 1.0))
    polymer = model.material("polymer", rgba=(0.69, 0.72, 0.75, 1.0))

    front_frame = model.part("front_frame")
    for side_name, y in (("left", 0.155), ("right", -0.155)):
        _add_box_member(
            front_frame,
            (0.030, y, 0.045),
            (0.166, y, 0.785),
            size=(0.026, 0.024),
            material=aluminum,
            name=f"front_rail_{side_name}",
        )
        front_frame.visual(
            Box((0.056, 0.046, 0.050)),
            origin=Origin(xyz=(0.026, y, 0.025)),
            material=rubber,
            name=f"front_foot_{side_name}",
        )
    for name, x, z in (
        ("lower_tread", 0.097, 0.190),
        ("middle_tread", 0.134, 0.390),
        ("upper_tread", 0.171, 0.590),
    ):
        front_frame.visual(
            Box((0.096, 0.304, 0.024)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=steel,
            name=name,
        )
        front_frame.visual(
            Box((0.008, 0.300, 0.028)),
            origin=Origin(xyz=(x - 0.042, 0.0, z + 0.002)),
            material=charcoal,
            name=f"{name}_lip",
        )
    front_frame.visual(
        Box((0.160, 0.286, 0.032)),
        origin=Origin(xyz=(0.145, 0.0, 0.742)),
        material=polymer,
        name="top_cap",
    )
    front_frame.visual(
        Box((0.042, 0.320, 0.026)),
        origin=Origin(xyz=(0.164, 0.0, 0.790)),
        material=charcoal,
        name="hinge_deck",
    )
    for name, y in (("left", 0.110), ("right", -0.110)):
        front_frame.visual(
            Cylinder(radius=0.013, length=0.110),
            origin=Origin(
                xyz=(0.178, y, 0.804),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=steel,
            name=f"hinge_barrel_{name}",
        )
        front_frame.visual(
            Box((0.028, 0.052, 0.026)),
            origin=Origin(xyz=(0.164, y, 0.792)),
            material=charcoal,
            name=f"hinge_cheek_{name}",
        )
    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        front_frame.visual(
            Box((0.020, 0.024, 0.060)),
            origin=Origin(xyz=(0.122, 0.215 * side_sign, 0.565)),
            material=charcoal,
            name=f"brace_mount_ear_{side_name}",
        )
        front_frame.visual(
            Box((0.022, 0.038, 0.034)),
            origin=Origin(xyz=(0.121, 0.185 * side_sign, 0.565)),
            material=charcoal,
            name=f"brace_mount_web_{side_name}",
        )
    front_frame.inertial = Inertial.from_geometry(
        Box((0.26, 0.34, 0.86)),
        mass=4.4,
        origin=Origin(xyz=(0.13, 0.0, 0.43)),
    )

    rear_frame = model.part("rear_frame")
    for side_name, y in (("left", 0.185), ("right", -0.185)):
        _add_box_member(
            rear_frame,
            (0.080, y, -0.090),
            (0.420, y, -0.780),
            size=(0.024, 0.022),
            material=aluminum,
            name=f"rear_rail_{side_name}",
        )
        rear_frame.visual(
            Box((0.054, 0.044, 0.034)),
            origin=Origin(xyz=(0.428, y, -0.797)),
            material=rubber,
            name=f"rear_foot_{side_name}",
        )
        side_sign = 1.0 if side_name == "left" else -1.0
        _add_box_member(
            rear_frame,
            (0.000, 0.118 * side_sign, 0.000),
            (0.082, 0.185 * side_sign, -0.090),
            size=(0.018, 0.022),
            material=steel,
            name=f"hinge_strap_{side_name}",
        )
        rear_frame.visual(
            Box((0.030, 0.010, 0.060)),
            origin=Origin(xyz=(0.233, 0.215 if side_name == "left" else -0.215, -0.400)),
            material=charcoal,
            name=f"brace_receiver_{side_name}",
        )
        rear_frame.visual(
            Box((0.028, 0.024, 0.024)),
            origin=Origin(xyz=(0.216, 0.202 if side_name == "left" else -0.202, -0.430)),
            material=charcoal,
            name=f"brace_receiver_web_{side_name}",
        )
    rear_frame.visual(
        Box((0.090, 0.376, 0.020)),
        origin=Origin(xyz=(0.102, 0.0, -0.110)),
        material=polymer,
        name="rear_top_bridge",
    )
    _add_box_member(
        rear_frame,
        (0.262, 0.185, -0.460),
        (0.262, -0.185, -0.460),
        size=(0.018, 0.020),
        material=steel,
        name="rear_cross_slat_upper",
    )
    _add_box_member(
        rear_frame,
        (0.351, 0.185, -0.640),
        (0.351, -0.185, -0.640),
        size=(0.018, 0.020),
        material=steel,
        name="rear_cross_slat_lower",
    )
    rear_frame.inertial = Inertial.from_geometry(
        Box((0.46, 0.40, 0.82)),
        mass=2.5,
        origin=Origin(xyz=(0.23, 0.0, -0.41)),
    )

    left_spreader_upper = model.part("left_spreader_upper")
    _build_upper_spreader(
        left_spreader_upper,
        brace_material=steel,
        pin_material=charcoal,
    )

    right_spreader_upper = model.part("right_spreader_upper")
    _build_upper_spreader(
        right_spreader_upper,
        brace_material=steel,
        pin_material=charcoal,
    )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_frame,
        origin=Origin(xyz=(0.178, 0.0, 0.804)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=1.6,
            lower=OPEN_REAR_ANGLE,
            upper=0.68,
        ),
    )
    model.articulation(
        "left_spreader_upper_joint",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=left_spreader_upper,
        origin=Origin(xyz=(0.130, 0.215, 0.565)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.2,
            lower=0.0,
            upper=1.30,
        ),
    )
    model.articulation(
        "right_spreader_upper_joint",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=right_spreader_upper,
        origin=Origin(xyz=(0.130, -0.215, 0.565)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.2,
            lower=0.0,
            upper=1.30,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    rear_frame = object_model.get_part("rear_frame")
    left_upper = object_model.get_part("left_spreader_upper")
    right_upper = object_model.get_part("right_spreader_upper")
    rear_hinge = object_model.get_articulation("rear_hinge")
    left_upper_joint = object_model.get_articulation("left_spreader_upper_joint")
    right_upper_joint = object_model.get_articulation("right_spreader_upper_joint")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        front_frame,
        rear_frame,
        elem_a="top_cap",
        elem_b="hinge_strap_left",
        reason="left rear hinge strap is enclosed beneath the molded top-cap hinge housing",
    )
    ctx.allow_overlap(
        front_frame,
        rear_frame,
        elem_a="top_cap",
        elem_b="hinge_strap_right",
        reason="right rear hinge strap is enclosed beneath the molded top-cap hinge housing",
    )
    ctx.allow_overlap(
        front_frame,
        rear_frame,
        elem_a="hinge_barrel_left",
        elem_b="hinge_strap_left",
        reason="left hinge strap passes into the left hinge barrel as a real pivot",
    )
    ctx.allow_overlap(
        front_frame,
        rear_frame,
        elem_a="hinge_barrel_right",
        elem_b="hinge_strap_right",
        reason="right hinge strap passes into the right hinge barrel as a real pivot",
    )
    ctx.allow_overlap(
        front_frame,
        rear_frame,
        elem_a="hinge_cheek_left",
        elem_b="hinge_strap_left",
        reason="left hinge cheek wraps around the rear strap as a captured hinge bracket",
    )
    ctx.allow_overlap(
        front_frame,
        rear_frame,
        elem_a="hinge_cheek_right",
        elem_b="hinge_strap_right",
        reason="right hinge cheek wraps around the rear strap as a captured hinge bracket",
    )
    ctx.allow_overlap(
        front_frame,
        rear_frame,
        elem_a="hinge_deck",
        elem_b="hinge_strap_left",
        reason="left hinge strap nests under the hinge deck in the folded region",
    )
    ctx.allow_overlap(
        front_frame,
        rear_frame,
        elem_a="hinge_deck",
        elem_b="hinge_strap_right",
        reason="right hinge strap nests under the hinge deck in the folded region",
    )
    ctx.allow_overlap(
        front_frame,
        left_upper,
        elem_a="brace_mount_ear_left",
        elem_b="root_knuckle",
        reason="left spreader root knuckle passes through the front ear as a real pivot",
    )
    ctx.allow_overlap(
        front_frame,
        right_upper,
        elem_a="brace_mount_ear_right",
        elem_b="root_knuckle",
        reason="right spreader root knuckle passes through the front ear as a real pivot",
    )
    ctx.allow_overlap(
        front_frame,
        left_upper,
        elem_a="brace_mount_ear_left",
        elem_b="brace_bar",
        reason="left spreader bar starts inside the boxed pivot ear before tapering outward",
    )
    ctx.allow_overlap(
        front_frame,
        right_upper,
        elem_a="brace_mount_ear_right",
        elem_b="brace_bar",
        reason="right spreader bar starts inside the boxed pivot ear before tapering outward",
    )
    ctx.allow_overlap(
        left_upper,
        rear_frame,
        elem_a="tip_shoe",
        elem_b="brace_receiver_left",
        reason="left spreader tip seats inside the rear receiver bracket at the open stop",
    )
    ctx.allow_overlap(
        left_upper,
        rear_frame,
        elem_a="brace_bar",
        elem_b="brace_receiver_left",
        reason="left spreader bar enters the receiver throat when the ladder is opened",
    )
    ctx.allow_overlap(
        right_upper,
        rear_frame,
        elem_a="tip_shoe",
        elem_b="brace_receiver_right",
        reason="right spreader tip seats inside the rear receiver bracket at the open stop",
    )
    ctx.allow_overlap(
        right_upper,
        rear_frame,
        elem_a="brace_bar",
        elem_b="brace_receiver_right",
        reason="right spreader bar enters the receiver throat when the ladder is opened",
    )

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
        left_upper,
        name="left_upper_spreader_is_mounted",
    )
    ctx.expect_contact(
        front_frame,
        right_upper,
        name="right_upper_spreader_is_mounted",
    )
    ctx.expect_contact(
        left_upper,
        rear_frame,
        elem_a="tip_shoe",
        elem_b="brace_receiver_left",
        name="left_spreader_sets_open_stop",
    )
    ctx.expect_contact(
        right_upper,
        rear_frame,
        elem_a="tip_shoe",
        elem_b="brace_receiver_right",
        name="right_spreader_sets_open_stop",
    )
    ctx.expect_gap(
        rear_frame,
        front_frame,
        axis="x",
        min_gap=0.30,
        positive_elem="rear_foot_left",
        negative_elem="front_foot_left",
        name="open_pose_has_stable_a_frame_depth",
    )

    with ctx.pose(
        {
            rear_hinge: CLOSED_REAR_ANGLE,
            left_upper_joint: CLOSED_UPPER_BRACE_ANGLE,
            right_upper_joint: CLOSED_UPPER_BRACE_ANGLE,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_closed_pose")
        ctx.expect_gap(
            rear_frame,
            front_frame,
            axis="x",
            min_gap=-0.03,
            max_gap=0.18,
            positive_elem="rear_foot_left",
            negative_elem="front_foot_left",
            name="closed_pose_is_stow_compact",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
