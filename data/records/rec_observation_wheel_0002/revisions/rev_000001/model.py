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
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

AXLE_Z = 0.56
SUPPORT_X = 0.135
BASE_Y = 0.17
RIM_RADIUS = 0.38
RIM_TUBE = 0.012
WHEEL_HALF_WIDTH = 0.06
AXLE_SHAFT_RADIUS = 0.014
AXLE_SHAFT_LENGTH = 0.268
PIVOT_RADIUS = 0.395
NUM_GONDOLAS = 8
ANGLE_OFFSET = -math.pi / 2.0


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _midpoint(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
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
    kwargs = {"material": material}
    if name is not None:
        kwargs["name"] = name
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        **kwargs,
    )


def _wheel_point(radius: float, angle: float, x: float = 0.0) -> tuple[float, float, float]:
    return (x, radius * math.cos(angle), radius * math.sin(angle))


def _tower_leg_point(x: float, foot_y: float, z: float, *, top_z: float = 0.42) -> tuple[float, float, float]:
    t = (z - 0.06) / (top_z - 0.06)
    return (x, foot_y * (1.0 - t), z)


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    low, high = aabb
    return (
        (low[0] + high[0]) * 0.5,
        (low[1] + high[1]) * 0.5,
        (low[2] + high[2]) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="observation_wheel", assets=ASSETS)

    support_paint = model.material("support_paint", rgba=(0.82, 0.84, 0.86, 1.0))
    wheel_white = model.material("wheel_white", rgba=(0.93, 0.94, 0.95, 1.0))
    axle_gray = model.material("axle_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    gondola_red = model.material("gondola_red", rgba=(0.71, 0.16, 0.16, 1.0))
    gondola_cream = model.material("gondola_cream", rgba=(0.95, 0.94, 0.88, 1.0))
    glass = model.material("glass", rgba=(0.70, 0.85, 0.93, 0.35))

    rim_mesh = _save_mesh(
        "observation_wheel_rim.obj",
        TorusGeometry(
            radius=RIM_RADIUS,
            tube=RIM_TUBE,
            radial_segments=18,
            tubular_segments=96,
        ).rotate_y(math.pi / 2.0),
    )

    support_frame = model.part("support_frame")
    support_frame.visual(
        Box((0.40, 0.060, 0.060)),
        origin=Origin(xyz=(0.0, BASE_Y, 0.03)),
        material=support_paint,
        name="front_base_beam",
    )
    support_frame.visual(
        Box((0.40, 0.060, 0.060)),
        origin=Origin(xyz=(0.0, -BASE_Y, 0.03)),
        material=support_paint,
        name="rear_base_beam",
    )
    support_frame.visual(
        Box((0.070, 0.340, 0.060)),
        origin=Origin(xyz=(-SUPPORT_X, 0.0, 0.03)),
        material=support_paint,
        name="left_side_sill",
    )
    support_frame.visual(
        Box((0.070, 0.340, 0.060)),
        origin=Origin(xyz=(SUPPORT_X, 0.0, 0.03)),
        material=support_paint,
        name="right_side_sill",
    )
    support_frame.visual(
        Box((0.20, 0.060, 0.018)),
        origin=Origin(xyz=(0.0, BASE_Y + 0.025, 0.069)),
        material=support_paint,
        name="front_walkway",
    )

    foot_y = BASE_Y - 0.03
    left_top = (-SUPPORT_X, 0.0, 0.44)
    right_top = (SUPPORT_X, 0.0, 0.44)
    left_front_foot = (-SUPPORT_X, foot_y, 0.06)
    left_rear_foot = (-SUPPORT_X, -foot_y, 0.06)
    right_front_foot = (SUPPORT_X, foot_y, 0.06)
    right_rear_foot = (SUPPORT_X, -foot_y, 0.06)
    left_front_mid = _tower_leg_point(-SUPPORT_X, foot_y, 0.26, top_z=0.44)
    left_rear_mid = _tower_leg_point(-SUPPORT_X, -foot_y, 0.26, top_z=0.44)
    right_front_mid = _tower_leg_point(SUPPORT_X, foot_y, 0.26, top_z=0.44)
    right_rear_mid = _tower_leg_point(SUPPORT_X, -foot_y, 0.26, top_z=0.44)
    left_front_tie = _tower_leg_point(-SUPPORT_X, foot_y, 0.16, top_z=0.44)
    left_rear_tie = _tower_leg_point(-SUPPORT_X, -foot_y, 0.16, top_z=0.44)
    right_front_tie = _tower_leg_point(SUPPORT_X, foot_y, 0.16, top_z=0.44)
    right_rear_tie = _tower_leg_point(SUPPORT_X, -foot_y, 0.16, top_z=0.44)

    for start, end, name in [
        (left_front_foot, left_top, "left_front_leg"),
        (left_rear_foot, left_top, "left_rear_leg"),
        (right_front_foot, right_top, "right_front_leg"),
        (right_rear_foot, right_top, "right_rear_leg"),
    ]:
        _add_member(
            support_frame,
            start,
            end,
            radius=0.010,
            material=support_paint,
            name=name,
        )

    for a, b, name in [
        (left_front_foot, left_rear_mid, "left_front_brace"),
        (left_rear_foot, left_front_mid, "left_rear_brace"),
        (right_front_foot, right_rear_mid, "right_front_brace"),
        (right_rear_foot, right_front_mid, "right_rear_brace"),
        (left_front_mid, left_rear_mid, "left_spreader"),
        (right_front_mid, right_rear_mid, "right_spreader"),
        (left_front_tie, right_front_tie, "front_lateral_tie"),
        (left_rear_tie, right_rear_tie, "rear_lateral_tie"),
    ]:
        _add_member(
            support_frame,
            a,
            b,
            radius=0.0075 if "tie" in name or "spreader" in name else 0.0065,
            material=support_paint,
            name=name,
        )

    support_frame.visual(
        Box((0.036, 0.074, 0.120)),
        origin=Origin(xyz=(-0.160, 0.0, 0.50)),
        material=support_paint,
        name="left_headstock",
    )
    support_frame.visual(
        Box((0.036, 0.074, 0.120)),
        origin=Origin(xyz=(0.160, 0.0, 0.50)),
        material=support_paint,
        name="right_headstock",
    )
    support_frame.visual(
        Box((0.032, 0.056, 0.042)),
        origin=Origin(xyz=(-0.139, 0.0, AXLE_Z)),
        material=axle_gray,
        name="left_bearing",
    )
    support_frame.visual(
        Box((0.032, 0.056, 0.042)),
        origin=Origin(xyz=(0.139, 0.0, AXLE_Z)),
        material=axle_gray,
        name="right_bearing",
    )
    support_frame.inertial = Inertial.from_geometry(
        Box((0.46, 0.42, 0.64)),
        mass=52.0,
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
    )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=AXLE_SHAFT_RADIUS, length=AXLE_SHAFT_LENGTH),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=axle_gray,
        name="axle_shaft",
    )
    wheel.visual(
        Cylinder(radius=0.047, length=0.084),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=axle_gray,
        name="hub_barrel",
    )
    wheel.visual(
        Cylinder(radius=0.066, length=0.010),
        origin=Origin(xyz=(WHEEL_HALF_WIDTH - 0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wheel_white,
        name="front_hub_cap",
    )
    wheel.visual(
        Cylinder(radius=0.066, length=0.010),
        origin=Origin(xyz=(-WHEEL_HALF_WIDTH + 0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wheel_white,
        name="rear_hub_cap",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(WHEEL_HALF_WIDTH, 0.0, 0.0)),
        material=wheel_white,
        name="front_rim",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(-WHEEL_HALF_WIDTH, 0.0, 0.0)),
        material=wheel_white,
        name="rear_rim",
    )

    spoke_inner_radius = 0.048
    spoke_outer_radius = RIM_RADIUS - RIM_TUBE
    for side_sign in (-1.0, 1.0):
        plane_x = side_sign * WHEEL_HALF_WIDTH
        side_prefix = "rear" if side_sign < 0.0 else "front"
        for index in range(NUM_GONDOLAS):
            angle = ANGLE_OFFSET + (2.0 * math.pi * index) / NUM_GONDOLAS
            _add_member(
                wheel,
                _wheel_point(spoke_inner_radius, angle, plane_x),
                _wheel_point(spoke_outer_radius, angle, plane_x),
                radius=0.0045,
                material=wheel_white,
                name=f"{side_prefix}_spoke_{index:02d}",
            )

    for index in range(NUM_GONDOLAS):
        angle = ANGLE_OFFSET + (2.0 * math.pi * index) / NUM_GONDOLAS
        rim_join = _wheel_point(RIM_RADIUS - RIM_TUBE * 0.45, angle, 0.0)
        pivot_point = _wheel_point(PIVOT_RADIUS, angle, 0.0)
        rear_cheek = (-(WHEEL_HALF_WIDTH - 0.016), pivot_point[1], pivot_point[2])
        front_cheek = (WHEEL_HALF_WIDTH - 0.016, pivot_point[1], pivot_point[2])
        wheel.visual(
            Cylinder(radius=0.006, length=0.088),
            origin=Origin(xyz=pivot_point, rpy=(0.0, math.pi / 2.0, 0.0)),
            material=axle_gray,
            name=f"pivot_barrel_{index:02d}",
        )
        wheel.visual(
            Box((0.010, 0.024, 0.036)),
            origin=Origin(xyz=rear_cheek),
            material=axle_gray,
            name=f"rear_pivot_cheek_{index:02d}",
        )
        wheel.visual(
            Box((0.010, 0.024, 0.036)),
            origin=Origin(xyz=front_cheek),
            material=axle_gray,
            name=f"front_pivot_cheek_{index:02d}",
        )
        _add_member(
            wheel,
            _wheel_point(RIM_RADIUS - RIM_TUBE * 0.45, angle, -WHEEL_HALF_WIDTH),
            rear_cheek,
            radius=0.0042,
            material=axle_gray,
            name=f"rear_hanger_strut_{index:02d}",
        )
        _add_member(
            wheel,
            _wheel_point(RIM_RADIUS - RIM_TUBE * 0.45, angle, WHEEL_HALF_WIDTH),
            front_cheek,
            radius=0.0042,
            material=axle_gray,
            name=f"front_hanger_strut_{index:02d}",
        )

    for index in range(NUM_GONDOLAS):
        spacer_angle = ANGLE_OFFSET + (2.0 * math.pi * (index + 0.5)) / NUM_GONDOLAS
        spacer_point = _wheel_point(RIM_RADIUS - RIM_TUBE * 0.8, spacer_angle, 0.0)
        _add_member(
            wheel,
            (-WHEEL_HALF_WIDTH, spacer_point[1], spacer_point[2]),
            (WHEEL_HALF_WIDTH, spacer_point[1], spacer_point[2]),
            radius=0.0048,
            material=axle_gray,
            name=f"inter_rim_spacer_{index:02d}",
        )

    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=RIM_RADIUS, length=AXLE_SHAFT_LENGTH),
        mass=22.0,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "support_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=support_frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2),
    )

    for index in range(NUM_GONDOLAS):
        gondola = model.part(f"gondola_{index:02d}")
        gondola.visual(
            Cylinder(radius=0.008, length=0.058),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=axle_gray,
            name="pivot_sleeve",
        )
        _add_member(
            gondola,
            (-0.016, 0.0, -0.008),
            (-0.016, 0.0, -0.062),
            radius=0.0036,
            material=axle_gray,
            name="left_hanger",
        )
        _add_member(
            gondola,
            (0.016, 0.0, -0.008),
            (0.016, 0.0, -0.062),
            radius=0.0036,
            material=axle_gray,
            name="right_hanger",
        )
        gondola.visual(
            Box((0.052, 0.018, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, -0.062)),
            material=axle_gray,
            name="roof_beam",
        )
        gondola.visual(
            Box((0.060, 0.076, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, -0.076)),
            material=gondola_red,
            name="roof_shell",
        )
        gondola.visual(
            Box((0.056, 0.070, 0.070)),
            origin=Origin(xyz=(0.0, 0.0, -0.116)),
            material=gondola_cream,
            name="cabin_body",
        )
        gondola.visual(
            Box((0.050, 0.062, 0.038)),
            origin=Origin(xyz=(0.0, 0.0, -0.112)),
            material=glass,
            name="window_band",
        )
        gondola.visual(
            Box((0.052, 0.066, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.156)),
            material=gondola_red,
            name="floor_pan",
        )
        gondola.inertial = Inertial.from_geometry(
            Box((0.064, 0.078, 0.172)),
            mass=2.8,
            origin=Origin(xyz=(0.0, 0.0, -0.098)),
        )

        angle = ANGLE_OFFSET + (2.0 * math.pi * index) / NUM_GONDOLAS
        model.articulation(
            f"wheel_to_gondola_{index:02d}",
            ArticulationType.REVOLUTE,
            parent=wheel,
            child=gondola,
            origin=Origin(xyz=_wheel_point(PIVOT_RADIUS, angle)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=3.0,
                lower=-math.pi,
                upper=math.pi,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    support_frame = object_model.get_part("support_frame")
    wheel = object_model.get_part("wheel")
    wheel_spin = object_model.get_articulation("support_to_wheel")
    axle_shaft = wheel.get_visual("axle_shaft")
    left_bearing = support_frame.get_visual("left_bearing")
    right_bearing = support_frame.get_visual("right_bearing")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.allow_overlap(
        support_frame,
        wheel,
        reason="The main axle seats inside the left bearing block.",
        elem_a=left_bearing,
        elem_b=axle_shaft,
    )
    ctx.allow_overlap(
        support_frame,
        wheel,
        reason="The main axle seats inside the right bearing block.",
        elem_a=right_bearing,
        elem_b=axle_shaft,
    )
    gondolas = [object_model.get_part(f"gondola_{index:02d}") for index in range(NUM_GONDOLAS)]
    gondola_joints = [
        object_model.get_articulation(f"wheel_to_gondola_{index:02d}") for index in range(NUM_GONDOLAS)
    ]
    for index, gondola in enumerate(gondolas):
        ctx.allow_overlap(
            gondola,
            wheel,
            reason="The gondola pivot sleeve wraps the wheel hanger barrel.",
            elem_a=gondola.get_visual("pivot_sleeve"),
            elem_b=wheel.get_visual(f"pivot_barrel_{index:02d}"),
        )

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.expect_origin_distance(
        wheel,
        support_frame,
        axes="xy",
        max_dist=0.001,
        name="wheel_is_centered_between_support_towers",
    )
    ctx.expect_origin_gap(
        wheel,
        support_frame,
        axis="z",
        min_gap=0.54,
        max_gap=0.58,
        name="wheel_axle_height_above_base",
    )
    ctx.expect_contact(
        wheel,
        support_frame,
        elem_a=axle_shaft,
        elem_b=left_bearing,
        name="wheel_contacts_left_bearing",
    )
    ctx.expect_contact(
        wheel,
        support_frame,
        elem_a=axle_shaft,
        elem_b=right_bearing,
        name="wheel_contacts_right_bearing",
    )

    pivot_barrel_00 = wheel.get_visual("pivot_barrel_00")
    rest_pivot_center = _aabb_center(ctx.part_element_world_aabb(wheel, elem=pivot_barrel_00))
    if rest_pivot_center is None:
        ctx.fail("wheel_pivot_barrel_measurable", "pivot_barrel_00 has no measurable world AABB")
    else:
        ctx.check(
            "wheel_bottom_mount_starts_below_axle",
            abs(rest_pivot_center[1]) < 0.015 and abs(rest_pivot_center[2] - (AXLE_Z - PIVOT_RADIUS)) < 0.02,
            details=f"pivot_barrel_00 center={rest_pivot_center}",
        )

    for index, gondola in enumerate(gondolas):
        sleeve = gondola.get_visual("pivot_sleeve")
        barrel = wheel.get_visual(f"pivot_barrel_{index:02d}")
        ctx.expect_contact(
            gondola,
            wheel,
            elem_a=sleeve,
            elem_b=barrel,
            name=f"gondola_{index:02d}_contacts_wheel_pivot",
        )
        pivot_pos = ctx.part_world_position(gondola)
        cabin_center = _aabb_center(ctx.part_element_world_aabb(gondola, elem=gondola.get_visual("cabin_body")))
        if pivot_pos is None or cabin_center is None:
            ctx.fail(
                f"gondola_{index:02d}_measurable",
                f"pivot_pos={pivot_pos}, cabin_center={cabin_center}",
            )
        else:
            ctx.check(
                f"gondola_{index:02d}_hangs_below_pivot_rest",
                cabin_center[2] < pivot_pos[2] - 0.07 and abs(cabin_center[1] - pivot_pos[1]) < 0.02,
                details=f"pivot={pivot_pos}, cabin_center={cabin_center}",
            )

    with ctx.pose({wheel_spin: math.pi / 2.0}):
        turned_pivot_center = _aabb_center(ctx.part_element_world_aabb(wheel, elem=pivot_barrel_00))
        if turned_pivot_center is None:
            ctx.fail("wheel_quarter_turn_measurable", "pivot_barrel_00 missing in quarter-turn pose")
        else:
            ctx.check(
                "wheel_quarter_turn_moves_bottom_mount_to_side",
                turned_pivot_center[1] > 0.30 and abs(turned_pivot_center[2] - AXLE_Z) < 0.02,
                details=f"turned_center={turned_pivot_center}",
            )

    compensation_pose = {wheel_spin: math.pi / 2.0}
    for gondola_joint in gondola_joints:
        compensation_pose[gondola_joint] = -math.pi / 2.0
    with ctx.pose(compensation_pose):
        for index, gondola in enumerate(gondolas):
            pivot_pos = ctx.part_world_position(gondola)
            cabin_center = _aabb_center(ctx.part_element_world_aabb(gondola, elem=gondola.get_visual("cabin_body")))
            if pivot_pos is None or cabin_center is None:
                ctx.fail(
                    f"gondola_{index:02d}_quarter_turn_measurable",
                    f"pivot_pos={pivot_pos}, cabin_center={cabin_center}",
                )
                continue
            ctx.check(
                f"gondola_{index:02d}_stays_upright_in_quarter_turn",
                cabin_center[2] < pivot_pos[2] - 0.07 and abs(cabin_center[1] - pivot_pos[1]) < 0.02,
                details=f"pivot={pivot_pos}, cabin_center={cabin_center}",
            )
            ctx.expect_contact(
                gondola,
                wheel,
                elem_a=gondola.get_visual("pivot_sleeve"),
                elem_b=wheel.get_visual(f"pivot_barrel_{index:02d}"),
                name=f"gondola_{index:02d}_keeps_pivot_contact_in_quarter_turn",
            )

    opposite_pose = {wheel_spin: -0.65}
    for gondola_joint in gondola_joints:
        opposite_pose[gondola_joint] = 0.65
    with ctx.pose(opposite_pose):
        for index, gondola in enumerate(gondolas):
            pivot_pos = ctx.part_world_position(gondola)
            cabin_center = _aabb_center(ctx.part_element_world_aabb(gondola, elem=gondola.get_visual("cabin_body")))
            if pivot_pos is None or cabin_center is None:
                ctx.fail(
                    f"gondola_{index:02d}_counter_pose_measurable",
                    f"pivot_pos={pivot_pos}, cabin_center={cabin_center}",
                )
                continue
            ctx.check(
                f"gondola_{index:02d}_stays_upright_in_counter_pose",
                cabin_center[2] < pivot_pos[2] - 0.07 and abs(cabin_center[1] - pivot_pos[1]) < 0.02,
                details=f"pivot={pivot_pos}, cabin_center={cabin_center}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
