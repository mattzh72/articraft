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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


CASE_OUTER_WIDTH = 0.332
CASE_DEPTH = 0.146
CASE_BACK_Y = 0.074
CASE_FRONT_Y = -0.072
BASE_THICKNESS = 0.0024
WALL_HEIGHT = 0.0100
PLATE_THICKNESS = 0.0026
PLATE_CENTER_Z = WALL_HEIGHT + (PLATE_THICKNESS * 0.5)
PLATE_TOP_Z = PLATE_CENTER_Z + (PLATE_THICKNESS * 0.5)
PLATE_UNDERSIDE_Z = PLATE_CENTER_Z - (PLATE_THICKNESS * 0.5)

CLUSTER_PLATE_WIDTH = 0.128
CLUSTER_PLATE_DEPTH = 0.118
LEFT_PLATE_CENTER = (-0.078, 0.012)
RIGHT_PLATE_CENTER = (0.078, 0.012)
CLUSTER_YAW = math.radians(13.0)

KEY_HOLE_SIZE = 0.0104
KEY_STEM_SIZE = 0.0094
KEY_STEM_HEIGHT = 0.0100
KEY_FLANGE_SIZE = 0.0156
KEY_FLANGE_HEIGHT = 0.0012
KEY_TRAVEL = 0.0022
KEY_ORIGIN_Z = 0.0144

FOOT_HINGE_Y = 0.0680
FOOT_HINGE_Z = -0.0040
FOOT_KNUCKLE_RADIUS = 0.0030
FOOT_KNUCKLE_LENGTH = 0.0100
FOOT_EAR_LENGTH = 0.0040
FOOT_EAR_RADIUS = 0.0032
FOOT_OPEN_ANGLE = 1.08


def _rect_profile(width: float, depth: float, *, center: tuple[float, float] = (0.0, 0.0)) -> list[tuple[float, float]]:
    cx, cy = center
    half_w = width * 0.5
    half_d = depth * 0.5
    return [
        (cx - half_w, cy - half_d),
        (cx + half_w, cy - half_d),
        (cx + half_w, cy + half_d),
        (cx - half_w, cy + half_d),
    ]


def _case_outer_profile() -> list[tuple[float, float]]:
    return [
        (-0.166, CASE_BACK_Y),
        (0.166, CASE_BACK_Y),
        (0.156, CASE_FRONT_Y),
        (0.056, CASE_FRONT_Y),
        (0.030, -0.036),
        (-0.030, -0.036),
        (-0.056, CASE_FRONT_Y),
        (-0.156, CASE_FRONT_Y),
    ]


def _case_inner_profile() -> list[tuple[float, float]]:
    return [
        (-0.148, 0.060),
        (0.148, 0.060),
        (0.136, -0.040),
        (0.050, -0.040),
        (0.020, -0.006),
        (-0.020, -0.006),
        (-0.050, -0.040),
        (-0.136, -0.040),
    ]


def _cluster_key_specs() -> list[dict[str, object]]:
    specs: list[dict[str, object]] = []
    column_x = (0.030, 0.008, -0.014, -0.036)
    column_y_offsets = (0.000, 0.002, 0.006, 0.010)
    row_y = (-0.006, 0.017, 0.040)
    for row_index, base_y in enumerate(row_y):
        for column_index, local_x in enumerate(column_x):
            specs.append(
                {
                    "name": f"key_{row_index}_{column_index}",
                    "local_x": local_x,
                    "local_y": base_y + column_y_offsets[column_index],
                }
            )
    for thumb_index, (local_x, local_y) in enumerate(((0.053, -0.028), (0.030, -0.039), (0.006, -0.047))):
        specs.append(
            {
                "name": f"thumb_{thumb_index}",
                "local_x": local_x,
                "local_y": local_y,
            }
        )
    return specs


def _all_key_names() -> list[str]:
    names: list[str] = []
    for side in ("left", "right"):
        for spec in _cluster_key_specs():
            names.append(f"{side}_{spec['name']}")
    return names


def _rotate_xy(x: float, y: float, angle: float) -> tuple[float, float]:
    c = math.cos(angle)
    s = math.sin(angle)
    return ((c * x) - (s * y), (s * x) + (c * y))


def _plate_origin(side: str) -> Origin:
    center_x, center_y = LEFT_PLATE_CENTER if side == "left" else RIGHT_PLATE_CENTER
    yaw = CLUSTER_YAW if side == "left" else -CLUSTER_YAW
    return Origin(xyz=(center_x, center_y, PLATE_CENTER_Z), rpy=(0.0, 0.0, yaw))


def _key_world_xy(side: str, local_x: float, local_y: float) -> tuple[float, float]:
    center_x, center_y = LEFT_PLATE_CENTER if side == "left" else RIGHT_PLATE_CENTER
    yaw = CLUSTER_YAW if side == "left" else -CLUSTER_YAW
    mirrored_x = local_x if side == "left" else -local_x
    dx, dy = _rotate_xy(mirrored_x, local_y, yaw)
    return (center_x + dx, center_y + dy)


def _cluster_plate_mesh(side: str):
    hole_profiles: list[list[tuple[float, float]]] = []
    for spec in _cluster_key_specs():
        local_x = float(spec["local_x"])
        local_y = float(spec["local_y"])
        mirrored_x = local_x if side == "left" else -local_x
        hole_profiles.append(_rect_profile(KEY_HOLE_SIZE, KEY_HOLE_SIZE, center=(mirrored_x, local_y)))
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(CLUSTER_PLATE_WIDTH, CLUSTER_PLATE_DEPTH, 0.010, corner_segments=8),
            hole_profiles,
            height=PLATE_THICKNESS,
            center=True,
        ),
        f"{side}_cluster_plate",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="alice_ergonomic_keyboard")

    case_charcoal = model.material("case_charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    deck_black = model.material("deck_black", rgba=(0.09, 0.10, 0.11, 1.0))
    key_ash = model.material("key_ash", rgba=(0.88, 0.89, 0.90, 1.0))
    key_top = model.material("key_top", rgba=(0.95, 0.96, 0.97, 1.0))
    foot_rubber = model.material("foot_rubber", rgba=(0.11, 0.11, 0.12, 1.0))
    bumper_rubber = model.material("bumper_rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    case = model.part("case")

    bottom_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(_case_outer_profile(), BASE_THICKNESS, cap=True, closed=True),
        "case_bottom_plate",
    )
    wall_ring_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _case_outer_profile(),
            [_case_inner_profile()],
            height=WALL_HEIGHT,
            center=False,
        ),
        "case_wall_ring",
    )

    case.visual(bottom_mesh, material=case_charcoal, name="bottom_plate")
    case.visual(wall_ring_mesh, material=case_charcoal, name="wall_ring")
    case.visual(_cluster_plate_mesh("left"), origin=_plate_origin("left"), material=deck_black, name="left_plate")
    case.visual(_cluster_plate_mesh("right"), origin=_plate_origin("right"), material=deck_black, name="right_plate")
    case.visual(
        Box((0.116, 0.016, 0.012)),
        origin=Origin(xyz=(0.000, 0.064, 0.006)),
        material=case_charcoal,
        name="rear_spine",
    )
    case.visual(
        Box((0.060, 0.024, 0.008)),
        origin=Origin(xyz=(0.000, 0.030, 0.004)),
        material=case_charcoal,
        name="center_bridge",
    )
    case.visual(
        Box((0.030, 0.022, 0.008)),
        origin=Origin(xyz=(-0.112, 0.044, 0.004)),
        material=case_charcoal,
        name="left_rear_support",
    )
    case.visual(
        Box((0.030, 0.022, 0.008)),
        origin=Origin(xyz=(0.112, 0.044, 0.004)),
        material=case_charcoal,
        name="right_rear_support",
    )

    hinge_x_positions = {"left": -0.124, "right": 0.124}
    for side, hinge_x in hinge_x_positions.items():
        case.visual(
            Box((0.028, 0.018, 0.007)),
            origin=Origin(xyz=(hinge_x, FOOT_HINGE_Y, -0.0005)),
            material=case_charcoal,
            name=f"{side}_hinge_block",
        )
        for ear_index, ear_offset in enumerate((-0.007, 0.007)):
            case.visual(
                Cylinder(radius=FOOT_EAR_RADIUS, length=FOOT_EAR_LENGTH),
                origin=Origin(
                    xyz=(hinge_x + ear_offset, FOOT_HINGE_Y, FOOT_HINGE_Z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=deck_black,
                name=f"{side}_hinge_ear_{ear_index}",
            )
        for pad_index, (px, py) in enumerate(((hinge_x, 0.060), (hinge_x * 0.88, -0.058))):
            case.visual(
                Box((0.016, 0.010, 0.0018)),
                origin=Origin(xyz=(px, py, 0.0009)),
                material=bumper_rubber,
                name=f"{side}_bumper_{pad_index}",
            )

    case.inertial = Inertial.from_geometry(
        Box((CASE_OUTER_WIDTH, CASE_DEPTH, 0.030)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.001, 0.015)),
    )

    for side in ("left", "right"):
        for spec in _cluster_key_specs():
            key_name = f"{side}_{spec['name']}"
            key_part = model.part(key_name)
            key_part.visual(
                Box((KEY_FLANGE_SIZE, KEY_FLANGE_SIZE, KEY_FLANGE_HEIGHT)),
                origin=Origin(xyz=(0.0, 0.0, -0.0050)),
                material=key_ash,
                name="retention_flange",
            )
            key_part.visual(
                Box((KEY_STEM_SIZE, KEY_STEM_SIZE, KEY_STEM_HEIGHT)),
                origin=Origin(xyz=(0.0, 0.0, 0.0000)),
                material=key_ash,
                name="stem",
            )
            key_part.visual(
                Box((0.0180, 0.0180, 0.0030)),
                origin=Origin(xyz=(0.0, 0.0, 0.0019)),
                material=key_ash,
                name="cap_lower",
            )
            key_part.visual(
                Box((0.0168, 0.0168, 0.0024)),
                origin=Origin(xyz=(0.0, 0.0, 0.0046)),
                material=key_top,
                name="cap_upper",
            )
            key_part.inertial = Inertial.from_geometry(
                Box((0.0180, 0.0180, 0.0120)),
                mass=0.010,
                origin=Origin(xyz=(0.0, 0.0, 0.0015)),
            )

            world_x, world_y = _key_world_xy(side, float(spec["local_x"]), float(spec["local_y"]))
            model.articulation(
                f"case_to_{key_name}",
                ArticulationType.PRISMATIC,
                parent=case,
                child=key_part,
                origin=Origin(xyz=(world_x, world_y, KEY_ORIGIN_Z)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=1.0,
                    velocity=0.08,
                    lower=0.0,
                    upper=KEY_TRAVEL,
                ),
            )

    for side, hinge_x in hinge_x_positions.items():
        foot = model.part(f"{side}_tent_foot")
        foot.visual(
            Cylinder(radius=FOOT_KNUCKLE_RADIUS, length=FOOT_KNUCKLE_LENGTH),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=deck_black,
            name="knuckle",
        )
        foot.visual(
            Box((0.008, 0.018, 0.012)),
            origin=Origin(xyz=(0.0, -0.009, -0.006)),
            material=deck_black,
            name="hanger",
        )
        foot.visual(
            Box((0.012, 0.034, 0.008)),
            origin=Origin(xyz=(0.0, -0.027, -0.012)),
            material=foot_rubber,
            name="leg",
        )
        foot.visual(
            Box((0.018, 0.012, 0.004)),
            origin=Origin(xyz=(0.0, -0.043, -0.017)),
            material=foot_rubber,
            name="pad",
        )
        foot.inertial = Inertial.from_geometry(
            Box((0.018, 0.050, 0.020)),
            mass=0.020,
            origin=Origin(xyz=(0.0, -0.020, -0.008)),
        )
        model.articulation(
            f"case_to_{side}_tent_foot",
            ArticulationType.REVOLUTE,
            parent=case,
            child=foot,
            origin=Origin(xyz=(hinge_x, FOOT_HINGE_Y, FOOT_HINGE_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=2.0,
                lower=0.0,
                upper=FOOT_OPEN_ANGLE,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    case = object_model.get_part("case")
    left_foot = object_model.get_part("left_tent_foot")
    right_foot = object_model.get_part("right_tent_foot")
    left_foot_joint = object_model.get_articulation("case_to_left_tent_foot")
    right_foot_joint = object_model.get_articulation("case_to_right_tent_foot")
    sample_key = object_model.get_part("left_key_1_1")
    sample_key_joint = object_model.get_articulation("case_to_left_key_1_1")

    ctx.check(
        "key_axis_vertical",
        tuple(sample_key_joint.axis) == (0.0, 0.0, -1.0),
        details=f"Expected vertical key travel axis, got {sample_key_joint.axis}.",
    )
    ctx.check(
        "tent_foot_axes_lateral",
        tuple(left_foot_joint.axis) == (1.0, 0.0, 0.0) and tuple(right_foot_joint.axis) == (1.0, 0.0, 0.0),
        details=f"Expected both tent feet to hinge about +X, got {left_foot_joint.axis} and {right_foot_joint.axis}.",
    )

    for key_name in _all_key_names():
        key_part = object_model.get_part(key_name)
        plate_name = "left_plate" if key_name.startswith("left_") else "right_plate"
        ctx.expect_contact(
            key_part,
            case,
            elem_b=plate_name,
            name=f"{key_name}_captured",
        )
        ctx.expect_overlap(
            key_part,
            case,
            elem_b=plate_name,
            axes="xy",
            min_overlap=0.010,
            name=f"{key_name}_seated_in_cluster",
        )

    ctx.expect_contact(left_foot, case, name="left_foot_hinge_contact")
    ctx.expect_contact(right_foot, case, name="right_foot_hinge_contact")

    sample_key_rest = ctx.part_world_position(sample_key)
    if sample_key_rest is None:
        ctx.fail("sample_key_rest_position", "Could not resolve sample key rest position.")
    else:
        with ctx.pose({sample_key_joint: KEY_TRAVEL}):
            sample_key_pressed = ctx.part_world_position(sample_key)
            if sample_key_pressed is None:
                ctx.fail("sample_key_pressed_position", "Could not resolve sample key pressed position.")
            else:
                ctx.check(
                    "sample_key_moves_down",
                    sample_key_pressed[2] < sample_key_rest[2] - 0.0018,
                    details=f"Expected pressed key to move down by about {KEY_TRAVEL:.4f} m, got {sample_key_rest[2] - sample_key_pressed[2]:.4f} m.",
                )
                ctx.expect_overlap(
                    sample_key,
                    case,
                    elem_b="left_plate",
                    axes="xy",
                    min_overlap=0.010,
                    name="sample_key_stays_over_plate",
                )

    left_foot_rest = ctx.part_world_aabb(left_foot)
    right_foot_rest = ctx.part_world_aabb(right_foot)
    with ctx.pose({left_foot_joint: FOOT_OPEN_ANGLE, right_foot_joint: FOOT_OPEN_ANGLE}):
        ctx.expect_contact(left_foot, case, name="left_foot_stays_clipped_open")
        ctx.expect_contact(right_foot, case, name="right_foot_stays_clipped_open")
        left_foot_open = ctx.part_world_aabb(left_foot)
        right_foot_open = ctx.part_world_aabb(right_foot)
        if left_foot_rest is None or left_foot_open is None:
            ctx.fail("left_foot_pose_aabb", "Could not compare left tent foot folded and open poses.")
        else:
            ctx.check(
                "left_foot_swings_down",
                left_foot_open[0][2] < left_foot_rest[0][2] - 0.015,
                details=f"Expected left tent foot to deploy downward, got folded zmin {left_foot_rest[0][2]:.4f} and open zmin {left_foot_open[0][2]:.4f}.",
            )
        if right_foot_rest is None or right_foot_open is None:
            ctx.fail("right_foot_pose_aabb", "Could not compare right tent foot folded and open poses.")
        else:
            ctx.check(
                "right_foot_swings_down",
                right_foot_open[0][2] < right_foot_rest[0][2] - 0.015,
                details=f"Expected right tent foot to deploy downward, got folded zmin {right_foot_rest[0][2]:.4f} and open zmin {right_foot_open[0][2]:.4f}.",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
