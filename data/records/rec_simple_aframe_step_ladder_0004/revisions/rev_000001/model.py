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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)

LADDER_HALF_WIDTH = 0.175
FRONT_FOOT_Y = 0.225
FRONT_TOP_Y = 0.050
REAR_HINGE_Y = -0.058
REAR_FOOT_Y = -0.253
FOOT_HEIGHT = 0.032
TOP_CAP_CENTER = (0.0, 0.005, 1.090)
TOP_CAP_HEIGHT = 0.056

STEP_SPECS = (
    ("step_1", 0.155, 0.255),
    ("step_2", 0.115, 0.500),
    ("step_3", 0.075, 0.745),
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _lerp_point(
    a: tuple[float, float, float], b: tuple[float, float, float], t: float
) -> tuple[float, float, float]:
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    )


def _rpy_for_z_member(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(xy, dz)
    return (0.0, pitch, yaw)


def _add_box_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    width: float,
    depth: float,
    material,
    name: str | None = None,
):
    return part.visual(
        Box((width, depth, _distance(a, b))),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_z_member(a, b)),
        material=material,
        name=name,
    )


def _save_mesh(geometry, filename: str):
    ASSETS.mesh_dir.mkdir(parents=True, exist_ok=True)
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _build_top_cap_shell():
    outer = ExtrudeGeometry(
        rounded_rect_profile(0.336, 0.196, 0.024, corner_segments=8),
        TOP_CAP_HEIGHT,
        center=True,
    )
    inner = ExtrudeGeometry(
        rounded_rect_profile(0.252, 0.102, 0.016, corner_segments=8),
        0.046,
        center=True,
    ).translate(0.0, 0.005, 0.016)
    return boolean_difference(outer, inner)


def _build_tray_insert():
    return ExtrudeGeometry(
        rounded_rect_profile(0.246, 0.096, 0.014, corner_segments=6),
        0.003,
        center=True,
    )


def _build_tread_mesh():
    return ExtrudeGeometry(
        rounded_rect_profile(0.288, 0.086, 0.008, corner_segments=6),
        0.022,
        center=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_aframe_step_ladder", assets=ASSETS)

    satin_aluminum = model.material("satin_aluminum", rgba=(0.72, 0.75, 0.78, 1.0))
    satin_hardware = model.material("satin_hardware", rgba=(0.56, 0.59, 0.62, 1.0))
    matte_graphite = model.material("matte_graphite", rgba=(0.19, 0.20, 0.22, 1.0))
    matte_charcoal = model.material("matte_charcoal", rgba=(0.14, 0.15, 0.16, 1.0))
    matte_rubber = model.material("matte_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    satin_insert = model.material("satin_insert", rgba=(0.47, 0.50, 0.54, 1.0))

    top_cap_shell_mesh = _save_mesh(_build_top_cap_shell(), "aframe_top_cap_shell.obj")
    tray_insert_mesh = _save_mesh(_build_tray_insert(), "aframe_top_cap_insert.obj")
    tread_mesh = _save_mesh(_build_tread_mesh(), "aframe_step_tread.obj")

    front_frame = model.part("front_frame")
    left_front_bottom = (-LADDER_HALF_WIDTH, FRONT_FOOT_Y, FOOT_HEIGHT)
    right_front_bottom = (LADDER_HALF_WIDTH, FRONT_FOOT_Y, FOOT_HEIGHT)
    left_front_top = (-LADDER_HALF_WIDTH, FRONT_TOP_Y, 1.052)
    right_front_top = (LADDER_HALF_WIDTH, FRONT_TOP_Y, 1.052)

    _add_box_member(
        front_frame,
        left_front_bottom,
        left_front_top,
        width=0.032,
        depth=0.060,
        material=satin_aluminum,
        name="left_rail",
    )
    _add_box_member(
        front_frame,
        right_front_bottom,
        right_front_top,
        width=0.032,
        depth=0.060,
        material=satin_aluminum,
        name="right_rail",
    )
    front_frame.visual(
        Box((0.300, 0.052, 0.020)),
        origin=Origin(xyz=(0.0, 0.050, 1.052)),
        material=satin_aluminum,
        name="crown_bridge",
    )
    front_frame.visual(
        Box((0.042, 0.042, 0.054)),
        origin=Origin(xyz=(-0.132, 0.053, 1.026)),
        material=satin_aluminum,
    )
    front_frame.visual(
        Box((0.042, 0.042, 0.054)),
        origin=Origin(xyz=(0.132, 0.053, 1.026)),
        material=satin_aluminum,
    )
    front_frame.visual(
        Box((0.048, 0.072, FOOT_HEIGHT)),
        origin=Origin(xyz=(-LADDER_HALF_WIDTH, FRONT_FOOT_Y + 0.013, FOOT_HEIGHT * 0.5)),
        material=matte_rubber,
        name="left_front_foot",
    )
    front_frame.visual(
        Box((0.048, 0.072, FOOT_HEIGHT)),
        origin=Origin(xyz=(LADDER_HALF_WIDTH, FRONT_FOOT_Y + 0.013, FOOT_HEIGHT * 0.5)),
        material=matte_rubber,
        name="right_front_foot",
    )

    spreader_pivot_x = 0.150
    spreader_pivot_y = 0.124
    spreader_pivot_z = 0.625
    front_frame.visual(
        Box((0.018, 0.024, 0.020)),
        origin=Origin(xyz=(-spreader_pivot_x, spreader_pivot_y, spreader_pivot_z)),
        material=matte_graphite,
        name="left_spreader_pad",
    )
    front_frame.visual(
        Box((0.018, 0.024, 0.020)),
        origin=Origin(xyz=(spreader_pivot_x, spreader_pivot_y, spreader_pivot_z)),
        material=matte_graphite,
        name="right_spreader_pad",
    )

    for step_name, step_y, step_z in STEP_SPECS:
        left_name = f"{step_name}_left_seat"
        right_name = f"{step_name}_right_seat"
        front_frame.visual(
            Box((0.014, 0.040, 0.018)),
            origin=Origin(xyz=(-0.151, step_y, step_z - 0.020)),
            material=satin_aluminum,
            name=left_name,
        )
        front_frame.visual(
            Box((0.014, 0.040, 0.018)),
            origin=Origin(xyz=(0.151, step_y, step_z - 0.020)),
            material=satin_aluminum,
            name=right_name,
        )

    front_frame.inertial = Inertial.from_geometry(
        Box((0.40, 0.28, 1.10)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.125, 0.55)),
    )

    top_cap = model.part("top_cap")
    top_cap.visual(top_cap_shell_mesh, material=matte_graphite, name="cap_shell")
    top_cap.visual(
        tray_insert_mesh,
        origin=Origin(xyz=(0.0, 0.005, -0.007)),
        material=satin_insert,
        name="tray_insert",
    )
    top_cap.visual(
        Box((0.286, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, -0.072, -0.022)),
        material=matte_graphite,
        name="rear_hinge_seat",
    )
    top_cap.visual(
        Box((0.214, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, 0.048, -0.0065)),
        material=satin_insert,
        name="front_accent",
    )
    top_cap.inertial = Inertial.from_geometry(
        Box((0.336, 0.196, TOP_CAP_HEIGHT)),
        mass=1.2,
    )

    rear_frame = model.part("rear_frame")
    left_rear_top = (-0.165, 0.0, -0.005)
    right_rear_top = (0.165, 0.0, -0.005)
    left_rear_bottom = (-0.165, REAR_FOOT_Y - REAR_HINGE_Y, FOOT_HEIGHT - 1.062)
    right_rear_bottom = (0.165, REAR_FOOT_Y - REAR_HINGE_Y, FOOT_HEIGHT - 1.062)
    _add_box_member(
        rear_frame,
        left_rear_top,
        left_rear_bottom,
        width=0.028,
        depth=0.048,
        material=satin_aluminum,
        name="left_rail",
    )
    _add_box_member(
        rear_frame,
        right_rear_top,
        right_rear_bottom,
        width=0.028,
        depth=0.048,
        material=satin_aluminum,
        name="right_rail",
    )
    rear_frame.visual(
        Box((0.288, 0.022, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=satin_aluminum,
        name="upper_shoe",
    )
    upper_crossbar_left = _lerp_point(left_rear_top, left_rear_bottom, 0.26)
    upper_crossbar_right = _lerp_point(right_rear_top, right_rear_bottom, 0.26)
    lower_crossbar_left = _lerp_point(left_rear_top, left_rear_bottom, 0.64)
    lower_crossbar_right = _lerp_point(right_rear_top, right_rear_bottom, 0.64)
    _add_box_member(
        rear_frame,
        upper_crossbar_left,
        upper_crossbar_right,
        width=0.022,
        depth=0.026,
        material=satin_aluminum,
        name="upper_crossbar",
    )
    _add_box_member(
        rear_frame,
        lower_crossbar_left,
        lower_crossbar_right,
        width=0.018,
        depth=0.022,
        material=matte_charcoal,
        name="lower_crossbar",
    )
    rear_frame.visual(
        Box((0.046, 0.066, FOOT_HEIGHT)),
        origin=Origin(xyz=(-0.165, REAR_FOOT_Y - REAR_HINGE_Y - 0.003, FOOT_HEIGHT * 0.5 - 1.062)),
        material=matte_rubber,
        name="left_rear_foot",
    )
    rear_frame.visual(
        Box((0.046, 0.066, FOOT_HEIGHT)),
        origin=Origin(xyz=(0.165, REAR_FOOT_Y - REAR_HINGE_Y - 0.003, FOOT_HEIGHT * 0.5 - 1.062)),
        material=matte_rubber,
        name="right_rear_foot",
    )
    rear_frame.visual(
        Box((0.018, 0.024, 0.020)),
        origin=Origin(xyz=(-0.146, -0.076, -0.497)),
        material=matte_graphite,
        name="left_spreader_pad",
    )
    rear_frame.visual(
        Box((0.018, 0.024, 0.020)),
        origin=Origin(xyz=(0.146, -0.076, -0.497)),
        material=matte_graphite,
        name="right_spreader_pad",
    )
    rear_frame.inertial = Inertial.from_geometry(
        Box((0.36, 0.24, 1.08)),
        mass=3.4,
        origin=Origin(xyz=(0.0, -0.120, -0.540)),
    )

    for step_name, step_y, step_z in STEP_SPECS:
        step_part = model.part(step_name)
        step_part.visual(
            Box((0.288, 0.086, 0.022)),
            origin=Origin(),
            material=satin_aluminum,
            name="deck_core",
        )
        step_part.visual(tread_mesh, material=satin_aluminum, name="deck")
        for strip_y in (-0.024, -0.008, 0.008, 0.024):
            step_part.visual(
                Box((0.254, 0.008, 0.003)),
                origin=Origin(xyz=(0.0, strip_y, 0.0125)),
                material=matte_charcoal,
            )
        step_part.visual(
            Box((0.288, 0.010, 0.006)),
            origin=Origin(xyz=(0.0, 0.038, 0.001)),
            material=satin_hardware,
            name="front_nosing",
        )
        step_part.inertial = Inertial.from_geometry(
            Box((0.288, 0.086, 0.025)),
            mass=0.65,
        )
        model.articulation(
            f"front_frame_to_{step_name}",
            ArticulationType.FIXED,
            parent=front_frame,
            child=step_part,
            origin=Origin(xyz=(0.0, step_y, step_z)),
        )

    spreader_front_vector = (0.0, -0.130, -0.040)
    spreader_rear_vector = (0.0, -0.128, -0.020)
    for side_name, side_x in (("left", -spreader_pivot_x), ("right", spreader_pivot_x)):
        front_spreader = model.part(f"{side_name}_spreader_front")
        _add_box_member(
            front_spreader,
            (0.0, 0.0, 0.0),
            spreader_front_vector,
            width=0.016,
            depth=0.004,
            material=matte_graphite,
            name="bar",
        )
        front_spreader.visual(
            Box((0.018, 0.016, 0.004)),
            origin=Origin(xyz=(0.0, -0.006, 0.0)),
            material=satin_hardware,
            name="front_mount_tab",
        )
        front_spreader.visual(
            Box((0.018, 0.016, 0.004)),
            origin=Origin(xyz=(0.0, spreader_front_vector[1] + 0.006, spreader_front_vector[2])),
            material=satin_hardware,
            name="rear_knuckle_tab",
        )
        front_spreader.inertial = Inertial.from_geometry(
            Box((0.020, 0.155, 0.050)),
            mass=0.16,
            origin=Origin(xyz=(0.0, -0.065, -0.020)),
        )

        rear_spreader = model.part(f"{side_name}_spreader_rear")
        _add_box_member(
            rear_spreader,
            (0.0, 0.0, 0.0),
            spreader_rear_vector,
            width=0.016,
            depth=0.004,
            material=matte_graphite,
            name="bar",
        )
        rear_spreader.visual(
            Box((0.018, 0.016, 0.004)),
            origin=Origin(xyz=(0.0, 0.006, 0.0)),
            material=satin_hardware,
            name="front_knuckle_tab",
        )
        rear_spreader.visual(
            Box((0.018, 0.016, 0.004)),
            origin=Origin(xyz=(0.0, spreader_rear_vector[1] - 0.006, spreader_rear_vector[2])),
            material=satin_hardware,
            name="rear_end_tab",
        )
        rear_spreader.inertial = Inertial.from_geometry(
            Box((0.020, 0.150, 0.040)),
            mass=0.14,
            origin=Origin(xyz=(0.0, -0.064, -0.010)),
        )

        model.articulation(
            f"{side_name}_spreader_pivot",
            ArticulationType.REVOLUTE,
            parent=front_frame,
            child=front_spreader,
            origin=Origin(xyz=(side_x, 0.112, spreader_pivot_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=2.0,
                lower=-0.10,
                upper=0.90,
            ),
        )
        model.articulation(
            f"{side_name}_spreader_knuckle",
            ArticulationType.REVOLUTE,
            parent=front_spreader,
            child=rear_spreader,
            origin=Origin(xyz=spreader_front_vector),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=2.5,
                lower=-0.15,
                upper=1.30,
            ),
        )

    model.articulation(
        "front_frame_to_top_cap",
        ArticulationType.FIXED,
        parent=front_frame,
        child=top_cap,
        origin=Origin(xyz=TOP_CAP_CENTER),
    )
    model.articulation(
        "rear_frame_hinge",
        ArticulationType.REVOLUTE,
        parent=top_cap,
        child=rear_frame,
        origin=Origin(xyz=(0.0, -0.072, -0.028)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(64.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    front_frame = object_model.get_part("front_frame")
    top_cap = object_model.get_part("top_cap")
    rear_frame = object_model.get_part("rear_frame")
    step_1 = object_model.get_part("step_1")
    step_2 = object_model.get_part("step_2")
    step_3 = object_model.get_part("step_3")
    left_spreader_front = object_model.get_part("left_spreader_front")
    left_spreader_rear = object_model.get_part("left_spreader_rear")
    right_spreader_front = object_model.get_part("right_spreader_front")
    right_spreader_rear = object_model.get_part("right_spreader_rear")

    rear_frame_hinge = object_model.get_articulation("rear_frame_hinge")
    left_spreader_pivot = object_model.get_articulation("left_spreader_pivot")
    right_spreader_pivot = object_model.get_articulation("right_spreader_pivot")
    left_spreader_knuckle = object_model.get_articulation("left_spreader_knuckle")
    right_spreader_knuckle = object_model.get_articulation("right_spreader_knuckle")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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

    ctx.expect_contact(top_cap, front_frame)
    ctx.expect_contact(top_cap, rear_frame)
    ctx.expect_contact(step_1, front_frame)
    ctx.expect_contact(step_2, front_frame)
    ctx.expect_contact(step_3, front_frame)
    ctx.expect_contact(left_spreader_front, front_frame)
    ctx.expect_contact(right_spreader_front, front_frame)
    ctx.expect_contact(left_spreader_front, left_spreader_rear)
    ctx.expect_contact(right_spreader_front, right_spreader_rear)
    ctx.expect_contact(left_spreader_rear, rear_frame)
    ctx.expect_contact(right_spreader_rear, rear_frame)

    ctx.expect_within(step_1, front_frame, axes="x", margin=0.04)
    ctx.expect_within(step_2, front_frame, axes="x", margin=0.04)
    ctx.expect_within(step_3, front_frame, axes="x", margin=0.04)
    ctx.expect_overlap(step_1, front_frame, axes="x", min_overlap=0.28)
    ctx.expect_overlap(step_2, front_frame, axes="x", min_overlap=0.28)
    ctx.expect_overlap(step_3, front_frame, axes="x", min_overlap=0.28)

    ctx.check(
        "rear hinge axis is ladder-width axis",
        tuple(rear_frame_hinge.axis) == (1.0, 0.0, 0.0),
        f"axis={rear_frame_hinge.axis}",
    )
    ctx.check(
        "spreader axes are ladder-width axis",
        tuple(left_spreader_pivot.axis) == (1.0, 0.0, 0.0)
        and tuple(right_spreader_pivot.axis) == (1.0, 0.0, 0.0)
        and tuple(left_spreader_knuckle.axis) == (1.0, 0.0, 0.0)
        and tuple(right_spreader_knuckle.axis) == (1.0, 0.0, 0.0),
        "all spreader pivots and knuckles should hinge around x",
    )

    front_aabb = ctx.part_world_aabb(front_frame)
    top_cap_aabb = ctx.part_world_aabb(top_cap)
    rear_aabb = ctx.part_world_aabb(rear_frame)
    step_1_aabb = ctx.part_world_aabb(step_1)
    step_2_aabb = ctx.part_world_aabb(step_2)
    step_3_aabb = ctx.part_world_aabb(step_3)
    step_1_pos = ctx.part_world_position(step_1)
    step_2_pos = ctx.part_world_position(step_2)
    step_3_pos = ctx.part_world_position(step_3)

    grounded = (
        front_aabb is not None
        and rear_aabb is not None
        and abs(front_aabb[0][2]) <= 0.002
        and abs(rear_aabb[0][2]) <= 0.002
    )
    ctx.check(
        "feet sit on the ground plane",
        grounded,
        f"front_min_z={None if front_aabb is None else front_aabb[0][2]}, "
        f"rear_min_z={None if rear_aabb is None else rear_aabb[0][2]}",
    )

    overall_height_ok = (
        front_aabb is not None
        and top_cap_aabb is not None
        and rear_aabb is not None
        and 1.06 <= max(front_aabb[1][2], top_cap_aabb[1][2], rear_aabb[1][2]) <= 1.16
    )
    ctx.check(
        "overall ladder height is realistic",
        overall_height_ok,
        "expected a compact household step ladder around 1.1 m tall",
    )

    stance_depth_ok = (
        front_aabb is not None
        and rear_aabb is not None
        and 0.50 <= (front_aabb[1][1] - rear_aabb[0][1]) <= 0.60
    )
    ctx.check(
        "open stance depth is realistic",
        stance_depth_ok,
        f"stance_depth={None if front_aabb is None or rear_aabb is None else front_aabb[1][1] - rear_aabb[0][1]}",
    )

    practical_tread_ok = (
        step_2_aabb is not None
        and (step_2_aabb[1][0] - step_2_aabb[0][0]) >= 0.285
        and (step_2_aabb[1][1] - step_2_aabb[0][1]) >= 0.086
    )
    ctx.check(
        "treads are sized for real use",
        practical_tread_ok,
        f"step_2_dims={None if step_2_aabb is None else (step_2_aabb[1][0] - step_2_aabb[0][0], step_2_aabb[1][1] - step_2_aabb[0][1], step_2_aabb[1][2] - step_2_aabb[0][2])}",
    )

    step_spacing_ok = (
        step_1_pos is not None
        and step_2_pos is not None
        and step_3_pos is not None
        and 0.21 <= (step_2_pos[2] - step_1_pos[2]) <= 0.27
        and 0.21 <= (step_3_pos[2] - step_2_pos[2]) <= 0.27
        and step_1_pos[1] > step_2_pos[1] > step_3_pos[1]
    )
    ctx.check(
        "step run and rise read as usable ladder geometry",
        step_spacing_ok,
        f"step_positions={step_1_pos}, {step_2_pos}, {step_3_pos}",
    )

    rear_open_aabb = ctx.part_world_aabb(rear_frame)
    left_open_aabb = ctx.part_world_aabb(left_spreader_rear)
    right_open_aabb = ctx.part_world_aabb(right_spreader_rear)
    with ctx.pose(
        {
            rear_frame_hinge: math.radians(58.0),
            left_spreader_pivot: 0.42,
            right_spreader_pivot: 0.42,
            left_spreader_knuckle: 0.86,
            right_spreader_knuckle: 0.86,
        }
    ):
        rear_folded_aabb = ctx.part_world_aabb(rear_frame)
        left_folded_aabb = ctx.part_world_aabb(left_spreader_rear)
        right_folded_aabb = ctx.part_world_aabb(right_spreader_rear)
        ctx.check(
            "rear frame folds forward toward the front frame",
            rear_open_aabb is not None
            and rear_folded_aabb is not None
            and rear_folded_aabb[1][1] > rear_open_aabb[1][1] + 0.16,
            f"open={None if rear_open_aabb is None else rear_open_aabb[1][1]}, "
            f"folded={None if rear_folded_aabb is None else rear_folded_aabb[1][1]}",
        )
        ctx.check(
            "spread-limit braces swing with the closing stance",
            left_open_aabb is not None
            and left_folded_aabb is not None
            and right_open_aabb is not None
            and right_folded_aabb is not None
            and left_folded_aabb[0][1] > left_open_aabb[0][1] + 0.08
            and right_folded_aabb[0][1] > right_open_aabb[0][1] + 0.08,
            "rear brace halves should move forward as the ladder closes",
        )
        ctx.expect_contact(left_spreader_front, left_spreader_rear)
        ctx.expect_contact(right_spreader_front, right_spreader_rear)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
