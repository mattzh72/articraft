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


def _rpy_for_member(
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
    sx: float,
    sy: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Box((sx, sy, _distance(a, b))),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_member(a, b)),
        material=material,
        name=name,
    )


def _add_bolt_head(
    part,
    *,
    center: tuple[float, float, float],
    axis: str,
    radius: float,
    length: float,
    material,
    name: str | None = None,
) -> None:
    rpy = {
        "x": (0.0, math.pi / 2.0, 0.0),
        "y": (math.pi / 2.0, 0.0, 0.0),
        "z": (0.0, 0.0, 0.0),
    }[axis]
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=rpy),
        material=material,
        name=name,
    )


def _add_tread(
    part,
    *,
    width: float,
    y: float,
    z: float,
    name_prefix: str,
    tread_material,
    bracket_material,
) -> None:
    tread_depth = 0.092
    tread_thickness = 0.028
    bracket_thickness = 0.010
    bracket_height = 0.072

    part.visual(
        Box((width, tread_depth, tread_thickness)),
        origin=Origin(xyz=(0.0, y, z)),
        material=tread_material,
        name=f"{name_prefix}_deck",
    )
    for side in (-1.0, 1.0):
        side_name = "left" if side < 0.0 else "right"
        plate_center_x = side * (width * 0.5 - bracket_thickness * 0.5)
        part.visual(
            Box((bracket_thickness, tread_depth * 0.92, bracket_height)),
            origin=Origin(xyz=(plate_center_x, y, z - 0.012)),
            material=bracket_material,
            name=f"{name_prefix}_{side_name}_bracket",
        )
        bolt_center_x = side * (width * 0.5 + 0.0015)
        _add_bolt_head(
            part,
            center=(bolt_center_x, y - 0.020, z - 0.004),
            axis="x",
            radius=0.007,
            length=0.006,
            material=bracket_material,
            name=f"{name_prefix}_{side_name}_bolt_low",
        )
        _add_bolt_head(
            part,
            center=(bolt_center_x, y + 0.020, z - 0.004),
            axis="x",
            radius=0.007,
            length=0.006,
            material=bracket_material,
            name=f"{name_prefix}_{side_name}_bolt_high",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_aframe_step_ladder")

    rail_orange = model.material("rail_orange", rgba=(0.90, 0.42, 0.12, 1.0))
    aluminum = model.material("aluminum", rgba=(0.80, 0.82, 0.84, 1.0))
    steel = model.material("steel", rgba=(0.38, 0.40, 0.43, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    safety_red = model.material("safety_red", rgba=(0.79, 0.12, 0.10, 1.0))

    front_frame = model.part("front_frame")
    front_frame.inertial = Inertial.from_geometry(
        Box((0.62, 0.44, 2.08)),
        mass=12.0,
        origin=Origin(xyz=(0.0, -0.12, -0.60)),
    )

    # Primary front rails.
    _add_box_member(
        front_frame,
        (-0.195, -0.045, -0.060),
        (-0.275, -0.300, -1.720),
        sx=0.055,
        sy=0.022,
        material=rail_orange,
        name="left_front_rail",
    )
    _add_box_member(
        front_frame,
        (0.195, -0.045, -0.060),
        (0.275, -0.300, -1.720),
        sx=0.055,
        sy=0.022,
        material=rail_orange,
        name="right_front_rail",
    )

    # Top standing platform and hinge bridge.
    front_frame.visual(
        Box((0.300, 0.220, 0.034)),
        origin=Origin(xyz=(0.0, -0.090, -0.067)),
        material=aluminum,
        name="platform_deck",
    )
    front_frame.visual(
        Box((0.430, 0.090, 0.065)),
        origin=Origin(xyz=(0.0, -0.020, -0.022)),
        material=steel,
        name="top_bridge",
    )

    # Upper hinge reinforcement and over-travel stop lugs.
    front_frame.visual(
        Box((0.012, 0.100, 0.155)),
        origin=Origin(xyz=(-0.206, 0.010, 0.010)),
        material=steel,
        name="left_hinge_plate",
    )
    front_frame.visual(
        Box((0.012, 0.100, 0.155)),
        origin=Origin(xyz=(0.206, 0.010, 0.010)),
        material=steel,
        name="right_hinge_plate",
    )
    front_frame.visual(
        Box((0.050, 0.033, 0.100)),
        origin=Origin(xyz=(-0.142, 0.0415, 0.010)),
        material=steel,
        name="left_front_stop",
    )
    front_frame.visual(
        Box((0.050, 0.033, 0.100)),
        origin=Origin(xyz=(0.142, 0.0415, 0.010)),
        material=steel,
        name="right_front_stop",
    )

    # Guard rail assembly integrated into the front frame for a safety-first stance.
    front_frame.visual(
        Box((0.060, 0.050, 0.012)),
        origin=Origin(xyz=(-0.120, -0.090, -0.044)),
        material=steel,
        name="left_guard_foot",
    )
    front_frame.visual(
        Box((0.060, 0.050, 0.012)),
        origin=Origin(xyz=(0.120, -0.090, -0.044)),
        material=steel,
        name="right_guard_foot",
    )
    front_frame.visual(
        Box((0.036, 0.018, 0.610)),
        origin=Origin(xyz=(-0.120, -0.090, 0.260)),
        material=rail_orange,
        name="left_guard_post",
    )
    front_frame.visual(
        Box((0.036, 0.018, 0.610)),
        origin=Origin(xyz=(0.120, -0.090, 0.260)),
        material=rail_orange,
        name="right_guard_post",
    )
    front_frame.visual(
        Box((0.300, 0.024, 0.028)),
        origin=Origin(xyz=(0.0, -0.090, 0.340)),
        material=rail_orange,
        name="guard_mid_rail",
    )
    front_frame.visual(
        Box((0.340, 0.040, 0.040)),
        origin=Origin(xyz=(0.0, -0.090, 0.580)),
        material=rail_orange,
        name="guard_top_bar",
    )
    _add_box_member(
        front_frame,
        (-0.120, -0.090, 0.080),
        (-0.205, -0.055, -0.210),
        sx=0.024,
        sy=0.014,
        material=steel,
        name="left_guard_kicker",
    )
    _add_box_member(
        front_frame,
        (0.120, -0.090, 0.080),
        (0.205, -0.055, -0.210),
        sx=0.024,
        sy=0.014,
        material=steel,
        name="right_guard_kicker",
    )

    # Treads with bracket and fastener logic.
    _add_tread(
        front_frame,
        width=0.500,
        y=-0.275,
        z=-1.305,
        name_prefix="tread_1",
        tread_material=aluminum,
        bracket_material=steel,
    )
    _add_tread(
        front_frame,
        width=0.468,
        y=-0.225,
        z=-0.985,
        name_prefix="tread_2",
        tread_material=aluminum,
        bracket_material=steel,
    )
    _add_tread(
        front_frame,
        width=0.442,
        y=-0.175,
        z=-0.665,
        name_prefix="tread_3",
        tread_material=aluminum,
        bracket_material=steel,
    )
    _add_tread(
        front_frame,
        width=0.418,
        y=-0.125,
        z=-0.345,
        name_prefix="tread_4",
        tread_material=aluminum,
        bracket_material=steel,
    )

    # Brace pivot brackets with visible hardware.
    front_frame.visual(
        Box((0.020, 0.050, 0.060)),
        origin=Origin(xyz=(-0.290, -0.175, -0.720)),
        material=steel,
        name="left_brace_pivot_bracket",
    )
    front_frame.visual(
        Box((0.020, 0.050, 0.060)),
        origin=Origin(xyz=(0.290, -0.175, -0.720)),
        material=steel,
        name="right_brace_pivot_bracket",
    )
    front_frame.visual(
        Box((0.040, 0.035, 0.115)),
        origin=Origin(xyz=(-0.272, -0.205, -0.775)),
        material=steel,
        name="left_brace_pivot_base",
    )
    front_frame.visual(
        Box((0.040, 0.035, 0.115)),
        origin=Origin(xyz=(0.272, -0.205, -0.775)),
        material=steel,
        name="right_brace_pivot_base",
    )
    front_frame.visual(
        Box((0.040, 0.045, 0.095)),
        origin=Origin(xyz=(-0.262, -0.198, -0.735)),
        material=steel,
        name="left_brace_pivot_web",
    )
    front_frame.visual(
        Box((0.040, 0.045, 0.095)),
        origin=Origin(xyz=(0.262, -0.198, -0.735)),
        material=steel,
        name="right_brace_pivot_web",
    )
    _add_box_member(
        front_frame,
        (-0.290, -0.175, -0.720),
        (-0.240, -0.210, -0.585),
        sx=0.020,
        sy=0.010,
        material=steel,
        name="left_brace_pivot_strap",
    )
    _add_box_member(
        front_frame,
        (0.290, -0.175, -0.720),
        (0.240, -0.210, -0.585),
        sx=0.020,
        sy=0.010,
        material=steel,
        name="right_brace_pivot_strap",
    )
    _add_bolt_head(
        front_frame,
        center=(-0.282, -0.175, -0.720),
        axis="x",
        radius=0.010,
        length=0.016,
        material=steel,
        name="left_brace_pivot_head",
    )
    _add_bolt_head(
        front_frame,
        center=(0.282, -0.175, -0.720),
        axis="x",
        radius=0.010,
        length=0.016,
        material=steel,
        name="right_brace_pivot_head",
    )

    # Top hinge hardware heads and lower anti-slip feet.
    for sx, side_name in ((-1.0, "left"), (1.0, "right")):
        _add_bolt_head(
            front_frame,
            center=(sx * 0.209, 0.008, 0.048),
            axis="x",
            radius=0.010,
            length=0.016,
            material=steel,
            name=f"{side_name}_hinge_head_outer",
        )
        _add_bolt_head(
            front_frame,
            center=(sx * 0.218, -0.010, -0.010),
            axis="x",
            radius=0.010,
            length=0.008,
            material=steel,
            name=f"{side_name}_hinge_head_inner",
        )
        front_frame.visual(
            Box((0.090, 0.050, 0.026)),
            origin=Origin(xyz=(sx * 0.275, -0.300, -1.733)),
            material=rubber,
            name=f"{side_name}_front_foot",
        )

    # Local gussets to show load path from hinge bridge into rails.
    _add_box_member(
        front_frame,
        (-0.165, -0.030, -0.040),
        (-0.215, -0.075, -0.255),
        sx=0.042,
        sy=0.010,
        material=steel,
        name="left_top_gusset",
    )
    _add_box_member(
        front_frame,
        (0.165, -0.030, -0.040),
        (0.215, -0.075, -0.255),
        sx=0.042,
        sy=0.010,
        material=steel,
        name="right_top_gusset",
    )

    rear_frame = model.part("rear_frame")
    rear_frame.inertial = Inertial.from_geometry(
        Box((0.54, 0.86, 1.88)),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.34, -0.74)),
    )

    _add_box_member(
        rear_frame,
        (-0.225, 0.085, -0.030),
        (-0.245, 0.735, -1.720),
        sx=0.045,
        sy=0.020,
        material=rail_orange,
        name="left_rear_rail",
    )
    _add_box_member(
        rear_frame,
        (0.225, 0.085, -0.030),
        (0.245, 0.735, -1.720),
        sx=0.045,
        sy=0.020,
        material=rail_orange,
        name="right_rear_rail",
    )
    rear_frame.visual(
        Box((0.500, 0.060, 0.060)),
        origin=Origin(xyz=(0.0, 0.125, -0.035)),
        material=steel,
        name="rear_top_crossbar",
    )
    rear_frame.visual(
        Box((0.470, 0.050, 0.028)),
        origin=Origin(xyz=(0.0, 0.410, -0.855)),
        material=steel,
        name="rear_mid_brace",
    )
    rear_frame.visual(
        Box((0.510, 0.050, 0.028)),
        origin=Origin(xyz=(0.0, 0.585, -1.255)),
        material=steel,
        name="rear_lower_brace",
    )

    rear_frame.visual(
        Box((0.018, 0.060, 0.130)),
        origin=Origin(xyz=(-0.205, 0.090, 0.025)),
        material=steel,
        name="left_rear_hinge_plate",
    )
    rear_frame.visual(
        Box((0.018, 0.060, 0.130)),
        origin=Origin(xyz=(0.205, 0.090, 0.025)),
        material=steel,
        name="right_rear_hinge_plate",
    )
    rear_frame.visual(
        Box((0.050, 0.018, 0.080)),
        origin=Origin(xyz=(-0.142, 0.067, 0.010)),
        material=steel,
        name="left_rear_stop",
    )
    rear_frame.visual(
        Box((0.050, 0.018, 0.080)),
        origin=Origin(xyz=(0.142, 0.067, 0.010)),
        material=steel,
        name="right_rear_stop",
    )
    _add_box_member(
        rear_frame,
        (-0.195, 0.088, 0.028),
        (-0.155, 0.073, 0.010),
        sx=0.022,
        sy=0.010,
        material=steel,
        name="left_rear_stop_web",
    )
    _add_box_member(
        rear_frame,
        (0.195, 0.088, 0.028),
        (0.155, 0.073, 0.010),
        sx=0.022,
        sy=0.010,
        material=steel,
        name="right_rear_stop_web",
    )
    rear_frame.visual(
        Box((0.040, 0.016, 0.045)),
        origin=Origin(xyz=(-0.335, 0.097, -0.865)),
        material=steel,
        name="left_brace_catch",
    )
    rear_frame.visual(
        Box((0.040, 0.016, 0.045)),
        origin=Origin(xyz=(0.335, 0.097, -0.865)),
        material=steel,
        name="right_brace_catch",
    )
    _add_box_member(
        rear_frame,
        (-0.225, 0.400, -0.855),
        (-0.335, 0.097, -0.865),
        sx=0.018,
        sy=0.008,
        material=steel,
        name="left_catch_tie",
    )
    _add_box_member(
        rear_frame,
        (0.225, 0.400, -0.855),
        (0.335, 0.097, -0.865),
        sx=0.018,
        sy=0.008,
        material=steel,
        name="right_catch_tie",
    )
    _add_box_member(
        rear_frame,
        (-0.195, 0.088, -0.020),
        (-0.225, 0.220, -0.390),
        sx=0.032,
        sy=0.010,
        material=steel,
        name="left_rear_gusset",
    )
    _add_box_member(
        rear_frame,
        (0.195, 0.088, -0.020),
        (0.225, 0.220, -0.390),
        sx=0.032,
        sy=0.010,
        material=steel,
        name="right_rear_gusset",
    )
    for sx, side_name in ((-1.0, "left"), (1.0, "right")):
        _add_bolt_head(
            rear_frame,
            center=(sx * 0.216, 0.090, 0.040),
            axis="x",
            radius=0.010,
            length=0.016,
            material=steel,
            name=f"{side_name}_rear_hinge_head",
        )
        _add_bolt_head(
            rear_frame,
            center=(sx * 0.347, 0.097, -0.865),
            axis="x",
            radius=0.009,
            length=0.008,
            material=steel,
            name=f"{side_name}_catch_head",
        )
        rear_frame.visual(
            Box((0.090, 0.050, 0.026)),
            origin=Origin(xyz=(sx * 0.245, 0.735, -1.733)),
            material=rubber,
            name=f"{side_name}_rear_foot",
        )

    left_spreader = model.part("left_spreader")
    left_spreader.inertial = Inertial.from_geometry(
        Box((0.10, 0.50, 0.12)),
        mass=1.0,
        origin=Origin(xyz=(0.01, 0.22, -0.03)),
    )
    left_spreader.visual(
        Box((0.024, 0.050, 0.060)),
        origin=Origin(xyz=(-0.035, 0.015, 0.0)),
        material=steel,
        name="left_spreader_clevis",
    )
    _add_box_member(
        left_spreader,
        (-0.035, 0.030, -0.005),
        (-0.035, 0.235, -0.145),
        sx=0.020,
        sy=0.010,
        material=steel,
        name="left_spreader_bar",
    )
    left_spreader.visual(
        Box((0.048, 0.028, 0.045)),
        origin=Origin(xyz=(-0.035, 0.250, -0.145)),
        material=steel,
        name="left_spreader_shoe",
    )
    left_spreader.visual(
        Box((0.036, 0.036, 0.016)),
        origin=Origin(xyz=(-0.035, 0.175, -0.090)),
        material=safety_red,
        name="left_lockout_tab",
    )
    _add_bolt_head(
        left_spreader,
        center=(-0.048, 0.015, 0.0),
        axis="x",
        radius=0.010,
        length=0.016,
        material=steel,
        name="left_spreader_pivot_head",
    )

    right_spreader = model.part("right_spreader")
    right_spreader.inertial = Inertial.from_geometry(
        Box((0.10, 0.50, 0.12)),
        mass=1.0,
        origin=Origin(xyz=(-0.01, 0.22, -0.03)),
    )
    right_spreader.visual(
        Box((0.024, 0.050, 0.060)),
        origin=Origin(xyz=(0.035, 0.015, 0.0)),
        material=steel,
        name="right_spreader_clevis",
    )
    _add_box_member(
        right_spreader,
        (0.035, 0.030, -0.005),
        (0.035, 0.235, -0.145),
        sx=0.020,
        sy=0.010,
        material=steel,
        name="right_spreader_bar",
    )
    right_spreader.visual(
        Box((0.048, 0.028, 0.045)),
        origin=Origin(xyz=(0.035, 0.250, -0.145)),
        material=steel,
        name="right_spreader_shoe",
    )
    right_spreader.visual(
        Box((0.036, 0.036, 0.016)),
        origin=Origin(xyz=(0.035, 0.175, -0.090)),
        material=safety_red,
        name="right_lockout_tab",
    )
    _add_bolt_head(
        right_spreader,
        center=(0.048, 0.015, 0.0),
        axis="x",
        radius=0.010,
        length=0.016,
        material=steel,
        name="right_spreader_pivot_head",
    )

    model.articulation(
        "front_to_rear",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=-0.58,
            upper=0.0,
        ),
    )
    model.articulation(
        "front_to_left_spreader",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=left_spreader,
        origin=Origin(xyz=(-0.300, -0.175, -0.720)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=-1.45,
            upper=0.0,
        ),
    )
    model.articulation(
        "front_to_right_spreader",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=right_spreader,
        origin=Origin(xyz=(0.300, -0.175, -0.720)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=-1.45,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    rear_frame = object_model.get_part("rear_frame")
    left_spreader = object_model.get_part("left_spreader")
    right_spreader = object_model.get_part("right_spreader")

    front_to_rear = object_model.get_articulation("front_to_rear")
    front_to_left_spreader = object_model.get_articulation("front_to_left_spreader")
    front_to_right_spreader = object_model.get_articulation("front_to_right_spreader")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        front_frame,
        rear_frame,
        reason="Stamped hinge stop tabs are simplified as rectangular boxes and slightly interleave in the folded pose.",
        elem_a="left_front_stop",
        elem_b="left_rear_stop",
    )
    ctx.allow_overlap(
        front_frame,
        rear_frame,
        reason="Stamped hinge stop tabs are simplified as rectangular boxes and slightly interleave in the folded pose.",
        elem_a="right_front_stop",
        elem_b="right_rear_stop",
    )
    ctx.allow_overlap(
        front_frame,
        rear_frame,
        reason="Front and rear hinge reinforcement plates are simplified as flat boxes around one shared pivot stack and slightly interleave when folded.",
        elem_a="left_hinge_plate",
        elem_b="left_rear_hinge_plate",
    )
    ctx.allow_overlap(
        front_frame,
        rear_frame,
        reason="Front and rear hinge reinforcement plates are simplified as flat boxes around one shared pivot stack and slightly interleave when folded.",
        elem_a="right_hinge_plate",
        elem_b="right_rear_hinge_plate",
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

    front_visuals = {visual.name for visual in front_frame.visuals if visual.name}
    rear_visuals = {visual.name for visual in rear_frame.visuals if visual.name}
    left_visuals = {visual.name for visual in left_spreader.visuals if visual.name}
    right_visuals = {visual.name for visual in right_spreader.visuals if visual.name}

    def _center_y(aabb):
        if aabb is None:
            return None
        return (aabb[0][1] + aabb[1][1]) * 0.5

    ctx.check(
        "safety_features_present",
        {
            "guard_top_bar",
            "left_front_stop",
            "right_front_stop",
            "left_brace_pivot_bracket",
            "right_brace_pivot_bracket",
        }.issubset(front_visuals)
        and {"left_rear_stop", "right_rear_stop", "left_brace_catch", "right_brace_catch"}.issubset(
            rear_visuals
        )
        and {"left_spreader_shoe", "left_lockout_tab"}.issubset(left_visuals)
        and {"right_spreader_shoe", "right_lockout_tab"}.issubset(right_visuals),
        "Critical guard, stop, catch, or lockout visuals are missing.",
    )

    guard_aabb = ctx.part_element_world_aabb(front_frame, elem="guard_top_bar")
    ctx.check(
        "guard_extends_above_platform",
        guard_aabb is not None and guard_aabb[1][2] > 0.55,
        f"Expected top guard bar above 0.55 m, got {guard_aabb}.",
    )

    with ctx.pose(
        {
            front_to_rear: 0.0,
            front_to_left_spreader: 0.0,
            front_to_right_spreader: 0.0,
        }
    ):
        ctx.expect_gap(
            rear_frame,
            front_frame,
            axis="y",
            positive_elem="left_rear_stop",
            negative_elem="left_front_stop",
            min_gap=0.0,
            max_gap=0.001,
            name="left_upper_stop_contacts_in_open_pose",
        )
        ctx.expect_gap(
            rear_frame,
            front_frame,
            axis="y",
            positive_elem="right_rear_stop",
            negative_elem="right_front_stop",
            min_gap=0.0,
            max_gap=0.001,
            name="right_upper_stop_contacts_in_open_pose",
        )
        ctx.expect_contact(
            rear_frame,
            left_spreader,
            elem_a="left_brace_catch",
            elem_b="left_spreader_shoe",
            name="left_spreader_catches_in_open_pose",
        )
        ctx.expect_contact(
            rear_frame,
            right_spreader,
            elem_a="right_brace_catch",
            elem_b="right_spreader_shoe",
            name="right_spreader_catches_in_open_pose",
        )
        rear_open_y = _center_y(ctx.part_element_world_aabb(rear_frame, elem="rear_lower_brace"))
        left_open_y = _center_y(ctx.part_element_world_aabb(left_spreader, elem="left_spreader_shoe"))
        right_open_y = _center_y(ctx.part_element_world_aabb(right_spreader, elem="right_spreader_shoe"))

    with ctx.pose(
        {
            front_to_rear: -0.54,
            front_to_left_spreader: -1.28,
            front_to_right_spreader: -1.28,
        }
    ):
        rear_closed_y = _center_y(ctx.part_element_world_aabb(rear_frame, elem="rear_lower_brace"))
        left_closed_y = _center_y(ctx.part_element_world_aabb(left_spreader, elem="left_spreader_shoe"))
        right_closed_y = _center_y(ctx.part_element_world_aabb(right_spreader, elem="right_spreader_shoe"))
        ctx.fail_if_parts_overlap_in_current_pose(name="closed_pose_no_part_overlap")

    ctx.check(
        "rear_frame_folds_toward_front",
        rear_open_y is not None and rear_closed_y is not None and rear_open_y - rear_closed_y > 0.18,
        (
            "Rear frame did not swing distinctly toward the front frame when closing: "
            f"open_y={rear_open_y}, closed_y={rear_closed_y}."
        ),
    )
    ctx.check(
        "spreaders_fold_with_closing_motion",
        left_open_y is not None
        and left_closed_y is not None
        and right_open_y is not None
        and right_closed_y is not None
        and left_open_y - left_closed_y > 0.10
        and right_open_y - right_closed_y > 0.10,
        (
            "Spreader braces did not retract toward the front frame enough to define a folded state: "
            f"left_open_y={left_open_y}, left_closed_y={left_closed_y}, "
            f"right_open_y={right_open_y}, right_closed_y={right_closed_y}."
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
