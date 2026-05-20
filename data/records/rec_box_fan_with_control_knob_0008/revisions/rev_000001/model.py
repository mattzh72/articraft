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

HOUSING_W = 0.50
HOUSING_H = 0.52
HOUSING_D = 0.18
FRAME_T = 0.03
BEZEL_T = 0.02
WIRE_R = 0.0035
PIVOT_Z = 0.325
PIVOT_X = HOUSING_W * 0.5 + 0.002
FRONT_BEZEL_Y = HOUSING_D * 0.5 - BEZEL_T * 0.5
REAR_BEZEL_Y = -FRONT_BEZEL_Y
FRONT_FACE_Y = FRONT_BEZEL_Y + BEZEL_T * 0.5
REAR_FACE_Y = REAR_BEZEL_Y - BEZEL_T * 0.5
INNER_HALF_W = HOUSING_W * 0.5 - FRAME_T
INNER_HALF_H = HOUSING_H * 0.5 - FRAME_T
GRILLE_HALF_W = 0.168
GRILLE_HALF_H = 0.178
FRONT_GRILLE_Y = FRONT_FACE_Y + WIRE_R
REAR_GRILLE_Y = REAR_FACE_Y - WIRE_R
KNOB_X = 0.214
KNOB_Z = -0.224


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


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


def _add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_rect_loop(part, *, y_pos: float, half_w: float, half_h: float, radius: float, material) -> None:
    _add_member(part, (-half_w, y_pos, half_h), (half_w, y_pos, half_h), radius, material)
    _add_member(part, (-half_w, y_pos, -half_h), (half_w, y_pos, -half_h), radius, material)
    _add_member(part, (-half_w, y_pos, -half_h), (-half_w, y_pos, half_h), radius, material)
    _add_member(part, (half_w, y_pos, -half_h), (half_w, y_pos, half_h), radius, material)


def _add_grille_face(part, *, y_pos: float, material, center_name: str, perimeter_name: str) -> None:
    _add_rect_loop(
        part,
        y_pos=y_pos,
        half_w=GRILLE_HALF_W,
        half_h=GRILLE_HALF_H,
        radius=WIRE_R,
        material=material,
    )
    _add_rect_loop(
        part,
        y_pos=y_pos,
        half_w=0.124,
        half_h=0.132,
        radius=WIRE_R * 0.88,
        material=material,
    )
    _add_rect_loop(
        part,
        y_pos=y_pos,
        half_w=0.078,
        half_h=0.084,
        radius=WIRE_R * 0.78,
        material=material,
    )

    _add_member(
        part,
        (-GRILLE_HALF_W, y_pos, 0.0),
        (GRILLE_HALF_W, y_pos, 0.0),
        WIRE_R * 0.92,
        material,
        name=perimeter_name,
    )
    _add_member(
        part,
        (0.0, y_pos, -GRILLE_HALF_H),
        (0.0, y_pos, GRILLE_HALF_H),
        WIRE_R * 0.92,
        material,
        name=center_name,
    )

    for sx in (-1.0, 1.0):
        for sz in (-1.0, 1.0):
            _add_member(part, (sx * 0.050, y_pos, sz * 0.050), (sx * 0.120, y_pos, sz * 0.122), WIRE_R * 0.72, material)
            _add_member(part, (sx * 0.036, y_pos, 0.0), (sx * 0.120, y_pos, sz * 0.122), WIRE_R * 0.64, material)
            _add_member(part, (0.0, y_pos, sz * 0.036), (sx * 0.120, y_pos, sz * 0.122), WIRE_R * 0.64, material)

    for x_pos in (-0.126, 0.126):
        _add_member(part, (x_pos, y_pos, 0.050), (x_pos, y_pos, 0.110), WIRE_R, material)
        _add_member(part, (x_pos, y_pos, -0.040), (x_pos, y_pos, -0.100), WIRE_R, material)
    _add_member(part, (0.0, y_pos, GRILLE_HALF_H), (0.0, y_pos, INNER_HALF_H), WIRE_R, material)
    _add_member(part, (-0.120, y_pos, 0.0), (-INNER_HALF_W, y_pos, 0.0), WIRE_R, material)


def _add_propeller_visuals(part, material) -> None:
    for index in range(4):
        angle = math.pi / 4.0 + (index * math.tau / 4.0)
        radial_x = math.sin(angle)
        radial_z = math.cos(angle)
        part.visual(
            Box((0.036, 0.008, 0.096)),
            origin=Origin(
                xyz=(radial_x * 0.070, 0.006, radial_z * 0.070),
                rpy=(0.18, angle, 0.0),
            ),
            material=material,
            name="blade_panel" if index == 0 else None,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_box_fan", assets=ASSETS)

    cream = model.material("cream_plastic", rgba=(0.90, 0.85, 0.73, 1.0))
    rib_shadow = model.material("rib_shadow", rgba=(0.77, 0.72, 0.61, 1.0))
    chrome = model.material("chrome", rgba=(0.84, 0.86, 0.90, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.33, 0.35, 0.37, 1.0))
    blade_amber = model.material("blade_amber", rgba=(0.70, 0.56, 0.22, 0.90))
    bakelite = model.material("bakelite", rgba=(0.18, 0.10, 0.06, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.38, 0.18, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=cream,
        name="pedestal",
    )
    base.visual(
        Box((0.086, 0.050, 0.010)),
        origin=Origin(xyz=(-0.233, 0.0, 0.045)),
        material=cream,
        name="left_upright_foot",
    )
    base.visual(
        Box((0.022, 0.050, 0.280)),
        origin=Origin(xyz=(-0.276, 0.0, 0.190)),
        material=cream,
        name="left_upright",
    )
    base.visual(
        Box((0.086, 0.050, 0.010)),
        origin=Origin(xyz=(0.233, 0.0, 0.045)),
        material=cream,
        name="right_upright_foot",
    )
    base.visual(
        Box((0.022, 0.050, 0.280)),
        origin=Origin(xyz=(0.276, 0.0, 0.190)),
        material=cream,
        name="right_upright",
    )
    base.visual(
        Cylinder(radius=0.024, length=0.030),
        origin=Origin(xyz=(-0.276, 0.0, PIVOT_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cream,
        name="left_yoke_cap",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.022),
        origin=Origin(xyz=(-0.263, 0.0, PIVOT_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="left_pivot_pin",
    )
    base.visual(
        Cylinder(radius=0.024, length=0.030),
        origin=Origin(xyz=(0.276, 0.0, PIVOT_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cream,
        name="right_yoke_cap",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.022),
        origin=Origin(xyz=(0.263, 0.0, PIVOT_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="right_pivot_pin",
    )
    base.visual(
        Box((0.09, 0.05, 0.010)),
        origin=Origin(xyz=(-0.115, 0.0, 0.005)),
        material=rubber,
        name="left_foot_pad",
    )
    base.visual(
        Box((0.09, 0.05, 0.010)),
        origin=Origin(xyz=(0.115, 0.0, 0.005)),
        material=rubber,
        name="right_foot_pad",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.62, 0.20, 0.36)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
    )

    housing = model.part("housing")
    for y_pos, prefix in ((FRONT_BEZEL_Y, "front"), (REAR_BEZEL_Y, "rear")):
        housing.visual(
            Box((HOUSING_W, BEZEL_T, FRAME_T)),
            origin=Origin(xyz=(0.0, y_pos, HOUSING_H * 0.5 - FRAME_T * 0.5)),
            material=cream,
            name=f"{prefix}_frame_top",
        )
        housing.visual(
            Box((HOUSING_W, BEZEL_T, FRAME_T)),
            origin=Origin(xyz=(0.0, y_pos, -HOUSING_H * 0.5 + FRAME_T * 0.5)),
            material=cream,
            name=f"{prefix}_frame_bottom",
        )
        housing.visual(
            Box((FRAME_T, BEZEL_T, HOUSING_H - 2.0 * FRAME_T)),
            origin=Origin(xyz=(HOUSING_W * 0.5 - FRAME_T * 0.5, y_pos, 0.0)),
            material=cream,
            name=f"{prefix}_frame_right",
        )
        housing.visual(
            Box((FRAME_T, BEZEL_T, HOUSING_H - 2.0 * FRAME_T)),
            origin=Origin(xyz=(-HOUSING_W * 0.5 + FRAME_T * 0.5, y_pos, 0.0)),
            material=cream,
            name=f"{prefix}_frame_left",
        )

    wall_depth = HOUSING_D - 2.0 * BEZEL_T
    housing.visual(
        Box((FRAME_T, wall_depth, HOUSING_H)),
        origin=Origin(xyz=(HOUSING_W * 0.5 - FRAME_T * 0.5, 0.0, 0.0)),
        material=cream,
        name="right_side_wall",
    )
    housing.visual(
        Box((FRAME_T, wall_depth, HOUSING_H)),
        origin=Origin(xyz=(-HOUSING_W * 0.5 + FRAME_T * 0.5, 0.0, 0.0)),
        material=cream,
        name="left_side_wall",
    )
    housing.visual(
        Box((HOUSING_W - 2.0 * FRAME_T, wall_depth, FRAME_T)),
        origin=Origin(xyz=(0.0, 0.0, HOUSING_H * 0.5 - FRAME_T * 0.5)),
        material=cream,
        name="top_wall",
    )
    housing.visual(
        Box((HOUSING_W - 2.0 * FRAME_T, wall_depth, FRAME_T)),
        origin=Origin(xyz=(0.0, 0.0, -HOUSING_H * 0.5 + FRAME_T * 0.5)),
        material=cream,
        name="bottom_wall",
    )

    for z_pos in (-0.15, -0.08, -0.01, 0.06, 0.13):
        housing.visual(
            Box((0.004, wall_depth * 0.84, 0.020)),
            origin=Origin(xyz=(HOUSING_W * 0.5 + 0.002, 0.0, z_pos)),
            material=rib_shadow,
        )
        housing.visual(
            Box((0.004, wall_depth * 0.84, 0.020)),
            origin=Origin(xyz=(-HOUSING_W * 0.5 - 0.002, 0.0, z_pos)),
            material=rib_shadow,
        )

    for x_pos in (-0.15, -0.05, 0.05, 0.15):
        housing.visual(
            Box((0.060, wall_depth * 0.82, 0.004)),
            origin=Origin(xyz=(x_pos, 0.0, HOUSING_H * 0.5 + 0.002)),
            material=rib_shadow,
        )

    housing.visual(
        Box((0.050, 0.012, 0.052)),
        origin=Origin(xyz=(KNOB_X, FRONT_FACE_Y + 0.006, KNOB_Z)),
        material=cream,
        name="control_panel",
    )
    housing.visual(
        Cylinder(radius=0.050, length=0.070),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="motor_pod",
    )
    housing.visual(
        Cylinder(radius=0.043, length=0.030),
        origin=Origin(xyz=(0.0, -0.056, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="rear_cap",
    )
    housing.visual(
        Cylinder(radius=0.012, length=0.046),
        origin=Origin(xyz=(0.0, 0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="spindle",
    )
    housing.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(-PIVOT_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cream,
        name="left_pivot_boss",
    )
    housing.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(PIVOT_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cream,
        name="right_pivot_boss",
    )

    for x_sign in (-1.0, 1.0):
        for z_sign in (-1.0, 1.0):
            _add_member(
                housing,
                (x_sign * 0.030, -0.006, 0.0),
                (x_sign * 0.075, 0.0, z_sign * (HOUSING_H * 0.5 - FRAME_T * 0.15)),
                0.005,
                dark_metal,
            )

    housing.inertial = Inertial.from_geometry(
        Box((HOUSING_W, HOUSING_D, HOUSING_H)),
        mass=4.6,
        origin=Origin(),
    )

    front_grille = model.part("front_grille")
    _add_grille_face(
        front_grille,
        y_pos=FRONT_GRILLE_Y,
        material=chrome,
        center_name="front_center_bar",
        perimeter_name="front_cross_bar",
    )
    front_grille.inertial = Inertial.from_geometry(
        Box((0.44, 0.010, 0.44)),
        mass=0.18,
        origin=Origin(xyz=(0.0, FRONT_GRILLE_Y, 0.0)),
    )

    rear_grille = model.part("rear_grille")
    _add_grille_face(
        rear_grille,
        y_pos=REAR_GRILLE_Y,
        material=chrome,
        center_name="rear_center_bar",
        perimeter_name="rear_cross_bar",
    )
    rear_grille.inertial = Inertial.from_geometry(
        Box((0.44, 0.010, 0.44)),
        mass=0.18,
        origin=Origin(xyz=(0.0, REAR_GRILLE_Y, 0.0)),
    )

    propeller = model.part("propeller")
    _add_propeller_visuals(propeller, blade_amber)
    propeller.visual(
        Cylinder(radius=0.045, length=0.020),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blade_amber,
        name="hub",
    )
    propeller.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(0.0, 0.026, 0.0)),
        material=chrome,
        name="nose_cap",
    )
    propeller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.17, length=0.07),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        Cylinder(radius=0.032, length=0.008),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bakelite,
        name="knob_collar",
    )
    speed_knob.visual(
        Cylinder(radius=0.029, length=0.026),
        origin=Origin(xyz=(0.0, 0.019, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bakelite,
        name="knob_body",
    )
    speed_knob.visual(
        Cylinder(radius=0.022, length=0.006),
        origin=Origin(xyz=(0.0, 0.035, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bakelite,
        name="knob_face",
    )
    speed_knob.visual(
        Box((0.018, 0.0025, 0.006)),
        origin=Origin(xyz=(0.010, 0.038, 0.0)),
        material=chrome,
        name="knob_pointer",
    )
    for rib_index in range(8):
        angle = rib_index * math.tau / 8.0
        speed_knob.visual(
            Box((0.006, 0.018, 0.010)),
            origin=Origin(
                xyz=(math.cos(angle) * 0.0255, 0.020, math.sin(angle) * 0.0255),
                rpy=(0.0, angle, 0.0),
            ),
            material=bakelite,
        )
    speed_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.034, length=0.040),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "housing_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=math.radians(-12.0),
            upper=math.radians(24.0),
        ),
    )
    model.articulation(
        "front_grille_mount",
        ArticulationType.FIXED,
        parent=housing,
        child=front_grille,
        origin=Origin(),
    )
    model.articulation(
        "rear_grille_mount",
        ArticulationType.FIXED,
        parent=housing,
        child=rear_grille,
        origin=Origin(),
    )
    model.articulation(
        "propeller_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=propeller,
        origin=Origin(xyz=(0.0, 0.036, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=28.0,
        ),
    )
    model.articulation(
        "speed_knob_turn",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=speed_knob,
        origin=Origin(xyz=(KNOB_X, FRONT_FACE_Y + 0.012, KNOB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(120.0),
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    housing = object_model.get_part("housing")
    front_grille = object_model.get_part("front_grille")
    rear_grille = object_model.get_part("rear_grille")
    propeller = object_model.get_part("propeller")
    speed_knob = object_model.get_part("speed_knob")

    housing_tilt = object_model.get_articulation("housing_tilt")
    propeller_spin = object_model.get_articulation("propeller_spin")
    speed_knob_turn = object_model.get_articulation("speed_knob_turn")

    left_pivot_pin = base.get_visual("left_pivot_pin")
    right_pivot_pin = base.get_visual("right_pivot_pin")
    left_pivot_boss = housing.get_visual("left_pivot_boss")
    right_pivot_boss = housing.get_visual("right_pivot_boss")
    pedestal = base.get_visual("pedestal")
    bottom_wall = housing.get_visual("bottom_wall")
    spindle = housing.get_visual("spindle")
    control_panel = housing.get_visual("control_panel")
    hub = propeller.get_visual("hub")
    blade_panel = propeller.get_visual("blade_panel")
    knob_collar = speed_knob.get_visual("knob_collar")
    knob_body = speed_knob.get_visual("knob_body")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        base,
        housing,
        reason="Steel pivot pins are seated inside the housing trunnion bosses; bearing holes are not modeled.",
        elem_a=left_pivot_pin,
        elem_b=left_pivot_boss,
    )
    ctx.allow_overlap(
        base,
        housing,
        reason="Steel pivot pins are seated inside the housing trunnion bosses; bearing holes are not modeled.",
        elem_a=right_pivot_pin,
        elem_b=right_pivot_boss,
    )
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(housing, propeller, reason="Drive spindle is intentionally inserted into the propeller hub.", elem_a=spindle, elem_b=hub)
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    ctx.expect_overlap(base, housing, axes="yz", elem_a=left_pivot_pin, elem_b=left_pivot_boss, min_overlap=0.018)
    ctx.expect_overlap(base, housing, axes="yz", elem_a=right_pivot_pin, elem_b=right_pivot_boss, min_overlap=0.018)
    ctx.expect_gap(housing, base, axis="z", min_gap=0.010, max_gap=0.030, positive_elem=bottom_wall, negative_elem=pedestal)
    ctx.expect_contact(front_grille, housing)
    ctx.expect_contact(rear_grille, housing)
    ctx.expect_contact(propeller, housing, elem_a=hub, elem_b=spindle)
    ctx.expect_within(propeller, housing, axes="xz")
    ctx.expect_gap(front_grille, propeller, axis="y", min_gap=0.015, max_gap=0.090)
    ctx.expect_gap(propeller, rear_grille, axis="y", min_gap=0.050, max_gap=0.150)
    ctx.expect_contact(speed_knob, housing, elem_a=knob_collar, elem_b=control_panel)
    ctx.expect_gap(
        speed_knob,
        housing,
        axis="y",
        max_gap=0.001,
        max_penetration=0.001,
        positive_elem=knob_collar,
        negative_elem=control_panel,
    )
    ctx.expect_gap(speed_knob, propeller, axis="x", min_gap=0.010, max_gap=0.090, positive_elem=knob_body, negative_elem=blade_panel)

    tilt_limits = housing_tilt.motion_limits
    if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
        with ctx.pose({housing_tilt: tilt_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="housing_tilt_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="housing_tilt_lower_no_floating")
            ctx.expect_gap(housing, base, axis="z", min_gap=0.002, positive_elem=bottom_wall, negative_elem=pedestal)
            ctx.expect_contact(front_grille, housing)
        with ctx.pose({housing_tilt: tilt_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="housing_tilt_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="housing_tilt_upper_no_floating")
            ctx.expect_gap(housing, base, axis="z", min_gap=0.002, positive_elem=bottom_wall, negative_elem=pedestal)
            ctx.expect_contact(rear_grille, housing)

    knob_limits = speed_knob_turn.motion_limits
    if knob_limits is not None and knob_limits.lower is not None and knob_limits.upper is not None:
        with ctx.pose({speed_knob_turn: knob_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="speed_knob_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="speed_knob_lower_no_floating")
            ctx.expect_contact(speed_knob, housing, elem_a=knob_collar, elem_b=control_panel)
        with ctx.pose({speed_knob_turn: knob_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="speed_knob_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="speed_knob_upper_no_floating")
            ctx.expect_contact(speed_knob, housing, elem_a=knob_collar, elem_b=control_panel)

    with ctx.pose({propeller_spin: math.pi / 4.0}):
        ctx.expect_within(propeller, housing, axes="xz")
        ctx.expect_gap(front_grille, propeller, axis="y", min_gap=0.015, max_gap=0.090)
        ctx.expect_gap(propeller, rear_grille, axis="y", min_gap=0.050, max_gap=0.150)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
