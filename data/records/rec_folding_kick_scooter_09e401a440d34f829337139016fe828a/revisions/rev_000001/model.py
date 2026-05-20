from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rear_deck_profile(length: float, width: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    return [
        (0.020, -half_w),
        (length * 0.74, -half_w),
        (length * 0.90, -half_w * 0.70),
        (length, 0.0),
        (length * 0.90, half_w * 0.70),
        (length * 0.74, half_w),
        (0.020, half_w),
    ]


def _front_deck_profile(length: float, width: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    return [
        (-0.020, half_w),
        (-length * 0.74, half_w),
        (-length * 0.90, half_w * 0.70),
        (-length, 0.0),
        (-length * 0.90, -half_w * 0.70),
        (-length * 0.74, -half_w),
        (-0.020, -half_w),
    ]


def _rear_fender_profile() -> list[tuple[float, float]]:
    return [
        (0.010, 0.006),
        (0.032, 0.013),
        (0.060, 0.018),
        (0.095, 0.020),
        (0.126, 0.016),
        (0.148, 0.007),
        (0.145, -0.001),
        (0.126, 0.004),
        (0.095, 0.008),
        (0.060, 0.006),
        (0.032, 0.001),
        (0.010, -0.003),
    ]


def _add_wheel_visuals(
    part,
    prefix: str,
    *,
    radius: float,
    width: float,
    rim_material,
) -> None:
    part.visual(
        Cylinder(radius=radius * 0.72, length=width * 0.86),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=rim_material,
        name=f"{prefix}_rim_ring",
    )
    part.visual(
        Cylinder(radius=radius * 0.24, length=0.078),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=rim_material,
        name=f"{prefix}_hub",
    )
    part.visual(
        Box((radius * 1.28, width * 0.28, radius * 0.10)),
        material=rim_material,
        name=f"{prefix}_spoke_cross_x",
    )
    part.visual(
        Box((radius * 0.10, width * 0.28, radius * 1.28)),
        material=rim_material,
        name=f"{prefix}_spoke_cross_z",
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_city_kick_scooter")

    aluminum = model.material("aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.25, 0.27, 0.30, 1.0))
    deck_black = model.material("deck_black", rgba=(0.12, 0.13, 0.14, 1.0))
    tire_black = model.material("tire_black", rgba=(0.05, 0.05, 0.06, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    deck_width = 0.125
    deck_thickness = 0.028
    rear_deck_length = 0.165
    front_deck_length = 0.290
    pad_thickness = 0.004
    wheel_radius = 0.090
    wheel_width = 0.032
    wheel_center_z = 0.030
    front_wheel_x = -0.390
    rear_wheel_x = 0.255
    neck_pitch = 0.14
    lower_neck_length = 0.270
    upper_stem_length = 0.580
    neck_hinge_x = -0.178
    neck_hinge_z = 0.299
    deck_hinge_radius = 0.016
    clamp_hinge_radius = 0.012
    brake_hinge_x = 0.162
    brake_hinge_z = 0.125

    rear_frame = model.part("rear_frame")
    rear_frame.visual(
        _mesh("rear_deck_shell_mesh", ExtrudeGeometry(_rear_deck_profile(rear_deck_length, deck_width), deck_thickness, center=True)),
        material=aluminum,
        name="rear_deck_shell",
    )
    rear_frame.visual(
        Box((rear_deck_length * 0.72, deck_width * 0.76, pad_thickness)),
        origin=Origin(xyz=(rear_deck_length * 0.39, 0.0, (deck_thickness + pad_thickness) * 0.5)),
        material=deck_black,
        name="rear_grip_pad",
    )
    rear_frame.visual(
        Box((0.022, 0.070, 0.028)),
        origin=Origin(xyz=(0.027, 0.0, 0.0)),
        material=dark_metal,
        name="rear_hinge_block",
    )
    rear_frame.visual(
        Cylinder(radius=deck_hinge_radius, length=0.026),
        origin=Origin(xyz=(0.0, -0.031, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="rear_hinge_left_barrel",
    )
    rear_frame.visual(
        Cylinder(radius=deck_hinge_radius, length=0.026),
        origin=Origin(xyz=(0.0, 0.031, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="rear_hinge_right_barrel",
    )
    rear_frame.visual(
        Box((0.160, 0.010, 0.028)),
        origin=Origin(xyz=(0.195, -0.046, 0.032), rpy=(0.0, -0.10, 0.0)),
        material=dark_metal,
        name="rear_left_stay",
    )
    rear_frame.visual(
        Box((0.160, 0.010, 0.028)),
        origin=Origin(xyz=(0.195, 0.046, 0.032), rpy=(0.0, -0.10, 0.0)),
        material=dark_metal,
        name="rear_right_stay",
    )
    rear_frame.visual(
        Box((0.090, 0.090, 0.022)),
        origin=Origin(xyz=(0.095, 0.0, 0.001)),
        material=dark_metal,
        name="rear_underbrace",
    )
    rear_frame.visual(
        Box((0.024, 0.012, 0.048)),
        origin=Origin(xyz=(rear_wheel_x, -0.042, wheel_center_z + 0.014)),
        material=dark_metal,
        name="rear_left_dropout",
    )
    rear_frame.visual(
        Box((0.024, 0.012, 0.048)),
        origin=Origin(xyz=(rear_wheel_x, 0.042, wheel_center_z + 0.014)),
        material=dark_metal,
        name="rear_right_dropout",
    )
    rear_frame.visual(
        Box((0.018, 0.014, 0.152)),
        origin=Origin(xyz=(brake_hinge_x, -0.030, 0.049)),
        material=dark_metal,
        name="rear_brake_support_left",
    )
    rear_frame.visual(
        Box((0.018, 0.014, 0.152)),
        origin=Origin(xyz=(brake_hinge_x, 0.030, 0.049)),
        material=dark_metal,
        name="rear_brake_support_right",
    )
    rear_frame.visual(
        Cylinder(radius=0.008, length=0.020),
        origin=Origin(xyz=(brake_hinge_x, -0.030, brake_hinge_z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="rear_brake_hinge_left_barrel",
    )
    rear_frame.visual(
        Cylinder(radius=0.008, length=0.020),
        origin=Origin(xyz=(brake_hinge_x, 0.030, brake_hinge_z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="rear_brake_hinge_right_barrel",
    )

    front_frame = model.part("front_frame")
    front_frame.visual(
        _mesh("front_deck_shell_mesh", ExtrudeGeometry(_front_deck_profile(front_deck_length, deck_width), deck_thickness, center=True)),
        material=aluminum,
        name="front_deck_shell",
    )
    front_frame.visual(
        Box((front_deck_length * 0.66, deck_width * 0.74, pad_thickness)),
        origin=Origin(xyz=(-front_deck_length * 0.36, 0.0, (deck_thickness + pad_thickness) * 0.5)),
        material=deck_black,
        name="front_grip_pad",
    )
    front_frame.visual(
        Box((0.022, 0.070, 0.028)),
        origin=Origin(xyz=(-0.027, 0.0, 0.0)),
        material=dark_metal,
        name="front_hinge_block",
    )
    front_frame.visual(
        Cylinder(radius=deck_hinge_radius, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="front_hinge_center_barrel",
    )
    front_frame.visual(
        Cylinder(radius=0.022, length=lower_neck_length),
        origin=Origin(xyz=(-0.197, 0.0, 0.136), rpy=(0.0, neck_pitch, 0.0)),
        material=aluminum,
        name="lower_neck_tube",
    )
    front_frame.visual(
        Box((0.060, 0.080, 0.030)),
        origin=Origin(xyz=(-0.194, 0.0, 0.015)),
        material=dark_metal,
        name="neck_base_gusset",
    )
    front_frame.visual(
        Box((0.045, 0.104, 0.032)),
        origin=Origin(xyz=(-0.224, 0.0, 0.086)),
        material=dark_metal,
        name="fork_crown",
    )
    front_frame.visual(
        Box((0.172, 0.010, 0.026)),
        origin=Origin(xyz=(-0.306, -0.041, 0.064), rpy=(0.0, 0.52, 0.0)),
        material=dark_metal,
        name="left_fork_blade",
    )
    front_frame.visual(
        Box((0.172, 0.010, 0.026)),
        origin=Origin(xyz=(-0.306, 0.041, 0.064), rpy=(0.0, 0.52, 0.0)),
        material=dark_metal,
        name="right_fork_blade",
    )
    front_frame.visual(
        Box((0.036, 0.012, 0.044)),
        origin=Origin(xyz=(front_wheel_x + 0.008, -0.042, wheel_center_z + 0.010)),
        material=dark_metal,
        name="front_left_dropout",
    )
    front_frame.visual(
        Box((0.036, 0.012, 0.044)),
        origin=Origin(xyz=(front_wheel_x + 0.008, 0.042, wheel_center_z + 0.010)),
        material=dark_metal,
        name="front_right_dropout",
    )
    front_frame.visual(
        Box((0.054, 0.014, 0.040)),
        origin=Origin(xyz=(front_wheel_x + 0.025, -0.045, wheel_center_z + 0.036)),
        material=dark_metal,
        name="front_left_fork_tip",
    )
    front_frame.visual(
        Box((0.054, 0.014, 0.040)),
        origin=Origin(xyz=(front_wheel_x + 0.025, 0.045, wheel_center_z + 0.036)),
        material=dark_metal,
        name="front_right_fork_tip",
    )
    front_frame.visual(
        Box((0.050, 0.080, 0.024)),
        origin=Origin(xyz=(neck_hinge_x - 0.029, 0.0, neck_hinge_z - 0.036)),
        material=dark_metal,
        name="neck_clamp_lower_yoke",
    )
    front_frame.visual(
        Box((0.040, 0.018, 0.034)),
        origin=Origin(xyz=(neck_hinge_x - 0.012, -0.040, neck_hinge_z - 0.012)),
        material=dark_metal,
        name="neck_clamp_left_cheek",
    )
    front_frame.visual(
        Box((0.040, 0.018, 0.034)),
        origin=Origin(xyz=(neck_hinge_x - 0.012, 0.040, neck_hinge_z - 0.012)),
        material=dark_metal,
        name="neck_clamp_right_cheek",
    )
    front_frame.visual(
        Cylinder(radius=clamp_hinge_radius, length=0.022),
        origin=Origin(xyz=(neck_hinge_x, -0.029, neck_hinge_z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="neck_clamp_left_barrel",
    )
    front_frame.visual(
        Cylinder(radius=clamp_hinge_radius, length=0.022),
        origin=Origin(xyz=(neck_hinge_x, 0.029, neck_hinge_z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="neck_clamp_right_barrel",
    )

    upper_stem = model.part("upper_stem")
    upper_stem.visual(
        Cylinder(radius=clamp_hinge_radius * 0.94, length=0.036),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="upper_stem_hinge_barrel",
    )
    upper_stem.visual(
        Box((0.022, 0.030, 0.026)),
        origin=Origin(xyz=(0.016, 0.0, 0.018)),
        material=dark_metal,
        name="upper_stem_hinge_block",
    )
    upper_stem.visual(
        Cylinder(radius=0.017, length=upper_stem_length),
        origin=Origin(
            xyz=(0.5 * upper_stem_length * 0.14, 0.0, 0.5 * upper_stem_length * 0.99),
            rpy=(0.0, neck_pitch, 0.0),
        ),
        material=aluminum,
        name="upper_stem_tube",
    )
    upper_stem.visual(
        Cylinder(radius=0.013, length=0.460),
        origin=Origin(xyz=(0.081, 0.0, 0.574), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="handlebar_bar",
    )
    upper_stem.visual(
        Cylinder(radius=0.016, length=0.110),
        origin=Origin(xyz=(0.081, -0.175, 0.574), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=grip_rubber,
        name="left_grip",
    )
    upper_stem.visual(
        Cylinder(radius=0.016, length=0.110),
        origin=Origin(xyz=(0.081, 0.175, 0.574), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=grip_rubber,
        name="right_grip",
    )

    rear_brake = model.part("rear_brake")
    rear_brake.visual(
        Cylinder(radius=0.0075, length=0.040),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="rear_brake_center_barrel",
    )
    rear_brake.visual(
        Box((0.028, 0.036, 0.020)),
        origin=Origin(xyz=(0.018, 0.0, 0.012)),
        material=dark_metal,
        name="rear_brake_bridge",
    )
    rear_brake.visual(
        _mesh(
            "rear_fender_shell_mesh",
            ExtrudeGeometry(_rear_fender_profile(), 0.106, center=True).rotate_x(pi / 2.0),
        ),
        material=aluminum,
        name="rear_fender_shell",
    )
    rear_brake.visual(
        Box((0.050, 0.012, 0.018)),
        origin=Origin(xyz=(0.046, -0.040, 0.012)),
        material=dark_metal,
        name="rear_fender_left_rib",
    )
    rear_brake.visual(
        Box((0.050, 0.012, 0.018)),
        origin=Origin(xyz=(0.046, 0.040, 0.012)),
        material=dark_metal,
        name="rear_fender_right_rib",
    )
    rear_brake.visual(
        Box((0.050, 0.094, 0.008)),
        origin=Origin(xyz=(0.082, 0.0, 0.016)),
        material=deck_black,
        name="rear_brake_pad",
    )

    front_wheel = model.part("front_wheel")
    _add_wheel_visuals(
        front_wheel,
        "front",
        radius=wheel_radius,
        width=wheel_width,
        rim_material=aluminum,
    )
    front_wheel.visual(
        _mesh(
            "front_tire_mesh",
            TorusGeometry(
                radius=wheel_radius - (wheel_width * 0.5),
                tube=wheel_width * 0.5,
                radial_segments=20,
                tubular_segments=44,
            ).rotate_x(pi / 2.0),
        ),
        material=tire_black,
        name="front_tire",
    )
    front_wheel.visual(
        Cylinder(radius=wheel_radius * 0.08, length=0.072),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="front_axle_cap",
    )
    front_wheel.visual(
        Box((0.012, wheel_width * 0.24, 0.016)),
        origin=Origin(xyz=(wheel_radius * 0.58, 0.0, 0.0)),
        material=aluminum,
        name="front_valve_stem",
    )

    rear_wheel = model.part("rear_wheel")
    _add_wheel_visuals(
        rear_wheel,
        "rear",
        radius=wheel_radius,
        width=wheel_width,
        rim_material=aluminum,
    )
    rear_wheel.visual(
        _mesh(
            "rear_tire_mesh",
            TorusGeometry(
                radius=wheel_radius - (wheel_width * 0.5),
                tube=wheel_width * 0.5,
                radial_segments=20,
                tubular_segments=44,
            ).rotate_x(pi / 2.0),
        ),
        material=tire_black,
        name="rear_tire",
    )
    rear_wheel.visual(
        Cylinder(radius=wheel_radius * 0.08, length=0.072),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="rear_axle_cap",
    )
    rear_wheel.visual(
        Box((0.012, wheel_width * 0.24, 0.016)),
        origin=Origin(xyz=(wheel_radius * 0.58, 0.0, 0.0)),
        material=aluminum,
        name="rear_valve_stem",
    )

    model.articulation(
        "deck_hinge",
        ArticulationType.REVOLUTE,
        parent=rear_frame,
        child=front_frame,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=0.0, upper=1.45),
    )
    model.articulation(
        "stem_fold",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=upper_stem,
        origin=Origin(xyz=(neck_hinge_x, 0.0, neck_hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.2, lower=0.0, upper=2.25),
    )
    model.articulation(
        "rear_brake_fold",
        ArticulationType.REVOLUTE,
        parent=rear_frame,
        child=rear_brake,
        origin=Origin(xyz=(brake_hinge_x, 0.0, brake_hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=0.0, upper=0.15),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_frame,
        child=front_wheel,
        origin=Origin(xyz=(front_wheel_x, 0.0, wheel_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )
    model.articulation(
        "rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=rear_frame,
        child=rear_wheel,
        origin=Origin(xyz=(rear_wheel_x, 0.0, wheel_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
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

    rear_frame = object_model.get_part("rear_frame")
    front_frame = object_model.get_part("front_frame")
    upper_stem = object_model.get_part("upper_stem")
    rear_brake = object_model.get_part("rear_brake")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")

    deck_hinge = object_model.get_articulation("deck_hinge")
    stem_fold = object_model.get_articulation("stem_fold")
    brake_fold = object_model.get_articulation("rear_brake_fold")
    front_spin = object_model.get_articulation("front_wheel_spin")
    rear_spin = object_model.get_articulation("rear_wheel_spin")

    ctx.check(
        "scooter articulation types are correct",
        deck_hinge.articulation_type == ArticulationType.REVOLUTE
        and stem_fold.articulation_type == ArticulationType.REVOLUTE
        and brake_fold.articulation_type == ArticulationType.REVOLUTE
        and front_spin.articulation_type == ArticulationType.CONTINUOUS
        and rear_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"deck={deck_hinge.articulation_type}, stem={stem_fold.articulation_type}, "
            f"brake={brake_fold.articulation_type}, front={front_spin.articulation_type}, "
            f"rear={rear_spin.articulation_type}"
        ),
    )
    ctx.check(
        "wheel spin joints are unbounded continuous joints",
        front_spin.motion_limits is not None
        and rear_spin.motion_limits is not None
        and front_spin.motion_limits.lower is None
        and front_spin.motion_limits.upper is None
        and rear_spin.motion_limits.lower is None
        and rear_spin.motion_limits.upper is None,
        details=f"front={front_spin.motion_limits}, rear={rear_spin.motion_limits}",
    )

    ctx.expect_origin_gap(
        rear_wheel,
        front_wheel,
        axis="x",
        min_gap=0.54,
        max_gap=0.68,
        name="city scooter wheelbase stays compact but rideable",
    )
    ctx.expect_contact(
        front_frame,
        rear_frame,
        elem_a="front_hinge_center_barrel",
        elem_b="rear_hinge_left_barrel",
        name="front deck hinge barrel meets left rear knuckle",
    )
    ctx.expect_contact(
        front_frame,
        rear_frame,
        elem_a="front_hinge_center_barrel",
        elem_b="rear_hinge_right_barrel",
        name="front deck hinge barrel meets right rear knuckle",
    )
    ctx.expect_contact(
        upper_stem,
        front_frame,
        elem_a="upper_stem_hinge_barrel",
        elem_b="neck_clamp_left_barrel",
        name="upper stem hinge barrel seats into left neck clamp",
    )
    ctx.expect_contact(
        upper_stem,
        front_frame,
        elem_a="upper_stem_hinge_barrel",
        elem_b="neck_clamp_right_barrel",
        name="upper stem hinge barrel seats into right neck clamp",
    )
    ctx.expect_contact(
        front_frame,
        front_wheel,
        elem_a="front_left_dropout",
        elem_b="front_axle_cap",
        name="front wheel axle seats in the fork dropout",
    )
    ctx.expect_contact(
        rear_frame,
        rear_wheel,
        elem_a="rear_left_dropout",
        elem_b="rear_axle_cap",
        name="rear wheel axle seats in the rear dropout",
    )
    ctx.expect_overlap(
        rear_brake,
        rear_wheel,
        axes="xy",
        elem_a="rear_brake_pad",
        elem_b="rear_tire",
        min_overlap=0.025,
        name="rear fender brake pad covers the tire footprint",
    )
    ctx.expect_gap(
        rear_brake,
        rear_wheel,
        axis="z",
        positive_elem="rear_brake_pad",
        negative_elem="rear_tire",
        min_gap=0.004,
        max_gap=0.018,
        name="rear fender brake rests just above the tire",
    )

    front_deck_rest = _aabb_center(ctx.part_element_world_aabb(front_frame, elem="front_deck_shell"))
    with ctx.pose({deck_hinge: deck_hinge.motion_limits.upper}):
        front_deck_folded = _aabb_center(ctx.part_element_world_aabb(front_frame, elem="front_deck_shell"))
    ctx.check(
        "deck center hinge folds the front half upward",
        front_deck_rest is not None
        and front_deck_folded is not None
        and front_deck_folded[2] > front_deck_rest[2] + 0.11
        and front_deck_folded[0] > front_deck_rest[0] + 0.08,
        details=f"rest={front_deck_rest}, folded={front_deck_folded}",
    )

    handlebar_rest = _aabb_center(ctx.part_element_world_aabb(upper_stem, elem="handlebar_bar"))
    with ctx.pose({stem_fold: stem_fold.motion_limits.upper}):
        handlebar_folded = _aabb_center(ctx.part_element_world_aabb(upper_stem, elem="handlebar_bar"))
    ctx.check(
        "upper neck clamp lets the handlebar stem fold rearward",
        handlebar_rest is not None
        and handlebar_folded is not None
        and handlebar_folded[0] > handlebar_rest[0] + 0.14
        and handlebar_folded[2] < handlebar_rest[2] - 0.18,
        details=f"rest={handlebar_rest}, folded={handlebar_folded}",
    )

    brake_rest = _aabb_center(ctx.part_element_world_aabb(rear_brake, elem="rear_brake_pad"))
    with ctx.pose({brake_fold: brake_fold.motion_limits.upper}):
        brake_pressed = _aabb_center(ctx.part_element_world_aabb(rear_brake, elem="rear_brake_pad"))
        ctx.expect_gap(
            rear_brake,
            rear_wheel,
            axis="z",
            positive_elem="rear_brake_pad",
            negative_elem="rear_tire",
            max_gap=0.008,
            max_penetration=0.0,
            name="rear fender brake can sweep down to the tire",
        )
    ctx.check(
        "rear fender brake pivots downward toward the tire",
        brake_rest is not None
        and brake_pressed is not None
        and brake_pressed[2] < brake_rest[2] - 0.008,
        details=f"rest={brake_rest}, pressed={brake_pressed}",
    )

    front_valve_rest = _aabb_center(ctx.part_element_world_aabb(front_wheel, elem="front_valve_stem"))
    rear_valve_rest = _aabb_center(ctx.part_element_world_aabb(rear_wheel, elem="rear_valve_stem"))
    with ctx.pose({front_spin: pi / 2.0, rear_spin: pi / 2.0}):
        front_valve_turned = _aabb_center(ctx.part_element_world_aabb(front_wheel, elem="front_valve_stem"))
        rear_valve_turned = _aabb_center(ctx.part_element_world_aabb(rear_wheel, elem="rear_valve_stem"))
    ctx.check(
        "front wheel spin moves a visible wheel feature around the axle",
        front_valve_rest is not None
        and front_valve_turned is not None
        and abs(front_valve_rest[0] - front_valve_turned[0]) > 0.05
        and abs(front_valve_rest[2] - front_valve_turned[2]) > 0.05,
        details=f"rest={front_valve_rest}, turned={front_valve_turned}",
    )
    ctx.check(
        "rear wheel spin moves a visible wheel feature around the axle",
        rear_valve_rest is not None
        and rear_valve_turned is not None
        and abs(rear_valve_rest[0] - rear_valve_turned[0]) > 0.05
        and abs(rear_valve_rest[2] - rear_valve_turned[2]) > 0.05,
        details=f"rest={rear_valve_rest}, turned={rear_valve_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
