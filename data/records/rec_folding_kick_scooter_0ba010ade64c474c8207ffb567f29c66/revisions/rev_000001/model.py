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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


WHEEL_RADIUS = 0.105
TIRE_WIDTH = 0.048
WHEEL_HUB_WIDTH = 0.060
INNER_TUBE_RADIUS = 0.0168
GUIDE_BUSHING_RADIUS = 0.0188
DECK_LENGTH = 0.460
DECK_WIDTH = 0.155
DECK_THICKNESS = 0.030
DECK_CENTER_Y = 0.005
DECK_BOTTOM_Z = 0.080
HINGE_Y = 0.245
HINGE_Z = 0.205
STEM_TILT = 0.18
FOLD_LIMIT = 1.16
STEM_EXTENSION = 0.130
LOWER_AXLE_Y = 0.110
LOWER_AXLE_Z = -0.110
SLIDE_JOINT_Z = 0.370
REAR_AXLE_Y = -0.335
REAR_AXLE_Z = 0.105


def _tube_shell(name: str, *, outer_radius: float, inner_radius: float, z0: float, z1: float):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, z0), (outer_radius, z1)],
            [(inner_radius, z0), (inner_radius, z1)],
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def _pneumatic_tire_mesh(name: str, *, wheel_radius: float, tire_width: float):
    half_width = tire_width * 0.5
    profile = [
        (wheel_radius * 0.56, -half_width * 0.98),
        (wheel_radius * 0.77, -half_width * 1.00),
        (wheel_radius * 0.92, -half_width * 0.80),
        (wheel_radius * 0.99, -half_width * 0.42),
        (wheel_radius, 0.0),
        (wheel_radius * 0.99, half_width * 0.42),
        (wheel_radius * 0.92, half_width * 0.80),
        (wheel_radius * 0.77, half_width * 1.00),
        (wheel_radius * 0.56, half_width * 0.98),
        (wheel_radius * 0.42, half_width * 0.34),
        (wheel_radius * 0.39, 0.0),
        (wheel_radius * 0.42, -half_width * 0.34),
        (wheel_radius * 0.56, -half_width * 0.98),
    ]
    return mesh_from_geometry(
        LatheGeometry(profile, segments=64).rotate_y(math.pi / 2.0),
        name,
    )


def _add_wheel(
    part,
    *,
    name_prefix: str,
    wheel_radius: float,
    tire_width: float,
    hub_width: float,
    rubber,
    rim,
    hub_material,
    motor_hub: bool = False,
) -> None:
    spin_origin = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    part.visual(
        _pneumatic_tire_mesh(
            f"{name_prefix}_tire",
            wheel_radius=wheel_radius,
            tire_width=tire_width,
        ),
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=wheel_radius * 0.60, length=tire_width * 0.18),
        origin=Origin(xyz=(tire_width * 0.18, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rim,
        name="left_rim_face",
    )
    part.visual(
        Cylinder(radius=wheel_radius * 0.60, length=tire_width * 0.18),
        origin=Origin(xyz=(-tire_width * 0.18, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rim,
        name="right_rim_face",
    )
    part.visual(
        Cylinder(radius=wheel_radius * (0.24 if motor_hub else 0.19), length=hub_width),
        origin=spin_origin,
        material=hub_material,
        name="hub",
    )
    part.visual(
        Cylinder(radius=wheel_radius * 0.08, length=hub_width * 0.92),
        origin=spin_origin,
        material=rim,
        name="axle_boss",
    )
    if motor_hub:
        part.visual(
            Cylinder(radius=wheel_radius * 0.31, length=hub_width * 0.72),
            origin=spin_origin,
            material=hub_material,
            name="motor_shell",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="foldable_commuter_scooter")

    matte_black = model.material("matte_black", rgba=(0.14, 0.14, 0.15, 1.0))
    satin_black = model.material("satin_black", rgba=(0.18, 0.18, 0.19, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    aluminum = model.material("aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    grip_tape = model.material("grip_tape", rgba=(0.10, 0.10, 0.11, 1.0))
    lens = model.material("lens", rgba=(0.82, 0.88, 0.92, 1.0))

    deck_top_z = DECK_BOTTOM_Z + DECK_THICKNESS

    chassis = model.part("chassis")
    chassis.inertial = Inertial.from_geometry(
        Box((0.78, 0.34, 0.26)),
        mass=11.5,
        origin=Origin(xyz=(0.0, -0.03, 0.13)),
    )

    deck_shell = ExtrudeGeometry.from_z0(
        rounded_rect_profile(DECK_WIDTH, DECK_LENGTH, radius=0.028),
        DECK_THICKNESS,
        cap=True,
        closed=True,
    ).translate(0.0, DECK_CENTER_Y, DECK_BOTTOM_Z)
    chassis.visual(
        mesh_from_geometry(deck_shell, "scooter_deck_shell"),
        material=matte_black,
        name="deck_shell",
    )
    chassis.visual(
        Box((0.128, 0.385, 0.004)),
        origin=Origin(xyz=(0.0, DECK_CENTER_Y, deck_top_z + 0.002)),
        material=grip_tape,
        name="deck_grip",
    )
    chassis.visual(
        Box((0.112, 0.320, 0.022)),
        origin=Origin(xyz=(0.0, -0.005, DECK_BOTTOM_Z - 0.011)),
        material=dark_gray,
        name="battery_case",
    )
    chassis.visual(
        Box((0.072, 0.060, 0.070)),
        origin=Origin(xyz=(0.0, 0.208, 0.145)),
        material=dark_gray,
        name="neck_block",
    )
    chassis.visual(
        Box((0.026, 0.040, 0.056)),
        origin=Origin(xyz=(0.055, HINGE_Y, HINGE_Z)),
        material=satin_black,
        name="left_hinge_lug",
    )
    chassis.visual(
        Box((0.026, 0.040, 0.056)),
        origin=Origin(xyz=(-0.055, HINGE_Y, HINGE_Z)),
        material=satin_black,
        name="right_hinge_lug",
    )
    chassis.visual(
        Box((0.070, 0.030, 0.028)),
        origin=Origin(xyz=(0.0, HINGE_Y - 0.015, HINGE_Z - 0.036)),
        material=satin_black,
        name="hinge_pedestal",
    )

    left_gusset = tube_from_spline_points(
        [
            (0.046, 0.168, deck_top_z - 0.002),
            (0.053, 0.204, 0.144),
            (0.060, HINGE_Y - 0.014, HINGE_Z - 0.014),
        ],
        radius=0.009,
        samples_per_segment=12,
        radial_segments=16,
        cap_ends=True,
    )
    right_gusset = tube_from_spline_points(
        [
            (-0.046, 0.168, deck_top_z - 0.002),
            (-0.053, 0.204, 0.144),
            (-0.060, HINGE_Y - 0.014, HINGE_Z - 0.014),
        ],
        radius=0.009,
        samples_per_segment=12,
        radial_segments=16,
        cap_ends=True,
    )
    chassis.visual(mesh_from_geometry(left_gusset, "scooter_left_neck_gusset"), material=matte_black, name="left_neck_gusset")
    chassis.visual(mesh_from_geometry(right_gusset, "scooter_right_neck_gusset"), material=matte_black, name="right_neck_gusset")

    chassis.visual(
        Box((0.026, 0.060, 0.080)),
        origin=Origin(xyz=(0.043, REAR_AXLE_Y, REAR_AXLE_Z + 0.005)),
        material=satin_black,
        name="left_rear_dropout",
    )
    chassis.visual(
        Box((0.026, 0.060, 0.080)),
        origin=Origin(xyz=(-0.043, REAR_AXLE_Y, REAR_AXLE_Z + 0.005)),
        material=satin_black,
        name="right_rear_dropout",
    )

    left_rear_stay = tube_from_spline_points(
        [
            (0.052, -0.215, DECK_BOTTOM_Z + 0.010),
            (0.047, -0.270, 0.145),
            (0.043, REAR_AXLE_Y, REAR_AXLE_Z + 0.030),
        ],
        radius=0.010,
        samples_per_segment=14,
        radial_segments=16,
        cap_ends=True,
    )
    right_rear_stay = tube_from_spline_points(
        [
            (-0.052, -0.215, DECK_BOTTOM_Z + 0.010),
            (-0.047, -0.270, 0.145),
            (-0.043, REAR_AXLE_Y, REAR_AXLE_Z + 0.030),
        ],
        radius=0.010,
        samples_per_segment=14,
        radial_segments=16,
        cap_ends=True,
    )
    chassis.visual(mesh_from_geometry(left_rear_stay, "scooter_left_rear_stay"), material=matte_black, name="left_rear_stay")
    chassis.visual(mesh_from_geometry(right_rear_stay, "scooter_right_rear_stay"), material=matte_black, name="right_rear_stay")

    chassis.visual(
        Box((0.098, 0.100, 0.008)),
        origin=Origin(xyz=(0.0, -0.335, 0.232)),
        material=dark_gray,
        name="rear_fender_top",
    )
    chassis.visual(
        Box((0.012, 0.060, 0.096)),
        origin=Origin(xyz=(0.043, -0.335, 0.184)),
        material=dark_gray,
        name="left_fender_support",
    )
    chassis.visual(
        Box((0.012, 0.060, 0.096)),
        origin=Origin(xyz=(-0.043, -0.335, 0.184)),
        material=dark_gray,
        name="right_fender_support",
    )
    chassis.visual(
        Box((0.030, 0.028, 0.058)),
        origin=Origin(xyz=(0.0, -0.318, 0.241)),
        material=satin_black,
        name="fender_catch_bracket",
    )
    chassis.visual(
        Cylinder(radius=0.013, length=0.082),
        origin=Origin(xyz=(0.0, -0.318, 0.225), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="fender_catch",
    )
    chassis.visual(
        Box((0.070, 0.012, 0.050)),
        origin=Origin(xyz=(0.0, -0.392, 0.264)),
        material=satin_black,
        name="rear_light_housing",
    )
    chassis.visual(
        Box((0.018, 0.074, 0.018)),
        origin=Origin(xyz=(0.0, -0.355, 0.252)),
        material=satin_black,
        name="rear_light_stalk",
    )

    lower_stem = model.part("lower_stem")
    lower_stem.inertial = Inertial.from_geometry(
        Box((0.20, 0.24, 0.62)),
        mass=4.4,
        origin=Origin(xyz=(0.0, 0.040, 0.140)),
    )
    lower_stem.visual(
        Cylinder(radius=0.020, length=0.084),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="hinge_barrel",
    )
    lower_stem.visual(
        Box((0.064, 0.038, 0.060)),
        origin=Origin(xyz=(0.0, 0.005, 0.015)),
        material=dark_gray,
        name="hinge_head",
    )
    lower_stem.visual(
        Cylinder(radius=0.023, length=0.375),
        origin=Origin(xyz=(0.0, 0.0, 0.2075)),
        material=matte_black,
        name="stem_sleeve",
    )
    lower_stem.visual(
        Cylinder(radius=0.028, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.365)),
        material=dark_gray,
        name="stem_collar",
    )
    lower_stem.visual(
        Box((0.020, 0.045, 0.065)),
        origin=Origin(xyz=(0.0, 0.030, 0.085)),
        material=dark_gray,
        name="headlight_mount",
    )
    lower_stem.visual(
        Box((0.016, 0.028, 0.026)),
        origin=Origin(xyz=(0.0, 0.055, 0.100)),
        material=lens,
        name="headlight",
    )

    lower_stem.visual(
        Box((0.076, 0.046, 0.030)),
        origin=Origin(xyz=(0.0, 0.045, 0.015)),
        material=dark_gray,
        name="fork_crown",
    )
    lower_stem.visual(
        Box((0.018, 0.060, 0.150)),
        origin=Origin(xyz=(0.046, 0.082, -0.045)),
        material=matte_black,
        name="left_fork_leg",
    )
    lower_stem.visual(
        Box((0.018, 0.060, 0.150)),
        origin=Origin(xyz=(-0.046, 0.082, -0.045)),
        material=matte_black,
        name="right_fork_leg",
    )
    lower_stem.visual(
        Box((0.020, 0.032, 0.052)),
        origin=Origin(xyz=(0.040, LOWER_AXLE_Y, LOWER_AXLE_Z)),
        material=satin_black,
        name="left_fork_end",
    )
    lower_stem.visual(
        Box((0.020, 0.032, 0.052)),
        origin=Origin(xyz=(-0.040, LOWER_AXLE_Y, LOWER_AXLE_Z)),
        material=satin_black,
        name="right_fork_end",
    )
    lower_stem.visual(
        Box((0.102, 0.066, 0.010)),
        origin=Origin(xyz=(0.0, LOWER_AXLE_Y, 0.007)),
        material=dark_gray,
        name="front_fender",
    )

    upper_stem = model.part("upper_stem")
    upper_stem.inertial = Inertial.from_geometry(
        Box((0.62, 0.20, 0.76)),
        mass=2.9,
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
    )
    upper_stem.visual(
        Cylinder(radius=INNER_TUBE_RADIUS, length=0.700),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=aluminum,
        name="inner_tube",
    )
    upper_stem.visual(
        Cylinder(radius=0.028, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.493)),
        material=dark_gray,
        name="bar_clamp",
    )

    handlebar = tube_from_spline_points(
        [
            (-0.275, 0.0, 0.515),
            (-0.170, 0.0, 0.548),
            (-0.060, 0.0, 0.565),
            (0.060, 0.0, 0.565),
            (0.170, 0.0, 0.548),
            (0.275, 0.0, 0.515),
        ],
        radius=0.012,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    upper_stem.visual(mesh_from_geometry(handlebar, "scooter_handlebar"), material=matte_black, name="handlebar")
    upper_stem.visual(
        Cylinder(radius=0.018, length=0.115),
        origin=Origin(xyz=(0.280, 0.0, 0.515), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="right_grip",
    )
    upper_stem.visual(
        Cylinder(radius=0.018, length=0.115),
        origin=Origin(xyz=(-0.280, 0.0, 0.515), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    upper_stem.visual(
        Box((0.060, 0.030, 0.045)),
        origin=Origin(xyz=(0.0, -0.008, 0.550)),
        material=dark_gray,
        name="display",
    )
    upper_stem.visual(
        Box((0.010, 0.050, 0.060)),
        origin=Origin(xyz=(0.238, -0.010, 0.485), rpy=(0.35, 0.0, 0.0)),
        material=matte_black,
        name="brake_lever",
    )
    upper_stem.visual(
        Box((0.022, 0.060, 0.078)),
        origin=Origin(xyz=(0.0, -0.030, 0.206)),
        material=dark_gray,
        name="hook_mount",
    )
    hook_latch = tube_from_spline_points(
        [
            (0.0, -0.058, 0.232),
            (0.0, -0.084, 0.220),
            (0.0, -0.094, 0.194),
            (0.0, -0.084, 0.166),
        ],
        radius=0.006,
        samples_per_segment=14,
        radial_segments=16,
        cap_ends=True,
    )
    upper_stem.visual(
        mesh_from_geometry(hook_latch, "scooter_hook_latch"),
        material=aluminum,
        name="hook_latch",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=TIRE_WIDTH),
        mass=1.9,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _add_wheel(
        front_wheel,
        name_prefix="scooter_front_wheel",
        wheel_radius=WHEEL_RADIUS,
        tire_width=TIRE_WIDTH,
        hub_width=WHEEL_HUB_WIDTH,
        rubber=rubber,
        rim=aluminum,
        hub_material=dark_gray,
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=TIRE_WIDTH),
        mass=2.4,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _add_wheel(
        rear_wheel,
        name_prefix="scooter_rear_wheel",
        wheel_radius=WHEEL_RADIUS,
        tire_width=TIRE_WIDTH,
        hub_width=WHEEL_HUB_WIDTH,
        rubber=rubber,
        rim=aluminum,
        hub_material=dark_gray,
        motor_hub=True,
    )

    model.articulation(
        "chassis_to_lower_stem",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=lower_stem,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(STEM_TILT, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=2.5,
            lower=0.0,
            upper=FOLD_LIMIT,
        ),
    )
    model.articulation(
        "lower_to_upper_stem",
        ArticulationType.PRISMATIC,
        parent=lower_stem,
        child=upper_stem,
        origin=Origin(xyz=(0.0, 0.0, SLIDE_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.20,
            lower=0.0,
            upper=STEM_EXTENSION,
        ),
    )
    model.articulation(
        "lower_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=lower_stem,
        child=front_wheel,
        origin=Origin(xyz=(0.0, LOWER_AXLE_Y, LOWER_AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=24.0),
    )
    model.articulation(
        "chassis_to_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=rear_wheel,
        origin=Origin(xyz=(0.0, REAR_AXLE_Y, REAR_AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=32.0, velocity=24.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    lower_stem = object_model.get_part("lower_stem")
    upper_stem = object_model.get_part("upper_stem")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")

    fold_joint = object_model.get_articulation("chassis_to_lower_stem")
    slide_joint = object_model.get_articulation("lower_to_upper_stem")

    left_hinge_lug = chassis.get_visual("left_hinge_lug")
    right_hinge_lug = chassis.get_visual("right_hinge_lug")
    left_rear_dropout = chassis.get_visual("left_rear_dropout")
    right_rear_dropout = chassis.get_visual("right_rear_dropout")
    fender_catch = chassis.get_visual("fender_catch")
    stem_sleeve = lower_stem.get_visual("stem_sleeve")
    hinge_barrel = lower_stem.get_visual("hinge_barrel")
    left_fork_end = lower_stem.get_visual("left_fork_end")
    right_fork_end = lower_stem.get_visual("right_fork_end")
    inner_tube = upper_stem.get_visual("inner_tube")
    hook_latch = upper_stem.get_visual("hook_latch")
    front_hub = front_wheel.get_visual("hub")
    rear_hub = rear_wheel.get_visual("hub")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        lower_stem,
        upper_stem,
        elem_a=stem_sleeve,
        elem_b=inner_tube,
        reason="The lower stem sleeve is represented as a solid exterior proxy around the telescoping inner tube.",
    )
    ctx.allow_overlap(
        lower_stem,
        upper_stem,
        elem_a="stem_collar",
        elem_b=inner_tube,
        reason="The clamp collar is part of the same telescoping sleeve proxy around the inner tube.",
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
        lower_stem,
        chassis,
        elem_a=hinge_barrel,
        elem_b=left_hinge_lug,
        name="left hinge lug carries the folding stem barrel",
    )
    ctx.expect_contact(
        lower_stem,
        chassis,
        elem_a=hinge_barrel,
        elem_b=right_hinge_lug,
        name="right hinge lug carries the folding stem barrel",
    )
    ctx.expect_contact(
        front_wheel,
        lower_stem,
        elem_a=front_hub,
        elem_b=left_fork_end,
        name="front hub seats against the left fork end",
    )
    ctx.expect_contact(
        front_wheel,
        lower_stem,
        elem_a=front_hub,
        elem_b=right_fork_end,
        name="front hub seats against the right fork end",
    )
    ctx.expect_contact(
        rear_wheel,
        chassis,
        elem_a=rear_hub,
        elem_b=left_rear_dropout,
        name="rear motor hub seats against the left dropout",
    )
    ctx.expect_contact(
        rear_wheel,
        chassis,
        elem_a=rear_hub,
        elem_b=right_rear_dropout,
        name="rear motor hub seats against the right dropout",
    )
    ctx.expect_overlap(
        upper_stem,
        lower_stem,
        axes="z",
        elem_a=inner_tube,
        elem_b=stem_sleeve,
        min_overlap=0.110,
        name="collapsed inner tube keeps deep retained insertion",
    )

    rest_upper_pos = ctx.part_world_position(upper_stem)
    with ctx.pose({slide_joint: STEM_EXTENSION}):
        ctx.expect_overlap(
            upper_stem,
            lower_stem,
            axes="z",
            elem_a=inner_tube,
            elem_b=stem_sleeve,
            min_overlap=0.075,
            name="extended inner tube still retains insertion",
        )
        extended_upper_pos = ctx.part_world_position(upper_stem)

    ctx.check(
        "telescoping stem extends upward",
        rest_upper_pos is not None
        and extended_upper_pos is not None
        and extended_upper_pos[2] > rest_upper_pos[2] + 0.11,
        details=f"rest={rest_upper_pos}, extended={extended_upper_pos}",
    )

    with ctx.pose({fold_joint: FOLD_LIMIT, slide_joint: 0.0}):
        folded_upper_pos = ctx.part_world_position(upper_stem)
        ctx.expect_contact(
            upper_stem,
            chassis,
            elem_a=hook_latch,
            elem_b=fender_catch,
            name="folded stem hook clips onto the rear fender catch",
        )

    ctx.check(
        "fold hinge swings the stem rearward over the deck",
        rest_upper_pos is not None
        and folded_upper_pos is not None
        and folded_upper_pos[1] < rest_upper_pos[1] - 0.25
        and folded_upper_pos[1] < -0.10
        and folded_upper_pos[2] < rest_upper_pos[2] - 0.25
        and folded_upper_pos[2] < 0.32,
        details=f"rest={rest_upper_pos}, folded={folded_upper_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
