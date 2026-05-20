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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)

BB_SHELL_LENGTH = 0.078
BB_OUTER_RADIUS = 0.020
BB_INNER_RADIUS = 0.011
SPINDLE_LENGTH = 0.170
SPINDLE_RADIUS = 0.011
CRANK_LENGTH = 0.168
ARM_BOSS_LENGTH = 0.022
ARM_BOSS_RADIUS = 0.018
PEDAL_EYE_LENGTH = 0.016
PEDAL_EYE_OUTER_RADIUS = 0.012
PEDAL_EYE_INNER_RADIUS = 0.006
SPROCKET_THICKNESS = 0.004
CYLINDER_X = Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _circle_profile(radius: float, *, segments: int = 32) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _ring_shell_geometry(length: float, outer_radius: float, inner_radius: float, *, segments: int = 56):
    half_length = length * 0.5
    return LatheGeometry.from_shell_profiles(
        [
            (outer_radius, -half_length),
            (outer_radius, half_length),
        ],
        [
            (inner_radius, -half_length),
            (inner_radius, half_length),
        ],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(math.pi / 2.0)


def _arm_section(width: float, height: float, z_pos: float, *, x_center: float = 0.011, y_center: float = 0.0):
    profile = rounded_rect_profile(
        width,
        height,
        radius=min(width, height) * 0.26,
        corner_segments=6,
    )
    return [(x_center + x, y_center + y, z_pos) for x, y in profile]


def _crank_arm_geometry():
    sections = [
        _arm_section(0.030, 0.018, -0.004, y_center=0.000),
        _arm_section(0.025, 0.016, -0.060, y_center=0.002),
        _arm_section(0.021, 0.014, -0.126, y_center=0.001),
        _arm_section(0.019, 0.013, -0.160, y_center=0.000),
    ]
    return repair_loft(section_loft(sections))


def _sprocket_geometry():
    tooth_count = 25
    root_radius = 0.067
    tip_radius = 0.072
    outer_profile: list[tuple[float, float]] = []
    tooth_step = (2.0 * math.pi) / tooth_count
    for tooth_index in range(tooth_count):
        base_angle = tooth_index * tooth_step
        outer_profile.extend(
            [
                (root_radius * math.cos(base_angle), root_radius * math.sin(base_angle)),
                (
                    tip_radius * math.cos(base_angle + tooth_step * 0.22),
                    tip_radius * math.sin(base_angle + tooth_step * 0.22),
                ),
                (
                    tip_radius * math.cos(base_angle + tooth_step * 0.52),
                    tip_radius * math.sin(base_angle + tooth_step * 0.52),
                ),
                (
                    root_radius * math.cos(base_angle + tooth_step * 0.86),
                    root_radius * math.sin(base_angle + tooth_step * 0.86),
                ),
            ]
        )

    bolt_circle_radius = 0.038
    hole_profiles = [_circle_profile(0.012, segments=32)]
    for bolt_index in range(5):
        angle = (2.0 * math.pi * bolt_index) / 5.0
        hole_profiles.append(
            [
                (
                    bolt_circle_radius * math.cos(angle) + 0.0045 * math.cos(theta),
                    bolt_circle_radius * math.sin(angle) + 0.0045 * math.sin(theta),
                )
                for theta in [2.0 * math.pi * step / 18.0 for step in range(18)]
            ]
        )
    for pocket_index in range(5):
        angle = (2.0 * math.pi * pocket_index) / 5.0 + math.pi / 5.0
        hole_profiles.append(
            [
                (
                    0.050 * math.cos(angle) + 0.0085 * math.cos(theta),
                    0.050 * math.sin(angle) + 0.0085 * math.sin(theta),
                )
                for theta in [2.0 * math.pi * step / 24.0 for step in range(24)]
            ]
        )

    return ExtrudeWithHolesGeometry(
        outer_profile,
        hole_profiles,
        height=SPROCKET_THICKNESS,
        center=True,
        closed=True,
    ).rotate_y(math.pi / 2.0)


def _build_pedal(part, body_material, deck_material, pin_material) -> None:
    part.visual(
        Cylinder(radius=PEDAL_EYE_INNER_RADIUS, length=0.066),
        origin=Origin(xyz=(0.033, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pin_material,
        name="axle",
    )
    part.visual(
        Box((0.010, 0.084, 0.012)),
        origin=Origin(xyz=(0.022, 0.0, 0.005)),
        material=body_material,
        name="inner_rail",
    )
    part.visual(
        Box((0.060, 0.012, 0.008)),
        origin=Origin(xyz=(0.048, 0.041, 0.008)),
        material=body_material,
        name="front_rail",
    )
    part.visual(
        Box((0.060, 0.012, 0.008)),
        origin=Origin(xyz=(0.048, -0.041, 0.008)),
        material=body_material,
        name="rear_rail",
    )
    part.visual(
        Box((0.010, 0.084, 0.012)),
        origin=Origin(xyz=(0.074, 0.0, 0.005)),
        material=body_material,
        name="outer_rail",
    )
    part.visual(
        Box((0.056, 0.066, 0.004)),
        origin=Origin(xyz=(0.048, 0.0, -0.002)),
        material=deck_material,
        name="deck",
    )
    pin_positions = (
        (0.028, 0.026),
        (0.028, -0.026),
        (0.046, 0.026),
        (0.046, -0.026),
        (0.064, 0.026),
        (0.064, -0.026),
    )
    for pin_index, (x_pos, y_pos) in enumerate(pin_positions):
        part.visual(
            Cylinder(radius=0.0016, length=0.012),
            origin=Origin(xyz=(x_pos, y_pos, 0.006)),
            material=pin_material,
            name=f"pin_{pin_index}",
        )


def _add_spider_tabs(part, material) -> None:
    for tab_index in range(5):
        angle = (2.0 * math.pi * tab_index) / 5.0
        part.visual(
            Box((0.006, 0.010, 0.042)),
            origin=Origin(
                xyz=(-0.001, 0.024 * math.sin(angle), 0.024 * math.cos(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=material,
            name=f"spider_tab_{tab_index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bmx_crankset", assets=ASSETS)

    black_oxide = model.material("black_oxide", rgba=(0.11, 0.11, 0.12, 1.0))
    forged_alloy = model.material("forged_alloy", rgba=(0.56, 0.57, 0.60, 1.0))
    chainring_alloy = model.material("chainring_alloy", rgba=(0.70, 0.72, 0.75, 1.0))
    bolt_steel = model.material("bolt_steel", rgba=(0.78, 0.79, 0.80, 1.0))
    pedal_black = model.material("pedal_black", rgba=(0.08, 0.08, 0.09, 1.0))
    pedal_deck = model.material("pedal_deck", rgba=(0.18, 0.19, 0.20, 1.0))

    shell_mesh = _save_mesh(
        "bmx_bottom_bracket_shell.obj",
        _ring_shell_geometry(BB_SHELL_LENGTH, BB_OUTER_RADIUS, BB_INNER_RADIUS),
    )
    cup_mesh = _save_mesh(
        "bmx_bottom_bracket_cup.obj",
        _ring_shell_geometry(0.010, 0.023, BB_INNER_RADIUS, segments=48),
    )
    arm_mesh = _save_mesh("bmx_crank_arm.obj", _crank_arm_geometry())
    pedal_eye_mesh = _save_mesh(
        "bmx_pedal_eye.obj",
        _ring_shell_geometry(PEDAL_EYE_LENGTH, PEDAL_EYE_OUTER_RADIUS, PEDAL_EYE_INNER_RADIUS, segments=48),
    )
    sprocket_mesh = _save_mesh("bmx_sprocket.obj", _sprocket_geometry())

    bottom_bracket = model.part("bottom_bracket")
    bottom_bracket.visual(shell_mesh, material=black_oxide, name="shell")
    bottom_bracket.visual(
        cup_mesh,
        origin=Origin(xyz=(0.034, 0.0, 0.0)),
        material=bolt_steel,
        name="right_cup",
    )
    bottom_bracket.visual(
        cup_mesh,
        origin=Origin(xyz=(-0.034, 0.0, 0.0)),
        material=bolt_steel,
        name="left_cup",
    )
    bottom_bracket.inertial = Inertial.from_geometry(
        Cylinder(radius=BB_OUTER_RADIUS, length=BB_SHELL_LENGTH),
        mass=0.42,
        origin=CYLINDER_X,
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=SPINDLE_RADIUS, length=SPINDLE_LENGTH),
        origin=CYLINDER_X,
        material=black_oxide,
        name="axle",
    )
    spindle.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(0.076, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_oxide,
        name="right_interface",
    )
    spindle.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(-0.076, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_oxide,
        name="left_interface",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.013, length=SPINDLE_LENGTH),
        mass=0.55,
        origin=CYLINDER_X,
    )

    right_arm = model.part("right_arm")
    right_arm.visual(
        Cylinder(radius=ARM_BOSS_RADIUS, length=ARM_BOSS_LENGTH),
        origin=Origin(xyz=(ARM_BOSS_LENGTH * 0.5, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=forged_alloy,
        name="boss",
    )
    right_arm.visual(arm_mesh, material=forged_alloy, name="beam")
    right_arm.visual(
        pedal_eye_mesh,
        origin=Origin(xyz=(PEDAL_EYE_LENGTH * 0.5, 0.0, -CRANK_LENGTH)),
        material=forged_alloy,
        name="pedal_eye",
    )
    _add_spider_tabs(right_arm, forged_alloy)
    right_arm.inertial = Inertial.from_geometry(
        Box((0.032, 0.020, 0.178)),
        mass=0.62,
        origin=Origin(xyz=(0.011, 0.0, -0.086)),
    )

    left_arm = model.part("left_arm")
    left_arm.visual(
        Cylinder(radius=ARM_BOSS_RADIUS, length=ARM_BOSS_LENGTH),
        origin=Origin(xyz=(ARM_BOSS_LENGTH * 0.5, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=forged_alloy,
        name="boss",
    )
    left_arm.visual(arm_mesh, material=forged_alloy, name="beam")
    left_arm.visual(
        pedal_eye_mesh,
        origin=Origin(xyz=(PEDAL_EYE_LENGTH * 0.5, 0.0, -CRANK_LENGTH)),
        material=forged_alloy,
        name="pedal_eye",
    )
    left_arm.inertial = Inertial.from_geometry(
        Box((0.032, 0.020, 0.178)),
        mass=0.58,
        origin=Origin(xyz=(0.011, 0.0, -0.086)),
    )

    sprocket = model.part("sprocket")
    sprocket.visual(
        sprocket_mesh,
        origin=Origin(xyz=(-0.008, 0.0, 0.0)),
        material=chainring_alloy,
        name="ring",
    )
    for bolt_index in range(5):
        angle = (2.0 * math.pi * bolt_index) / 5.0
        sprocket.visual(
            Cylinder(radius=0.0045, length=0.007),
            origin=Origin(
                xyz=(-0.0045, 0.038 * math.sin(angle), 0.038 * math.cos(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=bolt_steel,
            name=f"bolt_{bolt_index}",
        )
    sprocket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.072, length=0.010),
        mass=0.34,
        origin=Origin(xyz=(-0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    right_pedal = model.part("right_pedal")
    _build_pedal(right_pedal, pedal_black, pedal_deck, bolt_steel)
    right_pedal.inertial = Inertial.from_geometry(
        Box((0.080, 0.096, 0.020)),
        mass=0.26,
        origin=Origin(xyz=(0.040, 0.0, 0.004)),
    )

    left_pedal = model.part("left_pedal")
    _build_pedal(left_pedal, pedal_black, pedal_deck, bolt_steel)
    left_pedal.inertial = Inertial.from_geometry(
        Box((0.080, 0.096, 0.020)),
        mass=0.26,
        origin=Origin(xyz=(0.040, 0.0, 0.004)),
    )

    model.articulation(
        "bottom_bracket_spin",
        ArticulationType.CONTINUOUS,
        parent=bottom_bracket,
        child=spindle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=25.0),
    )
    model.articulation(
        "spindle_to_right_arm",
        ArticulationType.FIXED,
        parent=spindle,
        child=right_arm,
        origin=Origin(xyz=(SPINDLE_LENGTH * 0.5, 0.0, 0.0)),
    )
    model.articulation(
        "spindle_to_left_arm",
        ArticulationType.FIXED,
        parent=spindle,
        child=left_arm,
        origin=Origin(xyz=(-SPINDLE_LENGTH * 0.5, 0.0, 0.0), rpy=(0.0, math.pi, 0.0)),
    )
    model.articulation(
        "right_arm_to_sprocket",
        ArticulationType.FIXED,
        parent=right_arm,
        child=sprocket,
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
    )
    model.articulation(
        "right_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=right_arm,
        child=right_pedal,
        origin=Origin(xyz=(PEDAL_EYE_LENGTH * 0.5, 0.0, -CRANK_LENGTH)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=22.0),
    )
    model.articulation(
        "left_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent=left_arm,
        child=left_pedal,
        origin=Origin(xyz=(PEDAL_EYE_LENGTH * 0.5, 0.0, -CRANK_LENGTH)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=22.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    bottom_bracket = object_model.get_part("bottom_bracket")
    spindle = object_model.get_part("spindle")
    right_arm = object_model.get_part("right_arm")
    left_arm = object_model.get_part("left_arm")
    sprocket = object_model.get_part("sprocket")
    right_pedal = object_model.get_part("right_pedal")
    left_pedal = object_model.get_part("left_pedal")

    bb_spin = object_model.get_articulation("bottom_bracket_spin")
    right_pedal_spin = object_model.get_articulation("right_pedal_spin")
    left_pedal_spin = object_model.get_articulation("left_pedal_spin")

    shell_visual = bottom_bracket.get_visual("shell")
    spindle_axle = spindle.get_visual("axle")
    spindle_right_interface = spindle.get_visual("right_interface")
    right_boss = right_arm.get_visual("boss")
    left_boss = left_arm.get_visual("boss")
    right_beam = right_arm.get_visual("beam")
    right_eye = right_arm.get_visual("pedal_eye")
    left_eye = left_arm.get_visual("pedal_eye")
    right_spider_tab = right_arm.get_visual("spider_tab_0")
    sprocket_ring = sprocket.get_visual("ring")
    right_pedal_axle = right_pedal.get_visual("axle")
    left_pedal_axle = left_pedal.get_visual("axle")
    right_front_rail = right_pedal.get_visual("front_rail")
    right_rear_rail = right_pedal.get_visual("rear_rail")
    right_deck = right_pedal.get_visual("deck")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(bottom_bracket, spindle, reason="bottom bracket spindle nests concentrically inside the shell and cups")
    ctx.allow_overlap(right_arm, right_pedal, reason="right pedal axle passes through the crank arm pedal eye")
    ctx.allow_overlap(left_arm, left_pedal, reason="left pedal axle passes through the crank arm pedal eye")
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128, overlap_tol=0.001, overlap_volume_tol=0.0)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_contact(bottom_bracket, spindle, elem_a=shell_visual, elem_b=spindle_axle)
    ctx.expect_gap(
        right_arm,
        spindle,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=right_boss,
        negative_elem=spindle_axle,
    )
    ctx.expect_gap(
        spindle,
        left_arm,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=spindle_axle,
        negative_elem=left_boss,
    )
    ctx.expect_contact(sprocket, right_arm, elem_a=sprocket_ring, elem_b=right_spider_tab)
    ctx.expect_gap(
        sprocket,
        spindle,
        axis="x",
        min_gap=0.001,
        positive_elem=sprocket_ring,
        negative_elem=spindle_right_interface,
    )
    ctx.expect_overlap(
        right_arm,
        sprocket,
        axes="yz",
        min_overlap=0.004,
        elem_a=right_spider_tab,
        elem_b=sprocket_ring,
    )
    ctx.expect_overlap(
        right_arm,
        sprocket,
        axes="yz",
        min_overlap=0.015,
        elem_a=right_beam,
        elem_b=sprocket_ring,
    )
    ctx.expect_contact(right_pedal, right_arm, elem_a=right_pedal_axle, elem_b=right_eye)
    ctx.expect_contact(left_pedal, left_arm, elem_a=left_pedal_axle, elem_b=left_eye)
    ctx.expect_gap(
        left_arm,
        right_arm,
        axis="z",
        min_gap=0.28,
        positive_elem=left_eye,
        negative_elem=right_eye,
    )
    ctx.expect_gap(
        right_pedal,
        right_pedal,
        axis="z",
        min_gap=0.003,
        positive_elem=right_front_rail,
        negative_elem=right_deck,
    )
    ctx.expect_gap(
        right_pedal,
        right_pedal,
        axis="z",
        min_gap=0.003,
        positive_elem=right_rear_rail,
        negative_elem=right_deck,
    )

    with ctx.pose({bb_spin: math.pi / 2.0}):
        ctx.expect_gap(
            right_arm,
            left_arm,
            axis="y",
            min_gap=0.28,
            positive_elem=right_eye,
            negative_elem=left_eye,
        )
        ctx.expect_contact(right_pedal, right_arm, elem_a=right_pedal_axle, elem_b=right_eye)
        ctx.expect_contact(left_pedal, left_arm, elem_a=left_pedal_axle, elem_b=left_eye)
        ctx.expect_contact(sprocket, right_arm, elem_a=sprocket_ring, elem_b=right_spider_tab)
        ctx.expect_gap(
            sprocket,
            spindle,
            axis="x",
            min_gap=0.001,
            positive_elem=sprocket_ring,
            negative_elem=spindle_right_interface,
        )

    with ctx.pose({right_pedal_spin: math.pi / 2.0, left_pedal_spin: -math.pi / 2.0}):
        ctx.expect_contact(right_pedal, right_arm, elem_a=right_pedal_axle, elem_b=right_eye)
        ctx.expect_contact(left_pedal, left_arm, elem_a=left_pedal_axle, elem_b=left_eye)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
