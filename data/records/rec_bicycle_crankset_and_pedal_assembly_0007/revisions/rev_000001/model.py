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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


ARM_LENGTH = 0.142
ARM_ANGLE = math.radians(35.0)
ARM_SIDE_Y = 0.078
PEDAL_BOSS_Y = 0.090
PEDAL_MOUNT_Y = 0.100
CHAINRING_Y = 0.060


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _circle_profile(
    radius: float,
    *,
    segments: int = 48,
    center: tuple[float, float] = (0.0, 0.0),
    phase: float = 0.0,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos(phase + (math.tau * i / segments)),
            cy + radius * math.sin(phase + (math.tau * i / segments)),
        )
        for i in range(segments)
    ]


def _toothed_chainring_profile(
    *,
    root_radius: float,
    tip_radius: float,
    tooth_count: int,
) -> list[tuple[float, float]]:
    profile: list[tuple[float, float]] = []
    pitch = math.tau / tooth_count
    for tooth_index in range(tooth_count):
        base = tooth_index * pitch
        for angle, radius in (
            (base - 0.18 * pitch, root_radius),
            (base - 0.06 * pitch, tip_radius),
            (base + 0.06 * pitch, tip_radius),
            (base + 0.18 * pitch, root_radius),
        ):
            profile.append((radius * math.cos(angle), radius * math.sin(angle)))
    return profile


def _arm_outline(length: float) -> list[tuple[float, float]]:
    return [
        (-0.024, 0.000),
        (-0.019, 0.020),
        (-0.009, 0.034),
        (0.010, 0.044),
        (0.038, 0.035),
        (0.086, 0.020),
        (length - 0.024, 0.018),
        (length - 0.010, 0.027),
        (length + 0.007, 0.024),
        (length + 0.016, 0.015),
        (length + 0.020, 0.000),
        (length + 0.016, -0.015),
        (length + 0.007, -0.024),
        (length - 0.010, -0.027),
        (length - 0.024, -0.018),
        (0.086, -0.020),
        (0.038, -0.035),
        (0.010, -0.044),
        (-0.009, -0.034),
        (-0.019, -0.020),
    ]


def _arm_mesh() -> object:
    return ExtrudeWithHolesGeometry(
        _arm_outline(ARM_LENGTH),
        [
            _circle_profile(0.0105, segments=30),
            _circle_profile(0.0065, segments=24, center=(ARM_LENGTH, 0.0)),
        ],
        height=0.016,
        center=True,
    ).rotate_x(math.pi / 2.0)


def _chainring_mesh() -> object:
    return ExtrudeWithHolesGeometry(
        _toothed_chainring_profile(root_radius=0.101, tip_radius=0.106, tooth_count=48),
        [_circle_profile(0.081, segments=64)],
        height=0.004,
        center=True,
    ).rotate_x(math.pi / 2.0)


def _tip_position(angle: float) -> tuple[float, float, float]:
    return (
        ARM_LENGTH * math.cos(angle),
        0.0,
        -ARM_LENGTH * math.sin(angle),
    )


def _pedal_mount_origin(angle: float, side_sign: float) -> tuple[float, float, float]:
    tip_x, _, tip_z = _tip_position(angle)
    return (tip_x, side_sign * PEDAL_MOUNT_Y, tip_z)


def _add_pedal_visuals(part, *, side_sign: float, cage_material, axle_material) -> None:
    part.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.0, side_sign * 0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_material,
        name="axle_shoulder",
    )
    part.visual(
        Cylinder(radius=0.0048, length=0.028),
        origin=Origin(xyz=(0.0, side_sign * 0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_material,
        name="axle_spindle",
    )
    part.visual(
        Box((0.082, 0.004, 0.020)),
        origin=Origin(xyz=(0.0, side_sign * 0.034, 0.0)),
        material=cage_material,
        name="inner_plate",
    )
    part.visual(
        Box((0.088, 0.004, 0.020)),
        origin=Origin(xyz=(0.0, side_sign * 0.062, 0.0)),
        material=cage_material,
        name="outer_plate",
    )
    part.visual(
        Box((0.006, 0.034, 0.020)),
        origin=Origin(xyz=(0.038, side_sign * 0.048, 0.0)),
        material=cage_material,
        name="front_rail",
    )
    part.visual(
        Box((0.006, 0.034, 0.020)),
        origin=Origin(xyz=(-0.038, side_sign * 0.048, 0.0)),
        material=cage_material,
        name="rear_rail",
    )
    part.visual(
        Box((0.082, 0.034, 0.004)),
        origin=Origin(xyz=(0.0, side_sign * 0.048, 0.010)),
        material=cage_material,
        name="top_rail",
    )
    part.visual(
        Box((0.082, 0.034, 0.004)),
        origin=Origin(xyz=(0.0, side_sign * 0.048, -0.010)),
        material=cage_material,
        name="bottom_rail",
    )
    part.visual(
        Box((0.060, 0.024, 0.004)),
        origin=Origin(xyz=(0.0, side_sign * 0.048, 0.0)),
        material=axle_material,
        name="center_tread",
    )
    for x_pos in (-0.024, 0.0, 0.024):
        part.visual(
            Box((0.004, 0.028, 0.004)),
            origin=Origin(xyz=(x_pos, side_sign * 0.048, 0.0)),
            material=axle_material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="track_crankset", assets=ASSETS)

    satin_alloy = model.material("satin_alloy", rgba=(0.78, 0.80, 0.82, 1.0))
    polished_alloy = model.material("polished_alloy", rgba=(0.86, 0.88, 0.90, 1.0))
    chainring_steel = model.material("chainring_steel", rgba=(0.60, 0.62, 0.66, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.26, 0.29, 1.0))
    pedal_black = model.material("pedal_black", rgba=(0.10, 0.11, 0.12, 1.0))

    bottom_bracket = model.part("bottom_bracket")
    bottom_bracket.visual(
        Cylinder(radius=0.021, length=0.050),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="center_sleeve",
    )
    bottom_bracket.visual(
        Cylinder(radius=0.027, length=0.018),
        origin=Origin(xyz=(0.0, 0.034, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_cup",
    )
    bottom_bracket.visual(
        Cylinder(radius=0.027, length=0.018),
        origin=Origin(xyz=(0.0, -0.034, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_cup",
    )
    bottom_bracket.visual(
        Cylinder(radius=0.031, length=0.004),
        origin=Origin(xyz=(0.0, 0.045, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_alloy,
        name="right_flange",
    )
    bottom_bracket.visual(
        Cylinder(radius=0.031, length=0.004),
        origin=Origin(xyz=(0.0, -0.045, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_alloy,
        name="left_flange",
    )
    bottom_bracket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.031, length=0.094),
        mass=0.55,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    crankset = model.part("crankset")
    crankset.visual(
        Cylinder(radius=0.015, length=0.094),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="spindle",
    )
    crankset.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.0, 0.052, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_alloy,
        name="right_dust_cap",
    )
    crankset.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.0, -0.052, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_alloy,
        name="left_dust_cap",
    )
    crankset.visual(
        Cylinder(radius=0.026, length=0.036),
        origin=Origin(xyz=(0.0, 0.068, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_alloy,
        name="right_boss",
    )
    crankset.visual(
        Cylinder(radius=0.026, length=0.036),
        origin=Origin(xyz=(0.0, -0.068, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_alloy,
        name="left_boss",
    )
    crankset.visual(
        Cylinder(radius=0.032, length=0.010),
        origin=Origin(xyz=(0.0, CHAINRING_Y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_alloy,
        name="spider_core",
    )
    crankset.visual(
        _save_mesh("track_chainring.obj", _chainring_mesh()),
        origin=Origin(xyz=(0.0, CHAINRING_Y, 0.0)),
        material=chainring_steel,
        name="chainring",
    )
    for index in range(5):
        angle = index * math.tau / 5.0
        crankset.visual(
            Box((0.078, 0.010, 0.016)),
            origin=Origin(
                xyz=(0.057 * math.cos(angle), CHAINRING_Y, -0.057 * math.sin(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=polished_alloy,
            name=f"spider_arm_{index + 1}",
        )
        crankset.visual(
            Cylinder(radius=0.0045, length=0.012),
            origin=Origin(
                xyz=(0.041 * math.cos(angle), CHAINRING_Y + 0.001, -0.041 * math.sin(angle)),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_steel,
            name=f"chainring_bolt_{index + 1}",
        )
    crankset.visual(
        _save_mesh("right_crank_arm.obj", _arm_mesh()),
        origin=Origin(xyz=(0.0, ARM_SIDE_Y, 0.0), rpy=(0.0, ARM_ANGLE, 0.0)),
        material=satin_alloy,
        name="right_arm",
    )
    crankset.visual(
        _save_mesh("left_crank_arm.obj", _arm_mesh()),
        origin=Origin(xyz=(0.0, -ARM_SIDE_Y, 0.0), rpy=(0.0, ARM_ANGLE + math.pi, 0.0)),
        material=satin_alloy,
        name="left_arm",
    )

    right_tip_x, _, right_tip_z = _tip_position(ARM_ANGLE)
    left_tip_x, _, left_tip_z = _tip_position(ARM_ANGLE + math.pi)
    crankset.visual(
        Cylinder(radius=0.017, length=0.020),
        origin=Origin(
            xyz=(right_tip_x, PEDAL_BOSS_Y, right_tip_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=polished_alloy,
        name="right_pedal_boss",
    )
    crankset.visual(
        Cylinder(radius=0.017, length=0.020),
        origin=Origin(
            xyz=(left_tip_x, -PEDAL_BOSS_Y, left_tip_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=polished_alloy,
        name="left_pedal_boss",
    )
    crankset.inertial = Inertial.from_geometry(
        Box((0.34, 0.20, 0.28)),
        mass=1.9,
        origin=Origin(),
    )

    right_pedal = model.part("right_pedal")
    _add_pedal_visuals(
        right_pedal,
        side_sign=1.0,
        cage_material=pedal_black,
        axle_material=dark_steel,
    )
    right_pedal.inertial = Inertial.from_geometry(
        Box((0.09, 0.07, 0.03)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.048, 0.0)),
    )

    left_pedal = model.part("left_pedal")
    _add_pedal_visuals(
        left_pedal,
        side_sign=-1.0,
        cage_material=pedal_black,
        axle_material=dark_steel,
    )
    left_pedal.inertial = Inertial.from_geometry(
        Box((0.09, 0.07, 0.03)),
        mass=0.22,
        origin=Origin(xyz=(0.0, -0.048, 0.0)),
    )

    model.articulation(
        "bottom_bracket_spin",
        ArticulationType.REVOLUTE,
        parent=bottom_bracket,
        child=crankset,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=18.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "right_pedal_spin",
        ArticulationType.REVOLUTE,
        parent=crankset,
        child=right_pedal,
        origin=Origin(xyz=_pedal_mount_origin(ARM_ANGLE, 1.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=25.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "left_pedal_spin",
        ArticulationType.REVOLUTE,
        parent=crankset,
        child=left_pedal,
        origin=Origin(xyz=_pedal_mount_origin(ARM_ANGLE + math.pi, -1.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=25.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    bottom_bracket = object_model.get_part("bottom_bracket")
    crankset = object_model.get_part("crankset")
    right_pedal = object_model.get_part("right_pedal")
    left_pedal = object_model.get_part("left_pedal")

    bb_spin = object_model.get_articulation("bottom_bracket_spin")
    right_pedal_spin = object_model.get_articulation("right_pedal_spin")
    left_pedal_spin = object_model.get_articulation("left_pedal_spin")

    center_sleeve = bottom_bracket.get_visual("center_sleeve")
    right_flange = bottom_bracket.get_visual("right_flange")
    left_flange = bottom_bracket.get_visual("left_flange")

    chainring = crankset.get_visual("chainring")
    right_arm = crankset.get_visual("right_arm")
    left_arm = crankset.get_visual("left_arm")
    right_dust_cap = crankset.get_visual("right_dust_cap")
    left_dust_cap = crankset.get_visual("left_dust_cap")
    right_pedal_boss = crankset.get_visual("right_pedal_boss")
    left_pedal_boss = crankset.get_visual("left_pedal_boss")

    right_shoulder = right_pedal.get_visual("axle_shoulder")
    left_shoulder = left_pedal.get_visual("axle_shoulder")
    right_outer_plate = right_pedal.get_visual("outer_plate")
    left_outer_plate = left_pedal.get_visual("outer_plate")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(
        bottom_bracket,
        crankset,
        reason="press-fit bottom bracket cups visually enclose the crank spindle",
    )
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_within(
        bottom_bracket,
        crankset,
        axes="xz",
        inner_elem=center_sleeve,
        outer_elem=chainring,
    )
    ctx.expect_gap(
        crankset,
        bottom_bracket,
        axis="y",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem=right_dust_cap,
        negative_elem=right_flange,
        name="right_bottom_bracket_face_seated",
    )
    ctx.expect_gap(
        bottom_bracket,
        crankset,
        axis="y",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem=left_flange,
        negative_elem=left_dust_cap,
        name="left_bottom_bracket_face_seated",
    )
    ctx.expect_contact(
        right_pedal,
        crankset,
        elem_a=right_shoulder,
        elem_b=right_pedal_boss,
    )
    ctx.expect_contact(
        left_pedal,
        crankset,
        elem_a=left_shoulder,
        elem_b=left_pedal_boss,
    )
    ctx.expect_gap(
        crankset,
        crankset,
        axis="y",
        min_gap=0.12,
        positive_elem=chainring,
        negative_elem=left_arm,
        name="chainring_is_drive_side_and_clear_of_left_arm",
    )
    ctx.expect_gap(
        crankset,
        crankset,
        axis="y",
        min_gap=0.015,
        positive_elem=right_pedal_boss,
        negative_elem=chainring,
        name="right_pedal_mount_sits_outboard_of_chainring",
    )
    ctx.expect_gap(
        right_pedal,
        left_pedal,
        axis="x",
        min_gap=0.12,
        positive_elem=right_outer_plate,
        negative_elem=left_outer_plate,
        name="pedals_opposite_in_rest_pose",
    )

    with ctx.pose(
        {
            bb_spin: math.pi / 2.0,
            right_pedal_spin: 1.1,
            left_pedal_spin: -0.8,
        }
    ):
        ctx.expect_within(
            bottom_bracket,
            crankset,
            axes="xz",
            inner_elem=center_sleeve,
            outer_elem=chainring,
        )
        ctx.expect_contact(
            right_pedal,
            crankset,
            elem_a=right_shoulder,
            elem_b=right_pedal_boss,
        )
        ctx.expect_contact(
            left_pedal,
            crankset,
            elem_a=left_shoulder,
            elem_b=left_pedal_boss,
        )
        ctx.expect_gap(
            crankset,
            crankset,
            axis="y",
            min_gap=0.12,
            positive_elem=chainring,
            negative_elem=left_arm,
            name="chainring_remains_drive_side_at_quarter_turn",
        )
        ctx.expect_gap(
            left_pedal,
            right_pedal,
            axis="z",
            min_gap=0.15,
            positive_elem=left_outer_plate,
            negative_elem=right_outer_plate,
            name="pedals_separate_vertically_when_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
