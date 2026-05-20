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
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)

BB_SHELL_LENGTH = 0.076
BB_OUTER_RADIUS = 0.024
BB_INNER_RADIUS = 0.017
SPINDLE_RADIUS = 0.011
BEARING_LENGTH = 0.014
ARM_LENGTH = 0.165
ARM_THICKNESS = 0.012
ARM_ROOT_WIDTH = 0.032
ARM_MID_WIDTH = 0.024
ARM_TIP_WIDTH = 0.020
CRANK_OFFSET_X = 0.053
PEDAL_JOINT_X = 0.064
PEDAL_PLATFORM_WIDTH = 0.036
PEDAL_PLATFORM_LENGTH = 0.084
PEDAL_PLATFORM_THICKNESS = 0.010
PEDAL_PLATFORM_OFFSET = 0.024
CHAINRING_OUTER_RADIUS = 0.078
CHAINRING_INNER_RADIUS = 0.043
CHAINRING_THICKNESS = 0.004
CHAINRING_X = 0.046


def _circle_profile(radius: float, segments: int = 56) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _offset_profile(
    profile: list[tuple[float, float]],
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _build_bottom_bracket_shell():
    shell = LatheGeometry.from_shell_profiles(
        [
            (BB_OUTER_RADIUS, -BB_SHELL_LENGTH / 2.0),
            (BB_OUTER_RADIUS, BB_SHELL_LENGTH / 2.0),
        ],
        [
            (BB_INNER_RADIUS, -BB_SHELL_LENGTH / 2.0),
            (BB_INNER_RADIUS, BB_SHELL_LENGTH / 2.0),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    shell.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(shell, ASSETS.mesh_path("bb_shell.obj"))


def _build_crank_arm_mesh():
    profile = [
        (-ARM_ROOT_WIDTH / 2.0, 0.0),
        (ARM_ROOT_WIDTH / 2.0, 0.0),
        (ARM_ROOT_WIDTH / 2.0, 0.020),
        (ARM_MID_WIDTH / 2.0, 0.072),
        (ARM_TIP_WIDTH / 2.0, ARM_LENGTH - 0.018),
        (ARM_TIP_WIDTH / 2.0, ARM_LENGTH),
        (-ARM_TIP_WIDTH / 2.0, ARM_LENGTH),
        (-ARM_TIP_WIDTH / 2.0, ARM_LENGTH - 0.018),
        (-ARM_MID_WIDTH / 2.0, 0.072),
        (-ARM_ROOT_WIDTH / 2.0, 0.020),
    ]
    arm = ExtrudeGeometry.from_z0(profile, ARM_THICKNESS)
    arm.translate(0.0, 0.0, -ARM_THICKNESS / 2.0)
    arm.rotate_y(math.pi / 2.0).rotate_x(math.pi / 2.0)
    return mesh_from_geometry(arm, ASSETS.mesh_path("compact_crank_arm.obj"))


def _build_chainring_mesh():
    ring = ExtrudeWithHolesGeometry(
        _circle_profile(CHAINRING_OUTER_RADIUS),
        [_circle_profile(CHAINRING_INNER_RADIUS)],
        CHAINRING_THICKNESS,
        center=True,
    )
    ring.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(ring, ASSETS.mesh_path("compact_chainring.obj"))


def _build_pedal_platform_mesh():
    outer = rounded_rect_profile(
        PEDAL_PLATFORM_WIDTH,
        PEDAL_PLATFORM_LENGTH,
        radius=0.005,
        corner_segments=8,
    )
    slot = rounded_rect_profile(0.012, 0.028, radius=0.003, corner_segments=6)
    platform = ExtrudeWithHolesGeometry(
        outer,
        [
            _offset_profile(slot, dy=-0.021),
            _offset_profile(slot, dy=0.021),
        ],
        PEDAL_PLATFORM_THICKNESS,
        center=True,
    )
    return mesh_from_geometry(platform, ASSETS.mesh_path("folding_pedal_platform.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_bike_crankset", assets=ASSETS)

    alloy = model.material("alloy", rgba=(0.76, 0.78, 0.80, 1.0))
    anodized = model.material("anodized_black", rgba=(0.14, 0.14, 0.15, 1.0))
    steel = model.material("steel", rgba=(0.45, 0.47, 0.50, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    bb_shell_mesh = _build_bottom_bracket_shell()
    arm_mesh = _build_crank_arm_mesh()
    chainring_mesh = _build_chainring_mesh()
    pedal_platform_mesh = _build_pedal_platform_mesh()

    bottom_bracket = model.part("bottom_bracket")
    bottom_bracket.visual(bb_shell_mesh, material=anodized, name="bb_shell")
    bottom_bracket.visual(
        Cylinder(radius=0.027, length=0.004),
        origin=Origin(xyz=(-0.040, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="left_cup",
    )
    bottom_bracket.visual(
        Cylinder(radius=0.027, length=0.004),
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="right_cup",
    )

    crankset = model.part("crankset")
    crankset.visual(
        Cylinder(radius=SPINDLE_RADIUS, length=0.094),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="spindle",
    )
    crankset.visual(
        Cylinder(radius=BB_INNER_RADIUS, length=BEARING_LENGTH),
        origin=Origin(xyz=(-0.028, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="left_bearing",
    )
    crankset.visual(
        Cylinder(radius=BB_INNER_RADIUS, length=BEARING_LENGTH),
        origin=Origin(xyz=(0.028, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="right_bearing",
    )
    crankset.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(-0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=alloy,
        name="left_boss",
    )
    crankset.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized,
        name="right_boss",
    )
    crankset.visual(
        chainring_mesh,
        origin=Origin(xyz=(CHAINRING_X, 0.0, 0.0)),
        material=anodized,
        name="chainring",
    )
    for index, angle in enumerate((0.0, 1.26, 2.51, 3.77, 5.03), start=1):
        radial_center = 0.027
        crankset.visual(
            Box((0.004, 0.010, 0.050)),
            origin=Origin(
                xyz=(
                    0.044,
                    radial_center * math.sin(angle),
                    radial_center * math.cos(angle),
                ),
                rpy=(angle, 0.0, 0.0),
            ),
            material=alloy,
            name=f"spider_arm_{index}",
        )
    crankset.visual(
        arm_mesh,
        origin=Origin(xyz=(-CRANK_OFFSET_X, 0.0, 0.0)),
        material=alloy,
        name="left_arm",
    )
    crankset.visual(
        arm_mesh,
        origin=Origin(xyz=(CRANK_OFFSET_X, 0.0, 0.0), rpy=(math.pi, 0.0, 0.0)),
        material=alloy,
        name="right_arm",
    )
    crankset.visual(
        Box((0.012, 0.012, 0.020)),
        origin=Origin(xyz=(-0.058, 0.0, ARM_LENGTH)),
        material=alloy,
        name="left_pedal_bridge",
    )
    crankset.visual(
        Box((0.012, 0.012, 0.020)),
        origin=Origin(xyz=(0.058, 0.0, -ARM_LENGTH)),
        material=alloy,
        name="right_pedal_bridge",
    )
    crankset.visual(
        Cylinder(radius=0.0045, length=0.026),
        origin=Origin(
            xyz=(-PEDAL_JOINT_X, 0.0, ARM_LENGTH),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="left_hinge_pin",
    )
    crankset.visual(
        Cylinder(radius=0.0045, length=0.026),
        origin=Origin(
            xyz=(PEDAL_JOINT_X, 0.0, -ARM_LENGTH),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="right_hinge_pin",
    )

    right_pedal = model.part("right_pedal")
    right_pedal.visual(
        Cylinder(radius=0.008, length=0.026),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="hinge_barrel",
    )
    right_pedal.visual(
        Box((0.014, 0.022, 0.018)),
        origin=Origin(xyz=(0.009, 0.0, 0.0)),
        material=alloy,
        name="hinge_yoke",
    )
    right_pedal.visual(
        pedal_platform_mesh,
        origin=Origin(xyz=(PEDAL_PLATFORM_OFFSET, 0.0, 0.0)),
        material=alloy,
        name="platform",
    )
    right_pedal.visual(
        Box((0.018, 0.090, 0.002)),
        origin=Origin(xyz=(PEDAL_PLATFORM_OFFSET, 0.0, 0.0065)),
        material=rubber,
        name="grip_strip",
    )

    left_pedal = model.part("left_pedal")
    left_pedal.visual(
        Cylinder(radius=0.008, length=0.026),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="hinge_barrel",
    )
    left_pedal.visual(
        Box((0.014, 0.022, 0.018)),
        origin=Origin(xyz=(-0.009, 0.0, 0.0)),
        material=alloy,
        name="hinge_yoke",
    )
    left_pedal.visual(
        pedal_platform_mesh,
        origin=Origin(xyz=(-PEDAL_PLATFORM_OFFSET, 0.0, 0.0)),
        material=alloy,
        name="platform",
    )
    left_pedal.visual(
        Box((0.018, 0.090, 0.002)),
        origin=Origin(xyz=(-PEDAL_PLATFORM_OFFSET, 0.0, 0.0065)),
        material=rubber,
        name="grip_strip",
    )

    model.articulation(
        "bottom_bracket_spin",
        ArticulationType.REVOLUTE,
        parent=bottom_bracket,
        child=crankset,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=150.0,
            velocity=18.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "right_pedal_fold",
        ArticulationType.REVOLUTE,
        parent=crankset,
        child=right_pedal,
        origin=Origin(xyz=(PEDAL_JOINT_X, 0.0, -ARM_LENGTH)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=6.0,
            lower=0.0,
            upper=1.60,
        ),
    )
    model.articulation(
        "left_pedal_fold",
        ArticulationType.REVOLUTE,
        parent=crankset,
        child=left_pedal,
        origin=Origin(xyz=(-PEDAL_JOINT_X, 0.0, ARM_LENGTH)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=6.0,
            lower=-1.60,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    bottom_bracket = object_model.get_part("bottom_bracket")
    crankset = object_model.get_part("crankset")
    right_pedal = object_model.get_part("right_pedal")
    left_pedal = object_model.get_part("left_pedal")

    bottom_bracket_spin = object_model.get_articulation("bottom_bracket_spin")
    right_pedal_fold = object_model.get_articulation("right_pedal_fold")
    left_pedal_fold = object_model.get_articulation("left_pedal_fold")

    bb_shell = bottom_bracket.get_visual("bb_shell")
    chainring = crankset.get_visual("chainring")
    left_bearing = crankset.get_visual("left_bearing")
    right_bearing = crankset.get_visual("right_bearing")
    left_arm = crankset.get_visual("left_arm")
    right_arm = crankset.get_visual("right_arm")
    left_hinge_pin = crankset.get_visual("left_hinge_pin")
    right_hinge_pin = crankset.get_visual("right_hinge_pin")
    left_platform = left_pedal.get_visual("platform")
    right_platform = right_pedal.get_visual("platform")
    left_barrel = left_pedal.get_visual("hinge_barrel")
    right_barrel = right_pedal.get_visual("hinge_barrel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        bottom_bracket,
        crankset,
        reason="The spindle and bearing sleeves are intentionally nested inside the bottom bracket shell.",
    )
    ctx.allow_overlap(
        right_pedal,
        crankset,
        reason="The folding pedal barrel visually nests concentrically around the hinge pin.",
    )
    ctx.allow_overlap(
        left_pedal,
        crankset,
        reason="The folding pedal barrel visually nests concentrically around the hinge pin.",
    )

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.018)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128, overlap_tol=0.003, overlap_volume_tol=0.0)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_contact(crankset, bottom_bracket, elem_a=left_bearing, elem_b=bb_shell)
    ctx.expect_contact(crankset, bottom_bracket, elem_a=right_bearing, elem_b=bb_shell)
    ctx.expect_within(crankset, bottom_bracket, axes="yz", inner_elem=left_bearing, outer_elem=bb_shell)
    ctx.expect_within(crankset, bottom_bracket, axes="yz", inner_elem=right_bearing, outer_elem=bb_shell)
    ctx.expect_gap(
        crankset,
        bottom_bracket,
        axis="x",
        min_gap=0.004,
        positive_elem=chainring,
        negative_elem=bb_shell,
        name="chainring_sits_outboard_of_shell",
    )
    ctx.expect_within(
        crankset,
        right_pedal,
        axes="yz",
        inner_elem=right_hinge_pin,
        outer_elem=right_barrel,
        name="right_pedal_hinge_pin_is_captured_in_barrel",
    )
    ctx.expect_within(
        crankset,
        left_pedal,
        axes="yz",
        inner_elem=left_hinge_pin,
        outer_elem=left_barrel,
        name="left_pedal_hinge_pin_is_captured_in_barrel",
    )
    ctx.expect_overlap(right_pedal, crankset, axes="yz", min_overlap=0.0005, elem_a=right_barrel, elem_b=right_hinge_pin)
    ctx.expect_overlap(left_pedal, crankset, axes="yz", min_overlap=0.0005, elem_a=left_barrel, elem_b=left_hinge_pin)
    ctx.expect_gap(
        right_pedal,
        crankset,
        axis="x",
        min_gap=0.008,
        positive_elem=right_platform,
        negative_elem=right_arm,
        name="right_pedal_open_outboard_of_arm",
    )
    ctx.expect_gap(
        crankset,
        left_pedal,
        axis="x",
        min_gap=0.008,
        positive_elem=left_arm,
        negative_elem=left_platform,
        name="left_pedal_open_outboard_of_arm",
    )

    with ctx.pose({right_pedal_fold: 1.57, left_pedal_fold: -1.57}):
        ctx.expect_gap(
            right_pedal,
            crankset,
            axis="x",
            max_gap=0.004,
            max_penetration=0.004,
            positive_elem=right_platform,
            negative_elem=right_arm,
            name="right_pedal_folds_flat_against_arm",
        )
        ctx.expect_gap(
            crankset,
            left_pedal,
            axis="x",
            max_gap=0.004,
            max_penetration=0.004,
            positive_elem=left_arm,
            negative_elem=left_platform,
            name="left_pedal_folds_flat_against_arm",
        )

    with ctx.pose({bottom_bracket_spin: math.pi / 2.0}):
        ctx.expect_contact(crankset, bottom_bracket, elem_a=right_bearing, elem_b=bb_shell)
        ctx.expect_gap(
            right_pedal,
            crankset,
            axis="x",
            min_gap=0.008,
            positive_elem=right_platform,
            negative_elem=right_arm,
            name="open_pedal_stays_outboard_when_crank_rotates",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
