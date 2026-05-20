from __future__ import annotations

# The harness only exposes the editable block to the model.
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
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _circle_profile(
    radius: float,
    segments: int,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    phase: float = 0.0,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos(phase + (2.0 * math.pi * i / segments)),
            cy + radius * math.sin(phase + (2.0 * math.pi * i / segments)),
        )
        for i in range(segments)
    ]


def _chainring_profile(
    tooth_count: int,
    root_radius: float,
    tip_radius: float,
) -> list[tuple[float, float]]:
    profile: list[tuple[float, float]] = []
    tooth_pitch = 2.0 * math.pi / tooth_count
    for i in range(tooth_count):
        angle = tooth_pitch * i
        profile.extend(
            [
                (
                    root_radius * math.cos(angle - 0.46 * tooth_pitch),
                    root_radius * math.sin(angle - 0.46 * tooth_pitch),
                ),
                (
                    tip_radius * math.cos(angle - 0.18 * tooth_pitch),
                    tip_radius * math.sin(angle - 0.18 * tooth_pitch),
                ),
                (
                    tip_radius * math.cos(angle + 0.18 * tooth_pitch),
                    tip_radius * math.sin(angle + 0.18 * tooth_pitch),
                ),
                (
                    root_radius * math.cos(angle + 0.46 * tooth_pitch),
                    root_radius * math.sin(angle + 0.46 * tooth_pitch),
                ),
            ]
        )
    return profile


def _spider_profile(
    arm_count: int,
    valley_radius: float,
    shoulder_radius: float,
    lobe_radius: float,
) -> list[tuple[float, float]]:
    profile: list[tuple[float, float]] = []
    arm_pitch = 2.0 * math.pi / arm_count
    for i in range(arm_count):
        angle = arm_pitch * i
        profile.extend(
            [
                (
                    valley_radius * math.cos(angle - 0.46 * arm_pitch),
                    valley_radius * math.sin(angle - 0.46 * arm_pitch),
                ),
                (
                    shoulder_radius * math.cos(angle - 0.22 * arm_pitch),
                    shoulder_radius * math.sin(angle - 0.22 * arm_pitch),
                ),
                (
                    lobe_radius * math.cos(angle - 0.08 * arm_pitch),
                    lobe_radius * math.sin(angle - 0.08 * arm_pitch),
                ),
                (
                    lobe_radius * math.cos(angle + 0.08 * arm_pitch),
                    lobe_radius * math.sin(angle + 0.08 * arm_pitch),
                ),
                (
                    shoulder_radius * math.cos(angle + 0.22 * arm_pitch),
                    shoulder_radius * math.sin(angle + 0.22 * arm_pitch),
                ),
                (
                    valley_radius * math.cos(angle + 0.46 * arm_pitch),
                    valley_radius * math.sin(angle + 0.46 * arm_pitch),
                ),
            ]
        )
    return profile


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _crank_arm_mesh():
    length = 0.172
    sections = []
    for z_pos, width, thickness, radius in (
        (0.0, 0.036, 0.018, 0.0045),
        (0.040, 0.030, 0.015, 0.0038),
        (0.095, 0.024, 0.013, 0.0030),
        (0.150, 0.026, 0.014, 0.0032),
        (length, 0.032, 0.018, 0.0042),
    ):
        profile = rounded_rect_profile(width, thickness, radius, corner_segments=8)
        sections.append([(x, y, z_pos) for x, y in profile])

    arm = LoftGeometry(sections, cap=True, closed=True)
    eye = (
        CylinderGeometry(
            radius=0.0165,
            height=0.018,
            radial_segments=28,
        )
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.0, length)
    )
    arm.merge(eye)
    return _save_mesh("crank_arm.obj", arm)


def _spider_mesh():
    outer = _spider_profile(
        arm_count=5,
        valley_radius=0.026,
        shoulder_radius=0.042,
        lobe_radius=0.063,
    )
    holes = [_circle_profile(0.016, 32)]
    for i in range(5):
        angle = 2.0 * math.pi * i / 5.0
        holes.append(
            _circle_profile(
                0.0042,
                18,
                center=(0.046 * math.cos(angle), 0.046 * math.sin(angle)),
            )
        )
    spider = ExtrudeWithHolesGeometry(
        outer_profile=outer,
        hole_profiles=holes,
        height=0.0065,
        cap=True,
        center=True,
        closed=True,
    )
    return _save_mesh("spider.obj", spider)


def _chainring_mesh():
    outer = _chainring_profile(
        tooth_count=46,
        root_radius=0.094,
        tip_radius=0.101,
    )
    holes = [_circle_profile(0.044, 48)]
    for i in range(5):
        angle = 2.0 * math.pi * i / 5.0
        holes.append(
            _circle_profile(
                0.0039,
                14,
                center=(0.055 * math.cos(angle), 0.055 * math.sin(angle)),
            )
        )
    ring = ExtrudeWithHolesGeometry(
        outer_profile=outer,
        hole_profiles=holes,
        height=0.0042,
        cap=True,
        center=True,
        closed=True,
    )
    return _save_mesh("chainring.obj", ring)


def _pedal_body_mesh():
    sections = []
    for z_pos, width, thickness, radius in (
        (0.018, 0.062, 0.012, 0.0030),
        (0.040, 0.096, 0.015, 0.0038),
        (0.064, 0.092, 0.014, 0.0036),
        (0.084, 0.058, 0.011, 0.0028),
    ):
        profile = rounded_rect_profile(width, thickness, radius, corner_segments=8)
        sections.append([(x, y, z_pos) for x, y in profile])
    pedal = LoftGeometry(sections, cap=True, closed=True)
    return _save_mesh("pedal_body.obj", pedal)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bicycle_crankset", assets=ASSETS)

    frame_black = model.material("frame_black", rgba=(0.18, 0.19, 0.20, 1.0))
    alloy = model.material("alloy", rgba=(0.69, 0.71, 0.74, 1.0))
    machined = model.material("machined", rgba=(0.59, 0.61, 0.64, 1.0))
    anodized_black = model.material("anodized_black", rgba=(0.11, 0.11, 0.12, 1.0))
    steel = model.material("steel", rgba=(0.53, 0.56, 0.60, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    crank_arm = _crank_arm_mesh()
    spider = _spider_mesh()
    chainring = _chainring_mesh()
    pedal_body = _pedal_body_mesh()

    bb_shell = model.part("bb_shell")
    bb_shell.visual(
        Cylinder(radius=0.021, length=0.073),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_black,
        name="shell",
    )
    bb_shell.visual(
        Cylinder(radius=0.0245, length=0.010),
        origin=Origin(xyz=(0.0, -0.0415, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="drive_cup",
    )
    bb_shell.visual(
        Cylinder(radius=0.0245, length=0.010),
        origin=Origin(xyz=(0.0, 0.0415, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="non_drive_cup",
    )
    bb_shell.visual(
        Cylinder(radius=0.017, length=0.110),
        origin=Origin(xyz=(-0.015, 0.0, 0.072), rpy=(0.0, -0.20, 0.0)),
        material=frame_black,
        name="seat_tube_stub",
    )
    bb_shell.visual(
        Cylinder(radius=0.019, length=0.120),
        origin=Origin(xyz=(0.061, 0.0, 0.046), rpy=(0.0, 0.89, 0.0)),
        material=frame_black,
        name="down_tube_stub",
    )
    bb_shell.visual(
        Cylinder(radius=0.013, length=0.100),
        origin=Origin(xyz=(-0.054, 0.028, 0.002), rpy=(0.0, -1.31, 0.0)),
        material=frame_black,
        name="left_chainstay_stub",
    )
    bb_shell.visual(
        Cylinder(radius=0.013, length=0.100),
        origin=Origin(xyz=(-0.054, -0.028, 0.002), rpy=(0.0, -1.31, 0.0)),
        material=frame_black,
        name="right_chainstay_stub",
    )
    bb_shell.inertial = Inertial.from_geometry(
        Box((0.20, 0.10, 0.16)),
        mass=1.35,
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
    )

    crankset = model.part("crankset")
    crankset.visual(
        Cylinder(radius=0.0115, length=0.152),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="spindle",
    )
    crankset.visual(
        Cylinder(radius=0.032, length=0.022),
        origin=Origin(xyz=(0.0, -0.041, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized_black,
        name="drive_spider_mount",
    )
    crankset.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=(0.0, 0.044, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized_black,
        name="non_drive_interface",
    )

    right_pitch = 2.66
    left_pitch = right_pitch + math.pi
    arm_side_offset = 0.052
    arm_length = 0.172
    crank_limits = MotionLimits(effort=160.0, velocity=18.0)
    pedal_limits = MotionLimits(effort=35.0, velocity=28.0)

    crankset.visual(
        crank_arm,
        origin=Origin(xyz=(0.0, -arm_side_offset, 0.0), rpy=(0.0, right_pitch, 0.0)),
        material=alloy,
        name="right_crank_arm",
    )
    crankset.visual(
        crank_arm,
        origin=Origin(xyz=(0.0, arm_side_offset, 0.0), rpy=(0.0, left_pitch, 0.0)),
        material=alloy,
        name="left_crank_arm",
    )
    crankset.visual(
        spider,
        origin=Origin(xyz=(0.0, -0.033, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="spider",
    )
    crankset.visual(
        chainring,
        origin=Origin(xyz=(0.0, -0.039, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized_black,
        name="chainring",
    )
    for i in range(5):
        angle = 2.0 * math.pi * i / 5.0
        crankset.visual(
            Cylinder(radius=0.0044, length=0.014),
            origin=Origin(
                xyz=(0.055 * math.cos(angle), -0.036, 0.055 * math.sin(angle)),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=steel,
            name=f"chainring_bolt_{i + 1}",
        )
    crankset.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.0, 0.057, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="left_preload_cap",
    )
    crankset.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, -0.058, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="right_bolt_cap",
    )
    crankset.inertial = Inertial.from_geometry(
        Box((0.24, 0.18, 0.26)),
        mass=2.35,
        origin=Origin(xyz=(0.0, -0.01, 0.0)),
    )

    left_pedal = model.part("left_pedal")
    left_pedal.visual(
        pedal_body,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=anodized_black,
        name="left_pedal_body",
    )
    left_pedal.visual(
        Cylinder(radius=0.006, length=0.055),
        origin=Origin(xyz=(0.0, 0.0275, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_spindle",
    )
    left_pedal.visual(
        Box((0.076, 0.050, 0.005)),
        origin=Origin(xyz=(0.0, 0.052, 0.0055)),
        material=rubber,
        name="left_upper_tread",
    )
    left_pedal.visual(
        Box((0.076, 0.050, 0.005)),
        origin=Origin(xyz=(0.0, 0.052, -0.0055)),
        material=rubber,
        name="left_lower_tread",
    )
    left_pedal.visual(
        Cylinder(radius=0.0072, length=0.012),
        origin=Origin(xyz=(0.0, 0.084, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_end_cap",
    )
    left_pedal.inertial = Inertial.from_geometry(
        Box((0.10, 0.09, 0.025)),
        mass=0.34,
        origin=Origin(xyz=(0.0, 0.050, 0.0)),
    )

    right_pedal = model.part("right_pedal")
    right_pedal.visual(
        pedal_body,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized_black,
        name="right_pedal_body",
    )
    right_pedal.visual(
        Cylinder(radius=0.006, length=0.055),
        origin=Origin(xyz=(0.0, -0.0275, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_spindle",
    )
    right_pedal.visual(
        Box((0.076, 0.050, 0.005)),
        origin=Origin(xyz=(0.0, -0.052, 0.0055)),
        material=rubber,
        name="right_upper_tread",
    )
    right_pedal.visual(
        Box((0.076, 0.050, 0.005)),
        origin=Origin(xyz=(0.0, -0.052, -0.0055)),
        material=rubber,
        name="right_lower_tread",
    )
    right_pedal.visual(
        Cylinder(radius=0.0072, length=0.012),
        origin=Origin(xyz=(0.0, -0.084, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_end_cap",
    )
    right_pedal.inertial = Inertial.from_geometry(
        Box((0.10, 0.09, 0.025)),
        mass=0.34,
        origin=Origin(xyz=(0.0, -0.050, 0.0)),
    )

    right_pedal_origin = (
        arm_length * math.sin(right_pitch),
        -arm_side_offset,
        arm_length * math.cos(right_pitch),
    )
    left_pedal_origin = (
        arm_length * math.sin(left_pitch),
        arm_side_offset,
        arm_length * math.cos(left_pitch),
    )

    model.articulation(
        "bb_to_crank",
        ArticulationType.CONTINUOUS,
        parent="bb_shell",
        child="crankset",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=crank_limits,
    )
    model.articulation(
        "left_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent="crankset",
        child="left_pedal",
        origin=Origin(xyz=left_pedal_origin),
        axis=(0.0, 1.0, 0.0),
        motion_limits=pedal_limits,
    )
    model.articulation(
        "right_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent="crankset",
        child="right_pedal",
        origin=Origin(xyz=right_pedal_origin),
        axis=(0.0, 1.0, 0.0),
        motion_limits=pedal_limits,
    )

    return model


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "bb_shell",
        "crankset",
        reason="bottom bracket spindle passes through a visually solid shell that represents a hollow assembly",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("crankset", "bb_shell")
    ctx.expect_aabb_overlap("crankset", "bb_shell", axes="xz", min_overlap=0.035)
    ctx.expect_origin_distance("crankset", "bb_shell", axes="xz", max_dist=0.005)
    ctx.expect_aabb_contact("left_pedal", "crankset")
    ctx.expect_aabb_contact("right_pedal", "crankset")
    ctx.expect_joint_motion_axis(
        "bb_to_crank",
        "right_pedal",
        world_axis="z",
        direction="positive",
        min_delta=0.01,
    )
    ctx.expect_joint_motion_axis(
        "bb_to_crank",
        "left_pedal",
        world_axis="z",
        direction="negative",
        min_delta=0.01,
    )

    with ctx.pose(bb_to_crank=0.0, left_pedal_spin=0.0, right_pedal_spin=0.0):
        ctx.expect_aabb_contact("crankset", "bb_shell")
        ctx.expect_aabb_contact("left_pedal", "crankset")
        ctx.expect_aabb_contact("right_pedal", "crankset")
        right0 = ctx.part_world_position("right_pedal")
        left0 = ctx.part_world_position("left_pedal")
        _require(right0[2] < -0.12, "right pedal should start in a lower crank position")
        _require(left0[2] > 0.12, "left pedal should oppose the right pedal vertically")
        _require(abs(right0[0] + left0[0]) < 0.012, "pedals should balance fore-aft")
        _require(abs(right0[2] + left0[2]) < 0.012, "pedals should remain opposed in height")
        _require(
            abs(abs(left0[1] - right0[1]) - 0.104) < 0.012, "pedals need realistic lateral stance"
        )
        _require(0.165 < math.hypot(right0[0], right0[2]) < 0.180, "right crank radius drifted")
        _require(0.165 < math.hypot(left0[0], left0[2]) < 0.180, "left crank radius drifted")

    with ctx.pose(bb_to_crank=-math.pi / 2.0):
        ctx.expect_aabb_contact("left_pedal", "crankset")
        ctx.expect_aabb_contact("right_pedal", "crankset")
        right_q = ctx.part_world_position("right_pedal")
        left_q = ctx.part_world_position("left_pedal")
        _require(
            right_q[0] > 0.14 and right_q[2] > 0.07, "right pedal should sweep forward and upward"
        )
        _require(
            left_q[0] < -0.14 and left_q[2] < -0.07, "left pedal should mirror the right pedal"
        )

    with ctx.pose(bb_to_crank=math.pi):
        right_h = ctx.part_world_position("right_pedal")
        left_h = ctx.part_world_position("left_pedal")
        _require(right_h[2] > 0.12, "right pedal should reach the upper half-turn position")
        _require(left_h[2] < -0.12, "left pedal should reach the lower half-turn position")

    with ctx.pose(
        bb_to_crank=-math.pi / 2.0,
        left_pedal_spin=-math.pi / 3.0,
        right_pedal_spin=math.pi / 2.0,
    ):
        ctx.expect_aabb_contact("left_pedal", "crankset")
        ctx.expect_aabb_contact("right_pedal", "crankset")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
