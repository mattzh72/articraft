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
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def _rounded_section(
    width: float,
    height: float,
    radius: float,
    z: float,
    *,
    x_offset: float = 0.0,
    y_offset: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x + x_offset, y + y_offset, z)
        for x, y in rounded_rect_profile(width, height, radius)
    ]


def _yz_section(
    width: float,
    height: float,
    radius: float,
    x: float,
    *,
    y_offset: float = 0.0,
    z_offset: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x, y + y_offset, z + z_offset)
        for z, y in rounded_rect_profile(height, width, radius)
    ]


def _loft_mesh(name: str, sections: list[list[tuple[float, float, float]]]):
    return mesh_from_geometry(section_loft(sections), ASSETS.mesh_path(name))


def _x_loft_mesh(name: str, sections: list[list[tuple[float, float, float]]]):
    return mesh_from_geometry(section_loft(sections), ASSETS.mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_robotic_leg", assets=ASSETS)

    graphite = model.material("graphite_matte", rgba=(0.16, 0.17, 0.18, 1.0))
    satin_alloy = model.material("satin_alloy", rgba=(0.60, 0.63, 0.66, 1.0))
    dark_alloy = model.material("dark_alloy", rgba=(0.28, 0.30, 0.33, 1.0))
    polymer = model.material("polymer_black", rgba=(0.08, 0.09, 0.10, 1.0))
    rubber = model.material("sole_rubber", rgba=(0.12, 0.13, 0.14, 1.0))

    hip_frame = model.part("hip_frame")
    hip_frame.visual(
        _loft_mesh(
            "hip_shell.obj",
            [
                _rounded_section(0.118, 0.088, 0.020, 0.018),
                _rounded_section(0.128, 0.082, 0.024, 0.074),
                _rounded_section(0.112, 0.070, 0.020, 0.122),
            ],
        ),
        material=graphite,
        name="hip_shell",
    )
    hip_frame.visual(
        Box((0.032, 0.028, 0.092)),
        origin=Origin(xyz=(0.0, 0.043, -0.024)),
        material=dark_alloy,
        name="hip_left_cheek",
    )
    hip_frame.visual(
        Box((0.032, 0.028, 0.092)),
        origin=Origin(xyz=(0.0, -0.043, -0.024)),
        material=dark_alloy,
        name="hip_right_cheek",
    )
    hip_frame.visual(
        _x_loft_mesh(
            "hip_front_bay_cover.obj",
            [
                _yz_section(0.050, 0.014, 0.004, 0.010, z_offset=0.062),
                _yz_section(0.046, 0.010, 0.003, 0.043, z_offset=0.062),
                _yz_section(0.038, 0.006, 0.002, 0.072, z_offset=0.062),
            ],
        ),
        material=satin_alloy,
        name="hip_front_bay_cover",
    )
    hip_frame.visual(
        Box((0.050, 0.034, 0.004)),
        origin=Origin(xyz=(-0.041, 0.0, 0.070)),
        material=polymer,
        name="hip_rear_service_panel",
    )
    hip_frame.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.0, 0.062, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_alloy,
        name="hip_left_axis_cap",
    )
    hip_frame.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.0, -0.062, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_alloy,
        name="hip_right_axis_cap",
    )
    hip_frame.inertial = Inertial.from_geometry(
        Box((0.150, 0.120, 0.150)),
        mass=5.2,
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
    )

    thigh = model.part("thigh")
    thigh.visual(
        Cylinder(radius=0.0215, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_alloy,
        name="hip_barrel",
    )
    thigh.visual(
        _loft_mesh(
            "thigh_shell.obj",
            [
                _rounded_section(0.082, 0.062, 0.018, -0.012),
                _rounded_section(0.078, 0.060, 0.018, -0.120),
                _rounded_section(0.070, 0.052, 0.016, -0.226),
            ],
        ),
        material=graphite,
        name="thigh_shell",
    )
    thigh.visual(
        Box((0.036, 0.030, 0.212)),
        origin=Origin(xyz=(0.0, 0.0, -0.121)),
        material=dark_alloy,
        name="thigh_load_beam",
    )
    thigh.visual(
        _x_loft_mesh(
            "thigh_front_bay_cover.obj",
            [
                _yz_section(0.044, 0.126, 0.005, 0.020, z_offset=-0.108),
                _yz_section(0.040, 0.116, 0.004, 0.034, z_offset=-0.108),
                _yz_section(0.030, 0.098, 0.003, 0.044, z_offset=-0.104),
            ],
        ),
        material=satin_alloy,
        name="thigh_front_bay_cover",
    )
    thigh.visual(
        Box((0.005, 0.026, 0.150)),
        origin=Origin(xyz=(-0.034, 0.0, -0.116)),
        material=polymer,
        name="thigh_rear_spine",
    )
    thigh.visual(
        Cylinder(radius=0.0032, length=0.004),
        origin=Origin(xyz=(0.041, 0.012, -0.072), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_alloy,
        name="thigh_bolt_upper",
    )
    thigh.visual(
        Cylinder(radius=0.0032, length=0.006),
        origin=Origin(xyz=(0.040, -0.012, -0.146), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_alloy,
        name="thigh_bolt_lower",
    )
    thigh.visual(
        Box((0.028, 0.028, 0.082)),
        origin=Origin(xyz=(0.0, 0.043, -0.254)),
        material=dark_alloy,
        name="knee_left_cheek",
    )
    thigh.visual(
        Box((0.028, 0.028, 0.082)),
        origin=Origin(xyz=(0.0, -0.043, -0.254)),
        material=dark_alloy,
        name="knee_right_cheek",
    )
    thigh.visual(
        Box((0.020, 0.022, 0.044)),
        origin=Origin(xyz=(0.0, 0.020, -0.236)),
        material=dark_alloy,
        name="knee_left_web",
    )
    thigh.visual(
        Box((0.020, 0.022, 0.044)),
        origin=Origin(xyz=(0.0, -0.020, -0.236)),
        material=dark_alloy,
        name="knee_right_web",
    )
    thigh.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.0, 0.062, -0.285), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_alloy,
        name="knee_left_axis_cap",
    )
    thigh.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.0, -0.062, -0.285), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_alloy,
        name="knee_right_axis_cap",
    )
    thigh.inertial = Inertial.from_geometry(
        Box((0.095, 0.085, 0.315)),
        mass=4.3,
        origin=Origin(xyz=(0.0, 0.0, -0.155)),
    )

    shin = model.part("shin")
    shin.visual(
        Cylinder(radius=0.020, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_alloy,
        name="knee_barrel",
    )
    shin.visual(
        _loft_mesh(
            "shin_shell.obj",
            [
                _rounded_section(0.072, 0.056, 0.016, -0.012),
                _rounded_section(0.066, 0.050, 0.015, -0.120),
                _rounded_section(0.058, 0.044, 0.013, -0.208),
            ],
        ),
        material=graphite,
        name="shin_shell",
    )
    shin.visual(
        Box((0.032, 0.026, 0.188)),
        origin=Origin(xyz=(0.0, 0.0, -0.110)),
        material=dark_alloy,
        name="shin_load_beam",
    )
    shin.visual(
        _x_loft_mesh(
            "shin_front_bay_cover.obj",
            [
                _yz_section(0.038, 0.112, 0.0045, 0.018, z_offset=-0.108),
                _yz_section(0.034, 0.102, 0.0035, 0.030, z_offset=-0.108),
                _yz_section(0.026, 0.086, 0.003, 0.039, z_offset=-0.104),
            ],
        ),
        material=satin_alloy,
        name="shin_front_bay_cover",
    )
    shin.visual(
        Box((0.004, 0.024, 0.116)),
        origin=Origin(xyz=(-0.030, 0.0, -0.110)),
        material=polymer,
        name="shin_rear_spine",
    )
    shin.visual(
        Cylinder(radius=0.0030, length=0.004),
        origin=Origin(xyz=(0.037, 0.010, -0.076), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_alloy,
        name="shin_bolt_upper",
    )
    shin.visual(
        Cylinder(radius=0.0030, length=0.004),
        origin=Origin(xyz=(0.037, -0.010, -0.138), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_alloy,
        name="shin_bolt_lower",
    )
    shin.visual(
        Box((0.026, 0.024, 0.070)),
        origin=Origin(xyz=(0.0, 0.037, -0.235)),
        material=dark_alloy,
        name="ankle_left_cheek",
    )
    shin.visual(
        Box((0.026, 0.024, 0.070)),
        origin=Origin(xyz=(0.0, -0.037, -0.235)),
        material=dark_alloy,
        name="ankle_right_cheek",
    )
    shin.visual(
        Box((0.018, 0.020, 0.038)),
        origin=Origin(xyz=(0.0, 0.017, -0.226)),
        material=dark_alloy,
        name="ankle_left_web",
    )
    shin.visual(
        Box((0.018, 0.020, 0.038)),
        origin=Origin(xyz=(0.0, -0.017, -0.226)),
        material=dark_alloy,
        name="ankle_right_web",
    )
    shin.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.0, 0.054, -0.265), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_alloy,
        name="ankle_left_axis_cap",
    )
    shin.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(0.0, -0.054, -0.265), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_alloy,
        name="ankle_right_axis_cap",
    )
    shin.inertial = Inertial.from_geometry(
        Box((0.082, 0.072, 0.300)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, -0.145)),
    )

    foot = model.part("foot")
    foot.visual(
        Cylinder(radius=0.018, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_alloy,
        name="ankle_barrel",
    )
    foot.visual(
        _loft_mesh(
            "foot_shell.obj",
            [
                _rounded_section(0.072, 0.046, 0.014, -0.010, x_offset=0.012),
                _rounded_section(0.116, 0.060, 0.016, -0.045, x_offset=0.030),
                _rounded_section(0.168, 0.074, 0.018, -0.080, x_offset=0.060),
            ],
        ),
        material=graphite,
        name="foot_shell",
    )
    foot.visual(
        Box((0.238, 0.100, 0.026)),
        origin=Origin(xyz=(0.074, 0.0, -0.093)),
        material=dark_alloy,
        name="sole_carrier",
    )
    foot.visual(
        _x_loft_mesh(
            "toe_guard.obj",
            [
                _yz_section(0.086, 0.026, 0.006, 0.132, z_offset=-0.074),
                _yz_section(0.080, 0.020, 0.005, 0.172, z_offset=-0.071),
                _yz_section(0.064, 0.010, 0.003, 0.205, z_offset=-0.068),
            ],
        ),
        material=satin_alloy,
        name="toe_guard",
    )
    foot.visual(
        _x_loft_mesh(
            "heel_counter.obj",
            [
                _yz_section(0.058, 0.038, 0.006, -0.060, z_offset=-0.058),
                _yz_section(0.074, 0.056, 0.008, -0.030, z_offset=-0.058),
                _yz_section(0.068, 0.048, 0.007, 0.002, z_offset=-0.058),
            ],
        ),
        material=polymer,
        name="heel_counter",
    )
    foot.visual(
        Box((0.216, 0.092, 0.008)),
        origin=Origin(xyz=(0.074, 0.0, -0.110)),
        material=rubber,
        name="sole_pad",
    )
    foot.visual(
        Box((0.082, 0.052, 0.005)),
        origin=Origin(xyz=(0.058, 0.0, -0.066)),
        material=satin_alloy,
        name="foot_upper_panel",
    )
    foot.visual(
        Box((0.018, 0.020, 0.015)),
        origin=Origin(xyz=(0.036, 0.0, -0.073)),
        material=dark_alloy,
        name="foot_upper_panel_front_mount",
    )
    foot.visual(
        Box((0.018, 0.020, 0.015)),
        origin=Origin(xyz=(0.080, 0.0, -0.073)),
        material=dark_alloy,
        name="foot_upper_panel_rear_mount",
    )
    foot.inertial = Inertial.from_geometry(
        Box((0.250, 0.110, 0.130)),
        mass=1.9,
        origin=Origin(xyz=(0.074, 0.0, -0.080)),
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=hip_frame,
        child=thigh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=2.4,
            lower=-0.55,
            upper=1.05,
        ),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh,
        child=shin,
        origin=Origin(xyz=(0.0, 0.0, -0.285)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=240.0,
            velocity=3.0,
            lower=0.0,
            upper=2.25,
        ),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shin,
        child=foot,
        origin=Origin(xyz=(0.0, 0.0, -0.265)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=150.0,
            velocity=3.0,
            lower=-0.65,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    hip_frame = object_model.get_part("hip_frame")
    thigh = object_model.get_part("thigh")
    shin = object_model.get_part("shin")
    foot = object_model.get_part("foot")

    hip_pitch = object_model.get_articulation("hip_pitch")
    knee_pitch = object_model.get_articulation("knee_pitch")
    ankle_pitch = object_model.get_articulation("ankle_pitch")

    hip_left_cheek = hip_frame.get_visual("hip_left_cheek")
    hip_right_cheek = hip_frame.get_visual("hip_right_cheek")
    thigh_hip_barrel = thigh.get_visual("hip_barrel")
    knee_left_cheek = thigh.get_visual("knee_left_cheek")
    knee_right_cheek = thigh.get_visual("knee_right_cheek")
    shin_knee_barrel = shin.get_visual("knee_barrel")
    ankle_left_cheek = shin.get_visual("ankle_left_cheek")
    ankle_right_cheek = shin.get_visual("ankle_right_cheek")
    foot_ankle_barrel = foot.get_visual("ankle_barrel")
    sole_pad = foot.get_visual("sole_pad")
    toe_guard = foot.get_visual("toe_guard")

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

    ctx.expect_gap(
        hip_frame,
        thigh,
        axis="y",
        positive_elem=hip_left_cheek,
        negative_elem=thigh_hip_barrel,
        max_gap=0.0005,
        max_penetration=1e-6,
        name="hip_left_cheek_seated_to_barrel",
    )
    ctx.expect_gap(
        thigh,
        hip_frame,
        axis="y",
        positive_elem=thigh_hip_barrel,
        negative_elem=hip_right_cheek,
        max_gap=0.0005,
        max_penetration=1e-6,
        name="hip_right_cheek_seated_to_barrel",
    )
    ctx.expect_contact(
        hip_frame,
        thigh,
        elem_a=hip_left_cheek,
        elem_b=thigh_hip_barrel,
        name="hip_left_joint_contact",
    )
    ctx.expect_contact(
        hip_frame,
        thigh,
        elem_a=hip_right_cheek,
        elem_b=thigh_hip_barrel,
        name="hip_right_joint_contact",
    )
    ctx.expect_overlap(
        hip_frame,
        thigh,
        axes="xz",
        min_overlap=0.020,
        elem_a=hip_left_cheek,
        elem_b=thigh_hip_barrel,
        name="hip_joint_axis_capture",
    )

    ctx.expect_gap(
        thigh,
        shin,
        axis="y",
        positive_elem=knee_left_cheek,
        negative_elem=shin_knee_barrel,
        max_gap=0.0005,
        max_penetration=1e-6,
        name="knee_left_cheek_seated_to_barrel",
    )
    ctx.expect_gap(
        shin,
        thigh,
        axis="y",
        positive_elem=shin_knee_barrel,
        negative_elem=knee_right_cheek,
        max_gap=0.0005,
        max_penetration=1e-6,
        name="knee_right_cheek_seated_to_barrel",
    )
    ctx.expect_contact(
        thigh,
        shin,
        elem_a=knee_left_cheek,
        elem_b=shin_knee_barrel,
        name="knee_left_joint_contact",
    )
    ctx.expect_contact(
        thigh,
        shin,
        elem_a=knee_right_cheek,
        elem_b=shin_knee_barrel,
        name="knee_right_joint_contact",
    )
    ctx.expect_overlap(
        thigh,
        shin,
        axes="xz",
        min_overlap=0.018,
        elem_a=knee_left_cheek,
        elem_b=shin_knee_barrel,
        name="knee_joint_axis_capture",
    )

    ctx.expect_gap(
        shin,
        foot,
        axis="y",
        positive_elem=ankle_left_cheek,
        negative_elem=foot_ankle_barrel,
        max_gap=0.0005,
        max_penetration=1e-6,
        name="ankle_left_cheek_seated_to_barrel",
    )
    ctx.expect_gap(
        foot,
        shin,
        axis="y",
        positive_elem=foot_ankle_barrel,
        negative_elem=ankle_right_cheek,
        max_gap=0.0005,
        max_penetration=1e-6,
        name="ankle_right_cheek_seated_to_barrel",
    )
    ctx.expect_contact(
        shin,
        foot,
        elem_a=ankle_left_cheek,
        elem_b=foot_ankle_barrel,
        name="ankle_left_joint_contact",
    )
    ctx.expect_contact(
        shin,
        foot,
        elem_a=ankle_right_cheek,
        elem_b=foot_ankle_barrel,
        name="ankle_right_joint_contact",
    )
    ctx.expect_overlap(
        shin,
        foot,
        axes="xz",
        min_overlap=0.015,
        elem_a=ankle_left_cheek,
        elem_b=foot_ankle_barrel,
        name="ankle_joint_axis_capture",
    )

    hip_aabb = ctx.part_world_aabb(hip_frame)
    sole_aabb = ctx.part_element_world_aabb(foot, elem=sole_pad)
    foot_rest = ctx.part_world_position(foot)
    shin_rest = ctx.part_world_position(shin)
    toe_rest = ctx.part_element_world_aabb(foot, elem=toe_guard)
    assert hip_aabb is not None
    assert sole_aabb is not None
    assert foot_rest is not None
    assert shin_rest is not None
    assert toe_rest is not None

    total_height = hip_aabb[1][2] - sole_aabb[0][2]
    ctx.check(
        "leg_total_height_realistic",
        0.72 <= total_height <= 0.86,
        f"expected total height in [0.72, 0.86] m, got {total_height:.4f} m",
    )

    with ctx.pose({hip_pitch: 0.62}):
        foot_hip = ctx.part_world_position(foot)
        shin_hip = ctx.part_world_position(shin)
        assert foot_hip is not None
        assert shin_hip is not None
        ctx.check(
            "hip_pitch_swings_leg_forward",
            foot_hip[0] > foot_rest[0] + 0.22 and shin_hip[0] > shin_rest[0] + 0.12,
            (
                "expected positive hip pitch to move shin and foot forward; "
                f"rest foot x={foot_rest[0]:.4f}, posed foot x={foot_hip[0]:.4f}, "
                f"rest shin x={shin_rest[0]:.4f}, posed shin x={shin_hip[0]:.4f}"
            ),
        )
        ctx.expect_contact(hip_frame, thigh, elem_a=hip_left_cheek, elem_b=thigh_hip_barrel)
        ctx.expect_contact(hip_frame, thigh, elem_a=hip_right_cheek, elem_b=thigh_hip_barrel)

    with ctx.pose({knee_pitch: 1.25}):
        foot_knee = ctx.part_world_position(foot)
        assert foot_knee is not None
        ctx.check(
            "knee_pitch_retracts_lower_leg",
            foot_knee[0] > foot_rest[0] + 0.20 and foot_knee[2] > foot_rest[2] + 0.12,
            (
                "expected knee flexion to pull the foot forward and upward; "
                f"rest foot={foot_rest}, posed foot={foot_knee}"
            ),
        )
        ctx.expect_contact(thigh, shin, elem_a=knee_left_cheek, elem_b=shin_knee_barrel)
        ctx.expect_contact(thigh, shin, elem_a=knee_right_cheek, elem_b=shin_knee_barrel)

    with ctx.pose({ankle_pitch: 0.38}):
        toe_dorsiflex = ctx.part_element_world_aabb(foot, elem=toe_guard)
        assert toe_dorsiflex is not None
        ctx.check(
            "ankle_pitch_lifts_toe",
            toe_dorsiflex[1][2] > toe_rest[1][2] + 0.035,
            (
                "expected ankle dorsiflexion to raise the toe guard; "
                f"rest toe top z={toe_rest[1][2]:.4f}, posed toe top z={toe_dorsiflex[1][2]:.4f}"
            ),
        )
        ctx.expect_contact(shin, foot, elem_a=ankle_left_cheek, elem_b=foot_ankle_barrel)
        ctx.expect_contact(shin, foot, elem_a=ankle_right_cheek, elem_b=foot_ankle_barrel)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
