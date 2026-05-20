from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    LoftGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _save_mesh(name, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _profile_section(width, depth, z, y_shift=0.0, exponent=3.0, segments=56):
    return [
        (x, y + y_shift, z)
        for x, y in superellipse_profile(width, depth, exponent=exponent, segments=segments)
    ]


def _chair_materials():
    return {
        "graphite_plastic": Material("graphite_plastic", (0.18, 0.19, 0.21, 1.0)),
        "black_steel": Material("black_steel", (0.13, 0.13, 0.14, 1.0)),
        "matte_aluminum": Material("matte_aluminum", (0.72, 0.74, 0.76, 1.0)),
        "polished_chrome": Material("polished_chrome", (0.83, 0.84, 0.87, 1.0)),
        "seat_fabric": Material("seat_fabric", (0.09, 0.10, 0.11, 1.0)),
        "mesh_back": Material("mesh_back", (0.12, 0.13, 0.15, 0.94)),
        "arm_pad": Material("arm_pad", (0.11, 0.11, 0.12, 1.0)),
        "rubber": Material("rubber", (0.05, 0.05, 0.05, 1.0)),
        "accent_gray": Material("accent_gray", (0.34, 0.36, 0.39, 1.0)),
    }


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ergonomic_office_chair", assets=ASSETS)
    mats = _chair_materials()

    spoke_mesh = _save_mesh(
        "base_spoke.obj",
        tube_from_spline_points(
            [
                (0.0, 0.0, 0.0),
                (0.0, 0.11, -0.003),
                (0.0, 0.22, -0.010),
                (0.0, 0.335, -0.017),
            ],
            radius=0.017,
            samples_per_segment=18,
            radial_segments=20,
            cap_ends=True,
        ),
    )
    column_shell_mesh = _save_mesh(
        "column_shell.obj",
        LatheGeometry(
            [
                (0.028, 0.0),
                (0.031, 0.008),
                (0.036, 0.050),
                (0.044, 0.108),
                (0.040, 0.152),
                (0.033, 0.178),
                (0.029, 0.195),
            ],
            segments=40,
        ),
    )
    seat_shell_mesh = _save_mesh(
        "seat_shell.obj",
        LoftGeometry(
            [
                _profile_section(0.500, 0.415, 0.003, y_shift=0.000, exponent=3.2),
                _profile_section(0.515, 0.445, 0.015, y_shift=0.008, exponent=3.0),
                _profile_section(0.528, 0.470, 0.028, y_shift=0.014, exponent=3.0),
                _profile_section(0.518, 0.455, 0.034, y_shift=0.012, exponent=3.0),
            ],
            cap=True,
            closed=True,
        ),
    )
    seat_pad_mesh = _save_mesh(
        "seat_pad.obj",
        LoftGeometry(
            [
                _profile_section(0.445, 0.360, 0.018, y_shift=0.000, exponent=3.4),
                _profile_section(0.462, 0.392, 0.036, y_shift=0.008, exponent=3.2),
                _profile_section(0.478, 0.415, 0.051, y_shift=0.014, exponent=3.0),
                _profile_section(0.458, 0.395, 0.058, y_shift=0.018, exponent=3.1),
            ],
            cap=True,
            closed=True,
        ),
    )
    back_shell_mesh = _save_mesh(
        "back_shell.obj",
        LoftGeometry(
            [
                _profile_section(0.200, 0.040, 0.020, y_shift=-0.010, exponent=2.8),
                _profile_section(0.335, 0.075, 0.135, y_shift=0.006, exponent=2.7),
                _profile_section(0.435, 0.082, 0.290, y_shift=0.016, exponent=2.6),
                _profile_section(0.410, 0.060, 0.470, y_shift=0.000, exponent=2.8),
                _profile_section(0.300, 0.038, 0.600, y_shift=-0.016, exponent=3.0),
            ],
            cap=True,
            closed=True,
        ),
    )
    back_pad_mesh = _save_mesh(
        "back_pad.obj",
        LoftGeometry(
            [
                _profile_section(0.175, 0.018, 0.050, y_shift=0.010, exponent=3.2),
                _profile_section(0.295, 0.032, 0.165, y_shift=0.020, exponent=3.0),
                _profile_section(0.380, 0.036, 0.325, y_shift=0.020, exponent=2.8),
                _profile_section(0.265, 0.024, 0.500, y_shift=0.010, exponent=3.0),
            ],
            cap=True,
            closed=True,
        ),
    )
    arm_pad_mesh = _save_mesh(
        "arm_pad.obj",
        ExtrudeGeometry(
            rounded_rect_profile(0.080, 0.275, radius=0.025, corner_segments=12),
            height=0.028,
            center=True,
            cap=True,
            closed=True,
        ),
    )
    back_spine_mesh = _save_mesh(
        "back_spine.obj",
        tube_from_spline_points(
            [
                (0.0, -0.010, -0.006),
                (0.0, -0.028, 0.145),
                (0.0, -0.050, 0.340),
                (0.0, -0.060, 0.560),
            ],
            radius=0.018,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.066, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=mats["graphite_plastic"],
    )
    base.visual(
        Cylinder(radius=0.052, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=mats["accent_gray"],
    )
    for idx in range(5):
        theta = idx * (2.0 * pi / 5.0)
        spoke_yaw = theta - (pi / 2.0)
        wheel_x = 0.335 * cos(theta)
        wheel_y = 0.335 * sin(theta)
        base.visual(
            spoke_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.055), rpy=(0.0, 0.0, spoke_yaw)),
            material=mats["matte_aluminum"],
            name=f"spoke_{idx}",
        )
        caster_rpy = (0.0, pi / 2.0, theta + (pi / 2.0))
        base.visual(
            Box((0.028, 0.012, 0.006)),
            origin=Origin(xyz=(wheel_x, wheel_y, 0.039), rpy=caster_rpy),
            material=mats["black_steel"],
            name=f"caster_fork_arm_inner_{idx}",
        )
        base.visual(
            Box((0.028, 0.012, 0.006)),
            origin=Origin(xyz=(wheel_x, wheel_y, 0.039), rpy=caster_rpy),
            material=mats["black_steel"],
            name=f"caster_fork_arm_outer_{idx}",
        )
        base.visual(
            Box((0.014, 0.016, 0.034)),
            origin=Origin(xyz=(wheel_x, wheel_y, 0.053), rpy=caster_rpy),
            material=mats["graphite_plastic"],
            name=f"caster_fork_cap_{idx}",
        )
        base.visual(
            Box((0.014, 0.014, 0.012)),
            origin=Origin(xyz=(wheel_x, wheel_y, 0.049), rpy=caster_rpy),
            material=mats["accent_gray"],
            name=f"caster_stem_{idx}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.74, 0.74, 0.12)),
        mass=6.4,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    column_upper = model.part("column_upper")
    column_upper.visual(column_shell_mesh, origin=Origin(), material=mats["graphite_plastic"])
    column_upper.visual(
        Cylinder(radius=0.023, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=mats["polished_chrome"],
    )
    column_upper.visual(
        Cylinder(radius=0.032, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=mats["polished_chrome"],
    )
    column_upper.visual(
        Cylinder(radius=0.052, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.246)),
        material=mats["black_steel"],
    )
    column_upper.visual(
        Cylinder(radius=0.036, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.217)),
        material=mats["accent_gray"],
    )
    column_upper.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.265),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, 0.132)),
    )

    seat = model.part("seat")
    seat.visual(
        Box((0.210, 0.185, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=mats["black_steel"],
    )
    seat.visual(
        Cylinder(radius=0.028, length=0.020),
        origin=Origin(xyz=(0.090, -0.040, -0.022), rpy=(pi / 2.0, 0.0, 0.0)),
        material=mats["graphite_plastic"],
    )
    seat.visual(
        Box((0.075, 0.010, 0.008)),
        origin=Origin(xyz=(-0.128, -0.020, -0.020)),
        material=mats["black_steel"],
    )
    seat.visual(
        Box((0.010, 0.055, 0.010)),
        origin=Origin(xyz=(-0.162, -0.020, -0.020)),
        material=mats["black_steel"],
    )
    seat.visual(seat_shell_mesh, origin=Origin(), material=mats["graphite_plastic"])
    seat.visual(seat_pad_mesh, origin=Origin(), material=mats["seat_fabric"])
    seat.inertial = Inertial.from_geometry(
        Box((0.550, 0.520, 0.110)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    backrest = model.part("backrest")
    backrest.visual(
        Box((0.070, 0.050, 0.022)),
        origin=Origin(xyz=(0.0, -0.022, 0.011)),
        material=mats["black_steel"],
    )
    backrest.visual(
        back_spine_mesh,
        origin=Origin(xyz=(0.0, -0.028, 0.012)),
        material=mats["black_steel"],
    )
    backrest.visual(
        back_shell_mesh,
        origin=Origin(xyz=(0.0, -0.032, 0.028), rpy=(0.11, 0.0, 0.0)),
        material=mats["graphite_plastic"],
    )
    backrest.visual(
        back_pad_mesh,
        origin=Origin(xyz=(0.0, -0.032, 0.028), rpy=(0.11, 0.0, 0.0)),
        material=mats["mesh_back"],
    )
    backrest.visual(
        Cylinder(radius=0.013, length=0.210),
        origin=Origin(xyz=(0.0, -0.050, 0.140), rpy=(0.0, pi / 2.0, 0.0)),
        material=mats["accent_gray"],
    )
    backrest.inertial = Inertial.from_geometry(
        Box((0.470, 0.120, 0.650)),
        mass=4.1,
        origin=Origin(xyz=(0.0, -0.028, 0.315)),
    )

    for side_name in ("right", "left"):
        arm = model.part(f"armrest_{side_name}")
        arm.visual(
            Box((0.026, 0.180, 0.022)),
            origin=Origin(xyz=(0.002, 0.000, 0.011)),
            material=mats["graphite_plastic"],
        )
        arm.visual(
            Cylinder(radius=0.010, length=0.172),
            origin=Origin(xyz=(0.015, -0.074, 0.086)),
            material=mats["black_steel"],
        )
        arm.visual(
            Cylinder(radius=0.010, length=0.172),
            origin=Origin(xyz=(0.015, 0.074, 0.086)),
            material=mats["black_steel"],
        )
        arm.visual(
            Box((0.050, 0.020, 0.018)),
            origin=Origin(xyz=(0.034, -0.074, 0.170)),
            material=mats["graphite_plastic"],
        )
        arm.visual(
            Box((0.050, 0.020, 0.018)),
            origin=Origin(xyz=(0.034, 0.074, 0.170)),
            material=mats["graphite_plastic"],
        )
        arm.visual(
            Box((0.020, 0.170, 0.018)),
            origin=Origin(xyz=(0.054, 0.000, 0.170)),
            material=mats["graphite_plastic"],
        )
        arm.visual(
            arm_pad_mesh,
            origin=Origin(xyz=(0.056, 0.000, 0.192)),
            material=mats["arm_pad"],
        )
        arm.inertial = Inertial.from_geometry(
            Box((0.100, 0.290, 0.220)),
            mass=1.1,
            origin=Origin(xyz=(0.048, 0.000, 0.110)),
        )

    caster_radius = 0.027
    caster_thickness = 0.010
    caster_offset = 0.011
    for idx in range(5):
        wheel = model.part(f"caster_{idx}")
        for side in (-caster_offset, caster_offset):
            wheel.visual(
                Cylinder(radius=caster_radius, length=caster_thickness),
                origin=Origin(xyz=(0.0, 0.0, side)),
                material=mats["rubber"],
            )
            wheel.visual(
                Cylinder(radius=0.015, length=caster_thickness * 0.9),
                origin=Origin(xyz=(0.0, 0.0, side)),
                material=mats["accent_gray"],
            )
        wheel.visual(
            Cylinder(radius=0.006, length=0.034),
            origin=Origin(),
            material=mats["black_steel"],
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.030, length=0.040),
            mass=0.24,
            origin=Origin(),
        )

    model.articulation(
        "height_adjust",
        ArticulationType.PRISMATIC,
        parent="base",
        child="column_upper",
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1500.0, velocity=0.30, lower=0.0, upper=0.10),
    )
    model.articulation(
        "swivel",
        ArticulationType.CONTINUOUS,
        parent="column_upper",
        child="seat",
        origin=Origin(xyz=(0.0, 0.0, 0.262)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=6.0),
    )
    model.articulation(
        "back_tilt",
        ArticulationType.REVOLUTE,
        parent="seat",
        child="backrest",
        origin=Origin(xyz=(0.0, -0.176, 0.018)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.4, lower=0.0, upper=0.45),
    )
    model.articulation(
        "seat_to_armrest_right",
        ArticulationType.FIXED,
        parent="seat",
        child="armrest_right",
        origin=Origin(xyz=(0.214, -0.006, -0.004)),
    )
    model.articulation(
        "seat_to_armrest_left",
        ArticulationType.FIXED,
        parent="seat",
        child="armrest_left",
        origin=Origin(xyz=(-0.214, -0.006, -0.004), rpy=(0.0, 0.0, pi)),
    )
    for idx in range(5):
        theta = idx * (2.0 * pi / 5.0)
        model.articulation(
            f"caster_{idx}_roll",
            ArticulationType.CONTINUOUS,
            parent="base",
            child=f"caster_{idx}",
            origin=Origin(
                xyz=(0.335 * cos(theta), 0.335 * sin(theta), 0.027),
                rpy=(0.0, pi / 2.0, theta + (pi / 2.0)),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "column_upper",
        "seat",
        reason="swivel plate and seat mechanism are modeled as closely interfacing solids",
    )
    ctx.allow_overlap(
        "backrest",
        "seat",
        reason="the recline hinge carrier nests into the rear seat shell, and generated collision hulls conservatively merge that tight mounting pocket",
    )
    for idx in range(5):
        ctx.allow_overlap(
            "base",
            f"caster_{idx}",
            reason="tight caster fork clearances can appear overlapping in conservative collision QC",
        )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=128, overlap_tol=0.004, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("seat", "base", axes="xy", max_dist=0.03)
    ctx.expect_aabb_overlap("column_upper", "base", axes="xy", min_overlap=0.08)
    ctx.expect_aabb_overlap("seat", "column_upper", axes="xy", min_overlap=0.08)
    ctx.expect_aabb_overlap("backrest", "seat", axes="xy", min_overlap=0.02)
    ctx.expect_joint_motion_axis(
        "height_adjust", "seat", world_axis="z", direction="positive", min_delta=0.05
    )
    ctx.expect_joint_motion_axis(
        "back_tilt", "backrest", world_axis="y", direction="negative", min_delta=0.03
    )

    def upper_back_point(origin, tilt_angle):
        local_y = -0.040
        local_z = 0.540
        return (
            origin[0],
            origin[1] + (local_y * cos(tilt_angle)) - (local_z * sin(tilt_angle)),
            origin[2] + (local_y * sin(tilt_angle)) + (local_z * cos(tilt_angle)),
        )

    seat_pos = ctx.part_world_position("seat")
    back_pos = ctx.part_world_position("backrest")
    arm_right_pos = ctx.part_world_position("armrest_right")
    arm_left_pos = ctx.part_world_position("armrest_left")
    seat_top_z = seat_pos[2] + 0.058
    back_upper_rest = upper_back_point(back_pos, 0.0)
    arm_right_pad_z = arm_right_pos[2] + 0.190
    arm_left_pad_z = arm_left_pos[2] + 0.190
    assert back_pos[1] < seat_pos[1] - 0.12, "Backrest should sit clearly behind the seat pan"
    assert back_upper_rest[2] > seat_top_z + 0.28, (
        "Backrest should rise well above the seat cushion"
    )
    assert arm_right_pad_z > seat_top_z + 0.08 and arm_left_pad_z > seat_top_z + 0.08, (
        "Arm pads should be above the seat cushion"
    )
    assert arm_right_pos[0] > seat_pos[0] + 0.16 and arm_left_pos[0] < seat_pos[0] - 0.16, (
        "Armrests should flank the seat on both sides"
    )
    assert abs(arm_right_pos[1] - arm_left_pos[1]) < 0.03, (
        "Armrests should be aligned front-to-back"
    )
    assert abs(arm_right_pad_z - arm_left_pad_z) < 0.02, "Armrests should sit at similar heights"

    for idx in range(5):
        wheel_pos = ctx.part_world_position(f"caster_{idx}")
        wheel_radius = sqrt((wheel_pos[0] ** 2) + (wheel_pos[1] ** 2))
        assert 0.29 < wheel_radius < 0.38, (
            f"Caster {idx} should sit near the perimeter of the five-star base"
        )
        assert abs(wheel_pos[2] - 0.027) < 0.008, (
            f"Caster {idx} axle should ride just above the floor"
        )

    with ctx.pose(height_adjust=0.10):
        raised_seat = ctx.part_world_position("seat")
        raised_back = ctx.part_world_position("backrest")
        assert raised_seat[2] > seat_pos[2] + 0.08, (
            "Seat should noticeably rise at maximum gas-lift height"
        )
        assert upper_back_point(raised_back, 0.0)[2] > back_upper_rest[2] + 0.08, (
            "Backrest should rise with the seat assembly"
        )
        ctx.expect_origin_distance("seat", "base", axes="xy", max_dist=0.03)

    with ctx.pose(back_tilt=0.42):
        reclined_back_origin = ctx.part_world_position("backrest")
        reclined_upper = upper_back_point(reclined_back_origin, 0.42)
        assert reclined_upper[1] < back_upper_rest[1] - 0.12, (
            "Backrest should recline rearward at its upper limit"
        )
        assert reclined_upper[2] > seat_top_z + 0.22, (
            "Recline should still leave substantial upper-back support above the seat"
        )

    with ctx.pose(swivel=pi / 2.0):
        ctx.expect_origin_distance("seat", "base", axes="xy", max_dist=0.03)
        swivel_back = ctx.part_world_position("backrest")
        assert upper_back_point(swivel_back, 0.0)[2] > seat_top_z + 0.28, (
            "Swiveled backrest should stay mounted above the seat"
        )

    wheel_rest = ctx.part_world_position("caster_0")
    with ctx.pose(caster_0_roll=pi / 2.0):
        wheel_rot = ctx.part_world_position("caster_0")
        assert abs(wheel_rot[0] - wheel_rest[0]) < 1e-6
        assert abs(wheel_rot[1] - wheel_rest[1]) < 1e-6
        assert abs(wheel_rot[2] - wheel_rest[2]) < 1e-6

    with ctx.pose(height_adjust=0.08, swivel=pi / 3.0, back_tilt=0.30):
        ctx.expect_origin_distance("seat", "base", axes="xy", max_dist=0.03)
        combo_back = ctx.part_world_position("backrest")
        assert upper_back_point(combo_back, 0.30)[2] > 0.82, (
            "Chair should maintain a tall ergonomic back profile in combined adjustment poses"
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
