from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="city_bike_front_end", assets=ASSETS)

    enamel_black = model.material("enamel_black", rgba=(0.12, 0.13, 0.14, 1.0))
    chrome = model.material("chrome", rgba=(0.81, 0.83, 0.86, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.69, 0.71, 0.75, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
        return [(-x, y, z) for x, y, z in points]

    head_tube = model.part("head_tube")
    head_tube.inertial = Inertial.from_geometry(
        Box((0.09, 0.09, 0.22)),
        mass=1.6,
        origin=Origin(),
    )
    head_tube_shell = save_mesh(
        "head_tube_shell.obj",
        LatheGeometry.from_shell_profiles(
            [(0.028, -0.075), (0.028, 0.075)],
            [(0.019, -0.075), (0.019, 0.075)],
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    upper_cup = save_mesh(
        "upper_headset_cup.obj",
        LatheGeometry.from_shell_profiles(
            [(0.033, 0.075), (0.033, 0.098)],
            [(0.017, 0.075), (0.017, 0.098)],
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    lower_cup = save_mesh(
        "lower_headset_cup.obj",
        LatheGeometry.from_shell_profiles(
            [(0.033, -0.098), (0.033, -0.075)],
            [(0.017, -0.098), (0.017, -0.075)],
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    head_tube.visual(head_tube_shell, material=enamel_black, name="head_tube_shell")
    head_tube.visual(upper_cup, material=polished_steel, name="upper_cup")
    head_tube.visual(lower_cup, material=polished_steel, name="lower_cup")

    steer_assembly = model.part("steer_assembly")
    steer_assembly.inertial = Inertial.from_geometry(
        Box((0.82, 0.42, 0.86)),
        mass=3.8,
        origin=Origin(xyz=(0.0, -0.02, -0.14)),
    )
    steer_assembly.visual(
        Cylinder(radius=0.012, length=0.39),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=polished_steel,
        name="steerer",
    )
    steer_assembly.visual(
        Cylinder(radius=0.0135, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=chrome,
        name="threaded_section",
    )
    steer_assembly.visual(
        Cylinder(radius=0.022, length=0.01),
        origin=Origin(xyz=(0.0, 0.0, 0.103)),
        material=polished_steel,
        name="upper_locknut",
    )
    steer_assembly.visual(
        Cylinder(radius=0.021, length=0.01),
        origin=Origin(xyz=(0.0, 0.0, -0.103)),
        material=polished_steel,
        name="lower_race",
    )
    steer_assembly.visual(
        Cylinder(radius=0.011, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=chrome,
        name="quill_column",
    )
    steer_assembly.visual(
        save_mesh(
            "stem_extension.obj",
            tube_from_spline_points(
                [(0.0, 0.0, 0.195), (0.0, 0.04, 0.213), (0.0, 0.088, 0.228)],
                radius=0.011,
                samples_per_segment=14,
                radial_segments=18,
            ),
        ),
        material=chrome,
        name="stem_extension",
    )
    steer_assembly.visual(
        Cylinder(radius=0.016, length=0.072),
        origin=Origin(xyz=(0.0, 0.092, 0.228), rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="bar_clamp",
    )
    steer_assembly.visual(
        save_mesh(
            "cruiser_handlebar.obj",
            tube_from_spline_points(
                [
                    (-0.35, -0.145, 0.247),
                    (-0.29, -0.112, 0.262),
                    (-0.18, -0.052, 0.283),
                    (-0.08, 0.028, 0.266),
                    (0.0, 0.092, 0.228),
                    (0.08, 0.028, 0.266),
                    (0.18, -0.052, 0.283),
                    (0.29, -0.112, 0.262),
                    (0.35, -0.145, 0.247),
                ],
                radius=0.011,
                samples_per_segment=18,
                radial_segments=20,
            ),
        ),
        material=chrome,
        name="bar",
    )
    steer_assembly.visual(
        Cylinder(radius=0.017, length=0.105),
        origin=Origin(xyz=(-0.352, -0.145, 0.247), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    steer_assembly.visual(
        Cylinder(radius=0.017, length=0.105),
        origin=Origin(xyz=(0.352, -0.145, 0.247), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="right_grip",
    )
    steer_assembly.visual(
        Cylinder(radius=0.022, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.125)),
        material=chrome,
        name="fork_crown",
    )
    steer_assembly.visual(
        save_mesh(
            "left_unicrown_shoulder.obj",
            tube_from_spline_points(
                [(0.0, 0.0, -0.123), (0.022, 0.010, -0.130), (0.040, 0.020, -0.150), (0.049, 0.022, -0.178)],
                radius=0.0125,
                samples_per_segment=14,
                radial_segments=18,
            ),
        ),
        material=chrome,
        name="left_unicrown",
    )
    steer_assembly.visual(
        save_mesh(
            "right_unicrown_shoulder.obj",
            tube_from_spline_points(
                mirror_x(
                    [(0.0, 0.0, -0.123), (0.022, 0.010, -0.130), (0.040, 0.020, -0.150), (0.049, 0.022, -0.178)]
                ),
                radius=0.0125,
                samples_per_segment=14,
                radial_segments=18,
            ),
        ),
        material=chrome,
        name="right_unicrown",
    )
    steer_assembly.visual(
        save_mesh(
            "left_fork_blade.obj",
            tube_from_spline_points(
                [(0.049, 0.022, -0.178), (0.056, 0.035, -0.275), (0.061, 0.049, -0.410), (0.065, 0.060, -0.545)],
                radius=0.010,
                samples_per_segment=18,
                radial_segments=18,
            ),
        ),
        material=chrome,
        name="left_blade",
    )
    steer_assembly.visual(
        save_mesh(
            "right_fork_blade.obj",
            tube_from_spline_points(
                mirror_x([(0.049, 0.022, -0.178), (0.056, 0.035, -0.275), (0.061, 0.049, -0.410), (0.065, 0.060, -0.545)]),
                radius=0.010,
                samples_per_segment=18,
                radial_segments=18,
            ),
        ),
        material=chrome,
        name="right_blade",
    )
    steer_assembly.visual(
        Box((0.020, 0.010, 0.035)),
        origin=Origin(xyz=(0.065, 0.060, -0.5575)),
        material=polished_steel,
        name="left_dropout",
    )
    steer_assembly.visual(
        Box((0.020, 0.010, 0.035)),
        origin=Origin(xyz=(-0.065, 0.060, -0.5575)),
        material=polished_steel,
        name="right_dropout",
    )

    model.articulation(
        "steering",
        ArticulationType.REVOLUTE,
        parent=head_tube,
        child=steer_assembly,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=-0.65, upper=0.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    head_tube = object_model.get_part("head_tube")
    steer_assembly = object_model.get_part("steer_assembly")
    steering = object_model.get_articulation("steering")
    head_tube_shell = head_tube.get_visual("head_tube_shell")
    upper_cup = head_tube.get_visual("upper_cup")
    lower_cup = head_tube.get_visual("lower_cup")
    steerer = steer_assembly.get_visual("steerer")
    threaded_section = steer_assembly.get_visual("threaded_section")
    upper_locknut = steer_assembly.get_visual("upper_locknut")
    lower_race = steer_assembly.get_visual("lower_race")
    bar_clamp = steer_assembly.get_visual("bar_clamp")
    fork_crown = steer_assembly.get_visual("fork_crown")
    left_blade = steer_assembly.get_visual("left_blade")
    right_blade = steer_assembly.get_visual("right_blade")
    left_dropout = steer_assembly.get_visual("left_dropout")
    right_dropout = steer_assembly.get_visual("right_dropout")
    bar = steer_assembly.get_visual("bar")
    left_grip = steer_assembly.get_visual("left_grip")
    right_grip = steer_assembly.get_visual("right_grip")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # The steering axis runs through the hollow head tube shell, so this conservative
    # sensor needs a slightly looser tolerance than the compact default.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.02)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)
    ctx.expect_origin_distance(steer_assembly, head_tube, axes="xy", max_dist=0.0001)
    ctx.expect_overlap(steer_assembly, head_tube, axes="xy", min_overlap=0.020, elem_a=steerer, elem_b=head_tube_shell)
    ctx.expect_gap(
        steer_assembly,
        head_tube,
        axis="z",
        min_gap=0.005,
        positive_elem=threaded_section,
        negative_elem=upper_cup,
        name="threaded_steerer_projects_above_upper_cup",
    )
    ctx.expect_gap(
        steer_assembly,
        head_tube,
        axis="y",
        min_gap=0.040,
        positive_elem=bar_clamp,
        negative_elem=head_tube_shell,
        name="quill_stem_projects_forward_of_head_tube",
    )
    ctx.expect_gap(
        head_tube,
        steer_assembly,
        axis="x",
        min_gap=0.250,
        positive_elem=head_tube_shell,
        negative_elem=left_grip,
        name="left_grip_wide_of_head_tube",
    )
    ctx.expect_gap(
        steer_assembly,
        head_tube,
        axis="x",
        min_gap=0.250,
        positive_elem=right_grip,
        negative_elem=head_tube_shell,
        name="right_grip_wide_of_head_tube",
    )
    ctx.expect_gap(
        steer_assembly,
        head_tube,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0005,
        positive_elem=upper_locknut,
        negative_elem=upper_cup,
        name="upper_headset_locknut_seated",
    )
    ctx.expect_gap(
        head_tube,
        steer_assembly,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0005,
        positive_elem=lower_cup,
        negative_elem=lower_race,
        name="lower_headset_race_seated",
    )
    ctx.expect_gap(
        steer_assembly,
        head_tube,
        axis="x",
        min_gap=0.004,
        positive_elem=left_blade,
        negative_elem=head_tube_shell,
        name="left_blade_outboard_of_head_tube",
    )
    ctx.expect_gap(
        head_tube,
        steer_assembly,
        axis="x",
        min_gap=0.004,
        positive_elem=head_tube_shell,
        negative_elem=right_blade,
        name="right_blade_outboard_of_head_tube",
    )
    ctx.expect_gap(
        head_tube,
        steer_assembly,
        axis="z",
        min_gap=0.006,
        positive_elem=lower_cup,
        negative_elem=fork_crown,
        name="fork_crown_below_lower_cup",
    )
    ctx.expect_gap(
        head_tube,
        steer_assembly,
        axis="z",
        min_gap=0.380,
        positive_elem=lower_cup,
        negative_elem=left_dropout,
        name="left_dropout_well_below_head_tube",
    )
    ctx.expect_gap(
        head_tube,
        steer_assembly,
        axis="z",
        min_gap=0.380,
        positive_elem=lower_cup,
        negative_elem=right_dropout,
        name="right_dropout_well_below_head_tube",
    )
    ctx.expect_gap(
        head_tube,
        steer_assembly,
        axis="y",
        min_gap=0.080,
        positive_elem=head_tube_shell,
        negative_elem=left_grip,
        name="left_grip_sweeps_back",
    )
    ctx.expect_gap(
        head_tube,
        steer_assembly,
        axis="y",
        min_gap=0.080,
        positive_elem=head_tube_shell,
        negative_elem=right_grip,
        name="right_grip_sweeps_back",
    )
    ctx.expect_gap(
        steer_assembly,
        head_tube,
        axis="z",
        min_gap=0.100,
        positive_elem=bar,
        negative_elem=upper_cup,
        name="handlebar_rises_above_quill_stem_mount",
    )
    for angle in (0.55, -0.55):
        with ctx.pose({steering: angle}):
            ctx.expect_origin_distance(steer_assembly, head_tube, axes="xy", max_dist=0.0001)
            ctx.expect_overlap(steer_assembly, head_tube, axes="xy", min_overlap=0.020, elem_a=steerer, elem_b=head_tube_shell)
            ctx.expect_gap(
                steer_assembly,
                head_tube,
                axis="z",
                max_gap=0.001,
                max_penetration=0.0005,
                positive_elem=upper_locknut,
                negative_elem=upper_cup,
                name=f"upper_headset_locknut_stays_seated_{angle:+.2f}",
            )
            ctx.expect_gap(
                head_tube,
                steer_assembly,
                axis="z",
                max_gap=0.001,
                max_penetration=0.0005,
                positive_elem=lower_cup,
                negative_elem=lower_race,
                name=f"lower_headset_race_stays_seated_{angle:+.2f}",
            )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
