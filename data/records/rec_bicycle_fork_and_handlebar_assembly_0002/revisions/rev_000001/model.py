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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
STANCHION_X = 0.09
STANCHION_RADIUS = 0.0185
SLIDER_MOUTH_Z = 0.10
FORK_TRAVEL = 0.12


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _slider_shell_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.044, -0.390),
            (0.044, -0.342),
            (0.040, -0.240),
            (0.036, -0.118),
            (0.032, 0.000),
        ],
        [
            (0.000, -0.300),
            (0.0210, -0.300),
            (0.0215, -0.205),
            (0.0220, -0.108),
            (0.0220, 0.000),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )


def _seal_ring_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.0270, -0.003),
            (0.0262, 0.006),
        ],
        [
            (0.0182, -0.003),
            (0.0184, 0.006),
        ],
        segments=44,
        start_cap="flat",
        end_cap="flat",
    )


def _fork_arch_mesh():
    return sweep_profile_along_spline(
        [
            (-0.102, 0.036, -0.098),
            (-0.060, 0.040, -0.062),
            (0.000, 0.044, -0.030),
            (0.060, 0.040, -0.062),
            (0.102, 0.036, -0.098),
        ],
        profile=rounded_rect_profile(0.032, 0.060, radius=0.012, corner_segments=6),
        samples_per_segment=16,
        cap_profile=True,
    )


def _handlebar_mesh():
    return tube_from_spline_points(
        [
            (-0.345, 0.101, 0.772),
            (-0.225, 0.096, 0.799),
            (-0.105, 0.092, 0.779),
            (0.000, 0.095, 0.748),
            (0.105, 0.092, 0.779),
            (0.225, 0.096, 0.799),
            (0.345, 0.101, 0.772),
        ],
        radius=0.011,
        samples_per_segment=18,
        radial_segments=20,
        cap_ends=True,
    )


def _add_dropout_visuals(part, side_x: float, prefix: str, material) -> None:
    part.visual(
        Box((0.060, 0.036, 0.050)),
        origin=Origin(xyz=(side_x, 0.0, -0.350)),
        material=material,
        name=f"{prefix}_foot",
    )
    part.visual(
        Box((0.040, 0.020, 0.010)),
        origin=Origin(xyz=(side_x, 0.0, -0.365)),
        material=material,
        name=f"{prefix}_dropout_bridge",
    )
    part.visual(
        Box((0.040, 0.006, 0.040)),
        origin=Origin(xyz=(side_x, 0.007, -0.390)),
        material=material,
        name=f"{prefix}_dropout_front_jaw",
    )
    part.visual(
        Box((0.040, 0.006, 0.040)),
        origin=Origin(xyz=(side_x, -0.007, -0.390)),
        material=material,
        name=f"{prefix}_dropout_rear_jaw",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mountain_bike_suspension_fork", assets=ASSETS)

    anodized_gold = model.material("anodized_gold", rgba=(0.71, 0.58, 0.20, 1.0))
    casting_black = model.material("casting_black", rgba=(0.17, 0.17, 0.18, 1.0))
    cockpit_black = model.material("cockpit_black", rgba=(0.09, 0.09, 0.10, 1.0))
    silver = model.material("silver", rgba=(0.71, 0.73, 0.76, 1.0))
    grip_black = model.material("grip_black", rgba=(0.05, 0.05, 0.05, 1.0))
    seal_black = model.material("seal_black", rgba=(0.04, 0.04, 0.04, 1.0))

    slider_shell = _save_mesh("fork_slider_shell.obj", _slider_shell_mesh())
    seal_ring = _save_mesh("fork_wiper_ring.obj", _seal_ring_mesh())
    fork_arch = _save_mesh("fork_arch.obj", _fork_arch_mesh())
    handlebar_mesh = _save_mesh("fork_handlebar.obj", _handlebar_mesh())

    upper_fork = model.part("upper_fork")
    upper_fork.visual(
        Cylinder(radius=STANCHION_RADIUS, length=0.520),
        origin=Origin(xyz=(-STANCHION_X, 0.0, 0.290)),
        material=anodized_gold,
        name="left_stanchion",
    )
    upper_fork.visual(
        Cylinder(radius=STANCHION_RADIUS, length=0.520),
        origin=Origin(xyz=(STANCHION_X, 0.0, 0.290)),
        material=anodized_gold,
        name="right_stanchion",
    )
    upper_fork.visual(
        Box((0.236, 0.056, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.445)),
        material=casting_black,
        name="lower_crown",
    )
    upper_fork.visual(
        Cylinder(radius=0.028, length=0.056),
        origin=Origin(xyz=(-STANCHION_X, 0.0, 0.445), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=casting_black,
        name="left_lower_crown_clamp",
    )
    upper_fork.visual(
        Cylinder(radius=0.028, length=0.056),
        origin=Origin(xyz=(STANCHION_X, 0.0, 0.445), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=casting_black,
        name="right_lower_crown_clamp",
    )
    upper_fork.visual(
        Box((0.208, 0.050, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.555)),
        material=casting_black,
        name="upper_crown",
    )
    upper_fork.visual(
        Cylinder(radius=0.026, length=0.050),
        origin=Origin(xyz=(-STANCHION_X, 0.0, 0.555), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=casting_black,
        name="left_upper_crown_clamp",
    )
    upper_fork.visual(
        Cylinder(radius=0.026, length=0.050),
        origin=Origin(xyz=(STANCHION_X, 0.0, 0.555), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=casting_black,
        name="right_upper_crown_clamp",
    )
    upper_fork.visual(
        Cylinder(radius=0.026, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.555), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=casting_black,
        name="steerer_boss",
    )
    upper_fork.visual(
        Cylinder(radius=0.017, length=0.310),
        origin=Origin(xyz=(0.0, 0.0, 0.625)),
        material=silver,
        name="steerer_tube",
    )
    upper_fork.visual(
        Box((0.060, 0.050, 0.050)),
        origin=Origin(xyz=(0.0, 0.015, 0.682)),
        material=cockpit_black,
        name="steerer_clamp",
    )
    upper_fork.visual(
        Box((0.050, 0.105, 0.060)),
        origin=Origin(xyz=(0.0, 0.067, 0.708)),
        material=cockpit_black,
        name="stem_body",
    )
    upper_fork.visual(
        Box((0.046, 0.022, 0.056)),
        origin=Origin(xyz=(0.0, 0.106, 0.748)),
        material=cockpit_black,
        name="faceplate",
    )
    upper_fork.visual(
        Cylinder(radius=0.0065, length=0.016),
        origin=Origin(xyz=(0.0, 0.124, 0.764), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="upper_faceplate_bolt",
    )
    upper_fork.visual(
        Cylinder(radius=0.0065, length=0.016),
        origin=Origin(xyz=(0.0, 0.124, 0.732), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="lower_faceplate_bolt",
    )
    upper_fork.visual(handlebar_mesh, material=cockpit_black, name="handlebar_bar")
    upper_fork.visual(
        Cylinder(radius=0.014, length=0.105),
        origin=Origin(xyz=(-0.360, 0.101, 0.772), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    upper_fork.visual(
        Cylinder(radius=0.014, length=0.105),
        origin=Origin(xyz=(0.360, 0.101, 0.772), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )
    upper_fork.inertial = Inertial.from_geometry(
        Box((0.76, 0.14, 0.82)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.070, 0.410)),
    )

    lower_casting = model.part("lower_casting")
    lower_casting.visual(
        slider_shell,
        origin=Origin(xyz=(-STANCHION_X, 0.0, 0.0)),
        material=casting_black,
        name="left_slider",
    )
    lower_casting.visual(
        slider_shell,
        origin=Origin(xyz=(STANCHION_X, 0.0, 0.0)),
        material=casting_black,
        name="right_slider",
    )
    lower_casting.visual(
        seal_ring,
        origin=Origin(xyz=(-STANCHION_X, 0.0, -0.001)),
        material=seal_black,
        name="left_wiper",
    )
    lower_casting.visual(
        seal_ring,
        origin=Origin(xyz=(STANCHION_X, 0.0, -0.001)),
        material=seal_black,
        name="right_wiper",
    )
    lower_casting.visual(fork_arch, material=casting_black, name="fork_arch")
    _add_dropout_visuals(lower_casting, -STANCHION_X, "left", casting_black)
    _add_dropout_visuals(lower_casting, STANCHION_X, "right", casting_black)
    lower_casting.inertial = Inertial.from_geometry(
        Box((0.30, 0.11, 0.42)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, -0.200)),
    )

    model.articulation(
        "fork_travel",
        ArticulationType.PRISMATIC,
        parent=upper_fork,
        child=lower_casting,
        origin=Origin(xyz=(0.0, 0.0, SLIDER_MOUTH_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=1.4,
            lower=0.0,
            upper=FORK_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    upper_fork = object_model.get_part("upper_fork")
    lower_casting = object_model.get_part("lower_casting")
    fork_travel = object_model.get_articulation("fork_travel")

    steerer_tube = upper_fork.get_visual("steerer_tube")
    left_stanchion = upper_fork.get_visual("left_stanchion")
    right_stanchion = upper_fork.get_visual("right_stanchion")
    lower_crown = upper_fork.get_visual("lower_crown")
    upper_crown = upper_fork.get_visual("upper_crown")
    stem_clamp = upper_fork.get_visual("steerer_clamp")
    faceplate = upper_fork.get_visual("faceplate")
    upper_faceplate_bolt = upper_fork.get_visual("upper_faceplate_bolt")
    lower_faceplate_bolt = upper_fork.get_visual("lower_faceplate_bolt")
    handlebar_bar = upper_fork.get_visual("handlebar_bar")
    left_slider = lower_casting.get_visual("left_slider")
    right_slider = lower_casting.get_visual("right_slider")
    left_wiper = lower_casting.get_visual("left_wiper")
    right_wiper = lower_casting.get_visual("right_wiper")
    left_foot = lower_casting.get_visual("left_foot")
    right_foot = lower_casting.get_visual("right_foot")
    left_front_jaw = lower_casting.get_visual("left_dropout_front_jaw")
    left_rear_jaw = lower_casting.get_visual("left_dropout_rear_jaw")
    right_front_jaw = lower_casting.get_visual("right_dropout_front_jaw")
    right_rear_jaw = lower_casting.get_visual("right_dropout_rear_jaw")
    fork_arch = lower_casting.get_visual("fork_arch")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(
        lower_casting,
        upper_fork,
        reason="Wiper seals lightly preload on the stanchion tubes to keep the sliding interface seated.",
    )
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_overlap(
        upper_fork,
        upper_fork,
        axes="xy",
        min_overlap=0.0006,
        elem_a=stem_clamp,
        elem_b=steerer_tube,
        name="stem_clamps_steerer",
    )
    ctx.expect_overlap(
        upper_fork,
        upper_fork,
        axes="yz",
        min_overlap=0.0010,
        elem_a=handlebar_bar,
        elem_b=faceplate,
        name="riser_bar_sits_in_faceplate",
    )
    ctx.expect_gap(
        upper_fork,
        upper_fork,
        axis="z",
        min_gap=0.085,
        positive_elem=upper_crown,
        negative_elem=lower_crown,
        name="upper_crown_sits_above_lower_crown",
    )
    ctx.expect_within(
        upper_fork,
        upper_fork,
        axes="xy",
        inner_elem=steerer_tube,
        outer_elem=upper_crown,
        name="steerer_runs_through_upper_crown",
    )
    ctx.expect_contact(
        upper_fork,
        upper_fork,
        elem_a=upper_faceplate_bolt,
        elem_b=faceplate,
        name="upper_faceplate_bolt_is_visible_and_seated",
    )
    ctx.expect_contact(
        upper_fork,
        upper_fork,
        elem_a=lower_faceplate_bolt,
        elem_b=faceplate,
        name="lower_faceplate_bolt_is_visible_and_seated",
    )
    ctx.expect_overlap(
        lower_casting,
        upper_fork,
        axes="xy",
        min_overlap=0.0010,
        elem_a=left_slider,
        elem_b=left_stanchion,
        name="left_stanchion_runs_in_left_slider",
    )
    ctx.expect_overlap(
        lower_casting,
        upper_fork,
        axes="xy",
        min_overlap=0.0010,
        elem_a=right_slider,
        elem_b=right_stanchion,
        name="right_stanchion_runs_in_right_slider",
    )
    ctx.expect_overlap(
        lower_casting,
        upper_fork,
        axes="xy",
        min_overlap=0.0005,
        elem_a=left_wiper,
        elem_b=left_stanchion,
        name="left_wiper_wraps_left_stanchion",
    )
    ctx.expect_overlap(
        lower_casting,
        upper_fork,
        axes="xy",
        min_overlap=0.0005,
        elem_a=right_wiper,
        elem_b=right_stanchion,
        name="right_wiper_wraps_right_stanchion",
    )
    ctx.expect_within(
        lower_casting,
        lower_casting,
        axes="xy",
        inner_elem=left_wiper,
        outer_elem=left_slider,
        name="left_wiper_sits_at_left_slider_mouth",
    )
    ctx.expect_within(
        lower_casting,
        lower_casting,
        axes="xy",
        inner_elem=right_wiper,
        outer_elem=right_slider,
        name="right_wiper_sits_at_right_slider_mouth",
    )
    ctx.expect_gap(
        lower_casting,
        lower_casting,
        axis="y",
        min_gap=0.007,
        positive_elem=left_front_jaw,
        negative_elem=left_rear_jaw,
        name="left_dropout_has_quick_release_slot",
    )
    ctx.expect_gap(
        lower_casting,
        lower_casting,
        axis="y",
        min_gap=0.007,
        positive_elem=right_front_jaw,
        negative_elem=right_rear_jaw,
        name="right_dropout_has_quick_release_slot",
    )
    ctx.expect_gap(
        lower_casting,
        lower_casting,
        axis="z",
        min_gap=0.30,
        positive_elem=left_wiper,
        negative_elem=left_foot,
        name="left_dropout_sits_well_below_slider_mouth",
    )
    ctx.expect_gap(
        lower_casting,
        lower_casting,
        axis="z",
        min_gap=0.30,
        positive_elem=right_wiper,
        negative_elem=right_foot,
        name="right_dropout_sits_well_below_slider_mouth",
    )
    with ctx.pose({fork_travel: 0.10}):
        ctx.expect_overlap(
            lower_casting,
            upper_fork,
            axes="xy",
            min_overlap=0.0010,
            elem_a=left_slider,
            elem_b=left_stanchion,
            name="left_slider_tracks_left_stanchion_under_compression",
        )
        ctx.expect_overlap(
            lower_casting,
            upper_fork,
            axes="xy",
            min_overlap=0.0010,
            elem_a=right_slider,
            elem_b=right_stanchion,
            name="right_slider_tracks_right_stanchion_under_compression",
        )
        ctx.expect_overlap(
            lower_casting,
            upper_fork,
            axes="xy",
            min_overlap=0.0005,
            elem_a=left_wiper,
            elem_b=left_stanchion,
            name="left_wiper_stays_on_left_stanchion_under_compression",
        )
        ctx.expect_overlap(
            lower_casting,
            upper_fork,
            axes="xy",
            min_overlap=0.0005,
            elem_a=right_wiper,
            elem_b=right_stanchion,
            name="right_wiper_stays_on_right_stanchion_under_compression",
        )
        ctx.expect_gap(
            upper_fork,
            lower_casting,
            axis="z",
            min_gap=0.22,
            positive_elem=lower_crown,
            negative_elem=fork_arch,
            name="crown_stays_well_above_arch_at_compression",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
