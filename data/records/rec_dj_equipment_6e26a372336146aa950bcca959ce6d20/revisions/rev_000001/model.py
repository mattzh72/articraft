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


def _aabb_center(aabb):
    if aabb is None:
        return None
    minimum, maximum = aabb
    return tuple((minimum[index] + maximum[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="club_dj_turntable")

    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    deck_black = model.material("deck_black", rgba=(0.06, 0.06, 0.07, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_aluminum = model.material("dark_aluminum", rgba=(0.42, 0.44, 0.47, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.12, 0.12, 0.13, 1.0))
    satin_silver = model.material("satin_silver", rgba=(0.80, 0.82, 0.84, 1.0))
    accent_red = model.material("accent_red", rgba=(0.75, 0.12, 0.08, 1.0))

    plinth_shell_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.453, 0.353, 0.020, corner_segments=10),
            0.048,
            cap=True,
            center=True,
        ),
        "turntable_plinth_shell",
    )
    cue_sleeve_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.0115, -0.018), (0.0115, 0.0)],
            [(0.0082, -0.018), (0.0082, 0.0)],
            segments=36,
            start_cap="flat",
            end_cap="flat",
        ),
        "tonearm_cue_sleeve",
    )
    arm_tube_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.000, 0.000, 0.004),
                (0.060, 0.004, 0.005),
                (0.135, 0.012, 0.004),
                (0.222, 0.019, 0.002),
            ],
            radius=0.0046,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        "tonearm_tube",
    )

    plinth = model.part("plinth")
    plinth.visual(
        plinth_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=matte_black,
        name="plinth_shell",
    )
    plinth.visual(
        Box((0.443, 0.343, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        material=dark_aluminum,
        name="top_plate",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            plinth.visual(
                Cylinder(radius=0.024, length=0.016),
                origin=Origin(xyz=(0.180 * x_sign, 0.130 * y_sign, 0.008)),
                material=rubber_black,
                name=f"foot_{'r' if x_sign > 0.0 else 'l'}_{'b' if y_sign > 0.0 else 'f'}",
            )
    plinth.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(-0.188, -0.128, 0.070)),
        material=satin_silver,
        name="start_stop_button",
    )
    plinth.visual(
        Cylinder(radius=0.008, length=0.003),
        origin=Origin(xyz=(-0.156, -0.089, 0.0695)),
        material=satin_silver,
        name="speed_button",
    )
    plinth.visual(
        Box((0.012, 0.092, 0.004)),
        origin=Origin(xyz=(0.188, 0.000, 0.070)),
        material=deck_black,
        name="pitch_slider_slot",
    )
    plinth.visual(
        Box((0.014, 0.020, 0.005)),
        origin=Origin(xyz=(0.188, -0.034, 0.0705)),
        material=satin_silver,
        name="pitch_slider_cap",
    )
    plinth.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.126, 0.118, 0.072)),
        material=accent_red,
        name="target_light",
    )
    plinth.visual(
        Cylinder(radius=0.006, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0665)),
        material=dark_aluminum,
        name="bearing_hub",
    )
    plinth.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(0.170, -0.118, 0.074)),
        material=deck_black,
        name="tonearm_base",
    )
    plinth.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(xyz=(0.170, -0.118, 0.087)),
        material=satin_silver,
        name="tonearm_post",
    )
    plinth.inertial = Inertial.from_geometry(
        Box((0.453, 0.353, 0.084)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.166, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=brushed_aluminum,
        name="platter_disc",
    )
    platter.visual(
        Cylinder(radius=0.145, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=rubber_black,
        name="slipmat",
    )
    platter.visual(
        Cylinder(radius=0.052, length=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.0194)),
        material=deck_black,
        name="record_label_area",
    )
    platter.visual(
        Cylinder(radius=0.0018, length=0.027),
        origin=Origin(xyz=(0.0, 0.0, 0.0135)),
        material=satin_silver,
        name="spindle",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.166, length=0.020),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    cue_lift = model.part("cue_lift")
    cue_lift.visual(
        cue_sleeve_mesh,
        material=satin_silver,
        name="cue_sleeve",
    )
    cue_lift.visual(
        Cylinder(radius=0.010, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=deck_black,
        name="cue_cap",
    )
    cue_lift.visual(
        Box((0.030, 0.006, 0.003)),
        origin=Origin(xyz=(0.026, 0.0, -0.0025)),
        material=deck_black,
        name="cue_platform",
    )
    cue_lift.visual(
        Cylinder(radius=0.0025, length=0.016),
        origin=Origin(xyz=(0.026, 0.0, -0.0005), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="cue_pad",
    )
    cue_lift.inertial = Inertial.from_geometry(
        Box((0.042, 0.026, 0.022)),
        mass=0.08,
        origin=Origin(xyz=(0.012, 0.0, -0.006)),
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=deck_black,
        name="pivot_hub",
    )
    tonearm.visual(
        arm_tube_mesh,
        material=satin_silver,
        name="arm_tube",
    )
    tonearm.visual(
        Cylinder(radius=0.0032, length=0.072),
        origin=Origin(xyz=(-0.036, 0.0, 0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_silver,
        name="counterweight_shaft",
    )
    tonearm.visual(
        Cylinder(radius=0.011, length=0.032),
        origin=Origin(xyz=(-0.076, 0.0, 0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_aluminum,
        name="counterweight",
    )
    tonearm.visual(
        Box((0.026, 0.018, 0.004)),
        origin=Origin(xyz=(0.221, 0.020, 0.001), rpy=(0.0, 0.0, 0.26)),
        material=deck_black,
        name="headshell",
    )
    tonearm.visual(
        Box((0.010, 0.016, 0.007)),
        origin=Origin(xyz=(0.229, 0.021, -0.004)),
        material=deck_black,
        name="cartridge",
    )
    tonearm.visual(
        Cylinder(radius=0.0008, length=0.004),
        origin=Origin(xyz=(0.233, 0.021, -0.008), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_red,
        name="stylus",
    )
    tonearm.visual(
        Box((0.020, 0.003, 0.008)),
        origin=Origin(xyz=(0.214, 0.028, 0.005), rpy=(0.0, 0.0, 0.26)),
        material=satin_silver,
        name="finger_lift",
    )
    tonearm.inertial = Inertial.from_geometry(
        Box((0.330, 0.060, 0.030)),
        mass=0.35,
        origin=Origin(xyz=(0.080, 0.010, 0.004)),
    )

    model.articulation(
        "platter_spin",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=30.0),
    )
    model.articulation(
        "cueing_lift",
        ArticulationType.PRISMATIC,
        parent=plinth,
        child=cue_lift,
        origin=Origin(xyz=(0.170, -0.118, 0.094)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.05, lower=0.0, upper=0.007),
    )
    model.articulation(
        "tonearm_sweep",
        ArticulationType.REVOLUTE,
        parent=cue_lift,
        child=tonearm,
        origin=Origin(rpy=(0.0, 0.0, math.radians(35.0))),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=1.0,
            lower=0.0,
            upper=2.1,
        ),
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

    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    cue_lift = object_model.get_part("cue_lift")
    tonearm = object_model.get_part("tonearm")
    platter_spin = object_model.get_articulation("platter_spin")
    cueing_lift = object_model.get_articulation("cueing_lift")
    tonearm_sweep = object_model.get_articulation("tonearm_sweep")

    ctx.check(
        "prompt-critical parts exist",
        all(part is not None for part in (plinth, platter, cue_lift, tonearm)),
        details="Missing one or more of plinth, platter, cue lift, or tonearm.",
    )
    ctx.check(
        "prompt-critical articulations exist",
        all(joint is not None for joint in (platter_spin, cueing_lift, tonearm_sweep)),
        details="Missing platter spin, cueing lift, or tonearm sweep articulation.",
    )

    with ctx.pose({platter_spin: 0.0, cueing_lift: 0.0, tonearm_sweep: 0.0}):
        ctx.expect_gap(
            platter,
            plinth,
            axis="z",
            positive_elem="platter_disc",
            negative_elem="top_plate",
            min_gap=0.0005,
            max_gap=0.003,
            name="platter rides just above the top deck",
        )
        rest_headshell_aabb = ctx.part_element_world_aabb(tonearm, elem="headshell")
        rest_arm_pos = ctx.part_world_position(tonearm)

    lift_upper = cueing_lift.motion_limits.upper if cueing_lift.motion_limits is not None else 0.0
    with ctx.pose({cueing_lift: lift_upper}):
        lifted_arm_pos = ctx.part_world_position(tonearm)

    ctx.check(
        "cueing lift raises the tonearm",
        rest_arm_pos is not None
        and lifted_arm_pos is not None
        and lifted_arm_pos[2] > rest_arm_pos[2] + 0.005,
        details=f"rest={rest_arm_pos}, lifted={lifted_arm_pos}",
    )

    with ctx.pose({cueing_lift: lift_upper, tonearm_sweep: 1.55}):
        play_headshell_aabb = ctx.part_element_world_aabb(tonearm, elem="headshell")
        ctx.expect_overlap(
            tonearm,
            platter,
            axes="xy",
            elem_a="headshell",
            elem_b="slipmat",
            min_overlap=0.008,
            name="tonearm can sweep the headshell over the record area",
        )
        ctx.expect_gap(
            tonearm,
            platter,
            axis="z",
            positive_elem="cartridge",
            negative_elem="slipmat",
            min_gap=0.002,
            max_gap=0.025,
            name="lifted cartridge clears the platter surface",
        )

    rest_headshell_center = _aabb_center(rest_headshell_aabb)
    play_headshell_center = _aabb_center(play_headshell_aabb)
    ctx.check(
        "tonearm sweep moves the headshell inward across the deck",
        rest_headshell_center is not None
        and play_headshell_center is not None
        and play_headshell_center[0] < rest_headshell_center[0] - 0.10
        and play_headshell_center[1] > rest_headshell_center[1] + 0.025,
        details=f"rest={rest_headshell_center}, play={play_headshell_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
