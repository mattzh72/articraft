from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="box_fan")

    plastic = model.material("fan_plastic", rgba=(0.86, 0.88, 0.90, 1.0))
    dark_plastic = model.material("dark_trim", rgba=(0.18, 0.19, 0.22, 1.0))
    blade_tint = model.material("blade_tint", rgba=(0.72, 0.80, 0.86, 0.92))

    outer_w = 0.56
    outer_h = 0.56
    depth = 0.15
    frame_w = 0.055
    bezel_d = 0.018
    side_d = depth - 2.0 * bezel_d
    front_y = depth / 2.0 - bezel_d / 2.0
    rear_y = -depth / 2.0 + bezel_d / 2.0
    opening_half = outer_w / 2.0 - frame_w

    housing = model.part("housing")

    housing.visual(
        Box((frame_w, bezel_d, outer_h)),
        origin=Origin(xyz=(-outer_w / 2.0 + frame_w / 2.0, front_y, 0.0)),
        material=plastic,
        name="front_left_frame",
    )
    housing.visual(
        Box((frame_w, bezel_d, outer_h)),
        origin=Origin(xyz=(outer_w / 2.0 - frame_w / 2.0, front_y, 0.0)),
        material=plastic,
        name="front_right_frame",
    )
    housing.visual(
        Box((outer_w - 2.0 * frame_w, bezel_d, frame_w)),
        origin=Origin(xyz=(0.0, front_y, outer_h / 2.0 - frame_w / 2.0)),
        material=plastic,
        name="front_top_frame",
    )
    housing.visual(
        Box((outer_w - 2.0 * frame_w, bezel_d, frame_w)),
        origin=Origin(xyz=(0.0, front_y, -outer_h / 2.0 + frame_w / 2.0)),
        material=plastic,
        name="front_bottom_frame",
    )

    housing.visual(
        Box((frame_w, bezel_d, outer_h)),
        origin=Origin(xyz=(-outer_w / 2.0 + frame_w / 2.0, rear_y, 0.0)),
        material=plastic,
        name="rear_left_frame",
    )
    housing.visual(
        Box((frame_w, bezel_d, outer_h)),
        origin=Origin(xyz=(outer_w / 2.0 - frame_w / 2.0, rear_y, 0.0)),
        material=plastic,
        name="rear_right_frame",
    )
    housing.visual(
        Box((outer_w - 2.0 * frame_w, bezel_d, frame_w)),
        origin=Origin(xyz=(0.0, rear_y, outer_h / 2.0 - frame_w / 2.0)),
        material=plastic,
        name="rear_top_frame",
    )
    housing.visual(
        Box((outer_w - 2.0 * frame_w, bezel_d, frame_w)),
        origin=Origin(xyz=(0.0, rear_y, -outer_h / 2.0 + frame_w / 2.0)),
        material=plastic,
        name="rear_bottom_frame",
    )

    rear_inner_w = outer_w - 2.0 * frame_w
    rear_inner_h = outer_h - 2.0 * frame_w
    rear_bar_w = 0.006
    rear_bar_y = rear_y
    for idx in range(1, 6):
        x = -rear_inner_w / 2.0 + idx * rear_inner_w / 6.0
        housing.visual(
            Box((rear_bar_w, bezel_d, rear_inner_h)),
            origin=Origin(xyz=(x, rear_bar_y, 0.0)),
            material=plastic,
            name=f"rear_vertical_bar_{idx}",
        )
    for idx in range(1, 6):
        z = -rear_inner_h / 2.0 + idx * rear_inner_h / 6.0
        housing.visual(
            Box((rear_inner_w, bezel_d, rear_bar_w)),
            origin=Origin(xyz=(0.0, rear_bar_y, z)),
            material=plastic,
            name=f"rear_horizontal_bar_{idx}",
        )

    housing.visual(
        Box((frame_w, side_d, outer_h)),
        origin=Origin(xyz=(-outer_w / 2.0 + frame_w / 2.0, 0.0, 0.0)),
        material=plastic,
        name="left_side_shell",
    )
    housing.visual(
        Box((frame_w, side_d, outer_h)),
        origin=Origin(xyz=(outer_w / 2.0 - frame_w / 2.0, 0.0, 0.0)),
        material=plastic,
        name="right_side_shell",
    )
    housing.visual(
        Box((outer_w - 2.0 * frame_w, side_d, frame_w)),
        origin=Origin(xyz=(0.0, 0.0, outer_h / 2.0 - frame_w / 2.0)),
        material=plastic,
        name="top_shell",
    )
    housing.visual(
        Box((outer_w - 2.0 * frame_w, side_d, frame_w)),
        origin=Origin(xyz=(0.0, 0.0, -outer_h / 2.0 + frame_w / 2.0)),
        material=plastic,
        name="bottom_shell",
    )

    motor_center_y = -0.055
    housing.visual(
        Cylinder(radius=0.058, length=0.044),
        origin=Origin(xyz=(0.0, motor_center_y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="motor_housing",
    )

    brace_len = opening_half - 0.058
    brace_depth = 0.012
    brace_thickness = 0.018
    brace_y = rear_y + bezel_d / 2.0
    housing.visual(
        Box((brace_len, brace_depth, brace_thickness)),
        origin=Origin(xyz=(-(0.058 + brace_len / 2.0), brace_y, 0.0)),
        material=dark_plastic,
        name="left_motor_brace",
    )
    housing.visual(
        Box((brace_len, brace_depth, brace_thickness)),
        origin=Origin(xyz=((0.058 + brace_len / 2.0), brace_y, 0.0)),
        material=dark_plastic,
        name="right_motor_brace",
    )
    housing.visual(
        Box((brace_thickness, brace_depth, brace_len)),
        origin=Origin(xyz=(0.0, brace_y, -(0.058 + brace_len / 2.0))),
        material=dark_plastic,
        name="bottom_motor_brace",
    )
    housing.visual(
        Box((brace_thickness, brace_depth, brace_len)),
        origin=Origin(xyz=(0.0, brace_y, (0.058 + brace_len / 2.0))),
        material=dark_plastic,
        name="top_motor_brace",
    )

    control_boss_y = -0.008
    control_boss_z = 0.170
    housing.visual(
        Cylinder(radius=0.032, length=0.010),
        origin=Origin(
            xyz=(outer_w / 2.0 - 0.005, control_boss_y, control_boss_z),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=plastic,
        name="control_boss",
    )

    rotor = model.part("rotor")
    rotor_mesh = mesh_from_geometry(
        FanRotorGeometry(
            0.190,
            0.048,
            3,
            thickness=0.020,
            blade_pitch_deg=18.0,
            blade_sweep_deg=30.0,
            blade_root_chord=0.105,
            blade_tip_chord=0.070,
        ),
        "rotor_blades",
    )
    rotor.visual(
        rotor_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=blade_tint,
        name="blade_set",
    )
    rotor.visual(
        Cylinder(radius=0.010, length=0.028),
        origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="shaft_stub",
    )

    model.articulation(
        "housing_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=rotor,
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=25.0),
    )

    grille = model.part("front_grille")
    grille_w = 0.550
    grille_h = 0.550
    grille_t = 0.012
    grille_frame = 0.020
    inner_w = grille_w - 2.0 * grille_frame
    inner_h = grille_h - 2.0 * grille_frame

    grille.visual(
        Box((grille_frame, grille_t, grille_h)),
        origin=Origin(xyz=(grille_frame / 2.0, grille_t / 2.0, 0.0)),
        material=plastic,
        name="grille_left_frame",
    )
    grille.visual(
        Box((grille_frame, grille_t, grille_h)),
        origin=Origin(xyz=(grille_w - grille_frame / 2.0, grille_t / 2.0, 0.0)),
        material=plastic,
        name="grille_right_frame",
    )
    grille.visual(
        Box((grille_w - 2.0 * grille_frame, grille_t, grille_frame)),
        origin=Origin(xyz=(grille_w / 2.0, grille_t / 2.0, grille_h / 2.0 - grille_frame / 2.0)),
        material=plastic,
        name="grille_top_frame",
    )
    grille.visual(
        Box((grille_w - 2.0 * grille_frame, grille_t, grille_frame)),
        origin=Origin(xyz=(grille_w / 2.0, grille_t / 2.0, -grille_h / 2.0 + grille_frame / 2.0)),
        material=plastic,
        name="grille_bottom_frame",
    )

    vertical_bars = 6
    horizontal_bars = 6
    bar_w = 0.006
    for idx in range(1, vertical_bars + 1):
        x = grille_frame + idx * inner_w / (vertical_bars + 1)
        grille.visual(
            Box((bar_w, grille_t, inner_h)),
            origin=Origin(xyz=(x, grille_t / 2.0, 0.0)),
            material=plastic,
            name=f"vertical_bar_{idx}",
        )
    for idx in range(1, horizontal_bars + 1):
        z = -inner_h / 2.0 + idx * inner_h / (horizontal_bars + 1)
        grille.visual(
            Box((inner_w, grille_t, bar_w)),
            origin=Origin(xyz=(grille_w / 2.0, grille_t / 2.0, z)),
            material=plastic,
            name=f"horizontal_bar_{idx}",
        )

    grille.visual(
        Cylinder(radius=0.034, length=0.010),
        origin=Origin(xyz=(grille_w / 2.0, grille_t / 2.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="center_badge",
    )
    grille.visual(
        Box((0.012, 0.020, 0.060)),
        origin=Origin(xyz=(grille_w - 0.006, 0.010, 0.0)),
        material=dark_plastic,
        name="latch_tab",
    )

    model.articulation(
        "housing_to_front_grille",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=grille,
        origin=Origin(xyz=(-grille_w / 2.0, depth / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.75),
    )

    knob = model.part("speed_knob")
    knob.visual(
        Cylinder(radius=0.022, length=0.020),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_plastic,
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_plastic,
        name="knob_shaft",
    )
    knob.visual(
        Box((0.006, 0.007, 0.020)),
        origin=Origin(xyz=(0.018, 0.0, 0.012)),
        material=plastic,
        name="pointer_ridge",
    )

    model.articulation(
        "housing_to_speed_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=knob,
        origin=Origin(xyz=(outer_w / 2.0, control_boss_y, control_boss_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=12.0),
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
    housing = object_model.get_part("housing")
    rotor = object_model.get_part("rotor")
    grille = object_model.get_part("front_grille")
    knob = object_model.get_part("speed_knob")

    rotor_joint = object_model.get_articulation("housing_to_rotor")
    grille_joint = object_model.get_articulation("housing_to_front_grille")
    knob_joint = object_model.get_articulation("housing_to_speed_knob")

    ctx.check(
        "rotor articulation is continuous",
        rotor_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={rotor_joint.articulation_type}",
    )
    ctx.check(
        "front grille articulation is revolute",
        grille_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={grille_joint.articulation_type}",
    )
    ctx.check(
        "knob articulation is continuous",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type}",
    )

    ctx.expect_gap(
        grille,
        housing,
        axis="y",
        max_gap=0.003,
        max_penetration=0.0,
        positive_elem="grille_left_frame",
        negative_elem="front_left_frame",
        name="front grille sits just proud of housing",
    )
    ctx.expect_overlap(
        grille,
        housing,
        axes="xz",
        min_overlap=0.52,
        name="front grille covers the fan opening",
    )
    ctx.expect_origin_distance(
        rotor,
        housing,
        axes="xz",
        max_dist=0.001,
        name="rotor stays centered on the housing axis",
    )
    ctx.expect_gap(
        knob,
        housing,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="knob_body",
        negative_elem="control_boss",
        name="speed knob seats on the right-side control boss",
    )

    closed_latch = ctx.part_element_world_aabb(grille, elem="latch_tab")
    with ctx.pose({grille_joint: 1.40}):
        open_latch = ctx.part_element_world_aabb(grille, elem="latch_tab")
    ctx.check(
        "front grille swings forward on its side hinge",
        closed_latch is not None
        and open_latch is not None
        and open_latch[1][1] > closed_latch[1][1] + 0.12,
        details=f"closed={closed_latch}, open={open_latch}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
