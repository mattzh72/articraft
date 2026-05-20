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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_work_light_tower")

    dark_steel = model.material("dark_steel", rgba=(0.18, 0.18, 0.19, 1.0))
    black = model.material("black", rgba=(0.11, 0.11, 0.12, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.92, 0.76, 0.14, 1.0))
    lens_white = model.material("lens_white", rgba=(0.92, 0.94, 0.96, 0.95))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    base_section = model.part("base_section")
    base_section.visual(
        Box((0.12, 0.08, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_steel,
        name="base_shoe",
    )
    base_section.visual(
        Cylinder(radius=0.034, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=dark_steel,
        name="lower_mast",
    )
    base_section.visual(
        Cylinder(radius=0.074, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=dark_steel,
        name="base_ring",
    )

    outer_profile = [
        (0.030, 0.24),
        (0.030, 0.80),
        (0.039, 0.815),
        (0.039, 0.865),
    ]
    inner_profile = [
        (0.0245, 0.245),
        (0.0245, 0.805),
        (0.033, 0.820),
        (0.033, 0.860),
    ]
    outer_sleeve = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=48,
    )
    base_section.visual(
        mesh_from_geometry(outer_sleeve, "work_light_outer_sleeve"),
        material=black,
        name="outer_sleeve",
    )
    base_section.visual(
        Cylinder(radius=0.007, length=0.032),
        origin=Origin(
            xyz=(0.054, 0.0, 0.84),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=black,
        name="clamp_stem",
    )
    base_section.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(
            xyz=(0.078, 0.0, 0.84),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=rubber,
        name="clamp_knob",
    )

    hinge_y = 0.088
    hinge_plate_x = 0.020
    for side_name, side in (("left", 1.0), ("right", -1.0)):
        base_section.visual(
            Box((0.012, 0.040, 0.048)),
            origin=Origin(xyz=(hinge_plate_x, side * hinge_y, 0.145)),
            material=dark_steel,
            name=f"{side_name}_hinge_outer",
        )
        base_section.visual(
            Box((0.012, 0.040, 0.048)),
            origin=Origin(xyz=(-hinge_plate_x, side * hinge_y, 0.145)),
            material=dark_steel,
            name=f"{side_name}_hinge_inner",
        )

    base_section.inertial = Inertial.from_geometry(
        Box((0.22, 0.90, 0.90)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
    )

    def add_leg(name: str, side: float) -> None:
        leg = model.part(name)
        leg_path = tube_from_spline_points(
            [
                (0.0, 0.0, 0.0),
                (0.0, side * 0.10, -0.022),
                (0.0, side * 0.24, -0.080),
                (0.0, side * 0.36, -0.128),
            ],
            radius=0.012,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        )
        leg.visual(
            Cylinder(radius=0.015, length=0.028),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name="hinge_barrel",
        )
        leg.visual(
            mesh_from_geometry(leg_path, f"{name}_tube"),
            material=black,
            name="leg_tube",
        )
        leg.visual(
            Cylinder(radius=0.038, length=0.018),
            origin=Origin(xyz=(0.0, side * 0.36, -0.137)),
            material=rubber,
            name="foot_pad",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.06, 0.42, 0.18)),
            mass=0.9,
            origin=Origin(xyz=(0.0, side * 0.18, -0.06)),
        )

    add_leg("left_leg", 1.0)
    add_leg("right_leg", -1.0)

    model.articulation(
        "base_to_left_leg",
        ArticulationType.REVOLUTE,
        parent=base_section,
        child="left_leg",
        origin=Origin(xyz=(0.0, hinge_y, 0.145)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=-0.18,
            upper=1.05,
        ),
    )
    model.articulation(
        "base_to_right_leg",
        ArticulationType.REVOLUTE,
        parent=base_section,
        child="right_leg",
        origin=Origin(xyz=(0.0, -hinge_y, 0.145)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=-0.18,
            upper=1.05,
        ),
    )

    inner_mast = model.part("inner_mast")
    inner_mast.visual(
        Cylinder(radius=0.0205, length=1.08),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=dark_steel,
        name="inner_tube",
    )
    inner_mast.visual(
        Cylinder(radius=0.041, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=black,
        name="lower_stop_collar",
    )
    inner_mast.visual(
        Cylinder(radius=0.028, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.705)),
        material=black,
        name="top_stop_collar",
    )
    inner_mast.visual(
        Box((0.020, 0.086, 0.070)),
        origin=Origin(xyz=(-0.030, 0.0, 0.755)),
        material=black,
        name="top_bracket_bridge",
    )
    inner_mast.visual(
        Box((0.022, 0.014, 0.090)),
        origin=Origin(xyz=(-0.011, 0.040, 0.755)),
        material=black,
        name="top_bracket_left",
    )
    inner_mast.visual(
        Box((0.022, 0.014, 0.090)),
        origin=Origin(xyz=(-0.011, -0.040, 0.755)),
        material=black,
        name="top_bracket_right",
    )
    inner_mast.inertial = Inertial.from_geometry(
        Box((0.09, 0.12, 1.18)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
    )

    model.articulation(
        "base_to_inner_mast",
        ArticulationType.PRISMATIC,
        parent=base_section,
        child=inner_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.84)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.18,
            lower=0.0,
            upper=0.24,
        ),
    )

    lamp_bar = model.part("lamp_bar")
    lamp_bar.visual(
        Cylinder(radius=0.016, length=0.062),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion",
    )
    lamp_bar.visual(
        Box((0.060, 0.060, 0.050)),
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=black,
        name="center_hub",
    )
    lamp_bar.visual(
        Box((0.060, 0.520, 0.030)),
        origin=Origin(xyz=(0.075, 0.0, 0.005)),
        material=black,
        name="crossbar",
    )
    for side_name, side in (("left", 1.0), ("right", -1.0)):
        lamp_bar.visual(
            Box((0.042, 0.020, 0.100)),
            origin=Origin(xyz=(0.095, side * 0.150, -0.040)),
            material=black,
            name=f"{side_name}_head_bracket",
        )
        lamp_bar.visual(
            Box((0.120, 0.040, 0.160)),
            origin=Origin(xyz=(0.145, side * 0.150, -0.040)),
            material=safety_yellow,
            name=f"{side_name}_head_housing",
        )
        lamp_bar.visual(
            Box((0.008, 0.036, 0.142)),
            origin=Origin(xyz=(0.209, side * 0.150, -0.040)),
            material=lens_white,
            name=f"{side_name}_lens",
        )
        lamp_bar.visual(
            Box((0.028, 0.052, 0.144)),
            origin=Origin(xyz=(0.078, side * 0.150, -0.040)),
            material=black,
            name=f"{side_name}_heat_sink",
        )
    lamp_bar.inertial = Inertial.from_geometry(
        Box((0.26, 0.56, 0.22)),
        mass=2.3,
        origin=Origin(xyz=(0.11, 0.0, -0.02)),
    )

    model.articulation(
        "mast_to_lamp_bar",
        ArticulationType.REVOLUTE,
        parent=inner_mast,
        child=lamp_bar,
        origin=Origin(xyz=(0.0, 0.0, 0.755)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=-0.50,
            upper=1.10,
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
    base_section = object_model.get_part("base_section")
    left_leg = object_model.get_part("left_leg")
    right_leg = object_model.get_part("right_leg")
    inner_mast = object_model.get_part("inner_mast")
    lamp_bar = object_model.get_part("lamp_bar")

    left_leg_hinge = object_model.get_articulation("base_to_left_leg")
    right_leg_hinge = object_model.get_articulation("base_to_right_leg")
    mast_slide = object_model.get_articulation("base_to_inner_mast")
    lamp_tilt = object_model.get_articulation("mast_to_lamp_bar")

    ctx.expect_within(
        inner_mast,
        base_section,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="inner mast stays centered in the base sleeve at rest",
    )
    ctx.expect_overlap(
        inner_mast,
        base_section,
        axes="z",
        elem_a="inner_tube",
        elem_b="outer_sleeve",
        min_overlap=0.34,
        name="inner mast has deep insertion at rest",
    )
    ctx.expect_overlap(
        lamp_bar,
        inner_mast,
        axes="y",
        elem_a="center_hub",
        elem_b="top_bracket_bridge",
        min_overlap=0.04,
        name="lamp bar hinge stays centered on the mast bracket",
    )

    left_rest_aabb = ctx.part_element_world_aabb(left_leg, elem="foot_pad")
    right_rest_aabb = ctx.part_element_world_aabb(right_leg, elem="foot_pad")
    ctx.check(
        "legs form a wide deployed footprint",
        left_rest_aabb is not None
        and right_rest_aabb is not None
        and left_rest_aabb[0][1] > 0.40
        and right_rest_aabb[1][1] < -0.40
        and left_rest_aabb[0][2] < 0.02
        and right_rest_aabb[0][2] < 0.02,
        details=f"left={left_rest_aabb}, right={right_rest_aabb}",
    )

    with ctx.pose({mast_slide: 0.24}):
        ctx.expect_within(
            inner_mast,
            base_section,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="outer_sleeve",
            margin=0.002,
            name="extended mast stays centered in the base sleeve",
        )
        ctx.expect_overlap(
            inner_mast,
            base_section,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_sleeve",
            min_overlap=0.14,
            name="extended mast retains insertion in the sleeve",
        )

    mast_rest = ctx.part_world_position(inner_mast)
    with ctx.pose({mast_slide: 0.24}):
        mast_extended = ctx.part_world_position(inner_mast)
    ctx.check(
        "mast extends upward",
        mast_rest is not None
        and mast_extended is not None
        and mast_extended[2] > mast_rest[2] + 0.20,
        details=f"rest={mast_rest}, extended={mast_extended}",
    )

    with ctx.pose({left_leg_hinge: 1.0, right_leg_hinge: 1.0}):
        left_folded_aabb = ctx.part_element_world_aabb(left_leg, elem="foot_pad")
        right_folded_aabb = ctx.part_element_world_aabb(right_leg, elem="foot_pad")
    ctx.check(
        "positive leg motion folds both feet upward",
        left_rest_aabb is not None
        and right_rest_aabb is not None
        and left_folded_aabb is not None
        and right_folded_aabb is not None
        and left_folded_aabb[0][2] > left_rest_aabb[0][2] + 0.18
        and right_folded_aabb[0][2] > right_rest_aabb[0][2] + 0.18,
        details=(
            f"left_rest={left_rest_aabb}, left_folded={left_folded_aabb}, "
            f"right_rest={right_rest_aabb}, right_folded={right_folded_aabb}"
        ),
    )

    lamp_rest_aabb = ctx.part_world_aabb(lamp_bar)
    with ctx.pose({lamp_tilt: 0.85}):
        lamp_raised_aabb = ctx.part_world_aabb(lamp_bar)
    ctx.check(
        "positive lamp tilt raises the floodlight bar",
        lamp_rest_aabb is not None
        and lamp_raised_aabb is not None
        and lamp_raised_aabb[1][2] > lamp_rest_aabb[1][2] + 0.12,
        details=f"rest={lamp_rest_aabb}, raised={lamp_raised_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
