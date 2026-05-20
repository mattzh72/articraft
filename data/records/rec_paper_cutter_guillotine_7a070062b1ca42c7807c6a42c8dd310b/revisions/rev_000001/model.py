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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="large_format_guillotine_cutter")

    table_laminate = model.material("table_laminate", rgba=(0.90, 0.92, 0.86, 1.0))
    frame_gray = model.material("frame_gray", rgba=(0.44, 0.46, 0.50, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.19, 0.21, 0.23, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.77, 0.80, 0.83, 1.0))
    grip_black = model.material("grip_black", rgba=(0.08, 0.08, 0.09, 1.0))
    anodized_black = model.material("anodized_black", rgba=(0.15, 0.16, 0.18, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    table_w = 0.78
    table_d = 0.56
    surface_z = 0.048
    slab_t = 0.028

    base = model.part("base")
    base.visual(
        Box((table_w, table_d, slab_t)),
        origin=Origin(xyz=(0.0, 0.0, surface_z - slab_t * 0.5)),
        material=table_laminate,
        name="cutting_surface",
    )
    base.visual(
        Box((table_w, 0.032, 0.024)),
        origin=Origin(xyz=(0.0, table_d * 0.5 - 0.016, 0.012)),
        material=frame_gray,
        name="front_skirt",
    )
    base.visual(
        Box((table_w, 0.032, 0.024)),
        origin=Origin(xyz=(0.0, -table_d * 0.5 + 0.016, 0.012)),
        material=frame_gray,
        name="rear_skirt",
    )
    base.visual(
        Box((0.032, table_d - 0.048, 0.024)),
        origin=Origin(xyz=(table_w * 0.5 - 0.016, 0.0, 0.012)),
        material=frame_gray,
        name="right_skirt",
    )
    base.visual(
        Box((0.032, table_d - 0.048, 0.024)),
        origin=Origin(xyz=(-table_w * 0.5 + 0.016, 0.0, 0.012)),
        material=frame_gray,
        name="left_skirt",
    )
    base.visual(
        Box((0.70, 0.018, 0.024)),
        origin=Origin(xyz=(-0.010, -0.225, surface_z + 0.012)),
        material=anodized_black,
        name="rear_fence",
    )
    base.visual(
        Box((0.026, 0.470, 0.004)),
        origin=Origin(xyz=(0.337, 0.030, surface_z + 0.002)),
        material=blade_steel,
        name="cut_track",
    )
    base.visual(
        Box((0.660, 0.034, 0.004)),
        origin=Origin(xyz=(0.0, 0.215, surface_z + 0.002)),
        material=blade_steel,
        name="clamp_pad",
    )
    base.visual(
        Box((0.028, 0.118, 0.058)),
        origin=Origin(xyz=(0.376, -0.215, 0.077)),
        material=dark_steel,
        name="arm_pedestal",
    )
    base.visual(
        Box((0.028, 0.020, 0.018)),
        origin=Origin(xyz=(0.352, -0.232, 0.084)),
        material=dark_steel,
        name="arm_hinge_cheek",
    )
    base.visual(
        Box((0.026, 0.034, 0.046)),
        origin=Origin(xyz=(-0.367, 0.215, 0.071)),
        material=dark_steel,
        name="left_clamp_support",
    )
    base.visual(
        Box((0.026, 0.034, 0.046)),
        origin=Origin(xyz=(0.367, 0.215, 0.071)),
        material=dark_steel,
        name="right_clamp_support",
    )
    for index, (x_pos, y_pos) in enumerate(
        (
            (-0.325, -0.220),
            (0.325, -0.220),
            (-0.325, 0.220),
            (0.325, 0.220),
        )
    ):
        base.visual(
            Cylinder(radius=0.024, length=0.012),
            origin=Origin(
                xyz=(
                    -0.352 if x_pos < 0.0 else 0.352,
                    -0.248 if y_pos < 0.0 else 0.248,
                    0.006,
                )
            ),
            material=rubber,
            name=f"foot_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.80, 0.60, 0.10)),
        mass=15.0,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
    )

    cutting_arm = model.part("cutting_arm")
    cutting_arm.visual(
        Box((0.038, 0.110, 0.032)),
        origin=Origin(xyz=(-0.022, 0.060, -0.010)),
        material=dark_steel,
        name="hinge_block",
    )
    cutting_arm.visual(
        Box((0.034, 0.560, 0.020)),
        origin=Origin(xyz=(-0.023, 0.320, -0.009)),
        material=dark_steel,
        name="arm_spine",
    )
    cutting_arm.visual(
        Box((0.010, 0.500, 0.008)),
        origin=Origin(xyz=(-0.038, 0.300, -0.021)),
        material=blade_steel,
        name="blade_rail",
    )
    cutting_arm.visual(
        Box((0.028, 0.095, 0.048)),
        origin=Origin(xyz=(-0.021, 0.505, 0.008)),
        material=dark_steel,
        name="handle_stem",
    )
    cutting_arm.visual(
        Cylinder(radius=0.018, length=0.094),
        origin=Origin(xyz=(-0.021, 0.557, 0.035), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="handle_grip",
    )
    cutting_arm.inertial = Inertial.from_geometry(
        Box((0.08, 0.62, 0.10)),
        mass=2.8,
        origin=Origin(xyz=(-0.022, 0.300, 0.000)),
    )

    clamp_bar = model.part("clamp_bar")
    clamp_bar.visual(
        Cylinder(radius=0.010, length=0.620),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized_black,
        name="shaft",
    )
    clamp_bar.visual(
        Box((0.600, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, 0.026, -0.020)),
        material=dark_steel,
        name="pressure_bar",
    )
    clamp_bar.visual(
        Box((0.020, 0.036, 0.028)),
        origin=Origin(xyz=(-0.292, 0.016, -0.012)),
        material=dark_steel,
        name="left_ear",
    )
    clamp_bar.visual(
        Box((0.020, 0.036, 0.028)),
        origin=Origin(xyz=(0.292, 0.016, -0.012)),
        material=dark_steel,
        name="right_ear",
    )
    clamp_bar.visual(
        Box((0.620, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, 0.008, -0.010)),
        material=anodized_black,
        name="rear_bridge",
    )
    clamp_bar.visual(
        Box((0.044, 0.012, 0.012)),
        origin=Origin(xyz=(-0.332, 0.000, 0.006)),
        material=anodized_black,
        name="left_pivot_lug",
    )
    clamp_bar.visual(
        Box((0.044, 0.012, 0.012)),
        origin=Origin(xyz=(0.332, 0.000, 0.006)),
        material=anodized_black,
        name="right_pivot_lug",
    )
    clamp_bar.inertial = Inertial.from_geometry(
        Box((0.64, 0.06, 0.06)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.006, -0.010)),
    )

    arm_upper = math.radians(74.0)
    clamp_upper = math.radians(42.0)

    model.articulation(
        "base_to_cutting_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=cutting_arm,
        origin=Origin(xyz=(0.361, -0.215, 0.084)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=1.6,
            lower=0.0,
            upper=arm_upper,
        ),
    )
    model.articulation(
        "base_to_clamp_bar",
        ArticulationType.REVOLUTE,
        parent=base,
        child=clamp_bar,
        origin=Origin(xyz=(0.0, 0.215, 0.086)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=clamp_upper,
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
    base = object_model.get_part("base")
    cutting_arm = object_model.get_part("cutting_arm")
    clamp_bar = object_model.get_part("clamp_bar")
    arm_joint = object_model.get_articulation("base_to_cutting_arm")
    clamp_joint = object_model.get_articulation("base_to_clamp_bar")

    ctx.check("base part present", base.name == "base")
    ctx.check("cutting arm part present", cutting_arm.name == "cutting_arm")
    ctx.check("clamp bar part present", clamp_bar.name == "clamp_bar")

    with ctx.pose({arm_joint: 0.0, clamp_joint: 0.0}):
        ctx.expect_gap(
            cutting_arm,
            base,
            axis="z",
            positive_elem="blade_rail",
            negative_elem="cut_track",
            min_gap=0.006,
            max_gap=0.024,
            name="blade rail rides just above the cut track",
        )
        ctx.expect_gap(
            clamp_bar,
            base,
            axis="z",
            positive_elem="pressure_bar",
            negative_elem="clamp_pad",
            min_gap=0.002,
            max_gap=0.012,
            name="clamp bar sits just above the clamp pad",
        )
        ctx.expect_gap(
            cutting_arm,
            clamp_bar,
            axis="x",
            positive_elem="arm_spine",
            negative_elem="shaft",
            min_gap=0.008,
            name="cutting arm stays laterally clear of the clamp shaft",
        )
        ctx.expect_within(
            clamp_bar,
            base,
            axes="x",
            inner_elem="pressure_bar",
            outer_elem="cutting_surface",
            margin=0.050,
            name="clamp bar spans the table without overhanging the sides",
        )

    arm_rest_aabb = ctx.part_world_aabb(cutting_arm)
    with ctx.pose({arm_joint: arm_joint.motion_limits.upper}):
        arm_open_aabb = ctx.part_world_aabb(cutting_arm)
    ctx.check(
        "cutting arm raises upward",
        arm_rest_aabb is not None
        and arm_open_aabb is not None
        and arm_open_aabb[1][2] > arm_rest_aabb[1][2] + 0.20,
        details=f"rest={arm_rest_aabb}, open={arm_open_aabb}",
    )

    with ctx.pose({clamp_joint: clamp_joint.motion_limits.upper}):
        ctx.expect_gap(
            clamp_bar,
            base,
            axis="z",
            positive_elem="pressure_bar",
            negative_elem="clamp_pad",
            min_gap=0.018,
            name="clamp bar lifts well clear of the clamp pad",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
