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
    rounded_rect_profile,
    section_loft,
)


def _xz_section(width: float, height: float, radius: float, y: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, z in rounded_rect_profile(width, height, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clip_on_metronome")

    housing_plastic = model.material("housing_plastic", rgba=(0.18, 0.19, 0.21, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.08, 0.09, 0.10, 1.0))
    scale_tint = model.material("scale_tint", rgba=(0.22, 0.24, 0.26, 0.95))
    metal = model.material("metal", rgba=(0.72, 0.73, 0.75, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.78, 0.79, 0.81, 1.0))
    rubber = model.material("rubber", rgba=(0.13, 0.14, 0.15, 1.0))
    accent = model.material("accent", rgba=(0.88, 0.29, 0.20, 1.0))

    housing = model.part("housing")

    housing_shell = section_loft(
        [
            _xz_section(0.054, 0.058, 0.0055, -0.010),
            _xz_section(0.055, 0.060, 0.0065, 0.000),
            _xz_section(0.056, 0.060, 0.0060, 0.010),
        ]
    )
    housing.visual(
        mesh_from_geometry(housing_shell, "metronome_housing_shell"),
        material=housing_plastic,
        name="housing_shell",
    )
    housing.visual(
        Box((0.028, 0.0025, 0.040)),
        origin=Origin(xyz=(0.0, 0.0110, 0.003)),
        material=panel_dark,
        name="face_panel",
    )
    housing.visual(
        Box((0.004, 0.0018, 0.033)),
        origin=Origin(xyz=(0.0, 0.0122, 0.010)),
        material=scale_tint,
        name="tempo_window",
    )
    housing.visual(
        Box((0.012, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=housing_plastic,
        name="pendulum_pedestal",
    )
    housing.visual(
        Box((0.010, 0.003, 0.014)),
        origin=Origin(xyz=(0.0, 0.0045, 0.036)),
        material=housing_plastic,
        name="pivot_cheek_front",
    )
    housing.visual(
        Box((0.010, 0.003, 0.014)),
        origin=Origin(xyz=(0.0, -0.0045, 0.036)),
        material=housing_plastic,
        name="pivot_cheek_rear",
    )
    housing.visual(
        Cylinder(radius=0.0045, length=0.004),
        origin=Origin(xyz=(0.0275, 0.0, -0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_plastic,
        name="key_boss",
    )
    housing.visual(
        Box((0.044, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, -0.0140, -0.025)),
        material=housing_plastic,
        name="fixed_jaw_body",
    )
    housing.visual(
        Box((0.038, 0.002, 0.008)),
        origin=Origin(xyz=(0.0, -0.0170, -0.025)),
        material=rubber,
        name="fixed_jaw_pad",
    )
    housing.visual(
        Box((0.006, 0.006, 0.012)),
        origin=Origin(xyz=(-0.021, -0.012, 0.020)),
        material=housing_plastic,
        name="clip_hinge_left",
    )
    housing.visual(
        Box((0.006, 0.006, 0.012)),
        origin=Origin(xyz=(0.021, -0.012, 0.020)),
        material=housing_plastic,
        name="clip_hinge_right",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.060, 0.026, 0.064)),
        mass=0.11,
        origin=Origin(xyz=(0.0, -0.002, 0.0)),
    )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Cylinder(radius=0.0025, length=0.007),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="pivot_hub",
    )
    pendulum.visual(
        Cylinder(radius=0.0014, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.0525)),
        material=brushed_steel,
        name="rod",
    )
    pendulum.visual(
        Box((0.006, 0.002, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.091)),
        material=accent,
        name="rod_tip",
    )
    pendulum.inertial = Inertial.from_geometry(
        Box((0.010, 0.010, 0.110)),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    sliding_weight = model.part("sliding_weight")
    weight_shell = LatheGeometry.from_shell_profiles(
        [
            (0.0054, -0.005),
            (0.0066, -0.0035),
            (0.0066, 0.0035),
            (0.0054, 0.005),
        ],
        [
            (0.0014, -0.005),
            (0.0014, 0.005),
        ],
        segments=40,
    )
    sliding_weight.visual(
        mesh_from_geometry(weight_shell, "metronome_weight_shell"),
        material=metal,
        name="weight_shell",
    )
    sliding_weight.visual(
        Box((0.007, 0.004, 0.006)),
        origin=Origin(xyz=(0.0092, 0.0, 0.0)),
        material=metal,
        name="weight_thumb",
    )
    sliding_weight.inertial = Inertial.from_geometry(
        Box((0.020, 0.014, 0.012)),
        mass=0.012,
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
    )

    clip_arm = model.part("clip_arm")
    clip_arm.visual(
        Cylinder(radius=0.0028, length=0.036),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_plastic,
        name="hinge_barrel",
    )
    clip_arm.visual(
        Box((0.040, 0.005, 0.046)),
        origin=Origin(xyz=(0.0, -0.0045, -0.025)),
        material=housing_plastic,
        name="arm_web",
    )
    clip_arm.visual(
        Box((0.020, 0.005, 0.008)),
        origin=Origin(xyz=(0.0, -0.0025, -0.004)),
        material=housing_plastic,
        name="hinge_bridge",
    )
    clip_arm.visual(
        Box((0.046, 0.007, 0.008)),
        origin=Origin(xyz=(0.0, -0.0085, -0.049)),
        material=housing_plastic,
        name="jaw_body",
    )
    clip_arm.visual(
        Box((0.038, 0.002, 0.007)),
        origin=Origin(xyz=(0.0, -0.0075, -0.049)),
        material=rubber,
        name="jaw_pad",
    )
    clip_arm.visual(
        Box((0.030, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, 0.0010, -0.040)),
        material=housing_plastic,
        name="finger_tab",
    )
    clip_arm.inertial = Inertial.from_geometry(
        Box((0.050, 0.018, 0.055)),
        mass=0.03,
        origin=Origin(xyz=(0.0, -0.005, -0.028)),
    )

    winding_key = model.part("winding_key")
    winding_key.visual(
        Cylinder(radius=0.0023, length=0.005),
        origin=Origin(xyz=(0.0025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="key_stem",
    )
    winding_key.visual(
        Cylinder(radius=0.0044, length=0.004),
        origin=Origin(xyz=(0.0070, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="key_hub",
    )
    winding_key.visual(
        Box((0.003, 0.016, 0.004)),
        origin=Origin(xyz=(0.0105, 0.0, 0.0)),
        material=metal,
        name="key_wing",
    )
    winding_key.inertial = Inertial.from_geometry(
        Box((0.016, 0.018, 0.008)),
        mass=0.004,
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
    )

    model.articulation(
        "housing_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=4.0,
            lower=-0.65,
            upper=0.65,
        ),
    )
    model.articulation(
        "pendulum_to_sliding_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=sliding_weight,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=0.08,
            lower=0.0,
            upper=0.045,
        ),
    )
    model.articulation(
        "housing_to_clip_arm",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=clip_arm,
        origin=Origin(xyz=(0.0, -0.0115, 0.020)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=2.0,
            lower=0.0,
            upper=0.90,
        ),
    )
    model.articulation(
        "housing_to_winding_key",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=winding_key,
        origin=Origin(xyz=(0.028, 0.0, -0.005)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.08, velocity=12.0),
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
    pendulum = object_model.get_part("pendulum")
    sliding_weight = object_model.get_part("sliding_weight")
    clip_arm = object_model.get_part("clip_arm")
    winding_key = object_model.get_part("winding_key")

    pendulum_joint = object_model.get_articulation("housing_to_pendulum")
    weight_joint = object_model.get_articulation("pendulum_to_sliding_weight")
    clip_joint = object_model.get_articulation("housing_to_clip_arm")
    key_joint = object_model.get_articulation("housing_to_winding_key")

    ctx.check(
        "all major parts exist",
        all(part is not None for part in (housing, pendulum, sliding_weight, clip_arm, winding_key)),
        details="Expected housing, pendulum, sliding weight, clip arm, and winding key parts.",
    )
    ctx.check(
        "pendulum joint is top swing hinge",
        pendulum_joint.axis == (0.0, 1.0, 0.0)
        and pendulum_joint.motion_limits is not None
        and pendulum_joint.motion_limits.upper is not None
        and pendulum_joint.motion_limits.upper > 0.5,
        details=f"axis={pendulum_joint.axis}, limits={pendulum_joint.motion_limits}",
    )
    ctx.check(
        "sliding weight joint runs along rod",
        weight_joint.axis == (0.0, 0.0, 1.0)
        and weight_joint.motion_limits is not None
        and weight_joint.motion_limits.upper is not None
        and weight_joint.motion_limits.upper >= 0.04,
        details=f"axis={weight_joint.axis}, limits={weight_joint.motion_limits}",
    )
    ctx.check(
        "clip arm opens rearward from housing back",
        clip_joint.axis == (-1.0, 0.0, 0.0)
        and clip_joint.motion_limits is not None
        and clip_joint.motion_limits.upper is not None
        and clip_joint.motion_limits.upper >= 0.8,
        details=f"axis={clip_joint.axis}, limits={clip_joint.motion_limits}",
    )
    ctx.check(
        "winding key rotates continuously on side axis",
        key_joint.axis == (1.0, 0.0, 0.0)
        and key_joint.motion_limits is not None
        and key_joint.motion_limits.lower is None
        and key_joint.motion_limits.upper is None,
        details=f"axis={key_joint.axis}, limits={key_joint.motion_limits}",
    )

    ctx.expect_gap(
        housing,
        clip_arm,
        axis="y",
        positive_elem="fixed_jaw_pad",
        negative_elem="jaw_pad",
        max_gap=0.0015,
        max_penetration=0.0001,
        name="clip closes against fixed jaw with only a thin resting gap",
    )

    pendulum_tip_rest = ctx.part_element_world_aabb(pendulum, elem="rod_tip")
    with ctx.pose({pendulum_joint: 0.45}):
        pendulum_tip_swung = ctx.part_element_world_aabb(pendulum, elem="rod_tip")
    ctx.check(
        "pendulum tip swings sideways when articulated",
        pendulum_tip_rest is not None
        and pendulum_tip_swung is not None
        and pendulum_tip_swung[0][0] > pendulum_tip_rest[0][0] + 0.03,
        details=f"rest={pendulum_tip_rest}, swung={pendulum_tip_swung}",
    )

    weight_rest = None
    weight_high = None
    with ctx.pose({pendulum_joint: 0.0, weight_joint: 0.0}):
        weight_rest = ctx.part_world_position(sliding_weight)
    with ctx.pose({pendulum_joint: 0.0, weight_joint: 0.045}):
        weight_high = ctx.part_world_position(sliding_weight)
    ctx.check(
        "sliding weight travels upward along the pendulum rod",
        weight_rest is not None and weight_high is not None and weight_high[2] > weight_rest[2] + 0.035,
        details=f"rest={weight_rest}, high={weight_high}",
    )

    clip_rest = ctx.part_element_world_aabb(clip_arm, elem="jaw_pad")
    with ctx.pose({clip_joint: 0.75}):
        clip_open = ctx.part_element_world_aabb(clip_arm, elem="jaw_pad")
    ctx.check(
        "clip jaw swings farther behind the housing when opened",
        clip_rest is not None
        and clip_open is not None
        and clip_open[0][1] < clip_rest[0][1] - 0.01,
        details=f"rest={clip_rest}, open={clip_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
