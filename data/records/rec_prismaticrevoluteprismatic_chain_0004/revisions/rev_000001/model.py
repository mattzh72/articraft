from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
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
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prismatic_revolute_prismatic_chain", assets=ASSETS)

    painted_steel = model.material("painted_steel", rgba=(0.26, 0.29, 0.33, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.62, 0.65, 0.69, 1.0))
    anodized_black = model.material("anodized_black", rgba=(0.12, 0.13, 0.15, 1.0))
    guide_metal = model.material("guide_metal", rgba=(0.54, 0.56, 0.60, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.26, 0.06, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=painted_steel,
        name="base_plate",
    )
    base.visual(
        Box((0.23, 0.020, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=guide_metal,
        name="rail",
    )
    base.visual(
        Box((0.008, 0.028, 0.018)),
        origin=Origin(xyz=(-0.111, 0.0, 0.017)),
        material=machined_steel,
        name="rear_stop",
    )
    base.visual(
        Box((0.008, 0.028, 0.018)),
        origin=Origin(xyz=(0.111, 0.0, 0.017)),
        material=machined_steel,
        name="front_stop",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.26, 0.06, 0.028)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    root_slide = model.part("root_slide")
    root_slide.visual(
        Box((0.06, 0.008, 0.024)),
        origin=Origin(xyz=(0.0, 0.014, 0.020)),
        material=machined_steel,
        name="left_cheek",
    )
    root_slide.visual(
        Box((0.06, 0.008, 0.024)),
        origin=Origin(xyz=(0.0, -0.014, 0.020)),
        material=machined_steel,
        name="right_cheek",
    )
    root_slide.visual(
        Box((0.06, 0.036, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=anodized_black,
        name="carriage_top",
    )
    root_slide.visual(
        Box((0.028, 0.006, 0.070)),
        origin=Origin(xyz=(0.010, 0.010, 0.073)),
        material=anodized_black,
        name="left_fork",
    )
    root_slide.visual(
        Box((0.028, 0.006, 0.070)),
        origin=Origin(xyz=(0.010, -0.010, 0.073)),
        material=anodized_black,
        name="right_fork",
    )
    root_slide.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.020, 0.010, 0.100), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="left_barrel",
    )
    root_slide.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.020, -0.010, 0.100), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="right_barrel",
    )
    root_slide.inertial = Inertial.from_geometry(
        Box((0.07, 0.04, 0.115)),
        mass=1.2,
        origin=Origin(xyz=(0.005, 0.0, 0.0575)),
    )

    pivot_arm = model.part("pivot_arm")
    pivot_arm.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="center_barrel",
    )
    pivot_arm.visual(
        Box((0.010, 0.020, 0.018)),
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material=anodized_black,
        name="hinge_hub",
    )
    pivot_arm.visual(
        Box((0.026, 0.003, 0.014)),
        origin=Origin(xyz=(0.017, 0.0105, 0.0)),
        material=anodized_black,
        name="left_link_strut",
    )
    pivot_arm.visual(
        Box((0.026, 0.003, 0.014)),
        origin=Origin(xyz=(0.017, -0.0105, 0.0)),
        material=anodized_black,
        name="right_link_strut",
    )
    pivot_arm.visual(
        Box((0.13, 0.028, 0.003)),
        origin=Origin(xyz=(0.095, 0.0, 0.0095)),
        material=painted_steel,
        name="housing_top",
    )
    pivot_arm.visual(
        Box((0.13, 0.028, 0.003)),
        origin=Origin(xyz=(0.095, 0.0, -0.0095)),
        material=painted_steel,
        name="housing_bottom",
    )
    pivot_arm.visual(
        Box((0.13, 0.003, 0.016)),
        origin=Origin(xyz=(0.095, 0.0125, 0.0)),
        material=painted_steel,
        name="housing_left",
    )
    pivot_arm.visual(
        Box((0.13, 0.003, 0.016)),
        origin=Origin(xyz=(0.095, -0.0125, 0.0)),
        material=painted_steel,
        name="housing_right",
    )
    pivot_arm.inertial = Inertial.from_geometry(
        Box((0.16, 0.03, 0.026)),
        mass=0.9,
        origin=Origin(xyz=(0.080, 0.0, 0.0)),
    )

    distal_stage = model.part("distal_stage")
    distal_stage.visual(
        Box((0.170, 0.014, 0.010)),
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
        material=guide_metal,
        name="stage_bar",
    )
    distal_stage.visual(
        Box((0.120, 0.004, 0.012)),
        origin=Origin(xyz=(0.070, 0.009, 0.0)),
        material=machined_steel,
        name="left_guide_shoe",
    )
    distal_stage.visual(
        Box((0.120, 0.004, 0.012)),
        origin=Origin(xyz=(0.070, -0.009, 0.0)),
        material=machined_steel,
        name="right_guide_shoe",
    )
    distal_stage.visual(
        Box((0.012, 0.030, 0.024)),
        origin=Origin(xyz=(0.176, 0.0, 0.0)),
        material=machined_steel,
        name="tool_plate",
    )
    distal_stage.inertial = Inertial.from_geometry(
        Box((0.182, 0.03, 0.024)),
        mass=0.7,
        origin=Origin(xyz=(0.091, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_root_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=root_slide,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.30,
            lower=-0.075,
            upper=0.075,
        ),
    )
    model.articulation(
        "root_slide_to_pivot_arm",
        ArticulationType.REVOLUTE,
        parent=root_slide,
        child=pivot_arm,
        origin=Origin(xyz=(0.020, 0.0, 0.100)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=2.0,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "pivot_arm_to_distal_stage",
        ArticulationType.PRISMATIC,
        parent=pivot_arm,
        child=distal_stage,
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.20,
            lower=0.0,
            upper=0.080,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)

    base = object_model.get_part("base")
    root_slide = object_model.get_part("root_slide")
    pivot_arm = object_model.get_part("pivot_arm")
    distal_stage = object_model.get_part("distal_stage")

    root_joint = object_model.get_articulation("base_to_root_slide")
    pivot_joint = object_model.get_articulation("root_slide_to_pivot_arm")
    distal_joint = object_model.get_articulation("pivot_arm_to_distal_stage")

    rail = base.get_visual("rail")
    left_cheek = root_slide.get_visual("left_cheek")
    right_cheek = root_slide.get_visual("right_cheek")
    carriage_top = root_slide.get_visual("carriage_top")
    left_barrel = root_slide.get_visual("left_barrel")
    right_barrel = root_slide.get_visual("right_barrel")
    center_barrel = pivot_arm.get_visual("center_barrel")
    housing_top = pivot_arm.get_visual("housing_top")
    housing_left = pivot_arm.get_visual("housing_left")
    housing_right = pivot_arm.get_visual("housing_right")
    stage_bar = distal_stage.get_visual("stage_bar")
    left_guide_shoe = distal_stage.get_visual("left_guide_shoe")
    right_guide_shoe = distal_stage.get_visual("right_guide_shoe")
    tool_plate = distal_stage.get_visual("tool_plate")

    def aabb_center(part, elem):
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        lower, upper = bounds
        return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.check(
        "all parts present",
        all(part is not None for part in (base, root_slide, pivot_arm, distal_stage)),
        "expected four connected parts in the chain",
    )
    ctx.check(
        "joint travel ranges",
        (
            abs(root_joint.motion_limits.lower + 0.075) < 1e-9
            and abs(root_joint.motion_limits.upper - 0.075) < 1e-9
            and abs(pivot_joint.motion_limits.lower + math.pi / 2.0) < 1e-9
            and abs(pivot_joint.motion_limits.upper - math.pi / 2.0) < 1e-9
            and abs(distal_joint.motion_limits.lower - 0.0) < 1e-9
            and abs(distal_joint.motion_limits.upper - 0.080) < 1e-9
        ),
        "expected 150 mm root slide, +/-90 deg pivot, and 80 mm distal slide travel",
    )

    ctx.expect_contact(
        root_slide,
        base,
        elem_a=left_cheek,
        elem_b=rail,
        name="left carriage cheek bears on rail",
    )
    ctx.expect_contact(
        root_slide,
        base,
        elem_a=right_cheek,
        elem_b=rail,
        name="right carriage cheek bears on rail",
    )
    ctx.expect_overlap(
        root_slide,
        base,
        axes="x",
        elem_a=carriage_top,
        elem_b=rail,
        min_overlap=0.06,
        name="carriage remains over base rail footprint",
    )
    ctx.expect_contact(
        pivot_arm,
        root_slide,
        elem_a=center_barrel,
        elem_b=left_barrel,
        name="center hinge barrel contacts left fork barrel",
    )
    ctx.expect_contact(
        pivot_arm,
        root_slide,
        elem_a=center_barrel,
        elem_b=right_barrel,
        name="center hinge barrel contacts right fork barrel",
    )
    ctx.expect_within(
        distal_stage,
        pivot_arm,
        axes="yz",
        inner_elem=stage_bar,
        margin=0.0,
        name="distal stage remains within housing cross section at rest",
    )
    ctx.expect_contact(
        distal_stage,
        pivot_arm,
        elem_a=left_guide_shoe,
        elem_b=housing_left,
        name="left guide shoe bears on housing wall",
    )
    ctx.expect_contact(
        distal_stage,
        pivot_arm,
        elem_a=right_guide_shoe,
        elem_b=housing_right,
        name="right guide shoe bears on housing wall",
    )

    root_rest = ctx.part_world_position(root_slide)
    with ctx.pose({root_joint: 0.075}):
        root_forward = ctx.part_world_position(root_slide)
        ctx.expect_overlap(
            root_slide,
            base,
            axes="x",
            elem_a=carriage_top,
            elem_b=rail,
            min_overlap=0.06,
            name="carriage stays on rail at forward travel",
        )
    with ctx.pose({root_joint: -0.075}):
        root_back = ctx.part_world_position(root_slide)
        ctx.expect_overlap(
            root_slide,
            base,
            axes="x",
            elem_a=carriage_top,
            elem_b=rail,
            min_overlap=0.06,
            name="carriage stays on rail at rear travel",
        )
    ctx.check(
        "root stage translates 150 mm total",
        (
            root_rest is not None
            and root_forward is not None
            and root_back is not None
            and abs((root_forward[0] - root_back[0]) - 0.150) < 1e-6
        ),
        "root carriage origin should move 150 mm between prismatic endpoints",
    )

    rest_housing_center = aabb_center(pivot_arm, housing_top)
    with ctx.pose({pivot_joint: math.pi / 2.0}):
        swung_down_housing_center = aabb_center(pivot_arm, housing_top)
        ctx.expect_within(
            distal_stage,
            pivot_arm,
            axes="xy",
            inner_elem=stage_bar,
            margin=0.0,
            name="distal stage stays laterally captured when swung down",
        )
    with ctx.pose({pivot_joint: -math.pi / 2.0}):
        swung_up_housing_center = aabb_center(pivot_arm, housing_top)
        ctx.expect_within(
            distal_stage,
            pivot_arm,
            axes="xy",
            inner_elem=stage_bar,
            margin=0.0,
            name="distal stage stays laterally captured when swung up",
        )
    ctx.check(
        "pivot swings housing to opposite sides of hinge height",
        (
            rest_housing_center is not None
            and swung_down_housing_center is not None
            and swung_up_housing_center is not None
            and (
                (
                    swung_up_housing_center[2] > rest_housing_center[2] + 0.06
                    and swung_down_housing_center[2] < rest_housing_center[2] - 0.06
                )
                or (
                    swung_down_housing_center[2] > rest_housing_center[2] + 0.06
                    and swung_up_housing_center[2] < rest_housing_center[2] - 0.06
                )
            )
        ),
        "housing center should rotate substantially above and below its rest height",
    )

    rest_tool_center = aabb_center(distal_stage, tool_plate)
    with ctx.pose({distal_joint: 0.080}):
        extended_tool_center = aabb_center(distal_stage, tool_plate)
        ctx.expect_overlap(
            distal_stage,
            pivot_arm,
            axes="x",
            elem_a=stage_bar,
            min_overlap=0.045,
            name="stage remains engaged at full extension",
        )
    ctx.check(
        "distal stage extends 80 mm along arm axis at rest orientation",
        (
            rest_tool_center is not None
            and extended_tool_center is not None
            and abs((extended_tool_center[0] - rest_tool_center[0]) - 0.080) < 1e-6
        ),
        "tool plate center should advance 80 mm along x at full extension",
    )

    with ctx.pose({pivot_joint: math.pi / 2.0, distal_joint: 0.0}):
        upright_retracted_tool = aabb_center(distal_stage, tool_plate)
    with ctx.pose({pivot_joint: math.pi / 2.0, distal_joint: 0.080}):
        upright_extended_tool = aabb_center(distal_stage, tool_plate)
    ctx.check(
        "distal prismatic axis follows rotated arm",
        (
            upright_retracted_tool is not None
            and upright_extended_tool is not None
            and abs(upright_extended_tool[2] - upright_retracted_tool[2]) > 0.06
            and abs(upright_extended_tool[0] - upright_retracted_tool[0]) < 0.03
        ),
        "after a 90 degree pivot, the distal slider should extend mostly along world z rather than x",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
