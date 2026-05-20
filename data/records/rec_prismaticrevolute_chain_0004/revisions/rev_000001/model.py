from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
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
    model = ArticulatedObject(name="prismatic_revolute_chain", assets=ASSETS)

    rail_gray = model.material("rail_gray", rgba=(0.32, 0.35, 0.39, 1.0))
    carriage_silver = model.material("carriage_silver", rgba=(0.72, 0.74, 0.76, 1.0))
    arm_orange = model.material("arm_orange", rgba=(0.88, 0.48, 0.16, 1.0))

    base = model.part("base_rail")
    base.visual(
        Box((0.280, 0.060, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=rail_gray,
        name="base_plate",
    )
    base.visual(
        Box((0.240, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=rail_gray,
        name="guide",
    )
    base.visual(
        Box((0.018, 0.040, 0.012)),
        origin=Origin(xyz=(-0.131, 0.0, 0.022)),
        material=rail_gray,
        name="left_stop",
    )
    base.visual(
        Box((0.018, 0.040, 0.012)),
        origin=Origin(xyz=(0.131, 0.0, 0.022)),
        material=rail_gray,
        name="right_stop",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.280, 0.060, 0.034)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.055, 0.050, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=carriage_silver,
        name="bridge",
    )
    carriage.visual(
        Box((0.055, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, -0.0185, 0.026)),
        material=carriage_silver,
        name="left_shoe",
    )
    carriage.visual(
        Box((0.055, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.0185, 0.026)),
        material=carriage_silver,
        name="right_shoe",
    )
    carriage.visual(
        Box((0.055, 0.008, 0.030)),
        origin=Origin(xyz=(0.0, 0.029, 0.031)),
        material=carriage_silver,
        name="side_plate",
    )
    carriage.visual(
        Box((0.010, 0.017, 0.022)),
        origin=Origin(xyz=(-0.014, 0.0415, 0.055)),
        material=carriage_silver,
        name="clevis_left_ear",
    )
    carriage.visual(
        Box((0.010, 0.017, 0.022)),
        origin=Origin(xyz=(0.014, 0.0415, 0.055)),
        material=carriage_silver,
        name="clevis_right_ear",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.055, 0.058, 0.050)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.008, 0.029)),
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.0085, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        material=arm_orange,
        name="pivot_barrel",
    )
    arm.visual(
        Box((0.012, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        material=arm_orange,
        name="pivot_cheek",
    )
    arm.visual(
        Box((0.012, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.010, 0.015)),
        material=arm_orange,
        name="root_block",
    )
    arm.visual(
        Box((0.014, 0.010, 0.100)),
        origin=Origin(xyz=(0.0, 0.010, 0.074)),
        material=arm_orange,
        name="arm_bar",
    )
    arm.visual(
        Box((0.024, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.010, 0.133)),
        material=arm_orange,
        name="tip_pad",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.024, 0.020, 0.142)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.008, 0.071)),
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.25,
            lower=-0.075,
            upper=0.075,
        ),
    )
    model.articulation(
        "carriage_to_arm",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0415, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=-1.57079632679,
            upper=1.57079632679,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base_rail")
    carriage = object_model.get_part("carriage")
    arm = object_model.get_part("arm")
    slide = object_model.get_articulation("rail_to_carriage")
    hinge = object_model.get_articulation("carriage_to_arm")

    guide = base.get_visual("guide")
    bridge = carriage.get_visual("bridge")
    left_shoe = carriage.get_visual("left_shoe")
    right_shoe = carriage.get_visual("right_shoe")
    side_plate = carriage.get_visual("side_plate")
    clevis_left_ear = carriage.get_visual("clevis_left_ear")
    clevis_right_ear = carriage.get_visual("clevis_right_ear")
    pivot_barrel = arm.get_visual("pivot_barrel")
    arm_bar = arm.get_visual("arm_bar")
    arm_bar = arm.get_visual("arm_bar")

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
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=True,
        ignore_fixed=False,
    )

    ctx.check(
        "parts_present",
        all(part is not None for part in (base, carriage, arm)),
        "Base rail, carriage, and arm must all be present.",
    )
    ctx.check(
        "slide_joint_is_prismatic_x",
        slide.articulation_type == ArticulationType.PRISMATIC and tuple(slide.axis) == (1.0, 0.0, 0.0),
        f"Expected PRISMATIC joint on +X, got type={slide.articulation_type}, axis={slide.axis}.",
    )
    ctx.check(
        "slide_travel_is_150mm",
        abs(slide.motion_limits.lower + 0.075) < 1e-9 and abs(slide.motion_limits.upper - 0.075) < 1e-9,
        f"Expected ±75 mm slide limits, got {slide.motion_limits}.",
    )
    ctx.check(
        "hinge_joint_is_revolute_y",
        hinge.articulation_type == ArticulationType.REVOLUTE and tuple(hinge.axis) == (0.0, 1.0, 0.0),
        f"Expected REVOLUTE joint on +Y, got type={hinge.articulation_type}, axis={hinge.axis}.",
    )
    ctx.check(
        "hinge_range_is_90deg_each_way",
        abs(hinge.motion_limits.lower + 1.57079632679) < 1e-9
        and abs(hinge.motion_limits.upper - 1.57079632679) < 1e-9,
        f"Expected ±90 degree hinge limits, got {hinge.motion_limits}.",
    )

    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        positive_elem=bridge,
        negative_elem=guide,
        min_gap=0.001,
        max_gap=0.002,
        name="carriage_bridge_runs_just_above_guide",
    )
    ctx.expect_gap(
        carriage,
        base,
        axis="y",
        positive_elem=right_shoe,
        negative_elem=guide,
        min_gap=0.001,
        max_gap=0.0025,
        name="right_shoe_clears_guide_side",
    )
    ctx.expect_gap(
        base,
        carriage,
        axis="y",
        positive_elem=guide,
        negative_elem=left_shoe,
        min_gap=0.001,
        max_gap=0.0025,
        name="left_shoe_clears_guide_side",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="x",
        elem_a=bridge,
        elem_b=guide,
        min_overlap=0.050,
        name="carriage_spans_guide_in_x",
    )

    ctx.expect_contact(
        arm,
        carriage,
        elem_a=pivot_barrel,
        elem_b=clevis_left_ear,
        name="pivot_barrel_contacts_left_clevis_ear",
    )
    ctx.expect_contact(
        arm,
        carriage,
        elem_a=pivot_barrel,
        elem_b=clevis_right_ear,
        name="pivot_barrel_contacts_right_clevis_ear",
    )
    ctx.expect_gap(
        arm,
        carriage,
        axis="y",
        positive_elem=arm_bar,
        negative_elem=side_plate,
        min_gap=0.013,
        max_gap=0.014,
        name="arm_bar_stays_outboard_of_side_plate",
    )

    with ctx.pose({slide: -0.075}):
        ctx.expect_within(
            carriage,
            base,
            axes="x",
            inner_elem=bridge,
            outer_elem=guide,
            margin=0.0,
            name="carriage_left_extreme_stays_on_guide",
        )
    with ctx.pose({slide: 0.075}):
        ctx.expect_within(
            carriage,
            base,
            axes="x",
            inner_elem=bridge,
            outer_elem=guide,
            margin=0.0,
            name="carriage_right_extreme_stays_on_guide",
        )
    with ctx.pose({hinge: 1.57079632679}):
        ctx.expect_gap(
            arm,
            carriage,
            axis="y",
            positive_elem=arm_bar,
            negative_elem=side_plate,
            min_gap=0.013,
            max_gap=0.014,
            name="arm_clears_carriage_at_positive_90deg",
        )
    with ctx.pose({hinge: -1.57079632679}):
        ctx.expect_gap(
            arm,
            carriage,
            axis="y",
            positive_elem=arm_bar,
            negative_elem=side_plate,
            min_gap=0.013,
            max_gap=0.014,
            name="arm_clears_carriage_at_negative_90deg",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
