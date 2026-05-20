from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cheerful_retro_stand_mixer")

    enamel = model.material("sunny_enamel", rgba=(0.96, 0.27, 0.18, 1.0))
    cream = model.material("warm_cream", rgba=(1.0, 0.86, 0.55, 1.0))
    steel = model.material("brushed_steel", rgba=(0.78, 0.78, 0.74, 1.0))
    dark = model.material("dark_rubber", rgba=(0.05, 0.045, 0.04, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.90, 0.92, 0.92, 1.0))

    # Root base: broad rounded plinth, curved rear column, and exposed hinge yoke.
    base = model.part("base")
    base_shell = superellipse_side_loft(
        [
            (-0.150, 0.000, 0.056, 0.440),
            (-0.095, -0.002, 0.070, 0.515),
            (0.000, -0.004, 0.078, 0.540),
            (0.095, -0.002, 0.070, 0.515),
            (0.150, 0.000, 0.056, 0.440),
        ],
        exponents=3.0,
        segments=56,
    )
    base.visual(
        mesh_from_geometry(base_shell, "base_plinth"),
        origin=Origin(xyz=(0.035, 0.0, 0.004)),
        material=enamel,
        name="base_plinth",
    )
    rear_column = superellipse_side_loft(
        [
            (-0.088, 0.060, 0.270, 0.110),
            (-0.044, 0.054, 0.286, 0.145),
            (0.000, 0.052, 0.296, 0.160),
            (0.044, 0.054, 0.286, 0.145),
            (0.088, 0.060, 0.270, 0.110),
        ],
        exponents=2.6,
        segments=48,
    )
    base.visual(
        mesh_from_geometry(rear_column, "rear_column"),
        origin=Origin(xyz=(-0.135, 0.0, 0.0)),
        material=enamel,
        name="rear_column",
    )
    base.visual(
        Box((0.135, 0.150, 0.038)),
        origin=Origin(xyz=(-0.135, 0.0, 0.075)),
        material=enamel,
        name="column_neck",
    )
    base.visual(
        Box((0.060, 0.235, 0.064)),
        origin=Origin(xyz=(-0.155, 0.0, 0.315)),
        material=enamel,
        name="yoke_support",
    )
    for idx, y in enumerate((-0.116, 0.116)):
        base.visual(
            Box((0.084, 0.030, 0.090)),
            origin=Origin(xyz=(-0.130, y, 0.376)),
            material=enamel,
            name=f"hinge_cheek_{idx}",
        )
    base.visual(
        Cylinder(radius=0.018, length=0.232),
        origin=Origin(xyz=(-0.130, 0.0, 0.376), rpy=(-pi / 2, 0.0, 0.0)),
        material=chrome,
        name="hinge_pin",
    )
    base.visual(
        Box((0.290, 0.023, 0.012)),
        origin=Origin(xyz=(0.070, -0.110, 0.069)),
        material=cream,
        name="slide_rail_0",
    )
    base.visual(
        Box((0.290, 0.023, 0.012)),
        origin=Origin(xyz=(0.070, 0.110, 0.069)),
        material=cream,
        name="slide_rail_1",
    )
    base.visual(
        Box((0.020, 0.072, 0.030)),
        origin=Origin(xyz=(0.289, 0.070, 0.058)),
        material=enamel,
        name="lock_socket",
    )

    # Bowl carriage: a short sliding tray with a compact hollow steel bowl.
    bowl_carriage = model.part("bowl_carriage")
    bowl_carriage.visual(
        Box((0.235, 0.185, 0.016)),
        origin=Origin(xyz=(0.085, 0.0, 0.008)),
        material=cream,
        name="carriage_plate",
    )
    bowl_carriage.visual(
        Cylinder(radius=0.064, length=0.020),
        origin=Origin(xyz=(0.103, 0.0, 0.026)),
        material=steel,
        name="bowl_foot",
    )
    bowl_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.052, 0.000),
            (0.082, 0.026),
            (0.118, 0.090),
            (0.138, 0.145),
            (0.144, 0.162),
        ],
        inner_profile=[
            (0.038, 0.012),
            (0.067, 0.034),
            (0.103, 0.094),
            (0.126, 0.143),
            (0.132, 0.154),
        ],
        segments=96,
        start_cap="flat",
        end_cap="round",
        lip_samples=10,
    )
    bowl_carriage.visual(
        mesh_from_geometry(bowl_shell, "steel_bowl"),
        origin=Origin(xyz=(0.103, 0.0, 0.036)),
        material=steel,
        name="steel_bowl",
    )

    # Tilting head: hinge frame at rear; the rounded shell and short nose extend forward.
    head = model.part("head")
    head_shell = superellipse_side_loft(
        [
            (-0.096, -0.048, 0.080, 0.250),
            (-0.050, -0.064, 0.095, 0.325),
            (0.000, -0.068, 0.104, 0.350),
            (0.050, -0.064, 0.095, 0.325),
            (0.096, -0.048, 0.080, 0.250),
        ],
        exponents=2.55,
        segments=64,
    )
    head.visual(
        mesh_from_geometry(head_shell, "rounded_head_housing"),
        origin=Origin(xyz=(0.205, 0.0, 0.008)),
        material=enamel,
        name="rounded_housing",
    )
    head.visual(
        Cylinder(radius=0.040, length=0.118),
        origin=Origin(xyz=(0.230, 0.0, -0.026), rpy=(0.0, pi / 2, 0.0)),
        material=enamel,
        name="short_nose",
    )
    head.visual(
        Cylinder(radius=0.026, length=0.205),
        origin=Origin(xyz=(0.000, 0.0, 0.000), rpy=(-pi / 2, 0.0, 0.0)),
        material=chrome,
        name="head_hinge_barrel",
    )
    head.visual(
        Box((0.052, 0.150, 0.024)),
        origin=Origin(xyz=(0.026, 0.0, 0.036)),
        material=enamel,
        name="hinge_boss",
    )
    head.visual(
        Box((0.082, 0.118, 0.020)),
        origin=Origin(xyz=(0.218, 0.0, -0.075)),
        material=cream,
        name="nose_trim",
    )

    # Tool rotates continuously below the nose.  The dough hook is one continuous
    # swept tube, shaped to sit inside the bowl without intersecting the wall.
    tool = model.part("tool")
    tool.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=chrome,
        name="tool_shaft",
    )
    hook_path = [
        (0.000, 0.000, -0.054),
        (0.010, 0.000, -0.071),
        (0.038, -0.012, -0.095),
        (0.050, -0.002, -0.124),
        (0.034, 0.026, -0.146),
        (-0.002, 0.038, -0.150),
        (-0.030, 0.018, -0.135),
        (-0.022, -0.010, -0.114),
    ]
    tool.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                hook_path,
                radius=0.006,
                samples_per_segment=16,
                radial_segments=18,
                cap_ends=True,
            ),
            "dough_hook",
        ),
        material=chrome,
        name="dough_hook",
    )

    # Separate articulated controls on the base.
    speed_control = model.part("speed_control")
    speed_control.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.054,
                0.024,
                body_style="skirted",
                top_diameter=0.044,
                skirt=KnobSkirt(0.064, 0.006, flare=0.06, chamfer=0.001),
                grip=KnobGrip(style="fluted", count=18, depth=0.0014),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=20.0),
                center=False,
            ),
            "speed_control_knob",
        ),
        origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
        material=cream,
        name="speed_knob",
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Box((0.030, 0.060, 0.022)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=cream,
        name="lock_tab",
    )

    # Canonical stand-mixer articulation tree.
    model.articulation(
        "base_to_bowl_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl_carriage,
        origin=Origin(xyz=(-0.010, 0.0, 0.082)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.20, lower=0.0, upper=0.055),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.130, 0.0, 0.376)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=0.78),
    )
    model.articulation(
        "head_to_tool",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=tool,
        origin=Origin(xyz=(0.218, 0.0, -0.085)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=18.0),
    )
    model.articulation(
        "base_to_speed_control",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_control,
        origin=Origin(xyz=(-0.135, -0.088, 0.176)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(0.299, 0.070, 0.058)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.018),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    joints = {joint.name: joint for joint in object_model.articulations}
    expected = {
        "base_to_bowl_carriage": ArticulationType.PRISMATIC,
        "base_to_head": ArticulationType.REVOLUTE,
        "head_to_tool": ArticulationType.CONTINUOUS,
        "base_to_speed_control": ArticulationType.REVOLUTE,
        "base_to_head_lock": ArticulationType.PRISMATIC,
    }
    ctx.check(
        "canonical stand-mixer articulation set",
        set(joints) == set(expected),
        details=f"joints={sorted(joints)}",
    )
    for name, expected_type in expected.items():
        ctx.check(
            f"{name} uses required joint type",
            joints.get(name) is not None and joints[name].articulation_type == expected_type,
        )

    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl_carriage")
    head = object_model.get_part("head")
    tool = object_model.get_part("tool")
    lock = object_model.get_part("head_lock")

    ctx.expect_contact(
        bowl,
        base,
        elem_a="carriage_plate",
        elem_b="base_plinth",
        contact_tol=0.006,
        name="bowl carriage rides on base plinth",
    )
    ctx.expect_within(
        tool,
        bowl,
        axes="xy",
        inner_elem="dough_hook",
        outer_elem="steel_bowl",
        margin=0.004,
        name="dough hook sits inside compact bowl footprint",
    )
    ctx.expect_gap(
        head,
        bowl,
        axis="z",
        positive_elem="nose_trim",
        negative_elem="steel_bowl",
        min_gap=0.008,
        name="head nose clears bowl rim",
    )
    ctx.allow_overlap(
        base,
        head,
        elem_a="hinge_pin",
        elem_b="head_hinge_barrel",
        reason="The chrome hinge pin is intentionally captured inside the head hinge barrel.",
    )
    ctx.expect_overlap(
        base,
        head,
        axes="y",
        elem_a="hinge_pin",
        elem_b="head_hinge_barrel",
        min_overlap=0.150,
        name="hinge pin is retained through the barrel",
    )

    slide = joints["base_to_bowl_carriage"]
    hinge = joints["base_to_head"]
    lock_slide = joints["base_to_head_lock"]

    bowl_rest = ctx.part_world_position(bowl)
    head_rest_aabb = ctx.part_world_aabb(head)
    lock_rest = ctx.part_world_position(lock)
    with ctx.pose({slide: 0.055, hinge: 0.72, lock_slide: 0.018}):
        bowl_extended = ctx.part_world_position(bowl)
        head_open_aabb = ctx.part_world_aabb(head)
        lock_extended = ctx.part_world_position(lock)

    ctx.check(
        "bowl slide moves forward only a short travel",
        bowl_rest is not None
        and bowl_extended is not None
        and 0.045 <= bowl_extended[0] - bowl_rest[0] <= 0.065,
        details=f"rest={bowl_rest}, extended={bowl_extended}",
    )
    ctx.check(
        "rear hinge lifts mixer head",
        head_rest_aabb is not None
        and head_open_aabb is not None
        and head_open_aabb[1][2] > head_rest_aabb[1][2] + 0.045,
        details=f"rest={head_rest_aabb}, open={head_open_aabb}",
    )
    ctx.check(
        "head-lock control slides outward from the base",
        lock_rest is not None
        and lock_extended is not None
        and lock_extended[0] > lock_rest[0] + 0.012,
        details=f"rest={lock_rest}, extended={lock_extended}",
    )

    return ctx.report()


object_model = build_object_model()
