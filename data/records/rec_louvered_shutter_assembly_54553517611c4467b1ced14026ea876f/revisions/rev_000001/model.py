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
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _louver_blade_mesh(*, span: float, chord: float, thickness: float, name: str):
    half_span = span * 0.5
    half_chord = chord * 0.5
    half_thickness = thickness * 0.5
    profile_xy = [
        (half_chord, 0.0),
        (half_chord * 0.70, half_thickness * 0.55),
        (0.0, half_thickness),
        (-half_chord * 0.68, half_thickness * 0.50),
        (-half_chord, 0.0),
        (-half_chord * 0.45, -half_thickness * 0.25),
        (0.0, -half_thickness * 0.38),
        (half_chord * 0.45, -half_thickness * 0.25),
    ]
    geom = LoftGeometry(
        [
            [(x, y, -half_span) for x, y in profile_xy],
            [(x, y, half_span) for x, y in profile_xy],
        ],
        cap=True,
        closed=True,
    )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="storm_shutter")

    wall_trim = model.material("wall_trim", rgba=(0.94, 0.93, 0.90, 1.0))
    shutter_paint = model.material("shutter_paint", rgba=(0.22, 0.36, 0.30, 1.0))
    shutter_shadow = model.material("shutter_shadow", rgba=(0.16, 0.24, 0.21, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.23, 0.22, 0.20, 1.0))
    arm_metal = model.material("arm_metal", rgba=(0.35, 0.37, 0.39, 1.0))

    panel_width = 0.48
    panel_height = 1.24
    panel_depth = 0.028
    stile_width = 0.055
    rail_height = 0.070
    louver_span = 0.346
    louver_chord = 0.086
    louver_thickness = 0.012
    louver_default_angle = math.radians(-18.0)
    louver_positions = [-0.448, -0.320, -0.192, -0.064, 0.064, 0.192, 0.320, 0.448]
    hinge_x = -0.13
    hinge_y = 0.074
    hinge_z = 0.73

    louver_mesh = _louver_blade_mesh(
        span=louver_span,
        chord=louver_chord,
        thickness=louver_thickness,
        name="storm_shutter_louver_blade",
    )

    opening_frame = model.part("opening_frame")
    opening_frame.visual(
        Box((1.12, 0.090, 0.140)),
        origin=Origin(xyz=(-0.12, 0.0, 1.39)),
        material=wall_trim,
        name="header",
    )
    opening_frame.visual(
        Box((1.12, 0.090, 0.140)),
        origin=Origin(xyz=(-0.12, 0.0, 0.07)),
        material=wall_trim,
        name="sill",
    )
    opening_frame.visual(
        Box((0.580, 0.090, 1.18)),
        origin=Origin(xyz=(-0.39, 0.0, 0.73)),
        material=wall_trim,
        name="left_fascia",
    )
    opening_frame.visual(
        Box((0.120, 0.090, 1.18)),
        origin=Origin(xyz=(0.38, 0.0, 0.73)),
        material=wall_trim,
        name="right_jamb",
    )
    opening_frame.visual(
        Box((0.032, 0.050, 1.18)),
        origin=Origin(xyz=(-0.116, 0.020, 0.73)),
        material=wall_trim,
        name="hinge_jamb_edge",
    )
    opening_frame.visual(
        Box((0.032, 0.050, 1.18)),
        origin=Origin(xyz=(0.336, 0.020, 0.73)),
        material=wall_trim,
        name="latch_jamb_edge",
    )

    hinge_leaf_width = 0.030
    hinge_leaf_depth = 0.028
    hinge_leaf_height = 0.120
    for index, local_z in enumerate((-0.43, 0.0, 0.43)):
        world_z = hinge_z + local_z
        opening_frame.visual(
            Box((hinge_leaf_width, hinge_leaf_depth, hinge_leaf_height)),
            origin=Origin(
                xyz=(hinge_x - 0.017, 0.057, world_z),
            ),
            material=hinge_metal,
            name=f"frame_hinge_leaf_{index}",
        )
        opening_frame.visual(
            Cylinder(radius=0.008, length=0.118),
            origin=Origin(
                xyz=(hinge_x - 0.024, hinge_y, world_z),
            ),
            material=hinge_metal,
            name=f"frame_hinge_knuckle_{index}",
        )

    shutter_panel = model.part("shutter_panel")
    shutter_panel.visual(
        Box((stile_width, panel_depth, panel_height)),
        origin=Origin(xyz=(stile_width * 0.5, 0.0, 0.0)),
        material=shutter_paint,
        name="hinge_stile",
    )
    shutter_panel.visual(
        Box((stile_width, panel_depth, panel_height)),
        origin=Origin(xyz=(panel_width - stile_width * 0.5, 0.0, 0.0)),
        material=shutter_paint,
        name="outer_stile",
    )
    shutter_panel.visual(
        Box((panel_width, panel_depth, rail_height)),
        origin=Origin(xyz=(panel_width * 0.5, 0.0, panel_height * 0.5 - rail_height * 0.5)),
        material=shutter_paint,
        name="top_rail",
    )
    shutter_panel.visual(
        Box((panel_width, panel_depth, rail_height)),
        origin=Origin(xyz=(panel_width * 0.5, 0.0, -panel_height * 0.5 + rail_height * 0.5)),
        material=shutter_paint,
        name="bottom_rail",
    )
    shutter_panel.visual(
        Box((panel_width - 0.018, 0.010, 0.028)),
        origin=Origin(xyz=(panel_width * 0.5, 0.006, panel_height * 0.5 - 0.034)),
        material=shutter_shadow,
        name="top_cap_shadow",
    )
    shutter_panel.visual(
        Box((0.060, 0.014, 0.070)),
        origin=Origin(xyz=(0.430, -0.021, -0.180)),
        material=hinge_metal,
        name="keeper_pad",
    )
    shutter_panel.visual(
        Box((0.014, 0.010, 0.084)),
        origin=Origin(xyz=(0.416, -0.019, -0.180)),
        material=hinge_metal,
        name="keeper_inner_ear",
    )
    shutter_panel.visual(
        Box((0.014, 0.010, 0.084)),
        origin=Origin(xyz=(0.444, -0.019, -0.180)),
        material=hinge_metal,
        name="keeper_outer_ear",
    )
    shutter_panel.visual(
        Cylinder(radius=0.006, length=0.032),
        origin=Origin(
            xyz=(0.430, -0.018, -0.180),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hinge_metal,
        name="keeper_pivot_stub",
    )

    for index, local_z in enumerate((-0.43, 0.0, 0.43)):
        shutter_panel.visual(
            Cylinder(radius=0.008, length=0.118),
            origin=Origin(
                xyz=(-0.008, 0.0, local_z),
                rpy=(0.0, 0.0, 0.0),
            ),
            material=hinge_metal,
            name=f"panel_hinge_knuckle_{index}",
        )
        shutter_panel.visual(
            Box((0.020, 0.006, 0.102)),
            origin=Origin(xyz=(0.010, 0.017, local_z)),
            material=hinge_metal,
            name=f"panel_hinge_leaf_{index}",
        )

    support_arm = model.part("support_arm")
    support_arm.visual(
        Box((0.112, 0.012, 0.010)),
        origin=Origin(xyz=(0.066, 0.0, 0.0)),
        material=arm_metal,
        name="arm_bar",
    )
    support_arm.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=arm_metal,
        name="wall_pivot_barrel",
    )
    support_arm.visual(
        Cylinder(radius=0.008, length=0.008),
        origin=Origin(xyz=(0.130, 0.0, 0.006)),
        material=arm_metal,
        name="tip_pivot_barrel_upper",
    )
    support_arm.visual(
        Cylinder(radius=0.008, length=0.008),
        origin=Origin(xyz=(0.130, 0.0, -0.006)),
        material=arm_metal,
        name="tip_pivot_barrel_lower",
    )

    support_tip = model.part("support_tip")
    support_tip.visual(
        Box((0.022, 0.012, 0.052)),
        origin=Origin(xyz=(0.019, -0.004, 0.0)),
        material=arm_metal,
        name="tip_pad",
    )
    support_tip.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=arm_metal,
        name="tip_hinge_barrel",
    )
    support_tip.visual(
        Box((0.006, 0.010, 0.066)),
        origin=Origin(xyz=(0.032, -0.004, 0.0)),
        material=arm_metal,
        name="tip_stop",
    )

    for index, local_z in enumerate(louver_positions):
        louver_part = model.part(f"louver_{index:02d}")
        louver_part.visual(
            louver_mesh,
            origin=Origin(rpy=(louver_default_angle, 0.0, 0.0)),
            material=shutter_paint,
            name="blade",
        )
        louver_part.visual(
            Cylinder(radius=0.006, length=0.02134),
            origin=Origin(
                xyz=(-louver_span * 0.5, 0.0, 0.0),
                rpy=(louver_default_angle, math.pi / 2.0, 0.0),
            ),
            material=hinge_metal,
            name="left_journal",
        )
        louver_part.visual(
            Cylinder(radius=0.006, length=0.02134),
            origin=Origin(
                xyz=(louver_span * 0.5, 0.0, 0.0),
                rpy=(louver_default_angle, math.pi / 2.0, 0.0),
            ),
            material=hinge_metal,
            name="right_journal",
        )
        model.articulation(
            f"panel_to_louver_{index:02d}",
            ArticulationType.REVOLUTE,
            parent=shutter_panel,
            child=louver_part,
            origin=Origin(xyz=(panel_width * 0.5, 0.0, local_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=2.5,
                lower=math.radians(-28.0),
                upper=math.radians(34.0),
            ),
        )

    model.articulation(
        "frame_to_panel",
        ArticulationType.REVOLUTE,
        parent=opening_frame,
        child=shutter_panel,
        origin=Origin(xyz=(hinge_x, hinge_y + 0.005, hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(168.0),
        ),
    )
    model.articulation(
        "frame_to_support_arm",
        ArticulationType.REVOLUTE,
        parent=opening_frame,
        child=support_arm,
        origin=Origin(xyz=(-0.57, 0.052, 0.55)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=math.radians(-15.0),
            upper=math.radians(115.0),
        ),
    )
    model.articulation(
        "support_arm_to_tip",
        ArticulationType.REVOLUTE,
        parent=support_arm,
        child=support_tip,
        origin=Origin(xyz=(0.130, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.5,
            lower=math.radians(-55.0),
            upper=math.radians(55.0),
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

    opening_frame = object_model.get_part("opening_frame")
    shutter_panel = object_model.get_part("shutter_panel")
    support_arm = object_model.get_part("support_arm")
    support_tip = object_model.get_part("support_tip")
    panel_hinge = object_model.get_articulation("frame_to_panel")
    arm_joint = object_model.get_articulation("frame_to_support_arm")
    tip_joint = object_model.get_articulation("support_arm_to_tip")
    louver_parts = [object_model.get_part(f"louver_{index:02d}") for index in range(8)]
    louver_joints = [object_model.get_articulation(f"panel_to_louver_{index:02d}") for index in range(8)]

    ctx.check(
        "panel hinge is vertical",
        tuple(panel_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={panel_hinge.axis}",
    )
    ctx.check(
        "support arm pivots are vertical",
        tuple(arm_joint.axis) == (0.0, 0.0, 1.0) and tuple(tip_joint.axis) == (0.0, 0.0, 1.0),
        details=f"arm_axis={arm_joint.axis}, tip_axis={tip_joint.axis}",
    )
    ctx.check(
        "louvers pivot on their long axis",
        all(tuple(joint.axis) == (1.0, 0.0, 0.0) for joint in louver_joints),
        details=str([joint.axis for joint in louver_joints]),
    )

    with ctx.pose({panel_hinge: 0.0, arm_joint: 0.0, tip_joint: 0.0}):
        ctx.expect_gap(
            shutter_panel,
            opening_frame,
            axis="y",
            negative_elem="hinge_jamb_edge",
            min_gap=0.0,
            max_gap=0.035,
            name="closed shutter sits just proud of the trim",
        )
        ctx.expect_overlap(
            shutter_panel,
            opening_frame,
            axes="xz",
            elem_a="outer_stile",
            elem_b="right_jamb",
            min_overlap=0.020,
            name="closed shutter covers the right side of the opening",
        )
        for index, louver_part in enumerate(louver_parts):
            ctx.expect_within(
                louver_part,
                shutter_panel,
                axes="xz",
                inner_elem="blade",
                margin=0.010,
                name=f"louver {index:02d} stays within the panel frame",
            )

    with ctx.pose(
        {
            panel_hinge: math.radians(163.0),
            arm_joint: math.radians(78.0),
            tip_joint: math.radians(6.0),
        }
    ):
        ctx.expect_gap(
            shutter_panel,
            opening_frame,
            axis="y",
            positive_elem="outer_stile",
            negative_elem="right_jamb",
            min_gap=0.120,
            name="outer stile swings clear of the opening when opened",
        )
        ctx.expect_gap(
            shutter_panel,
            support_tip,
            axis="y",
            positive_elem="keeper_pad",
            negative_elem="tip_pad",
            min_gap=0.0,
            max_gap=0.020,
            name="support tip seats close behind the panel keeper",
        )
        ctx.expect_overlap(
            shutter_panel,
            support_tip,
            axes="xz",
            elem_a="keeper_pad",
            elem_b="tip_pad",
            min_overlap=0.014,
            name="support tip aligns with the keeper bracket in the hold-open pose",
        )

    with ctx.pose({louver_joints[3]: math.radians(22.0)}):
        ctx.expect_within(
            louver_parts[3],
            shutter_panel,
            axes="xz",
            inner_elem="blade",
            margin=0.012,
            name="representative louver can rotate without leaving the panel frame",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
