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
    model = ArticulatedObject(name="french_sketch_box_easel")

    oak = model.material("oak", rgba=(0.72, 0.56, 0.36, 1.0))
    oak_dark = model.material("oak_dark", rgba=(0.56, 0.40, 0.24, 1.0))
    brass = model.material("brass", rgba=(0.73, 0.60, 0.26, 1.0))
    iron = model.material("iron", rgba=(0.19, 0.20, 0.22, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.10, 1.0))

    apex_height = 1.605
    front_pitch_rest = 0.32
    rear_pitch = 0.48
    leg_side_splay = 0.16

    apex_block = model.part("apex_block")
    apex_block.visual(
        Box((0.090, 0.170, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, apex_height)),
        material=oak_dark,
        name="apex_head",
    )
    apex_block.visual(
        Cylinder(radius=0.009, length=0.210),
        origin=Origin(
            xyz=(0.0, 0.0, apex_height),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="hinge_pin",
    )
    rear_sleeve_length = 0.340
    rear_joint_origin = Origin(xyz=(-0.180, 0.0, apex_height - 0.030), rpy=(0.0, rear_pitch, 0.0))
    apex_block.visual(
        Box((0.032, 0.030, rear_sleeve_length)),
        origin=Origin(
            xyz=(
                rear_joint_origin.xyz[0] - 0.5 * rear_sleeve_length * math.sin(rear_pitch),
                0.0,
                apex_height - 0.030 - 0.5 * rear_sleeve_length * math.cos(rear_pitch),
            ),
            rpy=(0.0, rear_pitch, 0.0),
        ),
        material=brass,
        name="rear_sleeve",
    )
    apex_block.visual(
        Box((0.110, 0.075, 0.024)),
        origin=Origin(xyz=(-0.117, 0.0, apex_height - 0.030)),
        material=oak_dark,
        name="rear_socket_block",
    )
    apex_block.visual(
        Box((0.125, 0.010, 0.018)),
        origin=Origin(xyz=(-0.105, -0.026, apex_height - 0.030)),
        material=iron,
        name="left_rear_socket_strap",
    )
    apex_block.visual(
        Box((0.125, 0.010, 0.018)),
        origin=Origin(xyz=(-0.105, 0.026, apex_height - 0.030)),
        material=iron,
        name="right_rear_socket_strap",
    )
    apex_block.visual(
        Box((0.012, 0.010, 0.150)),
        origin=Origin(
            xyz=(-0.142, -0.019, apex_height - 0.108),
            rpy=(0.0, rear_pitch, 0.0),
        ),
        material=iron,
        name="left_rear_sleeve_brace",
    )
    apex_block.visual(
        Box((0.012, 0.010, 0.150)),
        origin=Origin(
            xyz=(-0.142, 0.019, apex_height - 0.108),
            rpy=(0.0, rear_pitch, 0.0),
        ),
        material=iron,
        name="right_rear_sleeve_brace",
    )
    apex_block.inertial = Inertial.from_geometry(
        Box((0.18, 0.18, 0.14)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, apex_height - 0.010)),
    )

    front_assembly = model.part("front_assembly")
    front_assembly.visual(
        Box((0.060, 0.180, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=oak_dark,
        name="top_yoke",
    )
    front_assembly.visual(
        Box((0.024, 0.030, 0.014)),
        origin=Origin(xyz=(0.032, -0.058, -0.0084)),
        material=iron,
        name="left_hinge_leaf",
    )
    front_assembly.visual(
        Box((0.024, 0.030, 0.014)),
        origin=Origin(xyz=(0.032, 0.058, -0.0084)),
        material=iron,
        name="right_hinge_leaf",
    )
    front_assembly.visual(
        Box((0.026, 0.024, 1.540)),
        origin=Origin(
            xyz=(0.0, -0.140, -0.810),
            rpy=(-leg_side_splay, 0.0, 0.0),
        ),
        material=oak,
        name="left_front_leg",
    )
    front_assembly.visual(
        Box((0.026, 0.024, 1.540)),
        origin=Origin(
            xyz=(0.0, 0.140, -0.810),
            rpy=(leg_side_splay, 0.0, 0.0),
        ),
        material=oak,
        name="right_front_leg",
    )
    front_assembly.visual(
        Box((0.052, 0.060, 0.020)),
        origin=Origin(
            xyz=(0.0, -0.266, -1.572),
            rpy=(-leg_side_splay, 0.0, 0.0),
        ),
        material=rubber,
        name="left_foot_pad",
    )
    front_assembly.visual(
        Box((0.052, 0.060, 0.020)),
        origin=Origin(
            xyz=(0.0, 0.266, -1.572),
            rpy=(leg_side_splay, 0.0, 0.0),
        ),
        material=rubber,
        name="right_foot_pad",
    )
    front_assembly.visual(
        Box((0.024, 0.080, 0.580)),
        origin=Origin(xyz=(0.0, 0.0, -0.310)),
        material=oak,
        name="center_mast",
    )
    box_center_z = -0.740
    box_height = 0.320
    box_width = 0.440
    box_depth = 0.092
    wall = 0.012
    front_assembly.visual(
        Box((wall, box_width, box_height)),
        origin=Origin(xyz=(0.5 * (box_depth - wall), 0.0, box_center_z)),
        material=oak,
        name="box_front_panel",
    )
    front_assembly.visual(
        Box((wall, box_width, box_height)),
        origin=Origin(xyz=(-0.5 * (box_depth - wall), 0.0, box_center_z)),
        material=oak_dark,
        name="box_back_panel",
    )
    front_assembly.visual(
        Box((box_depth, wall, box_height)),
        origin=Origin(xyz=(0.0, -0.5 * (box_width - wall), box_center_z)),
        material=oak,
        name="box_left_side",
    )
    front_assembly.visual(
        Box((box_depth, wall, box_height)),
        origin=Origin(xyz=(0.0, 0.5 * (box_width - wall), box_center_z)),
        material=oak,
        name="box_right_side",
    )
    front_assembly.visual(
        Box((box_depth, box_width, wall)),
        origin=Origin(xyz=(0.0, 0.0, box_center_z - 0.5 * (box_height - wall))),
        material=oak_dark,
        name="box_bottom",
    )
    front_assembly.visual(
        Box((box_depth, box_width, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, box_center_z + 0.5 * (box_height - 0.016))),
        material=oak_dark,
        name="box_upper_edge",
    )
    front_assembly.inertial = Inertial.from_geometry(
        Box((0.56, 0.56, 1.66)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, -0.820)),
    )

    rear_leg = model.part("rear_leg")
    rear_leg.visual(
        Box((0.024, 0.022, 1.980)),
        origin=Origin(xyz=(0.0, 0.0, -0.690)),
        material=oak,
        name="rear_stile",
    )
    rear_leg.visual(
        Box((0.050, 0.055, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -1.691)),
        material=rubber,
        name="rear_foot_pad",
    )
    rear_leg.inertial = Inertial.from_geometry(
        Box((0.07, 0.07, 2.02)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, -0.720)),
    )

    tray = model.part("canvas_tray")
    tray.visual(
        Box((0.012, 0.096, 0.136)),
        origin=Origin(xyz=(0.026, 0.0, 0.022)),
        material=brass,
        name="tray_carriage_face",
    )
    tray.visual(
        Box((0.014, 0.012, 0.124)),
        origin=Origin(xyz=(0.019, -0.046, 0.016)),
        material=brass,
        name="tray_left_guide",
    )
    tray.visual(
        Box((0.014, 0.012, 0.124)),
        origin=Origin(xyz=(0.019, 0.046, 0.016)),
        material=brass,
        name="tray_right_guide",
    )
    tray.visual(
        Box((0.018, 0.096, 0.014)),
        origin=Origin(xyz=(0.026, 0.0, 0.086)),
        material=brass,
        name="tray_top_bridge",
    )
    tray.visual(
        Box((0.114, 0.066, 0.034)),
        origin=Origin(xyz=(0.075, 0.0, -0.028)),
        material=oak_dark,
        name="tray_support_arm",
    )
    tray.visual(
        Box((0.072, 0.430, 0.018)),
        origin=Origin(xyz=(0.152, 0.0, -0.052)),
        material=oak,
        name="tray_shelf",
    )
    tray.visual(
        Box((0.018, 0.400, 0.030)),
        origin=Origin(xyz=(0.188, 0.0, -0.033)),
        material=oak_dark,
        name="tray_lip",
    )
    tray.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(
            xyz=(0.030, 0.0, 0.034),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=iron,
        name="tray_clamp_knob",
    )
    tray.inertial = Inertial.from_geometry(
        Box((0.28, 0.48, 0.10)),
        mass=0.9,
        origin=Origin(xyz=(0.110, 0.0, -0.010)),
    )

    front_spread = model.articulation(
        "apex_to_front_assembly",
        ArticulationType.REVOLUTE,
        parent=apex_block,
        child=front_assembly,
        origin=Origin(xyz=(0.0, 0.0, apex_height - 0.035), rpy=(0.0, -front_pitch_rest, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.8,
            lower=-0.18,
            upper=0.14,
        ),
    )

    rear_extension = model.articulation(
        "apex_to_rear_leg",
        ArticulationType.PRISMATIC,
        parent=apex_block,
        child=rear_leg,
        origin=rear_joint_origin,
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.18,
            lower=0.0,
            upper=0.22,
        ),
    )

    tray_slide = model.articulation(
        "front_leg_to_tray",
        ArticulationType.PRISMATIC,
        parent=front_assembly,
        child=tray,
        origin=Origin(
            xyz=(0.0, 0.0, -0.430),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.14,
            lower=0.0,
            upper=0.42,
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

    apex_block = object_model.get_part("apex_block")
    front_assembly = object_model.get_part("front_assembly")
    rear_leg = object_model.get_part("rear_leg")
    tray = object_model.get_part("canvas_tray")

    front_spread = object_model.get_articulation("apex_to_front_assembly")
    rear_extension = object_model.get_articulation("apex_to_rear_leg")
    tray_slide = object_model.get_articulation("front_leg_to_tray")

    ctx.allow_overlap(
        apex_block,
        rear_leg,
        elem_a="rear_sleeve",
        elem_b="rear_stile",
        reason="The rear stile is intentionally represented as telescoping inside the simplified apex sleeve.",
    )
    ctx.allow_overlap(
        apex_block,
        rear_leg,
        elem_a="rear_socket_block",
        elem_b="rear_stile",
        reason="The rear socket block is a simplified clamp housing around the telescoping rear stile at the apex.",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    ctx.expect_gap(
        tray,
        front_assembly,
        axis="z",
        positive_elem="tray_shelf",
        negative_elem="box_upper_edge",
        min_gap=0.020,
        name="canvas tray stays above the storage box",
    )
    ctx.expect_overlap(
        tray,
        front_assembly,
        axes="y",
        elem_a="tray_shelf",
        elem_b="box_upper_edge",
        min_overlap=0.260,
        name="canvas tray spans across the front frame width",
    )
    ctx.expect_contact(
        tray,
        front_assembly,
        elem_a="tray_left_guide",
        elem_b="center_mast",
        name="left tray guide bears against the center mast",
    )
    ctx.expect_contact(
        tray,
        front_assembly,
        elem_a="tray_right_guide",
        elem_b="center_mast",
        name="right tray guide bears against the center mast",
    )

    with ctx.pose({front_spread: front_spread.motion_limits.lower}):
        closed_right_foot = aabb_center(
            ctx.part_element_world_aabb(front_assembly, elem="right_foot_pad")
        )
    with ctx.pose({front_spread: front_spread.motion_limits.upper}):
        open_right_foot = aabb_center(
            ctx.part_element_world_aabb(front_assembly, elem="right_foot_pad")
        )
    ctx.check(
        "front legs spread forward from the apex hinge",
        closed_right_foot is not None
        and open_right_foot is not None
        and open_right_foot[0] > closed_right_foot[0] + 0.10,
        details=f"closed_right_foot={closed_right_foot}, open_right_foot={open_right_foot}",
    )

    with ctx.pose({tray_slide: tray_slide.motion_limits.lower}):
        tray_low = aabb_center(ctx.part_element_world_aabb(tray, elem="tray_shelf"))
    with ctx.pose({tray_slide: tray_slide.motion_limits.upper}):
        tray_high = aabb_center(ctx.part_element_world_aabb(tray, elem="tray_shelf"))
        ctx.expect_gap(
            tray,
            front_assembly,
            axis="z",
            positive_elem="tray_shelf",
            negative_elem="box_upper_edge",
            min_gap=0.180,
            name="raised tray clears the box well above its lid line",
        )
    ctx.check(
        "canvas tray slides upward on the front leg",
        tray_low is not None and tray_high is not None and tray_high[2] > tray_low[2] + 0.22,
        details=f"tray_low={tray_low}, tray_high={tray_high}",
    )

    ctx.expect_overlap(
        rear_leg,
        apex_block,
        axes="z",
        elem_a="rear_stile",
        elem_b="rear_sleeve",
        min_overlap=0.060,
        name="rear leg remains inserted in the apex sleeve at rest",
    )
    with ctx.pose({rear_extension: rear_extension.motion_limits.upper}):
        rear_extended = aabb_center(ctx.part_element_world_aabb(rear_leg, elem="rear_foot_pad"))
        ctx.expect_overlap(
            rear_leg,
            apex_block,
            axes="z",
            elem_a="rear_stile",
            elem_b="rear_sleeve",
            min_overlap=0.015,
            name="rear leg keeps retained insertion at full extension",
        )
    with ctx.pose({rear_extension: rear_extension.motion_limits.lower}):
        rear_retracted = aabb_center(ctx.part_element_world_aabb(rear_leg, elem="rear_foot_pad"))
    ctx.check(
        "rear leg extends downward and rearward from the apex",
        rear_retracted is not None
        and rear_extended is not None
        and rear_extended[0] < rear_retracted[0] - 0.06
        and rear_extended[2] < rear_retracted[2] - 0.06,
        details=f"rear_retracted={rear_retracted}, rear_extended={rear_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
