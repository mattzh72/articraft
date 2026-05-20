from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pickup_tailgate_with_ramp")

    body_paint = model.material("body_paint", rgba=(0.20, 0.28, 0.45, 1.0))
    liner_black = model.material("liner_black", rgba=(0.12, 0.12, 0.13, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.18, 0.19, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.65, 0.67, 1.0))

    bed_length = 0.72
    overall_width = 1.70
    wall_thickness = 0.05
    floor_thickness = 0.035
    bedside_height = 0.54

    tailgate_width = 1.58
    tailgate_height = 0.54
    tailgate_depth = 0.062
    tailgate_back_skin = 0.012
    tailgate_frame_depth = 0.048
    tailgate_top_frame = 0.060
    tailgate_bottom_frame = 0.080
    ramp_width = 0.62
    ramp_length = 0.40
    ramp_thickness = 0.022
    ramp_open_angle = 1.45

    side_frame_width = (tailgate_width - ramp_width) / 2.0
    opening_top_z = tailgate_height - tailgate_top_frame

    bed = model.part("bed")
    bed.visual(
        Box((bed_length, overall_width, floor_thickness)),
        origin=Origin(xyz=(bed_length / 2.0, 0.0, -floor_thickness / 2.0)),
        material=liner_black,
        name="bed_floor",
    )
    bed.visual(
        Box((bed_length, wall_thickness, bedside_height + 0.004)),
        origin=Origin(
            xyz=(
                bed_length / 2.0,
                (overall_width - wall_thickness) / 2.0,
                (bedside_height / 2.0) - 0.002,
            )
        ),
        material=body_paint,
        name="left_bedside",
    )
    bed.visual(
        Box((bed_length, wall_thickness, bedside_height + 0.004)),
        origin=Origin(
            xyz=(
                bed_length / 2.0,
                -(overall_width - wall_thickness) / 2.0,
                (bedside_height / 2.0) - 0.002,
            )
        ),
        material=body_paint,
        name="right_bedside",
    )
    bed.visual(
        Box((wall_thickness, overall_width - (2.0 * wall_thickness), bedside_height + 0.004)),
        origin=Origin(
            xyz=(
                bed_length - (wall_thickness / 2.0),
                0.0,
                (bedside_height / 2.0) - 0.002,
            )
        ),
        material=body_paint,
        name="front_bed_panel",
    )
    bed.visual(
        Box((bed_length - 0.08, overall_width - 0.18, 0.006)),
        origin=Origin(
            xyz=(
                (bed_length - 0.08) / 2.0 + 0.04,
                0.0,
                0.003,
            )
        ),
        material=dark_trim,
        name="bed_liner",
    )
    bed.inertial = Inertial.from_geometry(
        Box((bed_length, overall_width, bedside_height + floor_thickness)),
        mass=48.0,
        origin=Origin(xyz=(bed_length / 2.0, 0.0, (bedside_height - floor_thickness) / 2.0)),
    )

    tailgate = model.part("tailgate")
    tailgate.visual(
        Box((tailgate_back_skin, tailgate_width, tailgate_height)),
        origin=Origin(
            xyz=(
                -(tailgate_depth / 2.0) + (tailgate_back_skin / 2.0),
                0.0,
                tailgate_height / 2.0,
            )
        ),
        material=body_paint,
        name="tailgate_outer_skin",
    )
    tailgate.visual(
        Box((tailgate_frame_depth, tailgate_width, tailgate_bottom_frame)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                tailgate_bottom_frame / 2.0,
            )
        ),
        material=liner_black,
        name="tailgate_bottom_rail",
    )
    tailgate.visual(
        Box((tailgate_frame_depth, tailgate_width, tailgate_top_frame)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                tailgate_height - (tailgate_top_frame / 2.0),
            )
        ),
        material=liner_black,
        name="tailgate_top_rail",
    )
    tailgate.visual(
        Box((tailgate_frame_depth, side_frame_width, tailgate_height - (tailgate_top_frame + tailgate_bottom_frame))),
        origin=Origin(
            xyz=(
                0.0,
                (tailgate_width - side_frame_width) / 2.0,
                tailgate_bottom_frame
                + (tailgate_height - (tailgate_top_frame + tailgate_bottom_frame)) / 2.0,
            )
        ),
        material=liner_black,
        name="tailgate_left_frame",
    )
    tailgate.visual(
        Box((tailgate_frame_depth, side_frame_width, tailgate_height - (tailgate_top_frame + tailgate_bottom_frame))),
        origin=Origin(
            xyz=(
                0.0,
                -(tailgate_width - side_frame_width) / 2.0,
                tailgate_bottom_frame
                + (tailgate_height - (tailgate_top_frame + tailgate_bottom_frame)) / 2.0,
            )
        ),
        material=liner_black,
        name="tailgate_right_frame",
    )
    tailgate.visual(
        Box((0.014, 0.22, 0.045)),
        origin=Origin(xyz=(-0.018, 0.0, 0.455)),
        material=dark_trim,
        name="tailgate_handle_bezel",
    )
    tailgate.inertial = Inertial.from_geometry(
        Box((tailgate_depth, tailgate_width, tailgate_height)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, tailgate_height / 2.0)),
    )

    ramp_panel = model.part("ramp_panel")
    ramp_panel.visual(
        Box((ramp_thickness, ramp_width, ramp_length)),
        origin=Origin(xyz=(ramp_thickness / 2.0, 0.0, -ramp_length / 2.0)),
        material=steel,
        name="ramp_face",
    )
    ramp_panel.visual(
        Box((0.024, 0.045, ramp_length)),
        origin=Origin(
            xyz=(
                0.012,
                (ramp_width / 2.0) - 0.0225,
                -ramp_length / 2.0,
            )
        ),
        material=dark_trim,
        name="ramp_left_stiffener",
    )
    ramp_panel.visual(
        Box((0.024, 0.045, ramp_length)),
        origin=Origin(
            xyz=(
                0.012,
                -(ramp_width / 2.0) + 0.0225,
                -ramp_length / 2.0,
            )
        ),
        material=dark_trim,
        name="ramp_right_stiffener",
    )
    for index, z_pos in enumerate((-0.09, -0.19, -0.29), start=1):
        ramp_panel.visual(
            Box((0.004, ramp_width - 0.08, 0.016)),
            origin=Origin(xyz=(ramp_thickness - 0.002, 0.0, z_pos)),
            material=dark_trim,
            name=f"ramp_tread_{index}",
        )
    ramp_panel.inertial = Inertial.from_geometry(
        Box((0.024, ramp_width, ramp_length)),
        mass=7.0,
        origin=Origin(xyz=(0.012, 0.0, -ramp_length / 2.0)),
    )

    model.articulation(
        "bed_to_tailgate",
        ArticulationType.REVOLUTE,
        parent=bed,
        child=tailgate,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=1.2,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "tailgate_to_ramp_panel",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=ramp_panel,
        origin=Origin(xyz=(0.0, 0.0, opening_top_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.5,
            lower=0.0,
            upper=ramp_open_angle,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bed = object_model.get_part("bed")
    tailgate = object_model.get_part("tailgate")
    ramp_panel = object_model.get_part("ramp_panel")
    tailgate_hinge = object_model.get_articulation("bed_to_tailgate")
    ramp_hinge = object_model.get_articulation("tailgate_to_ramp_panel")

    tailgate_open = (
        tailgate_hinge.motion_limits.upper
        if tailgate_hinge.motion_limits is not None and tailgate_hinge.motion_limits.upper is not None
        else math.pi / 2.0
    )
    ramp_open = (
        ramp_hinge.motion_limits.upper
        if ramp_hinge.motion_limits is not None and ramp_hinge.motion_limits.upper is not None
        else 1.45
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[i] + high[i]) * 0.5 for i in range(3))

    with ctx.pose({tailgate_hinge: 0.0, ramp_hinge: 0.0}):
        ctx.expect_gap(
            tailgate,
            bed,
            axis="z",
            positive_elem="tailgate_bottom_rail",
            negative_elem="bed_floor",
            min_gap=0.0,
            max_gap=0.001,
            name="tailgate hinge line sits on the bed floor level",
        )
        ctx.expect_within(
            ramp_panel,
            tailgate,
            axes="yz",
            margin=0.015,
            name="ramp panel stows within the tailgate footprint",
        )
        closed_tailgate_aabb = ctx.part_world_aabb(tailgate)

    with ctx.pose({tailgate_hinge: tailgate_open, ramp_hinge: 0.0}):
        ctx.expect_overlap(
            tailgate,
            bed,
            axes="y",
            min_overlap=1.45,
            name="tailgate stays laterally aligned with the pickup bed",
        )
        open_tailgate_aabb = ctx.part_world_aabb(tailgate)
        stowed_ramp_aabb = ctx.part_world_aabb(ramp_panel)

    with ctx.pose({tailgate_hinge: tailgate_open, ramp_hinge: ramp_open}):
        ctx.expect_within(
            ramp_panel,
            tailgate,
            axes="y",
            margin=0.015,
            name="deployed ramp remains centered within tailgate width",
        )
        deployed_ramp_aabb = ctx.part_world_aabb(ramp_panel)

    closed_tailgate_center = aabb_center(closed_tailgate_aabb)
    open_tailgate_center = aabb_center(open_tailgate_aabb)
    stowed_ramp_center = aabb_center(stowed_ramp_aabb)
    deployed_ramp_center = aabb_center(deployed_ramp_aabb)

    ctx.check(
        "tailgate rotates down from the bed opening",
        closed_tailgate_center is not None
        and open_tailgate_center is not None
        and open_tailgate_center[0] < closed_tailgate_center[0] - 0.20
        and open_tailgate_center[2] < closed_tailgate_center[2] - 0.20,
        details=f"closed_center={closed_tailgate_center}, open_center={open_tailgate_center}",
    )
    ctx.check(
        "ramp panel folds farther down from the open tailgate",
        stowed_ramp_center is not None
        and deployed_ramp_center is not None
        and deployed_ramp_center[0] < stowed_ramp_center[0] - 0.10
        and deployed_ramp_center[2] < stowed_ramp_center[2] - 0.16,
        details=f"stowed_center={stowed_ramp_center}, deployed_center={deployed_ramp_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
