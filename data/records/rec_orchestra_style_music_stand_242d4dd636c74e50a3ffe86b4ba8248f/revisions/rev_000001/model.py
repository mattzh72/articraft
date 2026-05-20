from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

# Advanced; only use CadQuery if the native sdk is not enough to represent the shapes you want:
# import cadquery as cq
# from sdk import mesh_from_cadquery


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rehearsal_music_stand")

    powder_black = model.material("powder_black", rgba=(0.15, 0.15, 0.16, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    def tube_shell(outer_radius: float, inner_radius: float, length: float, name: str):
        return mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [(outer_radius, 0.0), (outer_radius, length)],
                [(inner_radius, 0.0), (inner_radius, length)],
                segments=48,
                start_cap="flat",
                end_cap="flat",
            ),
            name,
        )

    base = model.part("base")
    base.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(
                rounded_rect_profile(0.420, 0.300, 0.022),
                0.012,
            ),
            "base_plate",
        ),
        material=powder_black,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.040, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=powder_black,
        name="base_boss",
    )
    base.visual(
        tube_shell(0.022, 0.0185, 0.480, "lower_tube"),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=powder_black,
        name="lower_tube",
    )
    base.visual(
        tube_shell(0.029, 0.0215, 0.077, "sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.415)),
        material=powder_black,
        name="sleeve",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(0.028, 0.0, 0.455), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="clamp_stem",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.046, 0.0, 0.455), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="clamp_knob",
    )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.0165, length=0.920),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=graphite,
        name="inner_post",
    )
    mast.visual(
        Cylinder(radius=0.019, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.529)),
        material=powder_black,
        name="head_collar",
    )
    mast.visual(
        Box((0.050, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, -0.006, 0.546)),
        material=powder_black,
        name="yoke_pad",
    )
    mast.visual(
        Box((0.084, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, -0.020, 0.547)),
        material=powder_black,
        name="yoke_bridge",
    )
    mast.visual(
        Box((0.012, 0.044, 0.082)),
        origin=Origin(xyz=(-0.035, -0.009, 0.581)),
        material=powder_black,
        name="yoke_cheek_0",
    )
    mast.visual(
        Box((0.012, 0.044, 0.082)),
        origin=Origin(xyz=(0.035, -0.009, 0.581)),
        material=powder_black,
        name="yoke_cheek_1",
    )
    mast.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(-0.046, -0.004, 0.594), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="pivot_cap_0",
    )
    mast.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.046, -0.004, 0.594), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="pivot_cap_1",
    )

    model.articulation(
        "base_to_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.492)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=75.0,
            velocity=0.18,
            lower=0.0,
            upper=0.260,
        ),
    )

    desk = model.part("desk")
    panel_roll = math.pi / 2.0 + 0.30
    panel_center = (0.0, -0.010, 0.180)

    def desk_offset(x: float, vertical: float, normal: float) -> tuple[float, float, float]:
        cy = math.cos(panel_roll)
        sy = math.sin(panel_roll)
        return (
            panel_center[0] + x,
            panel_center[1] + vertical * cy - normal * sy,
            panel_center[2] + vertical * sy + normal * cy,
        )

    desk.visual(
        mesh_from_geometry(
            PerforatedPanelGeometry(
                (0.500, 0.340),
                0.0014,
                hole_diameter=0.008,
                pitch=(0.016, 0.016),
                frame=0.014,
                corner_radius=0.014,
                stagger=True,
            ),
            "desk_panel",
        ),
        origin=Origin(xyz=panel_center, rpy=(panel_roll, 0.0, 0.0)),
        material=graphite,
        name="desk_panel",
    )
    desk.visual(
        Box((0.016, 0.340, 0.018)),
        origin=Origin(xyz=desk_offset(-0.249, 0.0, 0.009), rpy=(panel_roll, 0.0, 0.0)),
        material=powder_black,
        name="side_flange_0",
    )
    desk.visual(
        Box((0.016, 0.340, 0.018)),
        origin=Origin(xyz=desk_offset(0.249, 0.0, 0.009), rpy=(panel_roll, 0.0, 0.0)),
        material=powder_black,
        name="side_flange_1",
    )
    desk.visual(
        Box((0.472, 0.014, 0.018)),
        origin=Origin(xyz=desk_offset(0.0, 0.171, 0.009), rpy=(panel_roll, 0.0, 0.0)),
        material=powder_black,
        name="top_rim",
    )
    desk.visual(
        Box((0.472, 0.032, 0.008)),
        origin=Origin(
            xyz=desk_offset(0.0, -0.164, -0.015),
            rpy=(0.82, 0.0, 0.0),
        ),
        material=powder_black,
        name="bottom_shelf",
    )
    desk.visual(
        Box((0.472, 0.008, 0.022)),
        origin=Origin(
            xyz=desk_offset(0.0, -0.158, -0.030),
            rpy=(0.82, 0.0, 0.0),
        ),
        material=powder_black,
        name="page_fence",
    )
    desk.visual(
        Box((0.472, 0.026, 0.020)),
        origin=Origin(
            xyz=desk_offset(0.0, -0.138, 0.008),
            rpy=(panel_roll, 0.0, 0.0),
        ),
        material=powder_black,
        name="lower_back_rail",
    )
    desk.visual(
        Box((0.096, 0.240, 0.014)),
        origin=Origin(
            xyz=desk_offset(0.0, -0.055, 0.007),
            rpy=(panel_roll, 0.0, 0.0),
        ),
        material=powder_black,
        name="center_spine",
    )
    desk.visual(
        Box((0.054, 0.018, 0.044)),
        origin=Origin(xyz=(0.0, -0.020, 0.020)),
        material=powder_black,
        name="pivot_block",
    )
    desk.visual(
        Box((0.060, 0.020, 0.110)),
        origin=Origin(xyz=(0.0, -0.018, 0.074)),
        material=powder_black,
        name="pivot_web",
    )

    model.articulation(
        "mast_to_desk",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=desk,
        origin=Origin(xyz=(0.0, 0.0, 0.594)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=-0.35,
            upper=0.50,
        ),
    )

    clip_specs = (
        ("page_clip_0", -0.148),
        ("page_clip_1", 0.148),
    )
    for clip_name, clip_x in clip_specs:
        clip = model.part(clip_name)
        clip.visual(
            Cylinder(radius=0.005, length=0.014),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=powder_black,
            name="pivot_barrel",
        )
        clip.visual(
            Box((0.020, 0.008, 0.010)),
            origin=Origin(xyz=(0.0, 0.008, -0.004)),
            material=powder_black,
            name="clip_neck",
        )
        clip.visual(
            Box((0.018, 0.003, 0.040)),
            origin=Origin(xyz=(0.0, 0.012, -0.024)),
            material=powder_black,
            name="clip_arm",
        )
        clip.visual(
            Box((0.018, 0.012, 0.004)),
            origin=Origin(xyz=(0.0, 0.014, -0.046)),
            material=dark_rubber,
            name="clip_pad",
        )

        model.articulation(
            f"desk_to_{clip_name}",
            ArticulationType.REVOLUTE,
            parent=desk,
            child=clip,
            origin=Origin(xyz=desk_offset(clip_x, 0.174, -0.004)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=2.5,
                lower=-0.25,
                upper=0.65,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    desk = object_model.get_part("desk")
    page_clip_0 = object_model.get_part("page_clip_0")
    page_clip_1 = object_model.get_part("page_clip_1")
    mast_slide = object_model.get_articulation("base_to_mast")
    desk_tilt = object_model.get_articulation("mast_to_desk")
    clip_joint_0 = object_model.get_articulation("desk_to_page_clip_0")
    clip_joint_1 = object_model.get_articulation("desk_to_page_clip_1")

    slide_limits = mast_slide.motion_limits
    tilt_limits = desk_tilt.motion_limits

    if slide_limits is not None and slide_limits.lower is not None and slide_limits.upper is not None:
        with ctx.pose({mast_slide: slide_limits.lower}):
            ctx.expect_within(
                mast,
                base,
                axes="xy",
                inner_elem="inner_post",
                outer_elem="lower_tube",
                margin=0.006,
                name="mast stays centered in lower tube at minimum height",
            )
            ctx.expect_overlap(
                mast,
                base,
                axes="z",
                elem_a="inner_post",
                elem_b="lower_tube",
                min_overlap=0.360,
                name="mast remains deeply inserted when collapsed",
            )
            rest_mast_pos = ctx.part_world_position(mast)

        with ctx.pose({mast_slide: slide_limits.upper}):
            ctx.expect_within(
                mast,
                base,
                axes="xy",
                inner_elem="inner_post",
                outer_elem="lower_tube",
                margin=0.006,
                name="mast stays centered in lower tube at full extension",
            )
            ctx.expect_overlap(
                mast,
                base,
                axes="z",
                elem_a="inner_post",
                elem_b="lower_tube",
                min_overlap=0.100,
                name="mast retains insertion at full extension",
            )
            extended_mast_pos = ctx.part_world_position(mast)

        ctx.check(
            "mast extends upward",
            rest_mast_pos is not None
            and extended_mast_pos is not None
            and extended_mast_pos[2] > rest_mast_pos[2] + 0.20,
            details=f"rest={rest_mast_pos}, extended={extended_mast_pos}",
        )

    if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
        with ctx.pose({desk_tilt: tilt_limits.lower}):
            low_aabb = ctx.part_element_world_aabb(desk, elem="top_rim")
        with ctx.pose({desk_tilt: tilt_limits.upper}):
            high_aabb = ctx.part_element_world_aabb(desk, elem="top_rim")

        def center_y(aabb):
            if aabb is None:
                return None
            return (aabb[0][1] + aabb[1][1]) / 2.0

        low_y = center_y(low_aabb)
        high_y = center_y(high_aabb)
        ctx.check(
            "desk tilts farther back at the upper limit",
            low_y is not None and high_y is not None and high_y < low_y - 0.025,
            details=f"lower_top_rim_y={low_y}, upper_top_rim_y={high_y}",
        )

    def center_from_aabb(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3))

    for clip_part, clip_joint, label in (
        (page_clip_0, clip_joint_0, "page clip 0"),
        (page_clip_1, clip_joint_1, "page clip 1"),
    ):
        clip_limits = clip_joint.motion_limits
        if clip_limits is None or clip_limits.lower is None or clip_limits.upper is None:
            continue

        with ctx.pose({clip_joint: clip_limits.lower}):
            closed_pad = center_from_aabb(ctx.part_element_world_aabb(clip_part, elem="clip_pad"))
        with ctx.pose({clip_joint: clip_limits.upper}):
            opened_pad = center_from_aabb(ctx.part_element_world_aabb(clip_part, elem="clip_pad"))

        ctx.check(
            f"{label} lifts away from the desk",
            closed_pad is not None
            and opened_pad is not None
            and opened_pad[1] > closed_pad[1] + 0.010,
            details=f"closed_pad={closed_pad}, opened_pad={opened_pad}",
        )

    return ctx.report()


object_model = build_object_model()
