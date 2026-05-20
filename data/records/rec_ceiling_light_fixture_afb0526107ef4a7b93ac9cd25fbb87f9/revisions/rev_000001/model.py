from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# Advanced; only use CadQuery if the native sdk is not enough to represent the shapes you want:
# import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flush_mount_gimbal_light")

    painted_metal = Material(name="painted_metal", rgba=(0.94, 0.94, 0.93, 1.0))
    satin_trim = Material(name="satin_trim", rgba=(0.86, 0.87, 0.88, 1.0))
    diffuser_white = Material(name="diffuser_white", rgba=(0.97, 0.97, 0.95, 1.0))

    model.materials.extend((painted_metal, satin_trim, diffuser_white))

    overall = 0.34
    top_plate_thickness = 0.004
    body_depth = 0.050
    wall_depth = body_depth - top_plate_thickness
    frame_width = 0.030
    clear_opening = overall - 2.0 * frame_width

    lip_width = 0.012
    lip_depth = 0.008
    lip_opening = clear_opening - 2.0 * lip_width

    wall_center_z = top_plate_thickness + wall_depth / 2.0
    lip_center_z = body_depth - lip_depth / 2.0
    pivot_z = 0.036

    base = model.part("base")
    base.visual(
        Box((overall, overall, top_plate_thickness)),
        origin=Origin(xyz=(0.0, 0.0, top_plate_thickness / 2.0)),
        material=painted_metal,
        name="ceiling_plate",
    )

    side_wall_size = (frame_width, overall, wall_depth)
    cross_wall_size = (clear_opening, frame_width, wall_depth)
    wall_offset = overall / 2.0 - frame_width / 2.0
    for idx, x_pos in enumerate((-wall_offset, wall_offset)):
        base.visual(
            Box(side_wall_size),
            origin=Origin(xyz=(x_pos, 0.0, wall_center_z)),
            material=painted_metal,
            name=f"side_wall_{idx}",
        )
    for idx, y_pos in enumerate((-wall_offset, wall_offset)):
        base.visual(
            Box(cross_wall_size),
            origin=Origin(xyz=(0.0, y_pos, wall_center_z)),
            material=painted_metal,
            name=f"cross_wall_{idx}",
        )

    lip_side_size = (lip_width, lip_opening, lip_depth)
    lip_cross_size = (lip_opening, lip_width, lip_depth)
    lip_offset = lip_opening / 2.0 + lip_width / 2.0
    for idx, x_pos in enumerate((-lip_offset, lip_offset)):
        base.visual(
            Box(lip_side_size),
            origin=Origin(xyz=(x_pos, 0.0, lip_center_z)),
            material=satin_trim,
            name=f"inner_lip_{idx}",
        )
    for idx, y_pos in enumerate((-lip_offset, lip_offset)):
        base.visual(
            Box(lip_cross_size),
            origin=Origin(xyz=(0.0, y_pos, lip_center_z)),
            material=satin_trim,
            name=f"inner_lip_{idx + 2}",
        )

    pivot_boss_length = 0.018
    pivot_boss_radius = 0.007
    pivot_boss_center_x = clear_opening / 2.0 + pivot_boss_length / 2.0 - 0.003
    for idx, x_pos in enumerate((-pivot_boss_center_x, pivot_boss_center_x)):
        base.visual(
            Cylinder(radius=pivot_boss_radius, length=pivot_boss_length),
            origin=Origin(xyz=(x_pos, 0.0, pivot_z), rpy=(0.0, 1.57079632679, 0.0)),
            material=painted_metal,
            name=f"pivot_boss_{idx}",
        )

    panel = model.part("light_panel")
    panel_outer = 0.246
    panel_core_thickness = 0.010
    panel_core_z = 0.004
    frame_band = 0.011
    diffuser_size = panel_outer - 2.0 * frame_band
    frame_face_depth = 0.004
    frame_face_z = 0.011
    pivot_stub_radius = 0.006
    pivot_stub_length = 0.016
    pivot_stub_center_x = panel_outer / 2.0 + pivot_stub_length / 2.0 - 0.002

    panel.visual(
        Box((panel_outer, panel_outer, panel_core_thickness)),
        origin=Origin(xyz=(0.0, 0.0, panel_core_z)),
        material=painted_metal,
        name="carrier",
    )
    panel.visual(
        Box((diffuser_size, diffuser_size, frame_face_depth)),
        origin=Origin(xyz=(0.0, 0.0, frame_face_z)),
        material=diffuser_white,
        name="diffuser",
    )

    frame_side_size = (frame_band, panel_outer, frame_face_depth)
    frame_cross_size = (diffuser_size, frame_band, frame_face_depth)
    frame_offset = diffuser_size / 2.0 + frame_band / 2.0
    for idx, x_pos in enumerate((-frame_offset, frame_offset)):
        panel.visual(
            Box(frame_side_size),
            origin=Origin(xyz=(x_pos, 0.0, frame_face_z)),
            material=satin_trim,
            name=f"panel_frame_{idx}",
        )
    for idx, y_pos in enumerate((-frame_offset, frame_offset)):
        panel.visual(
            Box(frame_cross_size),
            origin=Origin(xyz=(0.0, y_pos, frame_face_z)),
            material=satin_trim,
            name=f"panel_frame_{idx + 2}",
        )

    for idx, x_pos in enumerate((-pivot_stub_center_x, pivot_stub_center_x)):
        panel.visual(
            Cylinder(radius=pivot_stub_radius, length=pivot_stub_length),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
            material=satin_trim,
            name=f"pivot_stub_{idx}",
        )

    model.articulation(
        "panel_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=panel,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.0,
            lower=-0.24,
            upper=0.24,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    panel = object_model.get_part("light_panel")
    hinge = object_model.get_articulation("panel_tilt")

    limits = hinge.motion_limits
    upper = 0.0 if limits is None or limits.upper is None else limits.upper

    ctx.expect_within(
        panel,
        base,
        axes="xy",
        margin=0.0,
        name="light panel stays inside the square base footprint",
    )
    ctx.expect_gap(
        panel,
        base,
        axis="z",
        min_gap=0.024,
        max_gap=0.040,
        negative_elem="ceiling_plate",
        name="light panel hangs below the ceiling plate",
    )
    ctx.check(
        "tilt range remains shallow",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower >= -0.30
        and limits.upper <= 0.30,
        details=f"limits=({None if limits is None else limits.lower}, {upper})",
    )

    rest_aabb = ctx.part_element_world_aabb(panel, elem="diffuser")
    with ctx.pose({hinge: upper}):
        tilted_aabb = ctx.part_element_world_aabb(panel, elem="diffuser")
        ctx.expect_within(
            panel,
            base,
            axes="x",
            margin=0.0,
            name="tilted panel remains captured between the side pivots",
        )

    rest_max_z = None if rest_aabb is None else rest_aabb[1][2]
    tilted_max_z = None if tilted_aabb is None else tilted_aabb[1][2]
    ctx.check(
        "positive tilt drops one edge of the panel",
        rest_max_z is not None and tilted_max_z is not None and tilted_max_z > rest_max_z + 0.015,
        details=f"rest_max_z={rest_max_z}, tilted_max_z={tilted_max_z}",
    )

    return ctx.report()


object_model = build_object_model()
