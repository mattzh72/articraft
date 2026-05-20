from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _fuse(solids: list[cq.Workplane]) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _build_frame_geometry() -> tuple[cq.Workplane, cq.Workplane]:
    body = _fuse(
        [
            _box((0.26, 0.56, 0.06), (-0.02, 0.0, 0.03)),
            _box((0.12, 0.50, 0.76), (-0.03, 0.0, 0.44)),
            _box((0.05, 0.24, 0.76), (-0.095, 0.0, 0.44)),
            _box((0.04, 0.46, 0.64), (0.04, 0.0, 0.43)),
            _box((0.08, 0.03, 0.70), (-0.01, 0.235, 0.41)),
            _box((0.08, 0.03, 0.70), (-0.01, -0.235, 0.41)),
            _box((0.032, 0.014, 0.44), (0.055, 0.23, 0.43)),
            _box((0.032, 0.014, 0.44), (0.055, -0.23, 0.43)),
        ]
    )
    ways = _fuse(
        [
            _box((0.016, 0.43, 0.028), (0.066, 0.0, 0.56)),
            _box((0.016, 0.43, 0.028), (0.066, 0.0, 0.30)),
        ]
    )
    return body, ways


def _build_crosshead_geometry() -> tuple[cq.Workplane, cq.Workplane, cq.Workplane]:
    y_blocks = _fuse(
        [
            _box((0.04, 0.056, 0.062), (0.02, 0.075, 0.13)),
            _box((0.04, 0.056, 0.062), (0.02, -0.075, 0.13)),
            _box((0.04, 0.056, 0.062), (0.02, 0.075, -0.13)),
            _box((0.04, 0.056, 0.062), (0.02, -0.075, -0.13)),
        ]
    )
    body = _fuse(
        [
            _box((0.028, 0.24, 0.34), (0.052, 0.0, 0.0)),
            _box((0.082, 0.14, 0.60), (0.083, 0.0, -0.10)),
            _box((0.016, 0.11, 0.54), (0.117, 0.0, -0.08)),
            _box((0.052, 0.016, 0.52), (0.106, 0.079, -0.10)),
            _box((0.052, 0.016, 0.52), (0.106, -0.079, -0.10)),
            _box((0.034, 0.18, 0.024), (0.099, 0.0, 0.17)),
            _box((0.04, 0.17, 0.03), (0.102, 0.0, -0.355)),
        ]
    )
    z_ways = _fuse(
        [
            _box((0.014, 0.024, 0.52), (0.131, 0.044, -0.08)),
            _box((0.014, 0.024, 0.52), (0.131, -0.044, -0.08)),
        ]
    )
    return body, y_blocks, z_ways


def _build_tool_geometry() -> tuple[cq.Workplane, cq.Workplane]:
    z_blocks = _fuse(
        [
            _box((0.034, 0.038, 0.05), (0.017, 0.044, 0.07)),
            _box((0.034, 0.038, 0.05), (0.017, -0.044, 0.07)),
            _box((0.034, 0.038, 0.05), (0.017, 0.044, -0.07)),
            _box((0.034, 0.038, 0.05), (0.017, -0.044, -0.07)),
        ]
    )
    body = _fuse(
        [
            _box((0.046, 0.136, 0.26), (0.023, 0.0, 0.0)),
            _box((0.022, 0.22, 0.22), (0.057, 0.0, -0.02)),
            _box((0.028, 0.018, 0.18), (0.045, 0.101, -0.02)),
            _box((0.028, 0.018, 0.18), (0.045, -0.101, -0.02)),
            _box((0.03, 0.16, 0.04), (0.036, 0.0, 0.13)),
            _box((0.05, 0.10, 0.05), (0.058, 0.0, -0.15)),
        ]
    )
    return body, z_blocks


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_transfer_stage")

    frame_paint = model.material("frame_paint", rgba=(0.24, 0.25, 0.28, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.72, 0.75, 0.78, 1.0))
    carriage_gray = model.material("carriage_gray", rgba=(0.56, 0.58, 0.61, 1.0))
    block_dark = model.material("block_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    tool_aluminum = model.material("tool_aluminum", rgba=(0.80, 0.82, 0.85, 1.0))

    frame_body, frame_ways = _build_frame_geometry()
    crosshead_body, crosshead_y_blocks, crosshead_z_ways = _build_crosshead_geometry()
    tool_body, tool_z_blocks = _build_tool_geometry()

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(frame_body, "frame_body"),
        material=frame_paint,
        name="frame_body",
    )
    frame.visual(
        mesh_from_cadquery(frame_ways, "frame_y_ways"),
        material=machined_steel,
        name="y_ways",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.23, 0.56, 0.82)),
        mass=42.0,
        origin=Origin(xyz=(-0.038, 0.0, 0.41)),
    )

    crosshead = model.part("crosshead")
    crosshead.visual(
        mesh_from_cadquery(crosshead_body, "crosshead_body"),
        material=carriage_gray,
        name="crosshead_body",
    )
    crosshead.visual(
        mesh_from_cadquery(crosshead_y_blocks, "crosshead_y_blocks"),
        material=block_dark,
        name="y_blocks",
    )
    crosshead.visual(
        mesh_from_cadquery(crosshead_z_ways, "crosshead_z_ways"),
        material=machined_steel,
        name="z_ways",
    )
    crosshead.inertial = Inertial.from_geometry(
        Box((0.138, 0.24, 0.60)),
        mass=11.0,
        origin=Origin(xyz=(0.069, 0.0, -0.08)),
    )

    tool_plate = model.part("tool_plate")
    tool_plate.visual(
        mesh_from_cadquery(tool_body, "tool_plate_body"),
        material=tool_aluminum,
        name="tool_body",
    )
    tool_plate.visual(
        mesh_from_cadquery(tool_z_blocks, "tool_plate_z_blocks"),
        material=block_dark,
        name="z_blocks",
    )
    tool_plate.inertial = Inertial.from_geometry(
        Box((0.08, 0.22, 0.34)),
        mass=5.5,
        origin=Origin(xyz=(0.04, 0.0, -0.01)),
    )

    model.articulation(
        "frame_to_crosshead",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=crosshead,
        origin=Origin(xyz=(0.074, 0.0, 0.43)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=0.9, lower=-0.10, upper=0.10),
    )
    model.articulation(
        "crosshead_to_tool_plate",
        ArticulationType.PRISMATIC,
        parent=crosshead,
        child=tool_plate,
        origin=Origin(xyz=(0.138, 0.0, -0.08)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.7, lower=0.0, upper=0.155),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    crosshead = object_model.get_part("crosshead")
    tool_plate = object_model.get_part("tool_plate")

    y_slide = object_model.get_articulation("frame_to_crosshead")
    z_slide = object_model.get_articulation("crosshead_to_tool_plate")

    frame_ways = frame.get_visual("y_ways")
    crosshead_y_blocks = crosshead.get_visual("y_blocks")
    crosshead_z_ways = crosshead.get_visual("z_ways")
    tool_z_blocks = tool_plate.get_visual("z_blocks")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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

    y_limits = y_slide.motion_limits
    z_limits = z_slide.motion_limits
    ctx.check(
        "y_axis_is_lateral_prismatic",
        tuple(y_slide.axis) == (0.0, 1.0, 0.0)
        and y_limits is not None
        and y_limits.lower == -0.10
        and y_limits.upper == 0.10,
        details=f"axis={y_slide.axis}, limits={y_limits}",
    )
    ctx.check(
        "z_axis_descends_from_crosshead",
        tuple(z_slide.axis) == (0.0, 0.0, -1.0)
        and z_limits is not None
        and z_limits.lower == 0.0
        and z_limits.upper == 0.155,
        details=f"axis={z_slide.axis}, limits={z_limits}",
    )

    with ctx.pose({y_slide: 0.0, z_slide: 0.0}):
        ctx.expect_contact(
            crosshead,
            frame,
            elem_a=crosshead_y_blocks,
            elem_b=frame_ways,
            name="crosshead_blocks_contact_frame_ways",
        )
        ctx.expect_gap(
            crosshead,
            frame,
            axis="x",
            positive_elem=crosshead_y_blocks,
            negative_elem=frame_ways,
            max_gap=0.001,
            max_penetration=0.0,
            name="y_slide_runs_without_penetrating_frame",
        )
        ctx.expect_overlap(
            crosshead,
            frame,
            axes="yz",
            elem_a=crosshead_y_blocks,
            elem_b=frame_ways,
            min_overlap=0.10,
            name="crosshead_blocks_stay_engaged_on_y_ways",
        )
        ctx.expect_contact(
            tool_plate,
            crosshead,
            elem_a=tool_z_blocks,
            elem_b=crosshead_z_ways,
            name="tool_blocks_contact_crosshead_z_ways",
        )
        ctx.expect_gap(
            tool_plate,
            crosshead,
            axis="x",
            positive_elem=tool_z_blocks,
            negative_elem=crosshead_z_ways,
            max_gap=0.001,
            max_penetration=0.0,
            name="z_slide_runs_without_penetrating_crosshead",
        )
        ctx.expect_overlap(
            tool_plate,
            crosshead,
            axes="yz",
            elem_a=tool_z_blocks,
            elem_b=crosshead_z_ways,
            min_overlap=0.10,
            name="tool_blocks_stay_engaged_on_z_ways",
        )

    for y_pose, label in ((-0.10, "left"), (0.10, "right")):
        with ctx.pose({y_slide: y_pose, z_slide: 0.0}):
            ctx.expect_gap(
                crosshead,
                frame,
                axis="x",
                positive_elem=crosshead_y_blocks,
                negative_elem=frame_ways,
                max_gap=0.001,
                max_penetration=0.0,
                name=f"y_slide_clearance_at_{label}_travel",
            )
            ctx.expect_within(
                crosshead,
                frame,
                axes="y",
                margin=0.0,
                inner_elem=crosshead_y_blocks,
                outer_elem=frame_ways,
                name=f"crosshead_blocks_remain_on_y_ways_at_{label}_travel",
            )

    for z_pose, label in ((0.0, "upper"), (0.155, "lower")):
        with ctx.pose({y_slide: 0.0, z_slide: z_pose}):
            ctx.expect_gap(
                tool_plate,
                crosshead,
                axis="x",
                positive_elem=tool_z_blocks,
                negative_elem=crosshead_z_ways,
                max_gap=0.001,
                max_penetration=0.0,
                name=f"z_slide_clearance_at_{label}_travel",
            )
            ctx.expect_within(
                tool_plate,
                crosshead,
                axes="z",
                margin=0.0,
                inner_elem=tool_z_blocks,
                outer_elem=crosshead_z_ways,
                name=f"tool_blocks_remain_on_z_ways_at_{label}_travel",
            )

    with ctx.pose({y_slide: 0.10, z_slide: 0.155}):
        ctx.expect_within(
            crosshead,
            frame,
            axes="y",
            margin=0.0,
            inner_elem=crosshead_y_blocks,
            outer_elem=frame_ways,
            name="crosshead_engagement_at_combined_extreme_pose",
        )
        ctx.expect_within(
            tool_plate,
            crosshead,
            axes="z",
            margin=0.0,
            inner_elem=tool_z_blocks,
            outer_elem=crosshead_z_ways,
            name="tool_engagement_at_combined_extreme_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
