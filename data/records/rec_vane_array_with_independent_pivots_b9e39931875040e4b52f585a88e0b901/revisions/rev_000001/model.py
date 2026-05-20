from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


VANE_COUNT = 5
OUTER_WIDTH = 0.44
OUTER_HEIGHT = 0.30
FRAME_DEPTH = 0.055
SIDE_RAIL = 0.035
TOP_RAIL = 0.032
VANE_AXIS_ZS = (0.065, 0.105, 0.145, 0.185, 0.225)


def _cylinder_x(radius: float, length: float, x_start: float) -> cq.Workplane:
    """CadQuery cylinder whose axis is local +X and starts at x_start."""
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((x_start, 0.0, 0.0))


def _rounded_vane(span: float = 0.300) -> cq.Workplane:
    """One compact louver vane with a rounded blade, end collars, and pivot lugs."""
    blade_depth = 0.010
    blade_chord = 0.034
    pin_radius = 0.0055
    pin_extension = 0.032
    pin_overlap = 0.004
    collar_radius = 0.010
    collar_length = 0.010

    body = (
        cq.Workplane("XY")
        .box(span, blade_depth, blade_chord)
        .edges("|X")
        .fillet(0.0035)
    )

    # Small rolled lips make the flat plate read as a manufactured shutter vane.
    lip_radius = 0.0025
    lip_z = blade_chord * 0.36
    lip = _cylinder_x(lip_radius, span, -span / 2.0).translate((0.0, 0.0, lip_z))
    body = body.union(lip)
    lip = _cylinder_x(lip_radius, span, -span / 2.0).translate((0.0, 0.0, -lip_z))
    body = body.union(lip)

    right_pin = _cylinder_x(pin_radius, pin_extension + pin_overlap, span / 2.0 - pin_overlap)
    left_pin = _cylinder_x(pin_radius, pin_extension + pin_overlap, -span / 2.0 - pin_extension)
    body = body.union(right_pin).union(left_pin)

    right_collar = _cylinder_x(collar_radius, collar_length, span / 2.0 - collar_length / 2.0)
    left_collar = _cylinder_x(collar_radius, collar_length, -span / 2.0 - collar_length / 2.0)
    return body.union(right_collar).union(left_collar)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_shutter_vane_module")

    frame_mat = model.material("black_powder_coat", rgba=(0.025, 0.027, 0.028, 1.0))
    support_mat = model.material("dark_bearing_blocks", rgba=(0.045, 0.047, 0.050, 1.0))
    vane_mat = model.material("satin_aluminum_vanes", rgba=(0.62, 0.66, 0.68, 1.0))
    brass_mat = model.material("brass_bushings", rgba=(0.77, 0.58, 0.25, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((OUTER_WIDTH, FRAME_DEPTH, TOP_RAIL)),
        origin=Origin(xyz=(0.0, 0.0, TOP_RAIL / 2.0)),
        material=frame_mat,
        name="bottom_rail",
    )
    frame.visual(
        Box((OUTER_WIDTH, FRAME_DEPTH, TOP_RAIL)),
        origin=Origin(xyz=(0.0, 0.0, OUTER_HEIGHT - TOP_RAIL / 2.0)),
        material=frame_mat,
        name="top_rail",
    )
    side_x = OUTER_WIDTH / 2.0 - SIDE_RAIL / 2.0
    frame.visual(
        Box((SIDE_RAIL, FRAME_DEPTH, OUTER_HEIGHT)),
        origin=Origin(xyz=(-side_x, 0.0, OUTER_HEIGHT / 2.0)),
        material=frame_mat,
        name="side_rail_0",
    )
    frame.visual(
        Box((SIDE_RAIL, FRAME_DEPTH, OUTER_HEIGHT)),
        origin=Origin(xyz=(side_x, 0.0, OUTER_HEIGHT / 2.0)),
        material=frame_mat,
        name="side_rail_1",
    )

    # A small foot under the rail makes the module read as a grounded perimeter frame.
    frame.visual(
        Box((0.090, 0.066, 0.010)),
        origin=Origin(xyz=(-0.130, 0.0, -0.005)),
        material=frame_mat,
        name="foot_0",
    )
    frame.visual(
        Box((0.090, 0.066, 0.010)),
        origin=Origin(xyz=(0.130, 0.0, -0.005)),
        material=frame_mat,
        name="foot_1",
    )

    # Each vane has its own left/right fork support.  The fork cheeks are short
    # grounded lugs attached to the side rails; the vane pin passes through the
    # clear slot between the cheeks.
    inner_x = OUTER_WIDTH / 2.0 - SIDE_RAIL
    support_x = 0.028
    support_y = 0.040
    cheek_z = 0.006
    cheek_center_offset = 0.012
    for idx, z in enumerate(VANE_AXIS_ZS):
        for side_index, sign in enumerate((-1.0, 1.0)):
            support_center_x = sign * (inner_x - support_x / 2.0 + 0.004)
            for level, z_sign in (("upper", 1.0), ("lower", -1.0)):
                frame.visual(
                    Box((support_x, support_y, cheek_z)),
                    origin=Origin(xyz=(support_center_x, 0.0, z + z_sign * cheek_center_offset)),
                    material=support_mat,
                    name=f"support_{idx}_{side_index}_{level}",
                )
            frame.visual(
                Cylinder(radius=0.0105, length=0.004),
                origin=Origin(
                    xyz=(sign * (inner_x - 0.002), -0.001, z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=brass_mat,
                name=f"bushing_{idx}_{side_index}",
            )

    vane_mesh = mesh_from_cadquery(_rounded_vane(), "rounded_shutter_vane", tolerance=0.0007)
    for idx, z in enumerate(VANE_AXIS_ZS):
        vane = model.part(f"vane_{idx}")
        vane.visual(vane_mesh, origin=Origin(), material=vane_mat, name="vane_body")
        model.articulation(
            f"frame_to_vane_{idx}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=vane,
            origin=Origin(xyz=(0.0, 0.0, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=2.5, lower=-0.95, upper=0.95),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    frame = object_model.get_part("frame")
    joints = [object_model.get_articulation(f"frame_to_vane_{idx}") for idx in range(VANE_COUNT)]
    vanes = [object_model.get_part(f"vane_{idx}") for idx in range(VANE_COUNT)]

    ctx.check("five independent vane joints", len(joints) == VANE_COUNT and len(set(joints)) == VANE_COUNT)
    for idx, joint in enumerate(joints):
        limits = joint.motion_limits
        ctx.check(
            f"vane_{idx} is independently revolute",
            joint.parent == "frame"
            and joint.child == f"vane_{idx}"
            and joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(round(v, 6) for v in joint.axis) == (1.0, 0.0, 0.0)
            and limits is not None
            and limits.lower < -0.8
            and limits.upper > 0.8,
            details=f"joint={joint}",
        )
        ctx.expect_within(
            vanes[idx],
            frame,
            axes="xz",
            margin=0.004,
            elem_a="vane_body",
            name=f"vane_{idx} sits inside perimeter frame",
        )

    rest_aabb = ctx.part_world_aabb(vanes[2])
    with ctx.pose({joints[2]: 0.80}):
        open_aabb = ctx.part_world_aabb(vanes[2])
        neighbor_aabb = ctx.part_world_aabb(vanes[1])

    def _axis_extent(aabb, axis_index: int) -> float:
        return float(aabb[1][axis_index] - aabb[0][axis_index]) if aabb is not None else 0.0

    ctx.check(
        "commanded vane opens about its own long axis",
        rest_aabb is not None
        and open_aabb is not None
        and _axis_extent(open_aabb, 1) > _axis_extent(rest_aabb, 1) + 0.010,
        details=f"rest={rest_aabb}, open={open_aabb}",
    )
    baseline_neighbor = ctx.part_world_aabb(vanes[1])
    ctx.check(
        "neighboring vane remains uncommanded",
        baseline_neighbor == neighbor_aabb,
        details=f"baseline={baseline_neighbor}, posed={neighbor_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
