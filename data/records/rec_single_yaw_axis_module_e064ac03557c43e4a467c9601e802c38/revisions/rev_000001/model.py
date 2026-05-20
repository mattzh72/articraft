from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


def make_lower_saddle() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(0.130, 0.092, 0.012, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.005)
    )
    base = (
        base.faces(">Z")
        .workplane(centerOption="CenterOfBoundBox")
        .slot2D(0.082, 0.028, angle=0)
        .cutBlind(-0.004)
    )

    cheek = (
        cq.Workplane("XY")
        .box(0.082, 0.014, 0.040, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
    )
    left_cheek = cheek.translate((0.0, 0.033, 0.012))
    right_cheek = cheek.translate((0.0, -0.033, 0.012))

    bridge = (
        cq.Workplane("XY")
        .box(0.060, 0.078, 0.012, centered=(True, True, False))
        .edges("|X")
        .fillet(0.002)
        .translate((0.0, 0.0, 0.050))
    )

    body = base.union(left_cheek).union(right_cheek).union(bridge)

    bridge_hole = (
        cq.Workplane("XY")
        .circle(0.0135)
        .extrude(0.020)
        .translate((0.0, 0.0, 0.046))
    )
    arch_cut = (
        cq.Workplane("YZ")
        .center(0.0, 0.046)
        .circle(0.023)
        .extrude(0.090, both=True)
    )
    window_cut = (
        cq.Workplane("XY")
        .box(0.056, 0.052, 0.038, centered=(True, True, False))
        .translate((0.0, 0.0, 0.012))
    )

    return body.cut(bridge_hole).cut(arch_cut).cut(window_cut)


def make_rotor_head() -> cq.Workplane:
    lower_head = cq.Workplane("XY").circle(0.022).extrude(0.008).translate((0.0, 0.0, -0.018))
    neck = cq.Workplane("XY").circle(0.0105).extrude(0.037).translate((0.0, 0.0, -0.014))
    upper_collar = cq.Workplane("XY").circle(0.020).extrude(0.006).translate((0.0, 0.0, 0.022))
    upper_mount = (
        cq.Workplane("XY")
        .box(0.032, 0.024, 0.008, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.002)
        .translate((0.0, 0.0, 0.022))
    )
    return lower_head.union(neck).union(upper_collar).union(upper_mount)


def make_top_plate() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.112, 0.038, 0.008, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
        .translate((0.0, 0.0, 0.030))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_capped_yaw_module")

    saddle_color = model.material("saddle_graphite", rgba=(0.20, 0.22, 0.24, 1.0))
    head_color = model.material("head_metal", rgba=(0.62, 0.65, 0.69, 1.0))
    plate_color = model.material("plate_gunmetal", rgba=(0.42, 0.45, 0.49, 1.0))

    lower_saddle = model.part("lower_saddle")
    lower_saddle.visual(
        mesh_from_cadquery(make_lower_saddle(), "lower_saddle"),
        origin=Origin(),
        material=saddle_color,
        name="saddle_body",
    )
    lower_saddle.inertial = Inertial.from_geometry(
        Box((0.130, 0.092, 0.062)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
    )

    rotating_top = model.part("rotating_top")
    rotating_top.visual(
        mesh_from_cadquery(make_rotor_head(), "rotor_head"),
        origin=Origin(),
        material=head_color,
        name="rotor_head",
    )
    rotating_top.visual(
        mesh_from_cadquery(make_top_plate(), "top_plate"),
        origin=Origin(),
        material=plate_color,
        name="top_plate",
    )
    rotating_top.inertial = Inertial.from_geometry(
        Box((0.112, 0.038, 0.056)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    model.articulation(
        "lower_saddle_to_rotating_top",
        ArticulationType.REVOLUTE,
        parent=lower_saddle,
        child=rotating_top,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=-2.2,
            upper=2.2,
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

    lower_saddle = object_model.get_part("lower_saddle")
    rotating_top = object_model.get_part("rotating_top")
    yaw_joint = object_model.get_articulation("lower_saddle_to_rotating_top")

    limits = yaw_joint.motion_limits
    ctx.check(
        "yaw joint is a bounded vertical revolute axis",
        yaw_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in yaw_joint.axis) == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper,
        details=f"type={yaw_joint.articulation_type}, axis={yaw_joint.axis}, limits={limits}",
    )

    ctx.allow_overlap(
        lower_saddle,
        rotating_top,
        elem_a="saddle_body",
        elem_b="rotor_head",
        reason="The rotating head is intentionally captured in the bridge cap as a seated bearing fit.",
    )

    ctx.expect_origin_distance(
        lower_saddle,
        rotating_top,
        axes="xy",
        max_dist=1e-6,
        name="rotating head stays centered over the saddle",
    )

    with ctx.pose({yaw_joint: 0.0}):
        ctx.expect_contact(
            rotating_top,
            lower_saddle,
            elem_a="rotor_head",
            name="rotor head stays seated in the bridge cap",
        )
        ctx.expect_gap(
            rotating_top,
            lower_saddle,
            axis="z",
            min_gap=0.007,
            positive_elem="top_plate",
            name="top plate clears the bridge at rest",
        )
        rest_aabb = ctx.part_element_world_aabb(rotating_top, elem="top_plate")

    with ctx.pose({yaw_joint: math.pi / 2.0}):
        ctx.expect_gap(
            rotating_top,
            lower_saddle,
            axis="z",
            min_gap=0.007,
            positive_elem="top_plate",
            name="top plate clears the bridge through a quarter turn",
        )
        quarter_turn_aabb = ctx.part_element_world_aabb(rotating_top, elem="top_plate")

    def xy_size(aabb):
        if aabb is None:
            return None
        (xmin, ymin, _), (xmax, ymax, _) = aabb
        return (xmax - xmin, ymax - ymin)

    rest_xy = xy_size(rest_aabb)
    quarter_xy = xy_size(quarter_turn_aabb)
    ctx.check(
        "top plate yaws visibly in plan view",
        rest_xy is not None
        and quarter_xy is not None
        and rest_xy[0] > rest_xy[1] + 0.045
        and quarter_xy[1] > quarter_xy[0] + 0.045,
        details=f"rest_xy={rest_xy}, quarter_xy={quarter_xy}",
    )

    ctx.expect_overlap(
        rotating_top,
        lower_saddle,
        axes="xy",
        elem_a="rotor_head",
        min_overlap=0.030,
        name="rotor head remains nested within the saddle footprint",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
