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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


INPUT_PIVOT_B = (0.118, 0.052)
COUPLER_TO_OUTPUT = (0.082, -0.010)
OUTPUT_TIP = (0.226, 0.020)

INPUT_THICKNESS = 0.006
COUPLER_THICKNESS = 0.005
OUTPUT_THICKNESS = 0.006

INPUT_Z = 0.0
COUPLER_Z = 0.007
OUTPUT_Z = 0.0

BASE_PIN_D = 0.010
LINK_PIN_D = 0.008
BASE_HOLE_D = 0.0106
LINK_HOLE_D = 0.0086


def _disc(x: float, y: float, radius: float, thickness: float, z_center: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .center(x, y)
        .circle(radius)
        .extrude(thickness)
        .translate((0.0, 0.0, z_center - thickness / 2.0))
    )


def _box(
    x: float,
    y: float,
    z: float,
    sx: float,
    sy: float,
    sz: float,
    *,
    angle_deg: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(sx, sy, sz)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        .translate((x, y, z))
    )


def _cylinder_cut(x: float, y: float, diameter: float, z_min: float, z_max: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .center(x, y)
        .circle(diameter / 2.0)
        .extrude(z_max - z_min)
        .translate((0.0, 0.0, z_min))
    )


def _slot_cut(
    x: float,
    y: float,
    length: float,
    diameter: float,
    z_center: float,
    z_span: float,
    *,
    angle_deg: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .slot2D(length, diameter)
        .extrude(z_span)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        .translate((x, y, z_center - z_span / 2.0))
    )


def _fuse(*shapes: cq.Workplane) -> cq.Workplane:
    fused = None
    for shape in shapes:
        if fused is None:
            fused = shape
        else:
            fused = fused.union(shape)
    assert fused is not None
    return fused


def make_base_body() -> cq.Workplane:
    foot = _box(-0.030, 0.000, -0.009, 0.098, 0.052, 0.010)
    lug = _disc(0.000, 0.000, 0.020, 0.012, -0.009)
    back_pad = _disc(-0.028, 0.000, 0.017, 0.010, -0.009)
    rib = _box(-0.013, 0.000, -0.009, 0.034, 0.030, 0.010, angle_deg=12.0)
    body = _fuse(foot, lug, back_pad, rib)
    body = body.cut(_cylinder_cut(-0.050, 0.014, 0.008, -0.020, 0.002))
    body = body.cut(_cylinder_cut(-0.050, -0.014, 0.008, -0.020, 0.002))
    return body


def make_base_pin() -> cq.Workplane:
    root_boss = _disc(0.000, 0.000, 0.011, 0.003, -0.0045)
    shaft = _disc(0.000, 0.000, BASE_PIN_D / 2.0, 0.006, 0.000)
    top_washer = _disc(0.000, 0.000, 0.0085, 0.0013, 0.00365)
    head = _disc(0.000, 0.000, 0.0062, 0.0024, 0.0055)
    pin = _fuse(root_boss, shaft, top_washer, head)
    return pin


def make_input_body() -> cq.Workplane:
    angle_deg = math.degrees(math.atan2(INPUT_PIVOT_B[1], INPUT_PIVOT_B[0]))
    body = _fuse(
        _disc(0.000, 0.000, 0.018, INPUT_THICKNESS, INPUT_Z),
        _disc(0.038, 0.017, 0.013, INPUT_THICKNESS, INPUT_Z),
        _disc(0.079, 0.041, 0.011, INPUT_THICKNESS, INPUT_Z),
        _disc(INPUT_PIVOT_B[0], INPUT_PIVOT_B[1], 0.015, INPUT_THICKNESS, INPUT_Z),
        _box(0.060, 0.026, INPUT_Z, 0.108, 0.018, 0.0048, angle_deg=angle_deg),
    )
    body = body.cut(_cylinder_cut(0.000, 0.000, BASE_HOLE_D, -0.012, 0.012))
    body = body.cut(_slot_cut(0.061, 0.027, 0.064, 0.010, 0.000, 0.016, angle_deg=angle_deg))
    return body


def make_input_pin() -> cq.Workplane:
    x_b, y_b = INPUT_PIVOT_B
    collar = _disc(x_b, y_b, 0.0078, 0.0015, 0.00375)
    shaft = _disc(x_b, y_b, LINK_PIN_D / 2.0, 0.0050, 0.0070)
    top_washer = _disc(x_b, y_b, 0.0072, 0.0012, 0.0101)
    head = _disc(x_b, y_b, 0.0056, 0.0021, 0.01175)
    pin = _fuse(collar, shaft, top_washer, head)
    return pin


def make_coupler_body() -> cq.Workplane:
    x_c, y_c = COUPLER_TO_OUTPUT
    angle_deg = math.degrees(math.atan2(y_c, x_c))
    body = _fuse(
        _disc(0.000, 0.000, 0.014, COUPLER_THICKNESS, COUPLER_Z),
        _disc(0.041, -0.004, 0.010, COUPLER_THICKNESS, COUPLER_Z),
        _disc(x_c, y_c, 0.013, COUPLER_THICKNESS, COUPLER_Z),
        _box(0.041, -0.005, COUPLER_Z, 0.074, 0.014, 0.0045, angle_deg=angle_deg),
    )
    body = body.cut(_cylinder_cut(0.000, 0.000, LINK_HOLE_D, -0.004, 0.018))
    body = body.cut(_slot_cut(0.040, -0.005, 0.028, 0.0075, COUPLER_Z, 0.010, angle_deg=angle_deg))
    return body


def make_coupler_pin() -> cq.Workplane:
    x_c, y_c = COUPLER_TO_OUTPUT
    collar = _disc(x_c, y_c, 0.0078, 0.0015, 0.00375)
    shaft = _disc(x_c, y_c, LINK_PIN_D / 2.0, 0.0060, 0.0000)
    bottom_washer = _disc(x_c, y_c, 0.0072, 0.0012, -0.0036)
    head = _disc(x_c, y_c, 0.0056, 0.0021, -0.00525)
    pin = _fuse(collar, shaft, bottom_washer, head)
    return pin


def make_output_body() -> cq.Workplane:
    angle_deg = math.degrees(math.atan2(OUTPUT_TIP[1], OUTPUT_TIP[0]))
    body = _fuse(
        _disc(0.000, 0.000, 0.016, OUTPUT_THICKNESS, OUTPUT_Z),
        _disc(0.070, 0.003, 0.012, OUTPUT_THICKNESS, OUTPUT_Z),
        _disc(0.146, 0.010, 0.010, OUTPUT_THICKNESS, OUTPUT_Z),
        _disc(0.198, 0.016, 0.007, OUTPUT_THICKNESS, OUTPUT_Z),
        _disc(OUTPUT_TIP[0], OUTPUT_TIP[1], 0.0055, OUTPUT_THICKNESS, OUTPUT_Z),
        _box(0.103, 0.008, OUTPUT_Z, 0.176, 0.016, 0.0048, angle_deg=angle_deg),
        _box(0.204, 0.018, OUTPUT_Z, 0.050, 0.013, 0.0048, angle_deg=angle_deg),
    )
    body = body.cut(_cylinder_cut(0.000, 0.000, LINK_HOLE_D, -0.012, 0.012))
    body = body.cut(_slot_cut(0.100, 0.008, 0.102, 0.010, 0.000, 0.016, angle_deg=angle_deg))
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_lever_linkage")

    base_body_mat = model.material("base_body_mat", rgba=(0.24, 0.26, 0.29, 1.0))
    link_body_mat = model.material("link_body_mat", rgba=(0.47, 0.49, 0.52, 1.0))
    coupler_body_mat = model.material("coupler_body_mat", rgba=(0.40, 0.42, 0.45, 1.0))
    pin_mat = model.material("pin_mat", rgba=(0.75, 0.77, 0.80, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_base_body(), "base_body"),
        material=base_body_mat,
        name="body",
    )
    base.visual(
        mesh_from_cadquery(make_base_pin(), "base_pin_stack"),
        material=pin_mat,
        name="pin_stack",
    )
    base.inertial = None

    input_lever = model.part("input_lever")
    input_lever.visual(
        mesh_from_cadquery(make_input_body(), "input_lever_body"),
        material=link_body_mat,
        name="body",
    )
    input_lever.visual(
        mesh_from_cadquery(make_input_pin(), "input_pin_stack"),
        material=pin_mat,
        name="pin_stack",
    )

    coupler = model.part("coupler")
    coupler.visual(
        mesh_from_cadquery(make_coupler_body(), "coupler_body"),
        material=coupler_body_mat,
        name="body",
    )
    coupler.visual(
        mesh_from_cadquery(make_coupler_pin(), "coupler_pin_stack"),
        material=pin_mat,
        name="pin_stack",
    )

    output_lever = model.part("output_lever")
    output_lever.visual(
        mesh_from_cadquery(make_output_body(), "output_lever_body"),
        material=link_body_mat,
        name="body",
    )

    model.articulation(
        "base_to_input",
        ArticulationType.REVOLUTE,
        parent=base,
        child=input_lever,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.2, lower=-0.60, upper=0.85),
    )
    model.articulation(
        "input_to_coupler",
        ArticulationType.REVOLUTE,
        parent=input_lever,
        child=coupler,
        origin=Origin(xyz=(INPUT_PIVOT_B[0], INPUT_PIVOT_B[1], 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.4, lower=-0.95, upper=0.80),
    )
    model.articulation(
        "coupler_to_output",
        ArticulationType.REVOLUTE,
        parent=coupler,
        child=output_lever,
        origin=Origin(xyz=(COUPLER_TO_OUTPUT[0], COUPLER_TO_OUTPUT[1], 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.2, lower=-0.90, upper=1.00),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    input_lever = object_model.get_part("input_lever")
    coupler = object_model.get_part("coupler")
    output_lever = object_model.get_part("output_lever")

    base_to_input = object_model.get_articulation("base_to_input")
    input_to_coupler = object_model.get_articulation("input_to_coupler")
    coupler_to_output = object_model.get_articulation("coupler_to_output")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        base,
        input_lever,
        elem_a="pin_stack",
        elem_b="body",
        reason="grounded pivot pin occupies the bored input-lever joint region",
    )
    ctx.allow_overlap(
        coupler,
        output_lever,
        elem_a="pin_stack",
        elem_b="body",
        reason="output pivot pin occupies the bored output-lever joint region",
    )

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

    ctx.check(
        "three_parallel_revolute_joints",
        (
            base_to_input.articulation_type == ArticulationType.REVOLUTE
            and input_to_coupler.articulation_type == ArticulationType.REVOLUTE
            and coupler_to_output.articulation_type == ArticulationType.REVOLUTE
            and tuple(base_to_input.axis) == (0.0, 0.0, 1.0)
            and tuple(input_to_coupler.axis) == (0.0, 0.0, 1.0)
            and tuple(coupler_to_output.axis) == (0.0, 0.0, 1.0)
        ),
        "all three linkage articulations should be revolute joints about parallel z axes",
    )

    ctx.expect_contact(base, input_lever, elem_a="pin_stack", elem_b="body", name="base_pin_clamps_input")
    ctx.expect_contact(
        input_lever,
        coupler,
        elem_a="pin_stack",
        elem_b="body",
        name="input_pin_clamps_coupler",
    )
    ctx.expect_contact(
        coupler,
        output_lever,
        elem_a="pin_stack",
        elem_b="body",
        name="coupler_pin_clamps_output",
    )

    ctx.expect_gap(
        coupler,
        input_lever,
        axis="z",
        min_gap=0.001,
        positive_elem="body",
        negative_elem="body",
        name="coupler_plate_clears_input_plate",
    )
    ctx.expect_gap(
        coupler,
        output_lever,
        axis="z",
        min_gap=0.001,
        positive_elem="body",
        negative_elem="body",
        name="coupler_plate_clears_output_plate",
    )

    base_aabb = ctx.part_world_aabb(base)
    input_aabb = ctx.part_world_aabb(input_lever)
    coupler_aabb = ctx.part_world_aabb(coupler)
    output_aabb = ctx.part_world_aabb(output_lever)

    def _dx(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float:
        if aabb is None:
            return -1.0
        return aabb[1][0] - aabb[0][0]

    ctx.check(
        "output_lever_is_dominant_span",
        (
            base_aabb is not None
            and input_aabb is not None
            and coupler_aabb is not None
            and output_aabb is not None
            and _dx(output_aabb) > _dx(input_aabb) + 0.060
            and _dx(output_aabb) > _dx(coupler_aabb) + 0.120
        ),
        "the output lever should read as the longest member ending in the narrow tab",
    )

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=28,
        ignore_adjacent=True,
        ignore_fixed=True,
        name="sampled_pose_clearance",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
