from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _cylinder_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    """CadQuery cylinder whose local axis is world X."""
    cx, cy, cz = center
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((cx - length / 2.0, cy, cz))
    )


def _cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    """CadQuery cylinder whose local axis is world Y."""
    cx, cy, cz = center
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((cx, cy - length / 2.0, cz))
    )


def _binocular_side(sign: float) -> cq.Workplane:
    """One porro half: close rear ocular and wider, lower objective housing."""
    rear_y = sign * 0.041
    front_y = sign * 0.081

    rear_tube = _cylinder_x(0.0205, 0.092, (-0.066, rear_y, 0.024))
    objective_tube = _cylinder_x(0.0365, 0.086, (0.057, front_y, -0.005))

    prism_block = (
        cq.Workplane("XY")
        .box(0.120, 0.052, 0.060)
        .translate((-0.004, sign * 0.056, 0.012))
    )
    shoulder = (
        cq.Workplane("XY")
        .box(0.070, 0.045, 0.046)
        .translate((0.025, sign * 0.066, 0.012))
    )

    # A short bridge cheek reaches inward toward the central hinge without
    # interpenetrating the knurled hinge barrel.
    hinge_cheek = (
        cq.Workplane("XY")
        .box(0.112, 0.028, 0.024)
        .translate((-0.004, sign * 0.037, 0.000))
    )

    # Small lugs at the front and rear make the porro dog-leg read as one cast
    # chassis rather than two disconnected tubes.
    rear_lug = (
        cq.Workplane("XY")
        .box(0.055, 0.030, 0.038)
        .translate((-0.052, sign * 0.046, 0.022))
    )
    front_lug = (
        cq.Workplane("XY")
        .box(0.070, 0.035, 0.048)
        .translate((0.047, sign * 0.067, -0.003))
    )

    return (
        prism_block.union(shoulder)
        .union(rear_tube)
        .union(objective_tube)
        .union(hinge_cheek)
        .union(rear_lug)
        .union(front_lug)
    )


def _center_hinge_frame() -> cq.Workplane:
    """Connected hinge pin, focus shaft, and forked support frame."""
    frame = _cylinder_x(0.0100, 0.170, (0.0, 0.0, 0.0))

    # Raised ribs on the central barrel give the classic knurled grip/read.
    for i in range(19):
        x = -0.072 + i * 0.008
        frame = frame.union(_cylinder_x(0.0124, 0.0032, (x, 0.0, 0.0)))

    # The focus wheel rides on a transverse shaft between the oculars.
    shaft = _cylinder_y(0.0057, 0.043, (-0.052, 0.0, 0.056))
    frame = frame.union(shaft)

    focus_bridge = (
        cq.Workplane("XY")
        .box(0.014, 0.006, 0.060)
        .translate((-0.052, 0.0, 0.028))
    )
    frame = frame.union(focus_bridge)

    for sign in (-1.0, 1.0):
        hinge_pad = (
            cq.Workplane("XY")
            .box(0.116, 0.019, 0.020)
            .translate((-0.004, sign * 0.0145, 0.000))
        )
        foot = (
            cq.Workplane("XY")
            .box(0.026, 0.016, 0.016)
            .translate((-0.052, sign * 0.010, 0.006))
        )
        post = (
            cq.Workplane("XY")
            .box(0.012, 0.004, 0.058)
            .translate((-0.052, sign * 0.017, 0.032))
        )
        # Small thrust collars touch the side faces of the focus wheel, so the
        # wheel reads as retained on its shared shaft instead of hovering.
        collar = _cylinder_y(0.012, 0.003, (-0.052, sign * 0.0152, 0.056))
        frame = frame.union(hinge_pad).union(foot).union(post).union(collar)

    return frame


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_body_porro_binoculars")

    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    armor = model.material("olive_black_armor", rgba=(0.055, 0.075, 0.050, 1.0))
    metal = model.material("dark_blued_metal", rgba=(0.025, 0.030, 0.032, 1.0))
    glass = model.material("coated_glass", rgba=(0.10, 0.22, 0.32, 0.62))
    white = model.material("engraved_white", rgba=(0.88, 0.86, 0.76, 1.0))

    center_hinge = model.part("center_hinge")
    center_hinge.visual(
        Cylinder(radius=0.010, length=0.170),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="hinge_barrel",
    )
    for i in range(19):
        center_hinge.visual(
            Cylinder(radius=0.0124, length=0.0032),
            origin=Origin(xyz=(-0.072 + i * 0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name=f"hinge_rib_{i}",
        )
    center_hinge.visual(
        Box((0.116, 0.019, 0.020)),
        origin=Origin(xyz=(-0.004, 0.0145, 0.0)),
        material=rubber,
        name="left_hinge_pad",
    )
    center_hinge.visual(
        Box((0.026, 0.016, 0.016)),
        origin=Origin(xyz=(-0.052, 0.010, 0.006)),
        material=rubber,
        name="left_shaft_foot",
    )
    center_hinge.visual(
        Box((0.012, 0.004, 0.058)),
        origin=Origin(xyz=(-0.052, 0.017, 0.032)),
        material=rubber,
        name="left_shaft_post",
    )
    center_hinge.visual(
        Box((0.116, 0.019, 0.020)),
        origin=Origin(xyz=(-0.004, -0.0145, 0.0)),
        material=rubber,
        name="right_hinge_pad",
    )
    center_hinge.visual(
        Box((0.026, 0.016, 0.016)),
        origin=Origin(xyz=(-0.052, -0.010, 0.006)),
        material=rubber,
        name="right_shaft_foot",
    )
    center_hinge.visual(
        Box((0.012, 0.004, 0.058)),
        origin=Origin(xyz=(-0.052, -0.017, 0.032)),
        material=rubber,
        name="right_shaft_post",
    )
    center_hinge.visual(
        Cylinder(radius=0.0057, length=0.043),
        origin=Origin(xyz=(-0.052, 0.0, 0.056), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="focus_shaft",
    )
    for i, y in enumerate((-0.0152, 0.0152)):
        center_hinge.visual(
            Cylinder(radius=0.012, length=0.003),
            origin=Origin(xyz=(-0.052, y, 0.056), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"shaft_collar_{i}",
        )

    for side_name, sign in (("left", 1.0), ("right", -1.0)):
        body = model.part(f"{side_name}_body")
        body.visual(
            mesh_from_cadquery(_binocular_side(sign), f"{side_name}_body_shell", tolerance=0.0008),
            material=armor,
            name="body_shell",
        )

        # Black front retaining rim and blue-green coated glass disk.
        body.visual(
            mesh_from_geometry(
                KnobGeometry(
                    0.078,
                    0.014,
                    body_style="cylindrical",
                    bore=KnobBore(style="round", diameter=0.056),
                    grip=KnobGrip(style="ribbed", count=28, depth=0.00035),
                ),
                f"{side_name}_objective_rim",
            ),
            origin=Origin(xyz=(0.096, sign * 0.081, -0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name="objective_rim",
        )
        body.visual(
            Cylinder(radius=0.027, length=0.003),
            origin=Origin(xyz=(0.100, sign * 0.081, -0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=glass,
            name="objective_glass",
        )
        body.visual(
            Cylinder(radius=0.016, length=0.0025),
            origin=Origin(xyz=(-0.113, sign * 0.041, 0.024), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=glass,
            name="eyepiece_glass",
        )

    focus_knob = model.part("focus_knob")
    focus_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.038,
                0.028,
                body_style="hourglass",
                top_diameter=0.034,
                base_diameter=0.034,
                grip=KnobGrip(style="knurled", count=36, depth=0.0006, helix_angle_deg=20.0),
                bore=KnobBore(style="round", diameter=0.011),
            ),
            "focus_knob_mesh",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="focus_wheel",
    )
    focus_knob.visual(
        Box((0.007, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.0160)),
        material=white,
        name="focus_mark",
    )

    for side_name, sign in (("left", 1.0), ("right", -1.0)):
        ring = model.part(f"{side_name}_diopter")
        ring.visual(
            mesh_from_geometry(
                KnobGeometry(
                    0.053,
                    0.018,
                    body_style="cylindrical",
                    grip=KnobGrip(style="knurled", count=40, depth=0.00045, helix_angle_deg=-18.0),
                    bore=KnobBore(style="round", diameter=0.040),
                ),
                f"{side_name}_diopter_ring_mesh",
            ),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name="diopter_ring",
        )
        ring.visual(
            Box((0.006, 0.003, 0.003)),
            origin=Origin(xyz=(0.0, 0.0, 0.0267)),
            material=white,
            name="diopter_mark",
        )

    left_body = model.get_part("left_body")
    right_body = model.get_part("right_body")
    left_diopter = model.get_part("left_diopter")
    right_diopter = model.get_part("right_diopter")

    model.articulation(
        "left_body_mount",
        ArticulationType.FIXED,
        parent=center_hinge,
        child=left_body,
        origin=Origin(),
    )
    model.articulation(
        "right_body_hinge",
        ArticulationType.REVOLUTE,
        parent=center_hinge,
        child=right_body,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.5, lower=-0.12, upper=0.12),
    )
    model.articulation(
        "focus_rotation",
        ArticulationType.REVOLUTE,
        parent=center_hinge,
        child=focus_knob,
        origin=Origin(xyz=(-0.052, 0.0, 0.056)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=4.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "left_diopter_rotation",
        ArticulationType.REVOLUTE,
        parent=left_body,
        child=left_diopter,
        origin=Origin(xyz=(-0.100, 0.041, 0.024)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=2.0, lower=-0.85, upper=0.85),
    )
    model.articulation(
        "right_diopter_rotation",
        ArticulationType.REVOLUTE,
        parent=right_body,
        child=right_diopter,
        origin=Origin(xyz=(-0.100, -0.041, 0.024)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=2.0, lower=-0.85, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    focus = object_model.get_part("focus_knob")
    center_hinge = object_model.get_part("center_hinge")
    left_diopter = object_model.get_part("left_diopter")
    right_diopter = object_model.get_part("right_diopter")
    left_body = object_model.get_part("left_body")
    right_body = object_model.get_part("right_body")

    focus_joint = object_model.get_articulation("focus_rotation")
    left_ring_joint = object_model.get_articulation("left_diopter_rotation")
    right_ring_joint = object_model.get_articulation("right_diopter_rotation")
    hinge_joint = object_model.get_articulation("right_body_hinge")

    ctx.allow_overlap(
        center_hinge,
        focus,
        elem_a="focus_shaft",
        elem_b="focus_wheel",
        reason="The focus wheel is intentionally captured on the transverse metal shaft through its bore.",
    )
    ctx.allow_overlap(
        left_body,
        left_diopter,
        elem_a="body_shell",
        elem_b="diopter_ring",
        reason="The diopter ring is modeled as a close rotating sleeve seated around the rear eyepiece tube.",
    )
    ctx.allow_overlap(
        right_body,
        right_diopter,
        elem_a="body_shell",
        elem_b="diopter_ring",
        reason="The diopter ring is modeled as a close rotating sleeve seated around the rear eyepiece tube.",
    )
    ctx.allow_overlap(
        center_hinge,
        left_body,
        elem_a="left_hinge_pad",
        elem_b="body_shell",
        reason="The left porro chassis cheek is intentionally seated against the central hinge bearing pad.",
    )
    ctx.allow_overlap(
        center_hinge,
        right_body,
        elem_a="right_hinge_pad",
        elem_b="body_shell",
        reason="The right porro chassis cheek is intentionally seated against the central hinge bearing pad.",
    )

    ctx.check(
        "front objective centers are wider than rear eyepiece centers",
        0.081 > 0.041,
        details="front objective center offset 81 mm must exceed rear eyepiece offset 41 mm",
    )
    ctx.expect_origin_distance(
        left_diopter,
        right_diopter,
        axes="y",
        min_dist=0.070,
        max_dist=0.085,
        name="eyepiece diopter rings share a close rear spacing",
    )
    ctx.expect_overlap(
        focus,
        center_hinge,
        axes="xyz",
        elem_a="focus_wheel",
        elem_b="focus_shaft",
        min_overlap=0.010,
        name="focus wheel remains captured on shaft",
    )
    ctx.expect_overlap(
        center_hinge,
        left_body,
        axes="xy",
        elem_a="left_hinge_pad",
        elem_b="body_shell",
        min_overlap=0.003,
        name="left chassis cheek seats on hinge pad",
    )
    ctx.expect_overlap(
        center_hinge,
        right_body,
        axes="xy",
        elem_a="right_hinge_pad",
        elem_b="body_shell",
        min_overlap=0.003,
        name="right chassis cheek seats on hinge pad",
    )
    ctx.expect_overlap(
        left_diopter,
        left_body,
        axes="x",
        elem_a="diopter_ring",
        elem_b="body_shell",
        min_overlap=0.010,
        name="left diopter ring surrounds rear housing end",
    )
    ctx.expect_overlap(
        right_diopter,
        right_body,
        axes="x",
        elem_a="diopter_ring",
        elem_b="body_shell",
        min_overlap=0.010,
        name="right diopter ring surrounds rear housing end",
    )

    focus_rest = ctx.part_element_world_aabb(focus, elem="focus_mark")
    with ctx.pose({focus_joint: 0.75}):
        focus_turned = ctx.part_element_world_aabb(focus, elem="focus_mark")
    ctx.check(
        "focus knob mark rotates around transverse shaft",
        focus_rest is not None
        and focus_turned is not None
        and abs(((focus_turned[0][0] + focus_turned[1][0]) / 2.0) - ((focus_rest[0][0] + focus_rest[1][0]) / 2.0)) > 0.006,
        details=f"rest={focus_rest}, turned={focus_turned}",
    )

    left_rest = ctx.part_element_world_aabb(left_diopter, elem="diopter_mark")
    with ctx.pose({left_ring_joint: 0.65}):
        left_turned = ctx.part_element_world_aabb(left_diopter, elem="diopter_mark")
    ctx.check(
        "left diopter mark rotates independently",
        left_rest is not None
        and left_turned is not None
        and abs(((left_turned[0][1] + left_turned[1][1]) / 2.0) - ((left_rest[0][1] + left_rest[1][1]) / 2.0)) > 0.010,
        details=f"rest={left_rest}, turned={left_turned}",
    )

    right_rest = ctx.part_element_world_aabb(right_diopter, elem="diopter_mark")
    with ctx.pose({right_ring_joint: -0.65}):
        right_turned = ctx.part_element_world_aabb(right_diopter, elem="diopter_mark")
    ctx.check(
        "right diopter mark rotates independently",
        right_rest is not None
        and right_turned is not None
        and abs(((right_turned[0][1] + right_turned[1][1]) / 2.0) - ((right_rest[0][1] + right_rest[1][1]) / 2.0)) > 0.010,
        details=f"rest={right_rest}, turned={right_turned}",
    )

    right_rest_pose = ctx.part_world_aabb(right_body)
    with ctx.pose({hinge_joint: 0.10}):
        right_folded_pose = ctx.part_world_aabb(right_body)
    ctx.check(
        "right porro half folds around central hinge",
        right_rest_pose is not None
        and right_folded_pose is not None
        and abs(right_folded_pose[0][2] - right_rest_pose[0][2]) > 0.003,
        details=f"rest={right_rest_pose}, folded={right_folded_pose}",
    )

    return ctx.report()


object_model = build_object_model()
