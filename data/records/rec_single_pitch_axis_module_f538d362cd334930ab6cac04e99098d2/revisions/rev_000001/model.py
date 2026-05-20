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


def _annular_bearing() -> cq.Workplane:
    """Bearing-flange mesh: annular disk whose bore clears the live trunnion."""
    return cq.Workplane("YZ").circle(0.042).circle(0.0175).extrude(0.018, both=True)


def _support_yoke() -> cq.Workplane:
    """Boxed trunnion yoke with two cheeks and a real through-bore."""
    base_slab = cq.Workplane("XY").box(0.220, 0.120, 0.022).translate((0.0, 0.0, 0.011))
    cheek_0 = cq.Workplane("XY").box(0.045, 0.120, 0.150).translate((0.0875, 0.0, 0.075))
    cheek_1 = cq.Workplane("XY").box(0.045, 0.120, 0.150).translate((-0.0875, 0.0, 0.075))
    yoke = base_slab.union(cheek_0).union(cheek_1)
    bore = cq.Workplane("YZ").center(0.0, 0.100).circle(0.024).extrude(0.300, both=True)
    return yoke.cut(bore).edges("|Z").fillet(0.004)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_pitch_trunnion_module")

    cast_blue = model.material("painted_cast_blue", rgba=(0.08, 0.16, 0.25, 1.0))
    dark_plate = model.material("dark_ground_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    machined = model.material("machined_steel", rgba=(0.58, 0.60, 0.58, 1.0))
    polished = model.material("polished_shaft", rgba=(0.78, 0.78, 0.72, 1.0))
    black = model.material("black_hardware", rgba=(0.015, 0.015, 0.014, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.38, 0.25, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=dark_plate,
        name="base_plate",
    )
    base.visual(
        Box((0.25, 0.16, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 0.0545)),
        material=cast_blue,
        name="boxed_support",
    )
    base.visual(
        Box((0.265, 0.172, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.093)),
        material=dark_plate,
        name="support_cap",
    )

    yoke_bottom_z = 0.099
    trunnion_axis_z = yoke_bottom_z + 0.100
    base.visual(
        mesh_from_cadquery(_support_yoke(), "support_yoke"),
        origin=Origin(xyz=(0.0, 0.0, yoke_bottom_z)),
        material=cast_blue,
        name="support_yoke",
    )

    bearing_mesh = mesh_from_cadquery(_annular_bearing(), "bearing_flange_close_fit")
    base.visual(
        bearing_mesh,
        origin=Origin(xyz=(0.1175, 0.0, trunnion_axis_z)),
        material=machined,
        name="bearing_flange_0",
    )
    base.visual(
        bearing_mesh,
        origin=Origin(xyz=(-0.1175, 0.0, trunnion_axis_z)),
        material=machined,
        name="bearing_flange_1",
    )
    for sign, suffix in ((1.0, "0"), (-1.0, "1")):
        for yi, zi in ((0.027, 0.027), (0.027, -0.027), (-0.027, 0.027), (-0.027, -0.027)):
            base.visual(
                Cylinder(radius=0.005, length=0.006),
                origin=Origin(
                    xyz=(sign * 0.126, yi, trunnion_axis_z + zi),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=black,
                name=f"bearing_bolt_{suffix}_{'p' if yi > 0 else 'n'}_{'u' if zi > 0 else 'd'}",
            )

    for xi in (-0.155, 0.155):
        for yi in (-0.095, 0.095):
            base.visual(
                Cylinder(radius=0.007, length=0.005),
                origin=Origin(xyz=(xi, yi, 0.0245)),
                material=black,
                name=f"base_bolt_{'p' if xi > 0 else 'n'}_{'p' if yi > 0 else 'n'}",
            )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.018, length=0.250),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="trunnion_shaft",
    )
    head.visual(
        Cylinder(radius=0.030, length=0.096),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="central_hub",
    )
    head.visual(
        Cylinder(radius=0.027, length=0.012),
        origin=Origin(xyz=(0.053, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="journal_collar_0",
    )
    head.visual(
        Cylinder(radius=0.027, length=0.012),
        origin=Origin(xyz=(-0.053, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="journal_collar_1",
    )
    head.visual(
        Box((0.086, 0.058, 0.076)),
        origin=Origin(xyz=(0.0, -0.030, 0.0)),
        material=machined,
        name="tilt_block",
    )
    head.visual(
        Box((0.110, 0.014, 0.100)),
        origin=Origin(xyz=(0.0, -0.066, 0.006)),
        material=machined,
        name="tooling_face",
    )
    head.visual(
        Box((0.082, 0.004, 0.066)),
        origin=Origin(xyz=(0.0, -0.075, 0.006)),
        material=dark_plate,
        name="face_insert",
    )
    for xi, zi, suffix in (
        (-0.034, -0.026, "0"),
        (0.034, -0.026, "1"),
        (-0.034, 0.026, "2"),
        (0.034, 0.026, "3"),
        (0.0, 0.0, "4"),
    ):
        head.visual(
            Cylinder(radius=0.0045, length=0.005),
            origin=Origin(xyz=(xi, -0.079, 0.006 + zi), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name=f"tooling_hole_{suffix}",
        )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, trunnion_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=-0.50, upper=0.50),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    joint = object_model.get_articulation("base_to_head")

    for flange in ("bearing_flange_0", "bearing_flange_1"):
        ctx.allow_overlap(
            base,
            head,
            elem_a=flange,
            elem_b="trunnion_shaft",
            reason="The shaft is intentionally a close running fit inside the simplified bearing bore so the trunnion is physically captured.",
        )

    ctx.check(
        "single pitch joint",
        joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )
    ctx.expect_within(
        head,
        base,
        axes="yz",
        inner_elem="trunnion_shaft",
        outer_elem="support_yoke",
        margin=0.001,
        name="shaft is centered in the cheek bores",
    )
    ctx.expect_overlap(
        head,
        base,
        axes="x",
        elem_a="trunnion_shaft",
        elem_b="support_yoke",
        min_overlap=0.18,
        name="shaft spans both yoke cheeks",
    )
    for flange in ("bearing_flange_0", "bearing_flange_1"):
        ctx.expect_overlap(
            head,
            base,
            axes="x",
            elem_a="trunnion_shaft",
            elem_b=flange,
            min_overlap=0.012,
            name=f"shaft remains engaged in {flange}",
        )
        ctx.expect_within(
            head,
            base,
            axes="yz",
            inner_elem="trunnion_shaft",
            outer_elem=flange,
            margin=0.001,
            name=f"shaft is concentric with {flange}",
        )
    ctx.expect_gap(
        head,
        head,
        axis="x",
        positive_elem="journal_collar_0",
        negative_elem="journal_collar_1",
        min_gap=0.09,
        max_gap=0.12,
        name="journal collars sit inside the yoke span",
    )

    face_rest = ctx.part_element_world_aabb(head, elem="tooling_face")
    origin_rest = ctx.part_world_position(head)
    with ctx.pose({joint: 0.50}):
        face_down = ctx.part_element_world_aabb(head, elem="tooling_face")
        origin_down = ctx.part_world_position(head)
    with ctx.pose({joint: -0.50}):
        face_up = ctx.part_element_world_aabb(head, elem="tooling_face")
        origin_up = ctx.part_world_position(head)

    def _center_z(aabb):
        return None if aabb is None else 0.5 * (aabb[0][2] + aabb[1][2])

    ctx.check(
        "tooling face pitches about fixed trunnion",
        face_rest is not None
        and face_down is not None
        and face_up is not None
        and _center_z(face_down) < _center_z(face_rest) - 0.025
        and _center_z(face_up) > _center_z(face_rest) + 0.025
        and origin_rest == origin_down == origin_up,
        details=f"rest={face_rest}, down={face_down}, up={face_up}, origins={origin_rest, origin_down, origin_up}",
    )

    return ctx.report()


object_model = build_object_model()
