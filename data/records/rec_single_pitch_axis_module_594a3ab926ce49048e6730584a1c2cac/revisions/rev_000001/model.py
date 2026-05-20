from __future__ import annotations

import math

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
    TorusGeometry,
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


BASE_Z = 0.040
TRUNNION_Z = BASE_Z + 0.130


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_supported_trunnion_module")

    cast_iron = Material("graphite_cast_iron", rgba=(0.18, 0.19, 0.19, 1.0))
    dark_steel = Material("blackened_steel", rgba=(0.035, 0.038, 0.040, 1.0))
    machined = Material("machined_steel", rgba=(0.56, 0.57, 0.55, 1.0))
    bronze = Material("oiled_bronze", rgba=(0.74, 0.50, 0.22, 1.0))
    face_blue = Material("carried_face_blue", rgba=(0.05, 0.17, 0.32, 1.0))

    saddle = model.part("saddle")
    saddle.visual(
        Box((0.56, 0.38, BASE_Z)),
        origin=Origin(xyz=(0.0, 0.0, BASE_Z / 2.0)),
        material=cast_iron,
        name="base_plate",
    )
    saddle.visual(
        Box((0.42, 0.24, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, BASE_Z + 0.009)),
        material=machined,
        name="machined_saddle_pad",
    )

    yoke_body = TrunnionYokeGeometry(
        (0.42, 0.30, 0.230),
        span_width=0.240,
        trunnion_diameter=0.062,
        trunnion_center_z=TRUNNION_Z - BASE_Z,
        base_thickness=0.035,
        corner_radius=0.010,
        center=False,
    )
    saddle.visual(
        mesh_from_geometry(yoke_body, "yoke_body"),
        # A tiny seating overlap makes the cast yoke read as bolted into the saddle.
        origin=Origin(xyz=(0.0, 0.0, BASE_Z - 0.002)),
        material=cast_iron,
        name="yoke_body",
    )

    # Short rear cap bridge tying the cheeks together while leaving the trunnion
    # head visible and free to pitch in the central bay.
    saddle.visual(
        Box((0.44, 0.060, 0.040)),
        origin=Origin(xyz=(0.0, -0.135, BASE_Z + 0.222)),
        material=dark_steel,
        name="rear_cap_bridge",
    )

    bearing_ring = TorusGeometry(0.046, 0.012, radial_segments=32, tubular_segments=12)
    for side, x in (("near", -0.211), ("far", 0.211)):
        saddle.visual(
            mesh_from_geometry(bearing_ring, f"{side}_bearing_ring"),
            origin=Origin(xyz=(x, 0.0, TRUNNION_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bronze,
            name=f"{side}_bearing_ring",
        )

    for x in (-0.220, 0.220):
        for y in (-0.140, 0.140):
            saddle.visual(
                Cylinder(radius=0.014, length=0.012),
                origin=Origin(xyz=(x, y, BASE_Z + 0.004)),
                material=dark_steel,
                name=f"mount_bolt_{x:+.2f}_{y:+.2f}",
            )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.0315, length=0.370),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="trunnion_shaft",
    )
    head.visual(
        Box((0.180, 0.130, 0.120)),
        origin=Origin(),
        material=dark_steel,
        name="head_block",
    )
    for side, x in (("near", -0.106), ("far", 0.106)):
        head.visual(
            Cylinder(radius=0.040, length=0.018),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=machined,
            name=f"{side}_inner_hub",
        )

    head.visual(
        Box((0.210, 0.026, 0.160)),
        origin=Origin(xyz=(0.0, 0.077, 0.0)),
        material=machined,
        name="face_carrier",
    )
    head.visual(
        Box((0.165, 0.007, 0.112)),
        origin=Origin(xyz=(0.0, 0.0935, 0.0)),
        material=face_blue,
        name="carried_face",
    )
    for x in (-0.064, 0.064):
        for z in (-0.044, 0.044):
            head.visual(
                Cylinder(radius=0.007, length=0.008),
                origin=Origin(xyz=(x, 0.100, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=dark_steel,
                name=f"face_screw_{x:+.2f}_{z:+.2f}",
            )

    model.articulation(
        "saddle_to_head",
        ArticulationType.REVOLUTE,
        parent=saddle,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, TRUNNION_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=-0.52, upper=0.52),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    saddle = object_model.get_part("saddle")
    head = object_model.get_part("head")
    joint = object_model.get_articulation("saddle_to_head")

    ctx.allow_overlap(
        head,
        saddle,
        elem_a="trunnion_shaft",
        elem_b="yoke_body",
        reason="The trunnion shaft is modeled as a light captured bearing fit inside the simplified cheek bores.",
    )

    ctx.check(
        "single pitch revolute",
        joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={joint.articulation_type}, axis={joint.axis}",
    )

    ctx.expect_within(
        head,
        saddle,
        axes="yz",
        inner_elem="trunnion_shaft",
        outer_elem="yoke_body",
        margin=0.002,
        name="shaft is carried inside the cheek bores",
    )
    ctx.expect_overlap(
        head,
        saddle,
        axes="x",
        elem_a="trunnion_shaft",
        elem_b="yoke_body",
        min_overlap=0.32,
        name="shaft spans both side cheeks",
    )
    ctx.expect_within(
        head,
        saddle,
        axes="x",
        inner_elem="head_block",
        outer_elem="yoke_body",
        margin=0.0,
        name="moving head is between fixed supports",
    )

    def _center_z(aabb):
        return (aabb[0][2] + aabb[1][2]) / 2.0 if aabb is not None else None

    rest_face = ctx.part_element_world_aabb(head, elem="carried_face")
    with ctx.pose({joint: 0.42}):
        pitched_face = ctx.part_element_world_aabb(head, elem="carried_face")

    rest_z = _center_z(rest_face)
    pitched_z = _center_z(pitched_face)
    ctx.check(
        "positive pitch lifts carried face",
        rest_z is not None and pitched_z is not None and pitched_z > rest_z + 0.025,
        details=f"rest_z={rest_z}, pitched_z={pitched_z}",
    )

    return ctx.report()


object_model = build_object_model()
