from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="yaw_pitch_service_head")

    cast_iron = model.material("cast_iron", color=(0.16, 0.17, 0.18, 1.0))
    satin_steel = model.material("satin_steel", color=(0.68, 0.70, 0.72, 1.0))
    dark_bearing = model.material("dark_bearing", color=(0.04, 0.045, 0.05, 1.0))
    safety_yellow = model.material("safety_yellow", color=(0.95, 0.70, 0.12, 1.0))
    face_black = model.material("face_black", color=(0.015, 0.016, 0.018, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.18, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=cast_iron,
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.078, length=0.143),
        origin=Origin(xyz=(0.0, 0.0, 0.106)),
        material=cast_iron,
        name="pedestal_column",
    )
    base.visual(
        Cylinder(radius=0.112, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.1875)),
        material=dark_bearing,
        name="top_bearing",
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        Cylinder(radius=0.125, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=satin_steel,
        name="turntable",
    )
    lower_stage.visual(
        Cylinder(radius=0.114, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        material=dark_bearing,
        name="rotary_band",
    )
    lower_stage.visual(
        Cylinder(radius=0.070, length=0.083),
        origin=Origin(xyz=(0.0, 0.0, 0.1015)),
        material=satin_steel,
        name="yaw_neck",
    )
    lower_stage.visual(
        Box((0.190, 0.255, 0.032)),
        origin=Origin(xyz=(0.032, 0.0, 0.157)),
        material=satin_steel,
        name="cradle_base",
    )
    lower_stage.visual(
        Box((0.145, 0.026, 0.154)),
        origin=Origin(xyz=(0.045, 0.093, 0.245)),
        material=satin_steel,
        name="cradle_cheek_0",
    )
    lower_stage.visual(
        Box((0.145, 0.026, 0.154)),
        origin=Origin(xyz=(0.045, -0.093, 0.245)),
        material=satin_steel,
        name="cradle_cheek_1",
    )
    lower_stage.visual(
        Box((0.030, 0.212, 0.118)),
        origin=Origin(xyz=(-0.042, 0.0, 0.236)),
        material=satin_steel,
        name="rear_bridge",
    )
    lower_stage.visual(
        Cylinder(radius=0.040, length=0.012),
        origin=Origin(xyz=(0.045, 0.112, 0.245), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_bearing,
        name="bearing_cap_0",
    )
    lower_stage.visual(
        Cylinder(radius=0.040, length=0.012),
        origin=Origin(xyz=(0.045, -0.112, 0.245), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_bearing,
        name="bearing_cap_1",
    )

    output_head = model.part("output_head")
    output_head.visual(
        Cylinder(radius=0.045, length=0.160),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_bearing,
        name="trunnion_shaft",
    )
    output_head.visual(
        Cylinder(radius=0.043, length=0.112),
        origin=Origin(xyz=(0.079, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="output_nose",
    )
    output_head.visual(
        Box((0.026, 0.130, 0.130)),
        origin=Origin(xyz=(0.148, 0.0, 0.0)),
        material=safety_yellow,
        name="output_face",
    )
    output_head.visual(
        Box((0.008, 0.070, 0.070)),
        origin=Origin(xyz=(0.165, 0.0, 0.0)),
        material=face_black,
        name="tool_socket",
    )
    for y in (-0.045, 0.045):
        for z in (-0.045, 0.045):
            output_head.visual(
                Cylinder(radius=0.008, length=0.007),
                origin=Origin(xyz=(0.1585, y, z), rpy=(0.0, pi / 2.0, 0.0)),
                material=dark_bearing,
                name=f"face_bolt_{'p' if y > 0 else 'n'}_{'p' if z > 0 else 'n'}",
            )

    model.articulation(
        "yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.5, lower=-2.8, upper=2.8),
    )
    model.articulation(
        "pitch",
        ArticulationType.REVOLUTE,
        parent=lower_stage,
        child=output_head,
        origin=Origin(xyz=(0.045, 0.0, 0.245)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.8, lower=-0.75, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_stage = object_model.get_part("lower_stage")
    output_head = object_model.get_part("output_head")
    yaw = object_model.get_articulation("yaw")
    pitch = object_model.get_articulation("pitch")

    ctx.check(
        "two revolute service-head joints",
        len(object_model.articulations) == 2
        and yaw.articulation_type == ArticulationType.REVOLUTE
        and pitch.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations}",
    )
    ctx.expect_gap(
        lower_stage,
        base,
        axis="z",
        positive_elem="turntable",
        negative_elem="top_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable sits on base bearing",
    )
    ctx.expect_overlap(
        lower_stage,
        base,
        axes="xy",
        elem_a="turntable",
        elem_b="top_bearing",
        min_overlap=0.10,
        name="yaw bearing footprint is centered",
    )
    ctx.expect_overlap(
        output_head,
        lower_stage,
        axes="xz",
        elem_a="trunnion_shaft",
        elem_b="cradle_cheek_0",
        min_overlap=0.040,
        name="trunnion shaft aligns with cheek bore zone",
    )
    ctx.expect_contact(
        output_head,
        lower_stage,
        elem_a="trunnion_shaft",
        elem_b="cradle_cheek_0",
        contact_tol=0.0005,
        name="trunnion bears against one cradle cheek",
    )
    ctx.expect_contact(
        output_head,
        lower_stage,
        elem_a="trunnion_shaft",
        elem_b="cradle_cheek_1",
        contact_tol=0.0005,
        name="trunnion bears against opposite cradle cheek",
    )

    def _elem_center_z(part, elem: str) -> float | None:
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        low, high = bounds
        return (low[2] + high[2]) * 0.5

    def _elem_center_xy(part, elem: str) -> tuple[float, float] | None:
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        low, high = bounds
        return ((low[0] + high[0]) * 0.5, (low[1] + high[1]) * 0.5)

    rest_face_z = _elem_center_z(output_head, "output_face")
    with ctx.pose({pitch: pitch.motion_limits.upper}):
        pitched_face_z = _elem_center_z(output_head, "output_face")
    ctx.check(
        "positive pitch raises output face",
        rest_face_z is not None
        and pitched_face_z is not None
        and pitched_face_z > rest_face_z + 0.06,
        details=f"rest_z={rest_face_z}, pitched_z={pitched_face_z}",
    )

    rest_face_xy = _elem_center_xy(output_head, "output_face")
    with ctx.pose({yaw: 0.75}):
        yawed_face_xy = _elem_center_xy(output_head, "output_face")
    ctx.check(
        "positive yaw slews output face around base",
        rest_face_xy is not None
        and yawed_face_xy is not None
        and yawed_face_xy[1] > rest_face_xy[1] + 0.07,
        details=f"rest_xy={rest_face_xy}, yawed_xy={yawed_face_xy}",
    )

    return ctx.report()


object_model = build_object_model()
