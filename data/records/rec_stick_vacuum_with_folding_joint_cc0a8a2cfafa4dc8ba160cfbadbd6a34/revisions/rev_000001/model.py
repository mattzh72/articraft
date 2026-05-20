from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    CylinderGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_weatherproof_stick_vacuum")

    shell = model.material("sealed_graphite_shell", rgba=(0.10, 0.12, 0.13, 1.0))
    rubber = model.material("black_uv_rubber", rgba=(0.01, 0.012, 0.012, 1.0))
    metal = model.material("brushed_stainless_hardware", rgba=(0.72, 0.72, 0.68, 1.0))
    tube = model.material("anodized_dark_tube", rgba=(0.025, 0.035, 0.04, 1.0))
    accent = model.material("weatherproof_orange_control", rgba=(1.0, 0.33, 0.06, 1.0))
    intake = model.material("deep_nozzle_intake", rgba=(0.0, 0.0, 0.0, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(CapsuleGeometry(0.055, 0.32, radial_segments=32), "motor_capsule"),
        origin=Origin(xyz=(0.0, 0.0, 1.13)),
        material=shell,
        name="motor_capsule",
    )
    body.visual(
        Box((0.095, 0.082, 0.17)),
        origin=Origin(xyz=(0.014, 0.0, 0.985)),
        material=shell,
        name="sealed_battery_pod",
    )
    body.visual(
        Box((0.152, 0.128, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 1.348)),
        material=rubber,
        name="top_drip_hood",
    )
    body.visual(
        Box((0.128, 0.108, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 1.296)),
        material=rubber,
        name="upper_seam_gasket",
    )
    body.visual(
        mesh_from_geometry(TorusGeometry(0.057, 0.0045, radial_segments=36, tubular_segments=10), "lower_o_ring"),
        origin=Origin(xyz=(0.008, 0.0, 0.868)),
        material=rubber,
        name="lower_o_ring",
    )

    # Integral sealed carry handle, tied into the capsule at both ends.
    body.visual(
        Box((0.035, 0.044, 0.292)),
        origin=Origin(xyz=(-0.110, 0.0, 1.180)),
        material=rubber,
        name="handle_grip",
    )
    body.visual(
        Box((0.102, 0.046, 0.038)),
        origin=Origin(xyz=(-0.059, 0.0, 1.318)),
        material=rubber,
        name="handle_top_bridge",
    )
    body.visual(
        Box((0.096, 0.046, 0.038)),
        origin=Origin(xyz=(-0.062, 0.0, 1.045)),
        material=rubber,
        name="handle_lower_bridge",
    )

    # Overlapped rain-lap ribs and screw heads make the body read as sealed and serviceable.
    for idx, z in enumerate((1.245, 1.205, 1.165)):
        body.visual(
            Box((0.018, 0.112, 0.006)),
            origin=Origin(xyz=(0.052, 0.0, z)),
            material=rubber,
            name=f"protected_vent_slat_{idx}",
        )
    for y in (-0.054, 0.054):
        for z in (0.958,):
            body.visual(
                Cylinder(radius=0.0055, length=0.006),
                origin=Origin(xyz=(-0.020, math.copysign(0.043, y), z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=metal,
                name=f"stainless_body_screw_{'n' if y < 0 else 'p'}_{str(z).replace('.', '_')}",
            )

    # Main folding joint yoke: cheeks are part of the body and the pin heads sit proud.
    body.visual(
        Box((0.094, 0.094, 0.034)),
        origin=Origin(xyz=(0.004, 0.0, 0.902)),
        material=shell,
        name="fold_saddle",
    )
    for idx, y in enumerate((-0.039, 0.039)):
        body.visual(
            Box((0.083, 0.018, 0.086)),
            origin=Origin(xyz=(0.0, y, 0.858)),
            material=shell,
            name=f"fold_cheek_{idx}",
        )
        body.visual(
            Cylinder(radius=0.0155, length=0.010),
            origin=Origin(xyz=(0.0, y * 1.36, 0.858), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"fold_pin_cap_{idx}",
        )
        body.visual(
            Cylinder(radius=0.005, length=0.006),
            origin=Origin(xyz=(0.028, math.copysign(0.050, y), 0.886), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"fold_cheek_screw_{idx}",
        )
    body.visual(
        Cylinder(radius=0.010, length=0.104),
        origin=Origin(xyz=(0.0, 0.0, 0.858), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="fold_pin_shaft",
    )

    button = model.part("power_button")
    button.visual(
        Box((0.014, 0.040, 0.026)),
        origin=Origin(xyz=(-0.007, 0.0, 0.0)),
        material=accent,
        name="sealed_button_cap",
    )
    button.visual(
        Box((0.004, 0.047, 0.033)),
        origin=Origin(xyz=(-0.0015, 0.0, 0.0)),
        material=rubber,
        name="button_boot_flange",
    )

    wand = model.part("folding_wand")
    wand.visual(
        Box((0.055, 0.036, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=tube,
        name="fold_lug",
    )
    for idx, y in enumerate((-0.0195, 0.0195)):
        wand.visual(
            Cylinder(radius=0.018, length=0.006),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"fold_bushing_{idx}",
        )
    wand.visual(
        Cylinder(radius=0.024, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=tube,
        name="fold_neck",
    )
    for idx, z in enumerate((-0.042, -0.065, -0.088)):
        wand.visual(
            Cylinder(radius=0.034 - 0.003 * idx, length=0.016),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=rubber,
            name=f"fold_bellows_{idx}",
        )
    wand.visual(
        Cylinder(radius=0.0215, length=0.690),
        origin=Origin(xyz=(0.0, 0.0, -0.420)),
        material=tube,
        name="sealed_wand_tube",
    )
    wand.visual(
        Cylinder(radius=0.028, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.118)),
        material=rubber,
        name="upper_wand_boot",
    )
    wand.visual(
        Cylinder(radius=0.027, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, -0.736)),
        material=rubber,
        name="lower_wand_boot",
    )
    for idx, y in enumerate((-0.019, 0.019)):
        wand.visual(
            Box((0.006, 0.004, 0.520)),
            origin=Origin(xyz=(0.020, math.copysign(0.010, y), -0.420)),
            material=rubber,
            name=f"raised_drain_rib_{idx}",
        )
    wand.visual(
        Box((0.050, 0.034, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, -0.780)),
        material=tube,
        name="nozzle_trunnion",
    )
    for idx, y in enumerate((-0.0185, 0.0185)):
        wand.visual(
            Cylinder(radius=0.016, length=0.006),
            origin=Origin(xyz=(0.0, y, -0.780), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"nozzle_bushing_{idx}",
        )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Box((0.430, 0.292, 0.012)),
        origin=Origin(xyz=(0.104, 0.0, -0.055)),
        material=rubber,
        name="drip_overhang_lip",
    )
    floor_head.visual(
        Box((0.410, 0.268, 0.058)),
        origin=Origin(xyz=(0.105, 0.0, -0.094)),
        material=shell,
        name="sealed_nozzle_shell",
    )
    floor_head.visual(
        Box((0.128, 0.096, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
        material=shell,
        name="joint_bridge",
    )
    for idx, y in enumerate((-0.044, 0.044)):
        floor_head.visual(
            Box((0.082, 0.018, 0.074)),
            origin=Origin(xyz=(0.0, y, -0.015)),
            material=shell,
            name=f"nozzle_cheek_{idx}",
        )
        floor_head.visual(
            Cylinder(radius=0.015, length=0.010),
            origin=Origin(xyz=(0.0, y * 1.27, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"nozzle_pin_cap_{idx}",
        )
    floor_head.visual(
        Cylinder(radius=0.010, length=0.112),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="nozzle_pin_shaft",
    )
    floor_head.visual(
        Box((0.315, 0.030, 0.009)),
        origin=Origin(xyz=(0.160, 0.0, -0.127)),
        material=intake,
        name="suction_mouth_shadow",
    )
    floor_head.visual(
        Box((0.360, 0.014, 0.030)),
        origin=Origin(xyz=(0.205, 0.141, -0.103)),
        material=rubber,
        name="side_squeegee_0",
    )
    floor_head.visual(
        Box((0.360, 0.014, 0.030)),
        origin=Origin(xyz=(0.205, -0.141, -0.103)),
        material=rubber,
        name="side_squeegee_1",
    )
    for idx, y in enumerate((-0.112, 0.112)):
        floor_head.visual(
            Cylinder(radius=0.022, length=0.026),
            origin=Origin(xyz=(-0.094, y, -0.126), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"rear_roller_{idx}",
        )

    model.articulation(
        "body_to_power_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=(-0.1280, 0.0, 1.220)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.04, lower=0.0, upper=0.006),
    )
    model.articulation(
        "body_to_folding_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand,
        origin=Origin(xyz=(0.0, 0.0, 0.858)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.72),
    )
    model.articulation(
        "wand_to_floor_head",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.0, 0.0, -0.780)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.8, lower=-0.62, upper=0.62),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    button = object_model.get_part("power_button")
    wand = object_model.get_part("folding_wand")
    floor_head = object_model.get_part("floor_head")
    fold = object_model.get_articulation("body_to_folding_wand")
    nozzle = object_model.get_articulation("wand_to_floor_head")
    button_slide = object_model.get_articulation("body_to_power_button")

    ctx.allow_overlap(
        body,
        wand,
        elem_a="fold_pin_shaft",
        elem_b="fold_lug",
        reason="The corrosion-resistant hinge pin is intentionally modeled as captured through the folding wand lug bore.",
    )
    ctx.expect_within(
        body,
        wand,
        axes="xz",
        inner_elem="fold_pin_shaft",
        outer_elem="fold_lug",
        margin=0.002,
        name="folding pin passes through lug centerline",
    )
    ctx.expect_overlap(
        body,
        wand,
        axes="y",
        elem_a="fold_pin_shaft",
        elem_b="fold_lug",
        min_overlap=0.030,
        name="folding pin spans the lug",
    )
    for bushing_name in ("fold_bushing_0", "fold_bushing_1"):
        ctx.allow_overlap(
            body,
            wand,
            elem_a="fold_pin_shaft",
            elem_b=bushing_name,
            reason="The folding hinge pin is intentionally captured through the stainless bushing bore.",
        )
        ctx.expect_within(
            body,
            wand,
            axes="xz",
            inner_elem="fold_pin_shaft",
            outer_elem=bushing_name,
            margin=0.002,
            name=f"fold pin centered in {bushing_name}",
        )

    ctx.allow_overlap(
        floor_head,
        wand,
        elem_a="nozzle_pin_shaft",
        elem_b="nozzle_trunnion",
        reason="The floor-head pitch pin is intentionally captured through the wand trunnion bore.",
    )
    ctx.expect_within(
        floor_head,
        wand,
        axes="xz",
        inner_elem="nozzle_pin_shaft",
        outer_elem="nozzle_trunnion",
        margin=0.002,
        name="nozzle pin passes through trunnion centerline",
    )
    ctx.expect_overlap(
        floor_head,
        wand,
        axes="y",
        elem_a="nozzle_pin_shaft",
        elem_b="nozzle_trunnion",
        min_overlap=0.028,
        name="nozzle pin spans the trunnion",
    )
    for bushing_name in ("nozzle_bushing_0", "nozzle_bushing_1"):
        ctx.allow_overlap(
            floor_head,
            wand,
            elem_a="nozzle_pin_shaft",
            elem_b=bushing_name,
            reason="The nozzle pitch pin is intentionally captured through the stainless bushing bore.",
        )
        ctx.expect_within(
            floor_head,
            wand,
            axes="xz",
            inner_elem="nozzle_pin_shaft",
            outer_elem=bushing_name,
            margin=0.002,
            name=f"nozzle pin centered in {bushing_name}",
        )

    ctx.expect_gap(
        body,
        button,
        axis="x",
        positive_elem="handle_grip",
        negative_elem="button_boot_flange",
        max_gap=0.002,
        max_penetration=0.001,
        name="sealed button boot seats on handle",
    )

    rest_head = ctx.part_world_aabb(floor_head)
    with ctx.pose({fold: 1.45}):
        folded_head = ctx.part_world_aabb(floor_head)
    ctx.check(
        "fold joint lifts floor head into storage arc",
        rest_head is not None
        and folded_head is not None
        and folded_head[1][2] > rest_head[1][2] + 0.45
        and folded_head[0][0] < rest_head[0][0] - 0.45,
        details=f"rest={rest_head}, folded={folded_head}",
    )

    rest_nozzle = ctx.part_element_world_aabb(floor_head, elem="sealed_nozzle_shell")
    with ctx.pose({nozzle: 0.55}):
        pitched_nozzle = ctx.part_element_world_aabb(floor_head, elem="sealed_nozzle_shell")
    ctx.check(
        "nozzle pitch articulation moves the floor head",
        rest_nozzle is not None
        and pitched_nozzle is not None
        and pitched_nozzle[0][2] < rest_nozzle[0][2] - 0.045,
        details=f"rest={rest_nozzle}, pitched={pitched_nozzle}",
    )

    button_rest = ctx.part_world_position(button)
    with ctx.pose({button_slide: 0.006}):
        button_pressed = ctx.part_world_position(button)
    ctx.check(
        "sealed power button moves inward",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[0] > button_rest[0] + 0.004,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
