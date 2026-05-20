from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


SPINDLE_Z = 0.72
MOTOR_Z = 0.36
BELT_X = -0.34


def _circle_profile(radius: float, *, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (radius * cos(2.0 * pi * i / segments), radius * sin(2.0 * pi * i / segments))
        for i in range(segments)
    ]


def _capsule_profile(radius: float, half_distance: float, *, segments: int = 32) -> list[tuple[float, float]]:
    """2-D racetrack profile; local X maps to world Z after the mesh rotation."""
    points: list[tuple[float, float]] = []
    points.append((-half_distance, radius))
    points.append((half_distance, radius))
    for i in range(1, segments + 1):
        angle = pi / 2.0 - pi * i / segments
        points.append((half_distance + radius * cos(angle), radius * sin(angle)))
    points.append((-half_distance, -radius))
    for i in range(1, segments + 1):
        angle = -pi / 2.0 - pi * i / segments
        points.append((-half_distance + radius * cos(angle), radius * sin(angle)))
    return points


def _ring_mesh(name: str, outer_radius: float, inner_radius: float, width: float):
    outer = _circle_profile(outer_radius, segments=72)
    inner = list(reversed(_circle_profile(inner_radius, segments=72)))
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, [inner], height=width, center=True).rotate_y(pi / 2.0),
        name,
    )


def _belt_mesh(name: str):
    half_distance = (SPINDLE_Z - MOTOR_Z) * 0.5
    outer = _capsule_profile(0.098, half_distance, segments=36)
    inner = list(reversed(_capsule_profile(0.081, half_distance, segments=36)))
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, [inner], height=0.034, center=True).rotate_y(pi / 2.0),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="belt_driven_buffing_wheel")

    model.material("painted_steel", rgba=(0.18, 0.23, 0.27, 1.0))
    model.material("dark_cast_metal", rgba=(0.08, 0.09, 0.10, 1.0))
    model.material("brushed_steel", rgba=(0.66, 0.68, 0.70, 1.0))
    model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    model.material("buffing_cloth", rgba=(0.92, 0.84, 0.62, 1.0))
    model.material("cloth_shadow", rgba=(0.70, 0.61, 0.43, 1.0))
    model.material("brass_bushing", rgba=(0.72, 0.53, 0.22, 1.0))
    model.material("warning_label", rgba=(0.95, 0.70, 0.05, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.98, 0.38, 0.08)),
        origin=Origin(xyz=(0.05, 0.0, 0.04)),
        material="dark_cast_metal",
        name="base_plate",
    )
    frame.visual(
        Box((0.62, 0.20, 0.10)),
        origin=Origin(xyz=(0.12, 0.0, 0.13)),
        material="painted_steel",
        name="motor_pedestal",
    )
    frame.visual(
        Cylinder(radius=0.145, length=0.56),
        origin=Origin(xyz=(0.12, 0.0, MOTOR_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material="painted_steel",
        name="motor_shell",
    )
    frame.visual(
        Cylinder(radius=0.148, length=0.022),
        origin=Origin(xyz=(-0.17, 0.0, MOTOR_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_cast_metal",
        name="motor_end_cap",
    )
    frame.visual(
        Cylinder(radius=0.148, length=0.022),
        origin=Origin(xyz=(0.41, 0.0, MOTOR_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_cast_metal",
        name="rear_end_cap",
    )
    frame.visual(
        Box((0.18, 0.12, 0.075)),
        origin=Origin(xyz=(0.13, 0.0, 0.52)),
        material="dark_cast_metal",
        name="terminal_box",
    )
    frame.visual(
        Box((0.13, 0.004, 0.035)),
        origin=Origin(xyz=(0.13, -0.062, 0.52)),
        material="warning_label",
        name="nameplate",
    )

    frame.visual(
        Box((0.085, 0.11, SPINDLE_Z - 0.12)),
        origin=Origin(xyz=(-0.08, 0.0, (SPINDLE_Z - 0.12) * 0.5 + 0.08)),
        material="painted_steel",
        name="bearing_post_0",
    )
    frame.visual(
        _ring_mesh("bearing_ring_0", outer_radius=0.074, inner_radius=0.030, width=0.060),
        origin=Origin(xyz=(-0.08, 0.0, SPINDLE_Z)),
        material="dark_cast_metal",
        name="bearing_ring_0",
    )
    frame.visual(
        _ring_mesh("bushing_0", outer_radius=0.043, inner_radius=0.024, width=0.066),
        origin=Origin(xyz=(-0.08, 0.0, SPINDLE_Z)),
        material="brass_bushing",
        name="bushing_0",
    )
    frame.visual(
        Box((0.12, 0.15, 0.026)),
        origin=Origin(xyz=(-0.08, 0.0, 0.62)),
        material="dark_cast_metal",
        name="bearing_cap_foot_0",
    )
    frame.visual(
        Box((0.085, 0.11, SPINDLE_Z - 0.12)),
        origin=Origin(xyz=(0.18, 0.0, (SPINDLE_Z - 0.12) * 0.5 + 0.08)),
        material="painted_steel",
        name="bearing_post_1",
    )
    frame.visual(
        _ring_mesh("bearing_ring_1", outer_radius=0.074, inner_radius=0.030, width=0.060),
        origin=Origin(xyz=(0.18, 0.0, SPINDLE_Z)),
        material="dark_cast_metal",
        name="bearing_ring_1",
    )
    frame.visual(
        _ring_mesh("bushing_1", outer_radius=0.043, inner_radius=0.024, width=0.066),
        origin=Origin(xyz=(0.18, 0.0, SPINDLE_Z)),
        material="brass_bushing",
        name="bushing_1",
    )
    frame.visual(
        Box((0.12, 0.15, 0.026)),
        origin=Origin(xyz=(0.18, 0.0, 0.62)),
        material="dark_cast_metal",
        name="bearing_cap_foot_1",
    )

    frame.visual(
        Cylinder(radius=0.018, length=0.20),
        origin=Origin(xyz=(-0.255, 0.0, MOTOR_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material="brushed_steel",
        name="motor_output_shaft",
    )
    frame.visual(
        Cylinder(radius=0.084, length=0.045),
        origin=Origin(xyz=(BELT_X, 0.0, MOTOR_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_cast_metal",
        name="motor_pulley",
    )
    frame.visual(
        Cylinder(radius=0.052, length=0.052),
        origin=Origin(xyz=(BELT_X, 0.0, MOTOR_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material="brushed_steel",
        name="motor_pulley_hub",
    )
    frame.visual(
        Box((0.034, 0.017, 0.524)),
        origin=Origin(xyz=(BELT_X, 0.090, (SPINDLE_Z + MOTOR_Z) * 0.5)),
        material="black_rubber",
        name="drive_belt_side_0",
    )
    frame.visual(
        Box((0.034, 0.017, 0.524)),
        origin=Origin(xyz=(BELT_X, -0.090, (SPINDLE_Z + MOTOR_Z) * 0.5)),
        material="black_rubber",
        name="drive_belt_side_1",
    )
    frame.visual(
        Box((0.034, 0.198, 0.018)),
        origin=Origin(xyz=(BELT_X, 0.0, SPINDLE_Z + 0.089)),
        material="black_rubber",
        name="drive_belt_upper_span",
    )
    frame.visual(
        Box((0.034, 0.198, 0.018)),
        origin=Origin(xyz=(BELT_X, 0.0, MOTOR_Z - 0.089)),
        material="black_rubber",
        name="drive_belt_lower_span",
    )
    frame.visual(
        Box((0.026, 0.225, 0.48)),
        origin=Origin(xyz=(BELT_X - 0.065, 0.0, 0.54)),
        material="dark_cast_metal",
        name="belt_guard_backplate",
    )
    frame.visual(
        Box((0.040, 0.040, 0.30)),
        origin=Origin(xyz=(BELT_X - 0.065, 0.0, 0.23)),
        material="dark_cast_metal",
        name="belt_guard_stand",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.018, length=0.86),
        origin=Origin(xyz=(0.05, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="brushed_steel",
        name="shaft",
    )
    spindle.visual(
        Cylinder(radius=0.074, length=0.050),
        origin=Origin(xyz=(BELT_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_cast_metal",
        name="driven_pulley",
    )
    spindle.visual(
        Cylinder(radius=0.046, length=0.058),
        origin=Origin(xyz=(BELT_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="brushed_steel",
        name="driven_pulley_hub",
    )
    spindle.visual(
        Cylinder(radius=0.034, length=0.030),
        origin=Origin(xyz=(0.245, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="brushed_steel",
        name="inner_flange",
    )
    spindle.visual(
        Cylinder(radius=0.034, length=0.030),
        origin=Origin(xyz=(0.435, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="brushed_steel",
        name="outer_flange",
    )
    layer_xs = (0.286, 0.310, 0.334, 0.358, 0.382, 0.406)
    for idx, x in enumerate(layer_xs):
        spindle.visual(
            Cylinder(radius=0.158 if idx % 2 == 0 else 0.150, length=0.024),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material="buffing_cloth" if idx % 2 == 0 else "cloth_shadow",
            name=f"cloth_layer_{idx}",
        )
    spindle.visual(
        Cylinder(radius=0.060, length=0.150),
        origin=Origin(xyz=(0.346, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="buffing_cloth",
        name="stitched_center",
    )
    spindle.visual(
        Cylinder(radius=0.022, length=0.090),
        origin=Origin(xyz=(0.485, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="brushed_steel",
        name="threaded_nose",
    )

    model.articulation(
        "spindle_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, SPINDLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=45.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    spindle = object_model.get_part("spindle")
    spin = object_model.get_articulation("spindle_spin")

    ctx.check(
        "single continuous spindle joint",
        str(spin.articulation_type).endswith("CONTINUOUS"),
        details=f"joint type is {spin.articulation_type}",
    )
    ctx.check(
        "spindle rotates on horizontal shaft",
        tuple(round(v, 3) for v in spin.axis) == (1.0, 0.0, 0.0),
        details=f"axis={spin.axis}",
    )
    for bushing_name in ("bushing_0", "bushing_1", "bearing_ring_0", "bearing_ring_1"):
        ctx.allow_overlap(
            frame,
            spindle,
            elem_a=bushing_name,
            elem_b="shaft",
            reason="The visible steel shaft is intentionally captured through the bearing bore.",
        )
        ctx.expect_within(
            spindle,
            frame,
            axes="yz",
            inner_elem="shaft",
            outer_elem=bushing_name,
            margin=0.0,
            name=f"shaft is centered in {bushing_name}",
        )
        ctx.expect_overlap(
            spindle,
            frame,
            axes="x",
            elem_a="shaft",
            elem_b=bushing_name,
            min_overlap=0.040,
            name=f"shaft passes through {bushing_name}",
        )
    ctx.expect_overlap(
        spindle,
        frame,
        axes="x",
        elem_a="driven_pulley",
        elem_b="drive_belt_side_0",
        min_overlap=0.025,
        name="driven pulley sits in the belt plane",
    )
    ctx.expect_gap(
        frame,
        spindle,
        axis="y",
        positive_elem="drive_belt_side_0",
        negative_elem="driven_pulley",
        min_gap=0.001,
        max_gap=0.014,
        name="front belt run kisses the driven pulley",
    )
    ctx.expect_gap(
        spindle,
        frame,
        axis="y",
        positive_elem="driven_pulley",
        negative_elem="drive_belt_side_1",
        min_gap=0.001,
        max_gap=0.014,
        name="rear belt run kisses the driven pulley",
    )
    ctx.expect_gap(
        frame,
        spindle,
        axis="z",
        positive_elem="drive_belt_upper_span",
        negative_elem="driven_pulley",
        min_gap=0.001,
        max_gap=0.020,
        name="upper belt span clears the driven pulley crown",
    )
    with ctx.pose({spin: 1.25}):
        ctx.expect_overlap(
            spindle,
            frame,
            axes="x",
            elem_a="shaft",
            elem_b="bushing_0",
            min_overlap=0.040,
            name="rotating shaft remains captured in front bushing",
        )

    return ctx.report()


object_model = build_object_model()
