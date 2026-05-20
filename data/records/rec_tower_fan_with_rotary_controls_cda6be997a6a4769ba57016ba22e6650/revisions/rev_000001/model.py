from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    VentGrilleGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _build_body_shell() -> object:
    outer = (
        cq.Workplane("XY")
        .box(0.108, 0.118, 0.670, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
    )
    inner = cq.Workplane("XY").box(0.090, 0.098, 0.605, centered=(True, True, False)).translate(
        (0.0, 0.0, 0.032)
    )
    front_window = cq.Workplane("XY").box(0.078, 0.042, 0.555, centered=(True, True, False)).translate(
        (0.0, 0.051, 0.073)
    )
    rear_window = cq.Workplane("XY").box(0.066, 0.038, 0.420, centered=(True, True, False)).translate(
        (0.0, -0.051, 0.118)
    )
    return outer.cut(inner).cut(front_window).cut(rear_window)


def _build_blower_spider() -> object:
    ring = cq.Workplane("XY").circle(0.011).circle(0.0045).extrude(0.004).translate((0.0, 0.0, -0.002))
    arm = (
        cq.Workplane("XY")
        .box(0.022, 0.004, 0.004, centered=(False, True, True))
        .translate((0.011, 0.0, 0.0))
    )
    return ring.union(arm)


def _build_knob_part(
    model: ArticulatedObject,
    *,
    name: str,
    radius: float,
    body_height: float = 0.016,
):
    knob = model.part(name)
    knob.visual(
        Cylinder(radius=radius, length=body_height),
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
        material="knob_black",
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=radius * 0.72, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, body_height + 0.002)),
        material="knob_black",
        name="knob_cap",
    )
    knob.visual(
        Box((radius * 0.60, 0.003, 0.002)),
        origin=Origin(xyz=(radius * 0.45, 0.0, body_height + 0.003)),
        material="indicator",
        name="pointer",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=radius, length=body_height + 0.004),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )
    return knob


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_fan")

    model.material("base_silver", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("body_graphite", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("grille_black", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("knob_black", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("indicator", rgba=(0.82, 0.83, 0.84, 1.0))
    model.material("spindle_dark", rgba=(0.18, 0.18, 0.19, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.125, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material="base_silver",
        name="base_plate",
    )
    stand.visual(
        Cylinder(radius=0.108, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material="grille_black",
        name="foot_ring",
    )
    stand.visual(
        Cylinder(radius=0.034, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material="base_silver",
        name="neck",
    )
    stand.visual(
        Cylinder(radius=0.056, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material="body_graphite",
        name="turntable",
    )
    stand.inertial = Inertial.from_geometry(
        Cylinder(radius=0.125, length=0.084),
        mass=5.2,
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
    )

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.050, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material="body_graphite",
        name="base_ring",
    )
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "tower_fan_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material="body_graphite",
        name="shell",
    )
    body.visual(
        Box((0.074, 0.084, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.692)),
        material="body_graphite",
        name="control_deck",
    )
    body.visual(
        mesh_from_geometry(
            VentGrilleGeometry(
                (0.078, 0.555),
                frame=0.006,
                face_thickness=0.003,
                duct_depth=0.006,
                duct_wall=0.002,
                slat_pitch=0.016,
                slat_width=0.006,
                slat_angle_deg=18.0,
                corner_radius=0.010,
            ),
            "tower_fan_front_grille",
        ),
        origin=Origin(xyz=(0.0, 0.059, 0.3685), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="grille_black",
        name="front_grille",
    )
    body.visual(
        mesh_from_geometry(
            PerforatedPanelGeometry(
                (0.066, 0.420),
                0.003,
                hole_diameter=0.005,
                pitch=(0.010, 0.010),
                frame=0.007,
                corner_radius=0.012,
            ),
            "tower_fan_rear_intake",
        ),
        origin=Origin(xyz=(0.0, -0.0585, 0.3480), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="grille_black",
        name="rear_intake",
    )
    body.visual(
        Cylinder(radius=0.0045, length=0.637),
        origin=Origin(xyz=(0.0, 0.0, 0.3685)),
        material="spindle_dark",
        name="spindle",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.120, 0.120, 0.706)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.0, 0.353)),
    )

    blower = model.part("blower")
    blower.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                outer_radius=0.032,
                inner_radius=0.014,
                width=0.560,
                blade_count=30,
                blade_thickness=0.0022,
                blade_sweep_deg=32.0,
            ),
            "tower_fan_blower",
        ),
        material="base_silver",
        name="blower_wheel",
    )
    blower.visual(
        mesh_from_cadquery(_build_blower_spider(), "tower_fan_blower_spider"),
        origin=Origin(xyz=(0.0, 0.0, 0.278)),
        material="spindle_dark",
        name="bearing_spider",
    )
    blower.visual(
        Box((0.010, 0.004, 0.006)),
        origin=Origin(xyz=(0.021, 0.0, 0.276)),
        material="indicator",
        name="drive_tab",
    )
    blower.inertial = Inertial.from_geometry(
        Cylinder(radius=0.032, length=0.560),
        mass=0.55,
        origin=Origin(),
    )

    timer_knob = _build_knob_part(model, name="timer_knob", radius=0.0135)
    speed_knob = _build_knob_part(model, name="speed_knob", radius=0.0150)

    model.articulation(
        "stand_to_body",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=-0.95,
            upper=0.95,
        ),
    )
    model.articulation(
        "body_to_blower",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=blower,
        origin=Origin(xyz=(0.0, 0.0, 0.3685)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=28.0,
        ),
    )
    model.articulation(
        "body_to_timer_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=timer_knob,
        origin=Origin(xyz=(-0.022, -0.002, 0.696)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=3.0,
            lower=0.0,
            upper=4.8,
        ),
    )
    model.articulation(
        "body_to_speed_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=speed_knob,
        origin=Origin(xyz=(0.022, -0.002, 0.696)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=3.0,
            lower=0.0,
            upper=4.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    body = object_model.get_part("body")
    blower = object_model.get_part("blower")
    timer_knob = object_model.get_part("timer_knob")
    speed_knob = object_model.get_part("speed_knob")

    oscillation = object_model.get_articulation("stand_to_body")
    blower_spin = object_model.get_articulation("body_to_blower")
    timer_spin = object_model.get_articulation("body_to_timer_knob")
    speed_spin = object_model.get_articulation("body_to_speed_knob")

    with ctx.pose({oscillation: 0.0, blower_spin: 0.0, timer_spin: 0.0, speed_spin: 0.0}):
        ctx.expect_gap(
            body,
            stand,
            axis="z",
            positive_elem="base_ring",
            negative_elem="turntable",
            max_gap=0.001,
            max_penetration=1e-6,
            name="body sits on the stand turntable",
        )
        ctx.expect_overlap(
            body,
            stand,
            axes="xy",
            elem_a="base_ring",
            elem_b="turntable",
            min_overlap=0.090,
            name="body base stays centered over the stand",
        )
        ctx.expect_gap(
            timer_knob,
            body,
            axis="z",
            positive_elem="knob_body",
            negative_elem="control_deck",
            max_gap=0.001,
            max_penetration=0.0,
            name="timer knob seats on the control deck",
        )
        ctx.expect_gap(
            speed_knob,
            body,
            axis="z",
            positive_elem="knob_body",
            negative_elem="control_deck",
            max_gap=0.001,
            max_penetration=0.0,
            name="speed knob seats on the control deck",
        )
        ctx.expect_overlap(
            timer_knob,
            body,
            axes="xy",
            elem_a="knob_body",
            elem_b="control_deck",
            min_overlap=0.020,
            name="timer knob stays within the top cap footprint",
        )
        ctx.expect_overlap(
            speed_knob,
            body,
            axes="xy",
            elem_a="knob_body",
            elem_b="control_deck",
            min_overlap=0.020,
            name="speed knob stays within the top cap footprint",
        )

    rest_speed_pos = ctx.part_world_position(speed_knob)
    with ctx.pose({oscillation: 0.75}):
        swung_speed_pos = ctx.part_world_position(speed_knob)
    ctx.check(
        "body oscillation swings the control cluster",
        rest_speed_pos is not None
        and swung_speed_pos is not None
        and math.hypot(
            swung_speed_pos[0] - rest_speed_pos[0],
            swung_speed_pos[1] - rest_speed_pos[1],
        )
        > 0.012,
        details=f"rest={rest_speed_pos}, swung={swung_speed_pos}",
    )

    with ctx.pose({blower_spin: 0.0}):
        blower_tab_rest = _aabb_center(ctx.part_element_world_aabb(blower, elem="drive_tab"))
    with ctx.pose({blower_spin: math.pi / 2.0}):
        blower_tab_quarter = _aabb_center(ctx.part_element_world_aabb(blower, elem="drive_tab"))
    ctx.check(
        "blower wheel rotates inside the housing",
        blower_tab_rest is not None
        and blower_tab_quarter is not None
        and abs(blower_tab_quarter[0] - blower_tab_rest[0]) > 0.012
        and abs(blower_tab_quarter[1] - blower_tab_rest[1]) > 0.012,
        details=f"rest={blower_tab_rest}, quarter_turn={blower_tab_quarter}",
    )

    with ctx.pose({timer_spin: 0.0, speed_spin: 0.0}):
        timer_pointer_rest = _aabb_center(ctx.part_element_world_aabb(timer_knob, elem="pointer"))
        speed_pointer_rest = _aabb_center(ctx.part_element_world_aabb(speed_knob, elem="pointer"))
    with ctx.pose({timer_spin: math.pi / 2.0, speed_spin: 0.0}):
        timer_pointer_turned = _aabb_center(ctx.part_element_world_aabb(timer_knob, elem="pointer"))
        speed_pointer_still = _aabb_center(ctx.part_element_world_aabb(speed_knob, elem="pointer"))
    ctx.check(
        "timer knob rotates independently",
        timer_pointer_rest is not None
        and timer_pointer_turned is not None
        and speed_pointer_rest is not None
        and speed_pointer_still is not None
        and abs(timer_pointer_turned[1] - timer_pointer_rest[1]) > 0.006
        and abs(speed_pointer_still[1] - speed_pointer_rest[1]) < 0.001,
        details=(
            f"timer_rest={timer_pointer_rest}, timer_turned={timer_pointer_turned}, "
            f"speed_rest={speed_pointer_rest}, speed_still={speed_pointer_still}"
        ),
    )
    with ctx.pose({timer_spin: 0.0, speed_spin: math.pi / 2.0}):
        timer_pointer_still = _aabb_center(ctx.part_element_world_aabb(timer_knob, elem="pointer"))
        speed_pointer_turned = _aabb_center(ctx.part_element_world_aabb(speed_knob, elem="pointer"))
    ctx.check(
        "speed knob rotates independently",
        timer_pointer_rest is not None
        and timer_pointer_still is not None
        and speed_pointer_rest is not None
        and speed_pointer_turned is not None
        and abs(speed_pointer_turned[1] - speed_pointer_rest[1]) > 0.006
        and abs(timer_pointer_still[1] - timer_pointer_rest[1]) < 0.001,
        details=(
            f"timer_rest={timer_pointer_rest}, timer_still={timer_pointer_still}, "
            f"speed_rest={speed_pointer_rest}, speed_turned={speed_pointer_turned}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
