from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_geometry,
)


BODY_W = 0.60
BODY_D = 0.44
BODY_H = 0.37
FRONT_Y = -BODY_D / 2.0

LEFT_PANEL_W = 0.40
LEFT_PANEL_H = 0.31
LEFT_PANEL_T = 0.016
LEFT_PANEL_X = -0.085

CONTROL_POD_W = 0.15
CONTROL_POD_H = 0.27
CONTROL_POD_T = 0.016
CONTROL_POD_X = 0.205
CONTROL_POD_Z = 0.01

OUTLET_W = 0.34
OUTLET_H = 0.10
OUTLET_X = -0.085
OUTLET_Z = 0.095
OUTLET_DEPTH = 0.006

INTAKE_W = 0.34
INTAKE_H = 0.18
INTAKE_X = -0.085
INTAKE_Z = -0.065
INTAKE_DEPTH = 0.004

FLAP_W = 0.31
FLAP_D = 0.042
FLAP_T = 0.007
FLAP_HINGE_Y = FRONT_Y - 0.0165
FLAP_HINGE_Z = OUTLET_Z - OUTLET_H / 2.0

SLOT_TRACK_W = 0.088
SLOT_TRACK_H = 0.020
SLOT_TRACK_T = 0.002
SLIDER_TRAVEL = 0.060
SLIDER_Y = FRONT_Y - 0.021
FAN_SLOT_Z = 0.070
TEMP_SLOT_Z = -0.010
SLIDER_START_X = CONTROL_POD_X - 0.030


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="window_air_conditioner")

    cabinet_white = model.material("cabinet_white", color=(0.91, 0.92, 0.89))
    trim_white = model.material("trim_white", color=(0.96, 0.96, 0.94))
    vent_gray = model.material("vent_gray", color=(0.60, 0.64, 0.67))
    slot_gray = model.material("slot_gray", color=(0.28, 0.30, 0.32))
    lever_gray = model.material("lever_gray", color=(0.83, 0.84, 0.82))
    latch_gray = model.material("latch_gray", color=(0.71, 0.73, 0.74))

    body = model.part("body")
    body.visual(
        Box((BODY_W, BODY_D, BODY_H)),
        material=cabinet_white,
        name="cabinet_shell",
    )
    body.visual(
        Box((LEFT_PANEL_W, LEFT_PANEL_T, LEFT_PANEL_H)),
        origin=Origin(xyz=(LEFT_PANEL_X, FRONT_Y - LEFT_PANEL_T / 2.0, 0.0)),
        material=trim_white,
        name="front_panel",
    )
    body.visual(
        mesh_from_geometry(
            VentGrilleGeometry(
                (OUTLET_W, OUTLET_H),
                frame=0.010,
                face_thickness=OUTLET_DEPTH,
                duct_depth=0.010,
                duct_wall=0.003,
                slat_pitch=0.017,
                slat_width=0.008,
                slat_angle_deg=24.0,
                slats=VentGrilleSlats(profile="flat", direction="up", divider_count=3, divider_width=0.003),
                sleeve=VentGrilleSleeve(style="short", depth=0.010, wall=0.003),
            ),
            "outlet_grille",
        ),
        origin=Origin(
            xyz=(OUTLET_X, FRONT_Y - 0.011, OUTLET_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=vent_gray,
        name="outlet_grille",
    )
    body.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (INTAKE_W, INTAKE_H),
                INTAKE_DEPTH,
                slot_size=(0.024, 0.0045),
                pitch=(0.032, 0.015),
                frame=0.010,
                corner_radius=0.004,
                slot_angle_deg=0.0,
                stagger=False,
            ),
            "intake_grille",
        ),
        origin=Origin(
            xyz=(INTAKE_X, FRONT_Y - 0.0105, INTAKE_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_white,
        name="intake_grille",
    )
    body.visual(
        Box((CONTROL_POD_W, CONTROL_POD_T, CONTROL_POD_H)),
        origin=Origin(xyz=(CONTROL_POD_X, FRONT_Y - CONTROL_POD_T / 2.0, CONTROL_POD_Z)),
        material=trim_white,
        name="control_pod",
    )
    body.visual(
        Box((SLOT_TRACK_W, SLOT_TRACK_T, SLOT_TRACK_H)),
        origin=Origin(xyz=(CONTROL_POD_X, FRONT_Y - 0.015, FAN_SLOT_Z)),
        material=slot_gray,
        name="fan_slot_track",
    )
    body.visual(
        Box((SLOT_TRACK_W, SLOT_TRACK_T, SLOT_TRACK_H)),
        origin=Origin(xyz=(CONTROL_POD_X, FRONT_Y - 0.015, TEMP_SLOT_Z)),
        material=slot_gray,
        name="temp_slot_track",
    )
    body.visual(
        Box((0.042, 0.003, 0.010)),
        origin=Origin(xyz=(CONTROL_POD_X, FRONT_Y - 0.017, 0.125)),
        material=slot_gray,
        name="fan_label_bar",
    )
    body.visual(
        Box((0.060, 0.003, 0.010)),
        origin=Origin(xyz=(CONTROL_POD_X, FRONT_Y - 0.017, 0.044)),
        material=slot_gray,
        name="temp_label_bar",
    )

    vent_flap = model.part("vent_flap")
    vent_flap.visual(
        Cylinder(radius=0.0035, length=FLAP_W),
        origin=Origin(
            xyz=(0.0, -0.003, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=latch_gray,
        name="flap_barrel",
    )
    vent_flap.visual(
        Box((FLAP_W, FLAP_D, FLAP_T)),
        origin=Origin(xyz=(0.0, -FLAP_D / 2.0, -FLAP_T / 2.0)),
        material=trim_white,
        name="flap_blade",
    )

    fan_slider = model.part("fan_slider")
    fan_slider.visual(
        Box((0.024, 0.010, 0.032)),
        origin=Origin(),
        material=lever_gray,
        name="fan_handle",
    )
    fan_slider.visual(
        Box((0.010, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, -0.008, 0.010)),
        material=latch_gray,
        name="fan_thumb",
    )

    temperature_slider = model.part("temperature_slider")
    temperature_slider.visual(
        Box((0.024, 0.010, 0.032)),
        origin=Origin(),
        material=lever_gray,
        name="temp_handle",
    )
    temperature_slider.visual(
        Box((0.010, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, -0.008, 0.010)),
        material=latch_gray,
        name="temp_thumb",
    )

    model.articulation(
        "vent_tilt",
        ArticulationType.REVOLUTE,
        parent=body,
        child=vent_flap,
        origin=Origin(xyz=(OUTLET_X, FLAP_HINGE_Y, FLAP_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.45, upper=0.85),
    )
    model.articulation(
        "fan_control",
        ArticulationType.PRISMATIC,
        parent=body,
        child=fan_slider,
        origin=Origin(xyz=(SLIDER_START_X, SLIDER_Y, FAN_SLOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.20, lower=0.0, upper=SLIDER_TRAVEL),
    )
    model.articulation(
        "temp_control",
        ArticulationType.PRISMATIC,
        parent=body,
        child=temperature_slider,
        origin=Origin(xyz=(SLIDER_START_X, SLIDER_Y, TEMP_SLOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.20, lower=0.0, upper=SLIDER_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    vent_flap = object_model.get_part("vent_flap")
    fan_slider = object_model.get_part("fan_slider")
    temperature_slider = object_model.get_part("temperature_slider")
    vent_tilt = object_model.get_articulation("vent_tilt")
    fan_control = object_model.get_articulation("fan_control")
    temp_control = object_model.get_articulation("temp_control")

    body_aabb = ctx.part_world_aabb(body)
    ctx.check("body_aabb_present", body_aabb is not None, "Expected body bounds.")
    if body_aabb is not None:
        mins, maxs = body_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check("window_ac_width", 0.58 <= size[0] <= 0.62, details=f"size={size!r}")
        ctx.check("window_ac_depth", 0.44 <= size[1] <= 0.47, details=f"size={size!r}")
        ctx.check("window_ac_height", 0.36 <= size[2] <= 0.38, details=f"size={size!r}")

    ctx.expect_overlap(
        vent_flap,
        body,
        axes="x",
        elem_a="flap_blade",
        elem_b="outlet_grille",
        min_overlap=0.28,
        name="vent flap spans the outlet width",
    )
    ctx.expect_gap(
        body,
        vent_flap,
        axis="y",
        positive_elem="cabinet_shell",
        negative_elem="flap_blade",
        min_gap=0.005,
        max_gap=0.030,
        name="vent flap sits just proud of the cabinet face",
    )

    flap_rest = ctx.part_world_aabb(vent_flap)
    with ctx.pose({vent_tilt: vent_tilt.motion_limits.upper}):
        ctx.expect_overlap(
            vent_flap,
            body,
            axes="x",
            elem_a="flap_blade",
            elem_b="outlet_grille",
            min_overlap=0.26,
            name="opened vent flap stays aligned with the outlet",
        )
        flap_open = ctx.part_world_aabb(vent_flap)
    ctx.check(
        "vent flap opens upward",
        flap_rest is not None
        and flap_open is not None
        and float(flap_open[1][2]) > float(flap_rest[1][2]) + 0.020,
        details=f"rest={flap_rest}, open={flap_open}",
    )

    ctx.expect_overlap(
        fan_slider,
        body,
        axes="x",
        elem_a="fan_handle",
        elem_b="fan_slot_track",
        min_overlap=0.020,
        name="fan lever stays over its slot",
    )
    ctx.expect_overlap(
        temperature_slider,
        body,
        axes="x",
        elem_a="temp_handle",
        elem_b="temp_slot_track",
        min_overlap=0.020,
        name="temperature lever stays over its slot",
    )
    ctx.expect_gap(
        body,
        fan_slider,
        axis="y",
        positive_elem="control_pod",
        negative_elem="fan_handle",
        max_gap=0.015,
        max_penetration=1e-5,
        name="fan lever stays guided against the pod face",
    )
    ctx.expect_gap(
        body,
        temperature_slider,
        axis="y",
        positive_elem="control_pod",
        negative_elem="temp_handle",
        max_gap=0.015,
        max_penetration=1e-5,
        name="temperature lever stays guided against the pod face",
    )

    fan_rest = ctx.part_world_position(fan_slider)
    temp_rest = ctx.part_world_position(temperature_slider)
    with ctx.pose({fan_control: SLIDER_TRAVEL, temp_control: SLIDER_TRAVEL}):
        ctx.expect_overlap(
            fan_slider,
            body,
            axes="x",
            elem_a="fan_handle",
            elem_b="fan_slot_track",
            min_overlap=0.020,
            name="fan lever stays retained at max travel",
        )
        ctx.expect_overlap(
            temperature_slider,
            body,
            axes="x",
            elem_a="temp_handle",
            elem_b="temp_slot_track",
            min_overlap=0.020,
            name="temperature lever stays retained at max travel",
        )
        fan_extended = ctx.part_world_position(fan_slider)
        temp_extended = ctx.part_world_position(temperature_slider)
    ctx.check(
        "fan slider moves right",
        fan_rest is not None and fan_extended is not None and float(fan_extended[0]) > float(fan_rest[0]) + 0.045,
        details=f"rest={fan_rest}, extended={fan_extended}",
    )
    ctx.check(
        "temperature slider moves right",
        temp_rest is not None and temp_extended is not None and float(temp_extended[0]) > float(temp_rest[0]) + 0.045,
        details=f"rest={temp_rest}, extended={temp_extended}",
    )

    return ctx.report()


object_model = build_object_model()
