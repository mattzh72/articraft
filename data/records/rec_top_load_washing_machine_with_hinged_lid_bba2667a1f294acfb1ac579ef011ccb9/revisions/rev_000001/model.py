from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.56
BODY_D = 0.63
BODY_H = 0.80
TOP_Z = 0.815
LID_HINGE_Y = 0.245
LID_HINGE_Z = 0.8545
TUB_X = 0.0
TUB_Y = -0.040
TUB_Z = 0.290
CONSOLE_ROLL = math.radians(-20.0)
CONSOLE_CENTER = (0.0, 0.335, 0.925)
CONSOLE_THICKNESS = 0.004
CONTROL_RPY = (math.radians(70.0), 0.0, 0.0)


def _rot_x_point(x: float, y: float, z: float, roll: float) -> tuple[float, float, float]:
    c = math.cos(roll)
    s = math.sin(roll)
    return (x, y * c - z * s, y * s + z * c)


def _console_face_origin(local_x: float, local_z: float) -> Origin:
    """Point on the sloped console's front face, with +Z normal pointing outward."""
    px, py, pz = _rot_x_point(local_x, -CONSOLE_THICKNESS / 2.0, local_z, CONSOLE_ROLL)
    return Origin(
        xyz=(
            CONSOLE_CENTER[0] + px,
            CONSOLE_CENTER[1] + py,
            CONSOLE_CENTER[2] + pz,
        ),
        rpy=CONTROL_RPY,
    )


def _hollow_tub_mesh():
    height = 0.480
    bottom = 0.038
    shell = (
        cq.Workplane("XY")
        .circle(0.212)
        .extrude(height)
        .faces(">Z")
        .workplane()
        .circle(0.186)
        .cutBlind(-(height - bottom))
    )
    rolled_lip = (
        cq.Workplane("XY")
        .workplane(offset=height - 0.030)
        .circle(0.226)
        .circle(0.178)
        .extrude(0.030)
    )
    return shell.union(rolled_lip)


def _rounded_lid_mesh():
    return (
        cq.Workplane("XY")
        .box(0.510, 0.492, 0.030)
        .edges("|Z")
        .fillet(0.018)
        .edges("#Z")
        .fillet(0.004)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_apartment_top_load_washer")

    enamel = Material("white_enamel", rgba=(0.86, 0.88, 0.86, 1.0))
    seam = Material("shadow_seam", rgba=(0.035, 0.040, 0.045, 1.0))
    stainless = Material("perforated_stainless", rgba=(0.70, 0.74, 0.76, 1.0))
    dark_plastic = Material("dark_plastic", rgba=(0.045, 0.047, 0.052, 1.0))
    soft_blue = Material("softener_blue", rgba=(0.58, 0.77, 0.90, 1.0))
    green = Material("start_green", rgba=(0.10, 0.55, 0.22, 1.0))
    grey = Material("button_grey", rgba=(0.30, 0.32, 0.34, 1.0))
    dial_material = Material("warm_white_dial", rgba=(0.94, 0.93, 0.88, 1.0))
    metal = Material("hinge_metal", rgba=(0.55, 0.57, 0.58, 1.0))

    cabinet = model.part("cabinet")
    # Narrow apartment-width body with an open top deck rather than a solid block.
    cabinet.visual(Box((0.035, BODY_D, BODY_H)), origin=Origin(xyz=(-0.2625, 0.0, BODY_H / 2.0)), material=enamel, name="side_panel_0")
    cabinet.visual(Box((0.035, BODY_D, BODY_H)), origin=Origin(xyz=(0.2625, 0.0, BODY_H / 2.0)), material=enamel, name="side_panel_1")
    cabinet.visual(Box((BODY_W, 0.035, BODY_H)), origin=Origin(xyz=(0.0, -0.3125, BODY_H / 2.0)), material=enamel, name="front_panel")
    cabinet.visual(Box((BODY_W, 0.035, BODY_H)), origin=Origin(xyz=(0.0, 0.3125, BODY_H / 2.0)), material=enamel, name="rear_panel")
    cabinet.visual(Box((BODY_W, BODY_D, 0.060)), origin=Origin(xyz=(0.0, 0.0, 0.030)), material=enamel, name="bottom_plinth")
    cabinet.visual(Box((BODY_W, 0.120, 0.035)), origin=Origin(xyz=(0.0, -0.255, TOP_Z)), material=enamel, name="top_front_rail")
    cabinet.visual(Box((0.050, 0.490, 0.035)), origin=Origin(xyz=(-0.255, -0.010, TOP_Z)), material=enamel, name="top_side_rail_0")
    cabinet.visual(Box((0.050, 0.490, 0.035)), origin=Origin(xyz=(0.255, -0.010, TOP_Z)), material=enamel, name="top_side_rail_1")
    cabinet.visual(Box((BODY_W, 0.095, 0.035)), origin=Origin(xyz=(0.0, 0.235, TOP_Z)), material=enamel, name="top_rear_rail")
    cabinet.visual(Box((0.470, 0.475, 0.012)), origin=Origin(xyz=(0.0, -0.040, TOP_Z - 0.020)), material=seam, name="dark_opening_shadow")

    cabinet.visual(
        Box((0.520, CONSOLE_THICKNESS, 0.220)),
        origin=Origin(xyz=CONSOLE_CENTER, rpy=(CONSOLE_ROLL, 0.0, 0.0)),
        material=enamel,
        name="sloped_console_face",
    )
    cabinet.visual(Box((0.540, 0.120, 0.045)), origin=Origin(xyz=(0.0, 0.320, 0.814)), material=enamel, name="console_base_bridge")
    cabinet.visual(Box((BODY_W, 0.040, 0.215)), origin=Origin(xyz=(0.0, 0.394, 0.930)), material=enamel, name="console_back")
    cabinet.visual(Box((0.035, 0.130, 0.205)), origin=Origin(xyz=(-0.275, 0.352, 0.925)), material=enamel, name="console_cheek_0")
    cabinet.visual(Box((0.035, 0.130, 0.205)), origin=Origin(xyz=(0.275, 0.352, 0.925)), material=enamel, name="console_cheek_1")
    cabinet.visual(Box((0.540, 0.030, 0.035)), origin=Origin(xyz=(0.0, 0.376, 1.030)), material=enamel, name="console_top_lip")

    cabinet.visual(
        Cylinder(radius=0.030, length=0.231),
        origin=Origin(xyz=(TUB_X, TUB_Y, 0.1755)),
        material=metal,
        name="drive_spindle",
    )

    # Split rear hinge knuckles leave a clear gap for the moving lid barrel.
    for suffix, x in (("0", -0.205), ("1", 0.205)):
        cabinet.visual(
            Box((0.092, 0.030, 0.038)),
            origin=Origin(xyz=(x, LID_HINGE_Y + 0.022, LID_HINGE_Z - 0.017)),
            material=enamel,
            name=f"hinge_bracket_{suffix}",
        )
        cabinet.visual(
            Cylinder(radius=0.014, length=0.082),
            origin=Origin(xyz=(x, LID_HINGE_Y + 0.024, LID_HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name=f"hinge_knuckle_{suffix}",
        )

    tub = model.part("tub")
    tub.visual(mesh_from_cadquery(_hollow_tub_mesh(), "deep_hollow_tub"), material=stainless, name="deep_hollow_tub")
    tub.visual(Cylinder(radius=0.180, length=0.010), origin=Origin(xyz=(0.0, 0.0, 0.043)), material=dark_plastic, name="dark_tub_bottom")
    tub.visual(Cylinder(radius=0.045, length=0.340), origin=Origin(xyz=(0.0, 0.0, 0.190)), material=enamel, name="agitator_post")
    for i, yaw in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        tub.visual(
            Box((0.170, 0.026, 0.090)),
            origin=Origin(xyz=(0.074 * math.cos(yaw), 0.074 * math.sin(yaw), 0.088), rpy=(0.0, 0.0, yaw)),
            material=enamel,
            name=f"agitator_fin_{i}",
        )

    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_rounded_lid_mesh(), "flat_rounded_lid"), origin=Origin(xyz=(0.0, -0.246, -0.007)), material=enamel, name="flat_lid_panel")
    lid.visual(Box((0.155, 0.018, 0.006)), origin=Origin(xyz=(0.0, -0.340, 0.011)), material=dark_plastic, name="finger_recess")
    lid.visual(
        Cylinder(radius=0.012, length=0.290),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hinge_barrel",
    )

    timer_dial = model.part("timer_dial")
    timer_knob = KnobGeometry(
        0.088,
        0.034,
        body_style="skirted",
        top_diameter=0.066,
        skirt=KnobSkirt(0.100, 0.006, flare=0.06, chamfer=0.001),
        grip=KnobGrip(style="fluted", count=24, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
        center=False,
    )
    timer_dial.visual(mesh_from_geometry(timer_knob, "timer_dial_knob"), material=dial_material, name="timer_dial_knob")

    start_button = model.part("start_button")
    start_button.visual(Cylinder(radius=0.026, length=0.018), origin=Origin(xyz=(0.0, 0.0, 0.009)), material=green, name="round_start_cap")
    start_button.visual(Cylinder(radius=0.012, length=0.002), origin=Origin(xyz=(0.0, 0.0, 0.019)), material=dial_material, name="start_highlight")

    for idx in (0, 1):
        button = model.part(f"mode_button_{idx}")
        button.visual(Box((0.058, 0.036, 0.016)), origin=Origin(xyz=(0.0, 0.0, 0.008)), material=grey, name="mode_button_cap")

    softener_cap = model.part("softener_cap")
    softener_cap.visual(Cylinder(radius=0.057, length=0.046), origin=Origin(xyz=(0.0, 0.0, 0.023)), material=soft_blue, name="softener_cap")
    softener_cap.visual(Cylinder(radius=0.032, length=0.004), origin=Origin(xyz=(0.0, 0.0, 0.048)), material=dial_material, name="cap_top_insert")

    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, LID_HINGE_Y, LID_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.45),
    )
    model.articulation(
        "cabinet_to_tub",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=tub,
        origin=Origin(xyz=(TUB_X, TUB_Y, TUB_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=30.0),
    )
    model.articulation(
        "cabinet_to_timer_dial",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=timer_dial,
        origin=_console_face_origin(-0.150, 0.035),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )
    model.articulation(
        "cabinet_to_start_button",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=start_button,
        origin=_console_face_origin(0.055, 0.045),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.20, lower=0.0, upper=0.010),
    )
    for idx, x in enumerate((0.140, 0.220)):
        model.articulation(
            f"cabinet_to_mode_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=f"mode_button_{idx}",
            origin=_console_face_origin(x, -0.038),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.5, velocity=0.18, lower=0.0, upper=0.008),
        )
    model.articulation(
        "tub_to_softener_cap",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=softener_cap,
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    tub = object_model.get_part("tub")
    timer_dial = object_model.get_part("timer_dial")
    start_button = object_model.get_part("start_button")
    softener_cap = object_model.get_part("softener_cap")
    lid_hinge = object_model.get_articulation("cabinet_to_lid")
    tub_spin = object_model.get_articulation("cabinet_to_tub")
    timer_spin = object_model.get_articulation("cabinet_to_timer_dial")
    cap_spin = object_model.get_articulation("tub_to_softener_cap")
    start_push = object_model.get_articulation("cabinet_to_start_button")

    ctx.allow_overlap(
        cabinet,
        timer_dial,
        elem_a="sloped_console_face",
        elem_b="timer_dial_knob",
        reason="The timer dial skirt is intentionally seated flush into the simplified sloped console face.",
    )
    ctx.allow_overlap(
        cabinet,
        start_button,
        elem_a="sloped_console_face",
        elem_b="round_start_cap",
        reason="The start push cap is intentionally seated flush in the simplified sloped console face.",
    )

    ctx.check("tub spins continuously", tub_spin.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("timer dial spins continuously", timer_spin.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("softener cap spins continuously", cap_spin.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("start button is prismatic", start_push.articulation_type == ArticulationType.PRISMATIC)
    for idx in (0, 1):
        joint = object_model.get_articulation(f"cabinet_to_mode_button_{idx}")
        ctx.check(f"mode button {idx} is prismatic", joint.articulation_type == ArticulationType.PRISMATIC)

    ctx.expect_gap(lid, cabinet, axis="z", min_gap=0.0, max_gap=0.010, positive_elem="flat_lid_panel", negative_elem="top_front_rail", name="closed lid rests just above top deck")
    ctx.expect_overlap(lid, cabinet, axes="xy", min_overlap=0.18, elem_a="flat_lid_panel", elem_b="dark_opening_shadow", name="flat lid covers the wash opening footprint")
    ctx.expect_gap(cabinet, tub, axis="z", min_gap=0.020, positive_elem="top_front_rail", negative_elem="deep_hollow_tub", name="deep tub sits visibly below the deck opening")
    ctx.expect_within(tub, cabinet, axes="xy", margin=0.012, inner_elem="deep_hollow_tub", outer_elem="dark_opening_shadow", name="round hollow tub is centered under the opening")
    ctx.expect_contact(softener_cap, tub, elem_a="softener_cap", elem_b="agitator_post", contact_tol=0.002, name="softener cap sits on the agitator post")
    ctx.expect_overlap(timer_dial, cabinet, axes="xy", min_overlap=0.030, elem_a="timer_dial_knob", elem_b="sloped_console_face", name="timer dial is mounted on the rear console")
    ctx.expect_contact(timer_dial, cabinet, elem_a="timer_dial_knob", elem_b="sloped_console_face", contact_tol=0.001, name="timer dial skirt contacts console face")
    ctx.expect_contact(start_button, cabinet, elem_a="round_start_cap", elem_b="sloped_console_face", contact_tol=0.001, name="start button cap contacts console face")

    closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.20}):
        open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward on rear hinge",
        closed_aabb is not None and open_aabb is not None and open_aabb[1][2] > closed_aabb[1][2] + 0.30,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    rest_pos = ctx.part_world_position(start_button)
    with ctx.pose({start_push: 0.010}):
        pushed_pos = ctx.part_world_position(start_button)
    ctx.check(
        "start button pushes inward into console",
        rest_pos is not None
        and pushed_pos is not None
        and pushed_pos[1] > rest_pos[1] + 0.006
        and pushed_pos[2] < rest_pos[2] - 0.001,
        details=f"rest={rest_pos}, pushed={pushed_pos}",
    )

    return ctx.report()


object_model = build_object_model()
