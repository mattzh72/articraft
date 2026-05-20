from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


BODY_R = 0.160
BODY_H = 0.460
WALL = 0.007

HINGE_Y = 0.185
HINGE_Z = BODY_H + 0.023
HINGE_R = 0.0075

LID_R = 0.176
LID_CENTER_Y = -HINGE_Y

PEDAL_Y = -0.193
PEDAL_Z = 0.064


def _sector_profile(inner_r: float, outer_r: float, start_deg: float, end_deg: float, steps: int = 28):
    """Counter-clockwise annular-sector profile in the XY plane."""
    start = math.radians(start_deg)
    end = math.radians(end_deg)
    outer = [
        (outer_r * math.cos(start + (end - start) * i / steps), outer_r * math.sin(start + (end - start) * i / steps))
        for i in range(steps + 1)
    ]
    inner = [
        (inner_r * math.cos(end - (end - start) * i / steps), inner_r * math.sin(end - (end - start) * i / steps))
        for i in range(steps + 1)
    ]
    return outer + inner


def _body_shell_geometry():
    # A thin stainless shell with a real hollow interior and a slightly rolled top.
    return LatheGeometry.from_shell_profiles(
        [(BODY_R, 0.014), (BODY_R, BODY_H - 0.018), (BODY_R + 0.003, BODY_H + 0.002)],
        [(BODY_R - WALL, 0.032), (BODY_R - WALL, BODY_H - 0.020), (BODY_R - 0.021, BODY_H + 0.002)],
        segments=112,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )


def _domed_lid_geometry():
    dome_h = 0.056
    shell_t = 0.004
    skirt_h = 0.028
    outer = []
    inner = []
    for i in range(18):
        t = i / 17.0
        # Smooth shallow spherical-cap-like profile: full radius at the rim,
        # gently rising to a pressed-metal dome near the center.
        r = LID_R * math.cos(t * math.pi / 2.0)
        z = 0.004 + dome_h * math.sin(t * math.pi / 2.0) ** 0.72
        outer.append((r, z))
        inner.append((max(0.0, r - shell_t), z - shell_t))

    # Include a short outside skirt that drops around the bin rim with clearance.
    outer_profile = [(LID_R, -skirt_h), (LID_R, 0.004)] + outer[1:]
    inner_profile = [(LID_R - 0.009, -skirt_h + shell_t), (LID_R - 0.009, 0.001)] + inner[1:]
    lid = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=112,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )
    lid.translate(0.0, LID_CENTER_Y, 0.0)
    return lid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stainless_step_bin")

    steel = model.material("brushed_stainless", rgba=(0.70, 0.73, 0.72, 1.0))
    shadow_steel = model.material("shadowed_stainless", rgba=(0.43, 0.46, 0.46, 1.0))
    black = model.material("black_rubber", rgba=(0.025, 0.024, 0.022, 1.0))
    dark = model.material("dark_plastic", rgba=(0.08, 0.085, 0.085, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_body_shell_geometry(), "body_shell"),
        material=steel,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=BODY_R - WALL * 1.4, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark,
        name="inner_floor",
    )
    body.visual(
        mesh_from_geometry(TorusGeometry(BODY_R - 0.007, 0.007, radial_segments=20, tubular_segments=96), "rolled_rim"),
        origin=Origin(xyz=(0.0, 0.0, BODY_H + 0.002)),
        material=steel,
        name="rolled_rim",
    )
    body.visual(
        mesh_from_geometry(TorusGeometry(BODY_R - 0.006, 0.009, radial_segments=20, tubular_segments=96), "base_foot_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=black,
        name="base_foot_ring",
    )
    body.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(_sector_profile(BODY_R - 0.008, BODY_R + 0.008, 42.0, 138.0), 0.032),
            "rear_hinge_band",
        ),
        origin=Origin(xyz=(0.0, 0.0, BODY_H - 0.034)),
        material=shadow_steel,
        name="rear_hinge_band",
    )

    # Interleaved rear hinge support: fixed outer barrels on the body, a central
    # barrel on the lid.  The lower feet tie the standoff back into the round wall.
    for i, x in enumerate((-0.065, 0.065)):
        body.visual(
            Cylinder(radius=HINGE_R, length=0.030),
            origin=Origin(xyz=(x, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=shadow_steel,
            name=f"hinge_barrel_{i}",
        )
        body.visual(
            Box((0.036, 0.042, 0.014)),
            origin=Origin(xyz=(x, 0.164, BODY_H - 0.014)),
            material=shadow_steel,
            name=f"hinge_foot_{i}",
        )
        body.visual(
            Box((0.032, 0.014, 0.036)),
            origin=Origin(xyz=(x, HINGE_Y - 0.002, HINGE_Z - 0.017)),
            material=shadow_steel,
            name=f"hinge_upright_{i}",
        )
    body.visual(
        Cylinder(radius=0.0045, length=0.166),
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="hinge_pin",
    )

    # Low front bushings and under-rod webs for the pedal pivot.
    for i, x in enumerate((-0.104, 0.104)):
        body.visual(
            mesh_from_geometry(TorusGeometry(0.0130, 0.0035, radial_segments=16, tubular_segments=36), f"pedal_bushing_{i}"),
            origin=Origin(xyz=(x, PEDAL_Y, PEDAL_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=shadow_steel,
            name=f"pedal_bushing_{i}",
        )
        body.visual(
            Box((0.034, 0.074, 0.014)),
            origin=Origin(xyz=(x, -0.157, PEDAL_Z - 0.020)),
            material=shadow_steel,
            name=f"pedal_web_{i}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_domed_lid_geometry(), "lid_shell"),
        material=steel,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=HINGE_R, length=0.088),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shadow_steel,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.094, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.010, -0.006)),
        material=shadow_steel,
        name="hinge_leaf",
    )

    pedal = model.part("pedal")
    pedal_bar = ExtrudeGeometry.from_z0(rounded_rect_profile(0.215, 0.034, 0.008, corner_segments=8), 0.016)
    pedal_bar.translate(0.0, -0.066, -0.018)
    pedal.visual(
        mesh_from_geometry(pedal_bar, "foot_bar"),
        material=black,
        name="foot_bar",
    )
    for i, x in enumerate((-0.074, 0.074)):
        pedal.visual(
            Box((0.018, 0.070, 0.012)),
            origin=Origin(xyz=(x, -0.033, -0.009)),
            material=black,
            name=f"pedal_arm_{i}",
        )
    pedal.visual(
        Cylinder(radius=0.0097, length=0.232),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shadow_steel,
        name="pivot_rod",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.18, effort=6.0, velocity=2.5),
    )
    model.articulation(
        "pedal_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(0.0, PEDAL_Y, PEDAL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.35, effort=12.0, velocity=3.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    lid_hinge = object_model.get_articulation("lid_hinge")
    pedal_pivot = object_model.get_articulation("pedal_pivot")

    ctx.allow_overlap(
        body,
        lid,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The rear hinge pin is intentionally captured inside the lid hinge barrel.",
    )
    for bushing in ("pedal_bushing_0", "pedal_bushing_1"):
        ctx.allow_overlap(
            body,
            pedal,
            elem_a=bushing,
            elem_b="pivot_rod",
            reason="The pedal pivot rod is intentionally captured in the front wall bushing.",
        )

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_shell",
        elem_b="body_shell",
        min_overlap=0.25,
        name="domed lid covers the cylindrical body opening",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="x",
        elem_a="hinge_barrel",
        elem_b="hinge_pin",
        min_overlap=0.075,
        name="lid hinge barrel is retained on the pin",
    )
    for bushing in ("pedal_bushing_0", "pedal_bushing_1"):
        ctx.expect_overlap(
            pedal,
            body,
            axes="x",
            elem_a="pivot_rod",
            elem_b=bushing,
            min_overlap=0.006,
            name=f"{bushing} captures the pedal pivot rod",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_hinge: 0.92}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid rotates upward on rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.035,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    closed_pedal_aabb = ctx.part_element_world_aabb(pedal, elem="foot_bar")
    with ctx.pose({pedal_pivot: 0.30}):
        pressed_pedal_aabb = ctx.part_element_world_aabb(pedal, elem="foot_bar")
    ctx.check(
        "pedal bar presses downward",
        closed_pedal_aabb is not None
        and pressed_pedal_aabb is not None
        and pressed_pedal_aabb[0][2] < closed_pedal_aabb[0][2] - 0.010,
        details=f"closed={closed_pedal_aabb}, pressed={pressed_pedal_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
