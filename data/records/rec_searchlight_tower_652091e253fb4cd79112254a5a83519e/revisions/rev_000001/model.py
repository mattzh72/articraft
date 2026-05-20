from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


PAN_HEIGHT = 2.0
YOKE_BASE_Z = 0.11
TILT_AXIS_Z = YOKE_BASE_Z + 0.48


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="searchlight_tower")

    steel = model.material("dark_blued_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    black = model.material("matte_black", rgba=(0.01, 0.01, 0.012, 1.0))
    safety_yellow = model.material("weathered_yellow", rgba=(0.86, 0.63, 0.12, 1.0))
    lens_glass = model.material("pale_blue_glass", rgba=(0.55, 0.82, 1.0, 0.45))
    warm_reflector = model.material("warm_reflector", rgba=(1.0, 0.78, 0.28, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((0.80, 0.80, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=steel,
        name="base_plate",
    )
    tower.visual(
        Cylinder(radius=0.045, length=1.82),
        origin=Origin(xyz=(0.0, 0.0, 1.03)),
        material=steel,
        name="single_mast",
    )
    tower.visual(
        Cylinder(radius=0.090, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=black,
        name="mast_foot_collar",
    )
    tower.visual(
        Cylinder(radius=0.070, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 1.91)),
        material=black,
        name="upper_sleeve",
    )
    tower.visual(
        Cylinder(radius=0.160, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 1.98)),
        material=steel,
        name="fixed_bearing",
    )

    pan_stage = model.part("pan_stage")
    pan_stage.visual(
        Cylinder(radius=0.130, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=black,
        name="lower_bearing",
    )
    pan_stage.visual(
        Cylinder(radius=0.200, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=steel,
        name="turntable",
    )
    yoke_geom = TrunnionYokeGeometry(
        (0.62, 0.18, 0.68),
        span_width=0.42,
        trunnion_diameter=0.070,
        trunnion_center_z=0.48,
        base_thickness=0.10,
        corner_radius=0.018,
        center=False,
    )
    # The yoke helper spans across local X; rotate it so the searchlight sits
    # between arms along the tower's local Y axis.
    yoke_geom.rotate_z(math.pi / 2.0)
    pan_stage.visual(
        mesh_from_geometry(yoke_geom, "trunnion_yoke"),
        origin=Origin(xyz=(0.0, 0.0, YOKE_BASE_Z)),
        material=safety_yellow,
        name="yoke",
    )

    lamp_head = model.part("lamp_head")
    shell_geom = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.135, -0.23),
            (0.152, -0.12),
            (0.160, 0.10),
            (0.170, 0.25),
        ],
        inner_profile=[
            (0.115, -0.21),
            (0.132, -0.10),
            (0.140, 0.10),
            (0.146, 0.23),
        ],
        segments=72,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )
    shell_geom.rotate_y(math.pi / 2.0)
    lamp_head.visual(
        mesh_from_geometry(shell_geom, "lamp_body_shell"),
        material=safety_yellow,
        name="body_shell",
    )
    front_gasket = TorusGeometry(radius=0.151, tube=0.012, radial_segments=18, tubular_segments=72)
    front_gasket.rotate_y(math.pi / 2.0)
    lamp_head.visual(
        mesh_from_geometry(front_gasket, "front_gasket"),
        origin=Origin(xyz=(0.258, 0.0, 0.0)),
        material=black,
        name="front_gasket",
    )
    lamp_head.visual(
        Cylinder(radius=0.148, length=0.014),
        origin=Origin(xyz=(0.266, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    lamp_head.visual(
        Cylinder(radius=0.142, length=0.018),
        origin=Origin(xyz=(0.253, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_reflector,
        name="reflector",
    )
    lamp_head.visual(
        Cylinder(radius=0.137, length=0.035),
        origin=Origin(xyz=(-0.238, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="rear_cap",
    )
    lamp_head.visual(
        Cylinder(radius=0.045, length=0.045),
        origin=Origin(xyz=(-0.276, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="rear_socket",
    )
    lamp_head.visual(
        Cylinder(radius=0.036, length=0.62),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="tilt_pin",
    )
    for idx, y in enumerate((-0.045, 0.0, 0.045)):
        lamp_head.visual(
            Box((0.22, 0.012, 0.045)),
            origin=Origin(xyz=(0.020, y, 0.173)),
            material=black,
            name=f"cooling_fin_{idx}",
        )

    model.articulation(
        "tower_to_pan",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=pan_stage,
        origin=Origin(xyz=(0.0, 0.0, PAN_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "pan_to_lamp",
        ArticulationType.REVOLUTE,
        parent=pan_stage,
        child=lamp_head,
        origin=Origin(xyz=(0.0, 0.0, TILT_AXIS_Z)),
        # The lamp points along local +X, so -Y makes positive q tilt the beam upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.0, lower=-0.50, upper=0.90),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    pan_stage = object_model.get_part("pan_stage")
    lamp_head = object_model.get_part("lamp_head")
    pan = object_model.get_articulation("tower_to_pan")
    tilt = object_model.get_articulation("pan_to_lamp")

    ctx.allow_overlap(
        pan_stage,
        lamp_head,
        elem_a="yoke",
        elem_b="tilt_pin",
        reason="The trunnion pin is intentionally captured in the yoke bores with a slight interference fit.",
    )

    ctx.expect_contact(
        pan_stage,
        tower,
        elem_a="lower_bearing",
        elem_b="fixed_bearing",
        contact_tol=0.001,
        name="rotating stage sits on the single central support",
    )
    ctx.expect_overlap(
        pan_stage,
        tower,
        axes="xy",
        elem_a="lower_bearing",
        elem_b="fixed_bearing",
        min_overlap=0.10,
        name="pan bearing footprint is centered on mast",
    )
    ctx.expect_overlap(
        lamp_head,
        pan_stage,
        axes="y",
        elem_a="tilt_pin",
        elem_b="yoke",
        min_overlap=0.50,
        name="tilt pin spans through both yoke arms",
    )
    ctx.expect_within(
        lamp_head,
        pan_stage,
        axes="xz",
        inner_elem="tilt_pin",
        outer_elem="yoke",
        margin=0.010,
        name="tilt pin is centered in the yoke bores",
    )
    ctx.check(
        "pan joint is vertical revolute",
        pan.articulation_type == ArticulationType.REVOLUTE and tuple(pan.axis) == (0.0, 0.0, 1.0),
        details=f"type={pan.articulation_type}, axis={pan.axis}",
    )
    ctx.check(
        "tilt joint is horizontal revolute",
        tilt.articulation_type == ArticulationType.REVOLUTE and tuple(tilt.axis) == (0.0, -1.0, 0.0),
        details=f"type={tilt.articulation_type}, axis={tilt.axis}",
    )

    def aabb_center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3)) if aabb else None

    rest_lens = aabb_center(ctx.part_element_world_aabb(lamp_head, elem="front_lens"))
    with ctx.pose({pan: math.pi / 2.0}):
        panned_lens = aabb_center(ctx.part_element_world_aabb(lamp_head, elem="front_lens"))
    ctx.check(
        "pan swings the beam around the mast",
        rest_lens is not None
        and panned_lens is not None
        and panned_lens[1] > rest_lens[1] + 0.20
        and abs(panned_lens[2] - rest_lens[2]) < 0.010,
        details=f"rest={rest_lens}, panned={panned_lens}",
    )

    with ctx.pose({tilt: 0.70}):
        tilted_lens = aabb_center(ctx.part_element_world_aabb(lamp_head, elem="front_lens"))
    ctx.check(
        "positive tilt raises the lamp beam",
        rest_lens is not None and tilted_lens is not None and tilted_lens[2] > rest_lens[2] + 0.10,
        details=f"rest={rest_lens}, tilted={tilted_lens}",
    )

    return ctx.report()


object_model = build_object_model()
