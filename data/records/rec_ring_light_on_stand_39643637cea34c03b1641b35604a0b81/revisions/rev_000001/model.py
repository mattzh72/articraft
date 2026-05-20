from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _circle_profile(radius: float, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _hollow_cylinder_mesh(
    outer_radius: float,
    inner_radius: float,
    length: float,
    name: str,
    *,
    segments: int = 64,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments),
            [_circle_profile(inner_radius, segments)],
            length,
            center=False,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ring_light_boom_stand")

    black = model.material("matte_black", rgba=(0.02, 0.022, 0.024, 1.0))
    dark = model.material("dark_graphite", rgba=(0.12, 0.13, 0.14, 1.0))
    satin = model.material("satin_metal", rgba=(0.52, 0.54, 0.56, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.016, 1.0))
    diffuser = model.material("warm_diffuser", rgba=(1.0, 0.93, 0.70, 0.82))
    knob_mat = model.material("knob_black", rgba=(0.03, 0.03, 0.035, 1.0))

    sleeve_mesh = _hollow_cylinder_mesh(0.028, 0.0195, 1.08, "outer_sleeve")
    collar_mesh = _hollow_cylinder_mesh(0.041, 0.0205, 0.055, "stand_collar")
    yaw_collar_mesh = _hollow_cylinder_mesh(0.047, 0.022, 0.090, "yaw_collar")
    ring_back_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.158, tube=0.027, radial_segments=20, tubular_segments=96),
        "ring_back",
    )
    ring_diffuser_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.158, tube=0.018, radial_segments=20, tubular_segments=96),
        "ring_diffuser",
    )
    leg_tube_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.000, 0.000, 0.000),
                (0.120, 0.000, -0.020),
                (0.340, 0.000, -0.045),
                (0.575, 0.000, -0.055),
            ],
            radius=0.010,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        ),
        "tripod_leg_tube",
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.070, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=black,
        name="tripod_hub",
    )
    base.visual(
        Cylinder(radius=0.044, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=dark,
        name="lower_socket",
    )
    base.visual(
        sleeve_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=black,
        name="outer_sleeve",
    )
    base.visual(
        collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 1.165)),
        material=dark,
        name="height_clamp_collar",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.052),
        origin=Origin(xyz=(0.046, 0.0, 1.192), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin,
        name="clamp_screw",
    )
    base.visual(
        Sphere(radius=0.019),
        origin=Origin(xyz=(0.078, 0.0, 1.192)),
        material=knob_mat,
        name="height_knob",
    )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        base.visual(
            Box((0.082, 0.030, 0.030)),
            origin=Origin(
                xyz=(0.058 * c, 0.058 * s, 0.078),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark,
            name=f"hinge_lug_{index}",
        )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.0155, length=1.200),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=satin,
        name="inner_tube",
    )
    mast.visual(
        Cylinder(radius=0.021, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.670)),
        material=black,
        name="top_plug",
    )
    mast.visual(
        Cylinder(radius=0.005, length=0.034),
        origin=Origin(xyz=(-0.025, 0.0, 0.632), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin,
        name="safety_pin",
    )

    model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 1.200)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=75.0, velocity=0.16, lower=0.0, upper=0.340),
    )

    for index, angle in enumerate(leg_angles):
        leg = model.part(f"leg_{index}")
        leg.visual(
            Cylinder(radius=0.014, length=0.048),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin,
            name="leg_barrel",
        )
        leg.visual(leg_tube_mesh, material=black, name="leg_tube")
        leg.visual(
            Sphere(radius=0.024),
            origin=Origin(xyz=(0.590, 0.0, -0.055)),
            material=rubber,
            name="foot",
        )
        model.articulation(
            f"leg_hinge_{index}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=leg,
            origin=Origin(
                xyz=(0.084 * math.cos(angle), 0.084 * math.sin(angle), 0.079),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=1.2,
                lower=0.0,
                upper=math.radians(68.0),
            ),
        )

    boom = model.part("boom")
    boom.visual(
        yaw_collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=black,
        name="yaw_collar",
    )
    boom.visual(
        Box((0.034, 0.022, 0.055)),
        origin=Origin(xyz=(0.000, -0.056, 0.000)),
        material=dark,
        name="clamp_ear",
    )
    boom.visual(
        Cylinder(radius=0.0045, length=0.055),
        origin=Origin(xyz=(0.0, -0.070, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin,
        name="yaw_clamp_screw",
    )
    boom.visual(
        Sphere(radius=0.015),
        origin=Origin(xyz=(0.0, -0.103, 0.000)),
        material=knob_mat,
        name="yaw_knob",
    )
    boom.visual(
        Cylinder(radius=0.018, length=0.760),
        origin=Origin(xyz=(0.405, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="boom_tube",
    )
    boom.visual(
        Cylinder(radius=0.014, length=0.170),
        origin=Origin(xyz=(-0.115, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="weight_stem",
    )
    boom.visual(
        Cylinder(radius=0.040, length=0.060),
        origin=Origin(xyz=(-0.218, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="counterweight",
    )
    boom.visual(
        Box((0.078, 0.070, 0.050)),
        origin=Origin(xyz=(0.790, 0.0, 0.0)),
        material=dark,
        name="tip_block",
    )
    boom.visual(
        Box((0.060, 0.020, 0.320)),
        origin=Origin(xyz=(0.825, 0.236, 0.080)),
        material=black,
        name="yoke_side_0",
    )
    boom.visual(
        Box((0.060, 0.020, 0.320)),
        origin=Origin(xyz=(0.825, -0.236, 0.080)),
        material=black,
        name="yoke_side_1",
    )
    boom.visual(
        Box((0.080, 0.492, 0.026)),
        origin=Origin(xyz=(0.760, 0.0, 0.240)),
        material=black,
        name="yoke_bridge",
    )
    boom.visual(
        Box((0.052, 0.070, 0.380)),
        origin=Origin(xyz=(0.755, 0.0, 0.075)),
        material=black,
        name="yoke_spine",
    )
    boom.visual(
        Cylinder(radius=0.022, length=0.030),
        origin=Origin(xyz=(0.825, 0.259, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_mat,
        name="tilt_knob_0",
    )
    boom.visual(
        Cylinder(radius=0.022, length=0.030),
        origin=Origin(xyz=(0.825, -0.259, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_mat,
        name="tilt_knob_1",
    )

    model.articulation(
        "boom_yaw",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=boom,
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.0,
            lower=-math.radians(150.0),
            upper=math.radians(150.0),
        ),
    )

    ring = model.part("ring_head")
    ring.visual(
        ring_back_mesh,
        origin=Origin(xyz=(-0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="rear_housing",
    )
    ring.visual(
        ring_diffuser_mesh,
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=diffuser,
        name="front_diffuser",
    )
    ring.visual(
        Cylinder(radius=0.017, length=0.050),
        origin=Origin(xyz=(0.0, 0.218, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin,
        name="pin_0",
    )
    ring.visual(
        Cylinder(radius=0.017, length=0.050),
        origin=Origin(xyz=(0.0, -0.218, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin,
        name="pin_1",
    )
    ring.visual(
        Box((0.026, 0.050, 0.032)),
        origin=Origin(xyz=(-0.014, 0.185, 0.0)),
        material=black,
        name="side_lug_0",
    )
    ring.visual(
        Box((0.026, 0.050, 0.032)),
        origin=Origin(xyz=(-0.014, -0.185, 0.0)),
        material=black,
        name="side_lug_1",
    )
    ring.visual(
        Box((0.035, 0.075, 0.024)),
        origin=Origin(xyz=(-0.018, 0.0, -0.184)),
        material=dark,
        name="control_pod",
    )
    model.articulation(
        "ring_tilt",
        ArticulationType.REVOLUTE,
        parent=boom,
        child=ring,
        origin=Origin(xyz=(0.825, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=-math.radians(55.0),
            upper=math.radians(55.0),
        ),
    )

    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_mat,
        name="button_cap",
    )
    model.articulation(
        "power_press",
        ArticulationType.PRISMATIC,
        parent=ring,
        child=power_button,
        origin=Origin(xyz=(0.0055, -0.018, -0.184)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=0.05, lower=0.0, upper=0.004),
    )

    brightness_button = model.part("brightness_button")
    brightness_button.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_mat,
        name="button_cap",
    )
    model.articulation(
        "brightness_press",
        ArticulationType.PRISMATIC,
        parent=ring,
        child=brightness_button,
        origin=Origin(xyz=(0.0055, 0.018, -0.184)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=0.05, lower=0.0, upper=0.004),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    boom = object_model.get_part("boom")
    ring = object_model.get_part("ring_head")
    power_button = object_model.get_part("power_button")
    brightness_button = object_model.get_part("brightness_button")
    mast_slide = object_model.get_articulation("mast_slide")
    boom_yaw = object_model.get_articulation("boom_yaw")
    ring_tilt = object_model.get_articulation("ring_tilt")
    power_press = object_model.get_articulation("power_press")

    for index in range(3):
        leg = object_model.get_part(f"leg_{index}")
        ctx.allow_overlap(
            base,
            leg,
            elem_a=f"hinge_lug_{index}",
            elem_b="leg_barrel",
            reason="Each tripod leg barrel is intentionally captured inside its hinge lug.",
        )
        ctx.allow_overlap(
            base,
            leg,
            elem_a=f"hinge_lug_{index}",
            elem_b="leg_tube",
            reason="The leg tube root is intentionally welded into the hinge lug area.",
        )
        ctx.expect_overlap(
            base,
            leg,
            axes="xyz",
            elem_a=f"hinge_lug_{index}",
            elem_b="leg_barrel",
            min_overlap=0.006,
            name=f"leg {index} barrel is retained by lug",
        )
        ctx.expect_overlap(
            base,
            leg,
            axes="xyz",
            elem_a=f"hinge_lug_{index}",
            elem_b="leg_tube",
            min_overlap=0.006,
            name=f"leg {index} tube root meets hinge lug",
        )

    ctx.allow_overlap(
        boom,
        ring,
        elem_a="yoke_side_0",
        elem_b="pin_0",
        reason="The tilt trunnion pin is represented as seated through the yoke cheek.",
    )
    ctx.allow_overlap(
        boom,
        ring,
        elem_a="yoke_side_1",
        elem_b="pin_1",
        reason="The tilt trunnion pin is represented as seated through the yoke cheek.",
    )
    ctx.expect_overlap(
        boom,
        ring,
        axes="y",
        elem_a="yoke_side_0",
        elem_b="pin_0",
        min_overlap=0.006,
        name="upper yoke cheek captures ring pin",
    )
    ctx.expect_overlap(
        boom,
        ring,
        axes="y",
        elem_a="yoke_side_1",
        elem_b="pin_1",
        min_overlap=0.006,
        name="lower yoke cheek captures ring pin",
    )

    for button, label in ((power_button, "power"), (brightness_button, "brightness")):
        ctx.allow_overlap(
            button,
            ring,
            elem_a="button_cap",
            elem_b="rear_housing",
            reason=f"The {label} button cap is intentionally seated in the molded lower housing.",
        )
        ctx.expect_overlap(
            button,
            ring,
            axes="x",
            elem_a="button_cap",
            elem_b="rear_housing",
            min_overlap=0.006,
            name=f"{label} button is seated in housing",
        )

    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="sliding mast stays centered in sleeve",
    )
    ctx.allow_overlap(
        base,
        mast,
        elem_a="outer_sleeve",
        elem_b="inner_tube",
        reason="The inner mast is intentionally represented as sliding inside the sleeve proxy.",
    )
    ctx.allow_overlap(
        base,
        mast,
        elem_a="height_clamp_collar",
        elem_b="inner_tube",
        reason="The clamp collar is a simplified sleeve around the sliding mast tube.",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="outer_sleeve",
        min_overlap=0.18,
        name="collapsed mast remains inserted in sleeve",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="height_clamp_collar",
        min_overlap=0.040,
        name="mast passes through height clamp collar",
    )
    with ctx.pose({mast_slide: 0.340}):
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_sleeve",
            min_overlap=0.18,
            name="raised mast remains inserted in sleeve",
        )

    ctx.expect_within(
        mast,
        boom,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="yaw_collar",
        margin=0.002,
        name="boom yaw collar clips around mast",
    )
    ctx.allow_overlap(
        boom,
        mast,
        elem_a="yaw_collar",
        elem_b="top_plug",
        reason="The yaw collar clips around the top plug at the stand head so the boom stays captured.",
    )
    ctx.expect_overlap(
        boom,
        mast,
        axes="z",
        elem_a="yaw_collar",
        elem_b="top_plug",
        min_overlap=0.030,
        name="yaw collar vertically captures top plug",
    )

    rest_ring_pos = ctx.part_world_position(ring)
    with ctx.pose({mast_slide: 0.340}):
        raised_ring_pos = ctx.part_world_position(ring)
    ctx.check(
        "mast slide raises boom and ring",
        rest_ring_pos is not None
        and raised_ring_pos is not None
        and raised_ring_pos[2] > rest_ring_pos[2] + 0.30,
        details=f"rest={rest_ring_pos}, raised={raised_ring_pos}",
    )

    with ctx.pose({boom_yaw: math.pi / 2.0}):
        yawed_ring_pos = ctx.part_world_position(ring)
    ctx.check(
        "boom yaw carries ring around column",
        yawed_ring_pos is not None and yawed_ring_pos[1] > 0.75 and abs(yawed_ring_pos[0]) < 0.08,
        details=f"yawed_ring_pos={yawed_ring_pos}",
    )

    rest_ring_aabb = ctx.part_element_world_aabb(ring, elem="front_diffuser")
    with ctx.pose({ring_tilt: math.radians(45.0)}):
        tilted_ring_aabb = ctx.part_element_world_aabb(ring, elem="front_diffuser")
    if rest_ring_aabb is not None and tilted_ring_aabb is not None:
        rest_x = rest_ring_aabb[1][0] - rest_ring_aabb[0][0]
        tilted_x = tilted_ring_aabb[1][0] - tilted_ring_aabb[0][0]
    else:
        rest_x = tilted_x = None
    ctx.check(
        "ring tilt pitches light head",
        rest_x is not None and tilted_x is not None and tilted_x > rest_x + 0.10,
        details=f"rest_x={rest_x}, tilted_x={tilted_x}",
    )

    button_rest = ctx.part_world_position(power_button)
    with ctx.pose({power_press: 0.004}):
        button_pressed = ctx.part_world_position(power_button)
    ctx.check(
        "power button presses inward",
        button_rest is not None and button_pressed is not None and button_pressed[0] < button_rest[0] - 0.003,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    leg0 = object_model.get_part("leg_0")
    leg0_hinge = object_model.get_articulation("leg_hinge_0")
    foot_rest = ctx.part_element_world_aabb(leg0, elem="foot")
    with ctx.pose({leg0_hinge: math.radians(55.0)}):
        foot_folded = ctx.part_element_world_aabb(leg0, elem="foot")
    if foot_rest is not None and foot_folded is not None:
        rest_foot_z = (foot_rest[0][2] + foot_rest[1][2]) / 2.0
        folded_foot_z = (foot_folded[0][2] + foot_folded[1][2]) / 2.0
    else:
        rest_foot_z = folded_foot_z = None
    ctx.check(
        "tripod leg folds upward on hinge",
        rest_foot_z is not None and folded_foot_z is not None and folded_foot_z > rest_foot_z + 0.30,
        details=f"rest_foot_z={rest_foot_z}, folded_foot_z={folded_foot_z}",
    )

    return ctx.report()


object_model = build_object_model()
