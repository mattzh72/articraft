from __future__ import annotations

from math import pi

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    wire_from_points,
)


PANEL_LENGTH = 1.20
PANEL_WIDTH = 0.16
PANEL_THICKNESS = 0.040
ATTACH_X = 0.52
HINGE_Z = 0.055
LOOP_TOP_Z = 0.078
WIRE_LENGTH = 0.93
GRIP_REST_Z = 0.70
GRIP_TRAVEL = 0.18


def _panel_housing_mesh(name: str):
    profile = rounded_rect_profile(PANEL_LENGTH, PANEL_WIDTH, 0.018, corner_segments=10)
    return mesh_from_geometry(
        ExtrudeGeometry(profile, PANEL_THICKNESS, cap=True, center=True),
        name,
    )


def _diffuser_mesh(name: str):
    profile = rounded_rect_profile(1.10, 0.115, 0.012, corner_segments=8)
    return mesh_from_geometry(
        ExtrudeGeometry(profile, 0.006, cap=True, center=True),
        name,
    )


def _loop_mesh(name: str):
    # A single bent wire bail: bottom around the hinge pin, arcing up to the
    # cable eye.  It lies in the local XZ plane so a revolute Y-axis joint gives
    # the expected fore/aft swing.
    return mesh_from_geometry(
        wire_from_points(
            [
                (0.000, 0.0, 0.000),
                (-0.026, 0.0, 0.018),
                (-0.024, 0.0, 0.055),
                (0.000, 0.0, LOOP_TOP_Z),
                (0.024, 0.0, 0.055),
                (0.026, 0.0, 0.018),
            ],
            radius=0.0028,
            radial_segments=18,
            closed_path=True,
            corner_mode="fillet",
            corner_radius=0.010,
        ),
        name,
    )


def _friction_grip_mesh(name: str):
    # Hollow cable gripper sleeve with a visible pass-through bore.
    sleeve = (
        cq.Workplane("XY")
        .circle(0.014)
        .circle(0.0045)
        .extrude(0.070)
        .translate((0.0, 0.0, -0.035))
    )
    return mesh_from_cadquery(sleeve, name, tolerance=0.0008, angular_tolerance=0.08)


def _ceiling_neck_mesh(name: str):
    neck = (
        cq.Workplane("XY")
        .circle(0.026)
        .circle(0.0045)
        .extrude(0.016)
        .translate((0.0, 0.0, -0.008))
    )
    return mesh_from_cadquery(neck, name, tolerance=0.0008, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_led_wire_suspension")

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    end_cap = model.material("satin_end_cap", rgba=(0.18, 0.19, 0.20, 1.0))
    diffuser = model.material("milky_diffuser", rgba=(0.92, 0.97, 1.0, 0.62))
    led_glow = model.material("cool_led_glow", rgba=(0.75, 0.92, 1.0, 0.88))
    cable_steel = model.material("braided_steel", rgba=(0.58, 0.60, 0.61, 1.0))
    bracket_metal = model.material("polished_loop_bracket", rgba=(0.83, 0.84, 0.80, 1.0))
    ceiling_white = model.material("white_ceiling_anchor", rgba=(0.93, 0.93, 0.90, 1.0))
    black_rubber = model.material("black_release_button", rgba=(0.02, 0.02, 0.018, 1.0))

    panel = model.part("panel")
    panel.visual(
        _panel_housing_mesh("panel_housing"),
        origin=Origin(),
        material=aluminum,
        name="rounded_housing",
    )
    panel.visual(
        _diffuser_mesh("panel_diffuser"),
        origin=Origin(xyz=(0.0, 0.0, -0.023)),
        material=diffuser,
        name="diffuser_lens",
    )
    panel.visual(
        Box((PANEL_LENGTH - 0.045, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, PANEL_WIDTH / 2 - 0.009, 0.003)),
        material=aluminum,
        name="side_rail_0",
    )
    panel.visual(
        Box((PANEL_LENGTH - 0.045, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -PANEL_WIDTH / 2 + 0.009, 0.003)),
        material=aluminum,
        name="side_rail_1",
    )
    panel.visual(
        Box((0.026, PANEL_WIDTH, PANEL_THICKNESS + 0.004)),
        origin=Origin(xyz=(PANEL_LENGTH / 2 - 0.013, 0.0, 0.0)),
        material=end_cap,
        name="end_cap_0",
    )
    panel.visual(
        Box((0.026, PANEL_WIDTH, PANEL_THICKNESS + 0.004)),
        origin=Origin(xyz=(-PANEL_LENGTH / 2 + 0.013, 0.0, 0.0)),
        material=end_cap,
        name="end_cap_1",
    )
    for idx, x in enumerate([-0.45, -0.34, -0.23, -0.12, -0.01, 0.10, 0.21, 0.32, 0.43]):
        panel.visual(
            Box((0.050, 0.085, 0.0025)),
            origin=Origin(xyz=(x, 0.0, -0.027)),
            material=led_glow,
            name=f"led_patch_{idx}",
        )

    # Panel end yokes: two ears on each end with a clearance gap for the loop
    # barrel.  The ears overlap the housing top slightly as welded brackets.
    for side, x in enumerate((-ATTACH_X, ATTACH_X)):
        panel.visual(
            Box((0.060, 0.085, 0.007)),
            origin=Origin(xyz=(x, 0.0, PANEL_THICKNESS / 2 + 0.003)),
            material=bracket_metal,
            name=f"yoke_base_{side}",
        )
        for tab, y in enumerate((-0.033, 0.033)):
            panel.visual(
                Box((0.034, 0.008, 0.045)),
                origin=Origin(xyz=(x, y, HINGE_Z - 0.010)),
                material=bracket_metal,
                name=f"yoke_tab_{side}_{tab}",
            )

    for i, x in enumerate((-ATTACH_X, ATTACH_X)):
        loop = model.part(f"loop_{i}")
        loop.visual(
            _loop_mesh(f"loop_{i}_bail"),
            material=bracket_metal,
            name="bail_wire",
        )
        loop.visual(
            Cylinder(radius=0.0058, length=0.070),
            origin=Origin(rpy=(-pi / 2, 0.0, 0.0)),
            material=bracket_metal,
            name="hinge_barrel",
        )
        model.articulation(
            f"panel_to_loop_{i}",
            ArticulationType.REVOLUTE,
            parent=panel,
            child=loop,
            origin=Origin(xyz=(x, 0.0, HINGE_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=1.8, lower=-0.65, upper=0.65),
            motion_properties=MotionProperties(damping=0.05, friction=0.02),
        )

        wire = model.part(f"wire_{i}")
        wire.visual(
            Cylinder(radius=0.0018, length=WIRE_LENGTH),
            origin=Origin(xyz=(0.0, 0.0, WIRE_LENGTH / 2 + 0.002)),
            material=cable_steel,
            name="cable",
        )
        wire.visual(
            Cylinder(radius=0.006, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.000)),
            material=cable_steel,
            name="crimp_stop",
        )
        model.articulation(
            f"loop_{i}_to_wire_{i}",
            ArticulationType.FIXED,
            parent=loop,
            child=wire,
            origin=Origin(xyz=(0.0, 0.0, LOOP_TOP_Z + 0.001)),
        )

        grip = model.part(f"grip_{i}")
        grip.visual(
            _friction_grip_mesh(f"grip_{i}_sleeve"),
            material=ceiling_white,
            name="friction_sleeve",
        )
        grip.visual(
            _ceiling_neck_mesh(f"grip_{i}_ceiling_neck"),
            origin=Origin(xyz=(0.0, 0.0, 0.040)),
            material=ceiling_white,
            name="ceiling_neck",
        )
        grip.visual(
            mesh_from_geometry(
                TorusGeometry(radius=0.030, tube=0.0045, radial_segments=24, tubular_segments=40),
                f"grip_{i}_ceiling_cup",
            ),
            origin=Origin(xyz=(0.0, 0.0, 0.039)),
            material=ceiling_white,
            name="ceiling_cup",
        )
        grip.visual(
            Box((0.028, 0.052, 0.006)),
            origin=Origin(xyz=(-0.023, 0.0, 0.046)),
            material=ceiling_white,
            name="ceiling_plate_0",
        )
        grip.visual(
            Box((0.028, 0.052, 0.006)),
            origin=Origin(xyz=(0.023, 0.0, 0.046)),
            material=ceiling_white,
            name="ceiling_plate_1",
        )
        grip.visual(
            Box((0.0030, 0.0045, 0.024)),
            origin=Origin(xyz=(-0.0032, 0.0, -0.002)),
            material=black_rubber,
            name="clamp_pad_0",
        )
        grip.visual(
            Box((0.0030, 0.0045, 0.024)),
            origin=Origin(xyz=(0.0032, 0.0, -0.002)),
            material=black_rubber,
            name="clamp_pad_1",
        )
        grip.visual(
            Box((0.007, 0.016, 0.010)),
            origin=Origin(xyz=(0.0, 0.012, -0.003)),
            material=black_rubber,
            name="release_button",
        )
        model.articulation(
            f"wire_{i}_to_grip_{i}",
            ArticulationType.PRISMATIC,
            parent=wire,
            child=grip,
            origin=Origin(xyz=(0.0, 0.0, GRIP_REST_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=18.0, velocity=0.08, lower=0.0, upper=GRIP_TRAVEL),
            motion_properties=MotionProperties(damping=0.4, friction=1.2),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    panel = object_model.get_part("panel")

    ctx.check(
        "two loop revolutes",
        all(
            object_model.get_articulation(f"panel_to_loop_{i}").articulation_type
            == ArticulationType.REVOLUTE
            for i in (0, 1)
        ),
        details="Each panel end should have a revolute loop bracket.",
    )
    ctx.check(
        "two prismatic cable grips",
        all(
            object_model.get_articulation(f"wire_{i}_to_grip_{i}").articulation_type
            == ArticulationType.PRISMATIC
            for i in (0, 1)
        ),
        details="Each ceiling gripper should slide along its hanger wire.",
    )

    for i in (0, 1):
        loop = object_model.get_part(f"loop_{i}")
        wire = object_model.get_part(f"wire_{i}")
        grip = object_model.get_part(f"grip_{i}")
        loop_joint = object_model.get_articulation(f"panel_to_loop_{i}")
        grip_joint = object_model.get_articulation(f"wire_{i}_to_grip_{i}")

        for tab in (0, 1):
            ctx.allow_overlap(
                loop,
                panel,
                elem_a="hinge_barrel",
                elem_b=f"yoke_tab_{i}_{tab}",
                reason="The hinge pin is intentionally captured through the yoke-tab hole proxy.",
            )
            ctx.expect_overlap(
                loop,
                panel,
                axes="xyz",
                min_overlap=0.003,
                elem_a="hinge_barrel",
                elem_b=f"yoke_tab_{i}_{tab}",
                name=f"loop {i} hinge pin is retained by yoke tab {tab}",
            )
        ctx.allow_overlap(
            loop,
            wire,
            elem_a="bail_wire",
            elem_b="crimp_stop",
            reason="The crimped cable stop is locally swaged through the loop eye.",
        )
        ctx.expect_overlap(
            loop,
            wire,
            axes="xz",
            min_overlap=0.003,
            elem_a="bail_wire",
            elem_b="crimp_stop",
            name=f"wire {i} crimp is seated in the loop eye",
        )
        for pad in (0, 1):
            ctx.allow_overlap(
                wire,
                grip,
                elem_a="cable",
                elem_b=f"clamp_pad_{pad}",
                reason="The spring clamp pads intentionally pinch the cable inside the friction grip.",
            )
            ctx.expect_overlap(
                wire,
                grip,
                axes="z",
                min_overlap=0.015,
                elem_a="cable",
                elem_b=f"clamp_pad_{pad}",
                name=f"grip {i} clamp pad {pad} engages the cable",
            )

        ctx.expect_overlap(
            loop,
            panel,
            axes="xy",
            min_overlap=0.008,
            elem_a="hinge_barrel",
            name=f"loop {i} hinge sits in panel end yoke",
        )
        ctx.expect_gap(
            loop,
            panel,
            axis="z",
            max_penetration=0.030,
            positive_elem="hinge_barrel",
            name=f"loop {i} barrel is captured at top bracket height",
        )
        ctx.expect_overlap(
            wire,
            grip,
            axes="z",
            min_overlap=0.040,
            elem_a="cable",
            elem_b="friction_sleeve",
            name=f"wire {i} passes through friction sleeve",
        )
        ctx.expect_within(
            wire,
            grip,
            axes="xy",
            margin=0.002,
            inner_elem="cable",
            outer_elem="friction_sleeve",
            name=f"wire {i} is centered in the ceiling grip",
        )

        rest_grip_pos = ctx.part_world_position(grip)
        with ctx.pose({grip_joint: GRIP_TRAVEL}):
            extended_grip_pos = ctx.part_world_position(grip)
            ctx.expect_overlap(
                wire,
                grip,
                axes="z",
                min_overlap=0.040,
                elem_a="cable",
                elem_b="friction_sleeve",
                name=f"wire {i} remains captured at full adjustment",
            )

        ctx.check(
            f"grip {i} slides upward on cable",
            rest_grip_pos is not None
            and extended_grip_pos is not None
            and extended_grip_pos[2] > rest_grip_pos[2] + 0.12,
            details=f"rest={rest_grip_pos}, extended={extended_grip_pos}",
        )

        rest_wire_pos = ctx.part_world_position(wire)
        with ctx.pose({loop_joint: 0.45}):
            swung_wire_pos = ctx.part_world_position(wire)
        ctx.check(
            f"loop {i} revolute swings hanger wire",
            rest_wire_pos is not None
            and swung_wire_pos is not None
            and abs(swung_wire_pos[0] - rest_wire_pos[0]) > 0.020,
            details=f"rest={rest_wire_pos}, swung={swung_wire_pos}",
        )

    return ctx.report()


object_model = build_object_model()
