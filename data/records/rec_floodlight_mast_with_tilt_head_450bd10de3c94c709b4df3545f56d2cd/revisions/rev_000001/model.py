from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="construction_floodlight_tower")

    yellow = model.material("safety_yellow", rgba=(1.0, 0.72, 0.08, 1.0))
    aluminium = model.material("brushed_aluminium", rgba=(0.72, 0.74, 0.72, 1.0))
    dark = model.material("matte_black", rgba=(0.015, 0.014, 0.012, 1.0))
    tire_mat = model.material("rubber_tire", rgba=(0.02, 0.018, 0.016, 1.0))
    steel = model.material("galvanized_steel", rgba=(0.45, 0.48, 0.48, 1.0))
    lens = model.material("warm_led_lens", rgba=(1.0, 0.92, 0.58, 0.78))
    led = model.material("led_emitters", rgba=(1.0, 0.96, 0.72, 1.0))

    trailer = model.part("trailer_base")
    trailer.visual(Box((1.25, 0.72, 0.12)), origin=Origin(xyz=(0.18, 0.0, 0.42)), material=yellow, name="deck")
    trailer.visual(Box((0.50, 0.44, 0.36)), origin=Origin(xyz=(-0.02, 0.0, 0.655)), material=yellow, name="generator_case")
    trailer.visual(Box((0.54, 0.035, 0.20)), origin=Origin(xyz=(-0.02, -0.226, 0.66)), material=dark, name="side_vent_0")
    trailer.visual(Box((0.54, 0.035, 0.20)), origin=Origin(xyz=(-0.02, 0.226, 0.66)), material=dark, name="side_vent_1")
    trailer.visual(Box((0.82, 0.08, 0.07)), origin=Origin(xyz=(-0.80, 0.0, 0.39)), material=steel, name="tow_tongue")
    trailer.visual(Box((0.10, 0.16, 0.055)), origin=Origin(xyz=(-1.24, 0.0, 0.385)), material=steel, name="hitch_coupler")

    # Axle and simple outriggers are fixed to the trailer frame; the wheels spin
    # as separate children on revolute joints.
    trailer.visual(
        Cylinder(radius=0.035, length=0.86),
        origin=Origin(xyz=(0.16, 0.0, 0.27), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=steel,
        name="axle",
    )
    for y, suffix in [(-0.33, "0"), (0.33, "1")]:
        trailer.visual(Box((0.10, 0.06, 0.16)), origin=Origin(xyz=(0.16, y, 0.335)), material=steel, name=f"axle_hanger_{suffix}")
    for x in (-0.32, 0.62):
        for y in (-0.58, 0.58):
            name_suffix = f"{0 if x < 0 else 1}_{0 if y < 0 else 1}"
            trailer.visual(Box((0.12, 0.34, 0.080)), origin=Origin(xyz=(x, y * 0.80, 0.36)), material=yellow, name=f"outrigger_arm_{name_suffix}")
            trailer.visual(Box((0.055, 0.055, 0.33)), origin=Origin(xyz=(x, y, 0.18)), material=steel, name=f"jack_leg_{name_suffix}")
            trailer.visual(Box((0.18, 0.14, 0.035)), origin=Origin(xyz=(x, y, 0.018)), material=dark, name=f"foot_pad_{name_suffix}")

    pivot_xyz = (0.56, 0.0, 0.86)
    trailer.visual(Box((0.18, 0.38, 0.31)), origin=Origin(xyz=(pivot_xyz[0], 0.0, 0.62)), material=yellow, name="pivot_pedestal")
    trailer.visual(Box((0.18, 0.06, 0.28)), origin=Origin(xyz=(pivot_xyz[0], -0.165, pivot_xyz[2])), material=yellow, name="pivot_cheek_0")
    trailer.visual(Box((0.18, 0.06, 0.28)), origin=Origin(xyz=(pivot_xyz[0], 0.165, pivot_xyz[2])), material=yellow, name="pivot_cheek_1")
    trailer.visual(
        Cylinder(radius=0.022, length=0.43),
        origin=Origin(xyz=pivot_xyz, rpy=(-math.pi / 2, 0.0, 0.0)),
        material=steel,
        name="base_pivot_pin",
    )

    chain_start = (-0.28, 0.0, 0.96)
    chain_end = (0.44, 0.0, 1.77)
    trailer.visual(Box((0.09, 0.12, 0.20)), origin=Origin(xyz=(-0.28, 0.0, 0.88)), material=steel, name="chain_anchor")
    chain_geom = tube_from_spline_points(
        [chain_start, (-0.06, 0.0, 1.12), (0.22, 0.0, 1.48), chain_end],
        radius=0.008,
        samples_per_segment=8,
        radial_segments=12,
        cap_ends=True,
    )
    trailer.visual(mesh_from_geometry(chain_geom, "chain_brace_mesh"), material=steel, name="chain_brace")

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.250,
            0.14,
            inner_radius=0.166,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.05),
            tread=TireTread(style="block", depth=0.010, count=22, land_ratio=0.56),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.012, radius=0.004),
        ),
        "utility_tire",
    )
    for index, y in enumerate((-0.50, 0.50)):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(tire_mesh, origin=Origin(rpy=(0.0, 0.0, math.pi / 2)), material=tire_mat, name="tire")
        wheel.visual(
            Cylinder(radius=0.170, length=0.130),
            origin=Origin(rpy=(-math.pi / 2, 0.0, 0.0)),
            material=steel,
            name="rim_disc",
        )
        wheel.visual(
            Cylinder(radius=0.055, length=0.150),
            origin=Origin(rpy=(-math.pi / 2, 0.0, 0.0)),
            material=steel,
            name="hub_sleeve",
        )
        model.articulation(
            f"wheel_joint_{index}",
            ArticulationType.CONTINUOUS,
            parent=trailer,
            child=wheel,
            origin=Origin(xyz=(0.16, y, 0.27)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=20.0, velocity=8.0),
        )

    mast = model.part("folding_mast")
    mast.visual(
        Cylinder(radius=0.055, length=0.24),
        origin=Origin(rpy=(-math.pi / 2, 0.0, 0.0)),
        material=aluminium,
        name="mast_hinge_barrel",
    )
    mast.visual(Box((0.08, 0.08, 2.55)), origin=Origin(xyz=(0.0, 0.0, 1.315)), material=aluminium, name="mast_tube")
    for z, suffix in [(0.78, "0"), (1.55, "1"), (2.30, "2")]:
        mast.visual(Box((0.12, 0.12, 0.055)), origin=Origin(xyz=(0.0, 0.0, z)), material=steel, name=f"mast_collar_{suffix}")
    mast.visual(Box((0.09, 0.12, 0.06)), origin=Origin(xyz=(-0.075, 0.0, 0.91)), material=steel, name="chain_lug")
    mast.visual(Box((0.07, 0.80, 0.07)), origin=Origin(xyz=(-0.055, 0.0, 2.52)), material=aluminium, name="top_yoke_crossbar")
    mast.visual(Box((0.15, 0.05, 0.07)), origin=Origin(xyz=(0.005, -0.405, 2.52)), material=aluminium, name="top_yoke_bridge_0")
    mast.visual(Box((0.15, 0.05, 0.07)), origin=Origin(xyz=(0.005, 0.405, 2.52)), material=aluminium, name="top_yoke_bridge_1")
    mast.visual(Box((0.11, 0.05, 0.28)), origin=Origin(xyz=(0.075, -0.405, 2.63)), material=aluminium, name="top_yoke_0")
    mast.visual(Box((0.11, 0.05, 0.28)), origin=Origin(xyz=(0.075, 0.405, 2.63)), material=aluminium, name="top_yoke_1")

    mast_joint = model.articulation(
        "base_pivot",
        ArticulationType.REVOLUTE,
        parent=trailer,
        child=mast,
        origin=Origin(xyz=pivot_xyz),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.6, lower=-1.45, upper=0.0),
    )
    mast_joint.meta["description"] = "Folding mast pivot: q=0 is the upright locked position; negative q folds the mast down over the trailer tongue."

    head = model.part("flood_head")
    head.visual(
        Cylinder(radius=0.030, length=0.760),
        origin=Origin(rpy=(-math.pi / 2, 0.0, 0.0)),
        material=steel,
        name="trunnion_pin",
    )
    head.visual(Box((0.18, 0.05, 0.08)), origin=Origin(xyz=(0.07, -0.32, 0.0)), material=steel, name="tilt_ear_0")
    head.visual(Box((0.18, 0.05, 0.08)), origin=Origin(xyz=(0.07, 0.32, 0.0)), material=steel, name="tilt_ear_1")
    head.visual(Box((0.16, 0.72, 0.36)), origin=Origin(xyz=(0.23, 0.0, 0.0)), material=dark, name="head_housing")
    head.visual(Box((0.018, 0.64, 0.28)), origin=Origin(xyz=(0.319, 0.0, 0.0)), material=lens, name="front_lens")
    for row, z in enumerate((-0.09, 0.0, 0.09)):
        for col, y in enumerate((-0.24, -0.08, 0.08, 0.24)):
            head.visual(Box((0.012, 0.050, 0.035)), origin=Origin(xyz=(0.333, y, z)), material=led, name=f"led_{row}_{col}")
    for index, y in enumerate((-0.27, -0.18, -0.09, 0.0, 0.09, 0.18, 0.27)):
        head.visual(Box((0.060, 0.018, 0.30)), origin=Origin(xyz=(0.125, y, 0.0)), material=steel, name=f"heat_sink_{index}")

    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=head,
        origin=Origin(xyz=(0.075, 0.0, 2.63)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=-0.60, upper=0.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    trailer = object_model.get_part("trailer_base")
    mast = object_model.get_part("folding_mast")
    head = object_model.get_part("flood_head")
    mast_joint = object_model.get_articulation("base_pivot")
    tilt_joint = object_model.get_articulation("head_tilt")

    ctx.allow_overlap(
        trailer,
        mast,
        elem_a="base_pivot_pin",
        elem_b="mast_hinge_barrel",
        reason="The steel pivot pin is intentionally captured through the mast hinge barrel.",
    )
    ctx.expect_overlap(
        mast,
        trailer,
        axes="y",
        elem_a="mast_hinge_barrel",
        elem_b="base_pivot_pin",
        min_overlap=0.20,
        name="mast hinge barrel is retained on the base pivot pin",
    )

    ctx.allow_overlap(
        trailer,
        mast,
        elem_a="chain_brace",
        elem_b="chain_lug",
        reason="The chain hook is seated into the mast lug in the upright locked pose.",
    )
    ctx.expect_gap(
        mast,
        trailer,
        axis="x",
        positive_elem="chain_lug",
        negative_elem="chain_brace",
        max_penetration=0.02,
        name="chain brace seats locally into the mast lug",
    )

    for wheel_name, wheel_is_positive_y in (("wheel_0", False), ("wheel_1", True)):
        wheel = object_model.get_part(wheel_name)
        ctx.allow_overlap(
            trailer,
            wheel,
            elem_a="axle",
            elem_b="hub_sleeve",
            reason="The wheel hub sleeve is intentionally captured on the trailer axle end.",
        )
        if wheel_is_positive_y:
            ctx.expect_gap(
                wheel,
                trailer,
                axis="y",
                positive_elem="hub_sleeve",
                negative_elem="axle",
                max_penetration=0.012,
                name=f"{wheel_name} hub is seated on the axle",
            )
        else:
            ctx.expect_gap(
                trailer,
                wheel,
                axis="y",
                positive_elem="axle",
                negative_elem="hub_sleeve",
                max_penetration=0.012,
                name=f"{wheel_name} hub is seated on the axle",
            )

    deployed_mast_aabb = ctx.part_element_world_aabb(mast, elem="mast_tube")
    deployed_head_aabb = ctx.part_element_world_aabb(head, elem="front_lens")
    ctx.check(
        "mast is upright at rest",
        deployed_mast_aabb is not None and deployed_mast_aabb[1][2] > 3.35,
        details=f"mast_tube_aabb={deployed_mast_aabb}",
    )

    with ctx.pose({mast_joint: -1.20}):
        folded_mast_aabb = ctx.part_element_world_aabb(mast, elem="mast_tube")
        ctx.check(
            "base pivot folds mast down over the trailer",
            deployed_mast_aabb is not None
            and folded_mast_aabb is not None
            and folded_mast_aabb[1][2] < deployed_mast_aabb[1][2] - 1.0
            and folded_mast_aabb[0][0] < 0.10,
            details=f"deployed={deployed_mast_aabb}, folded={folded_mast_aabb}",
        )

    with ctx.pose({tilt_joint: 0.55}):
        tilted_head_aabb = ctx.part_element_world_aabb(head, elem="front_lens")
        ctx.check(
            "head knuckle tilts the LED face downward",
            deployed_head_aabb is not None
            and tilted_head_aabb is not None
            and tilted_head_aabb[0][2] < deployed_head_aabb[0][2] - 0.08,
            details=f"level={deployed_head_aabb}, tilted={tilted_head_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
