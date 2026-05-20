from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="event_floodlight_tripod")

    dark = Material("matte black powder-coated steel", rgba=(0.015, 0.014, 0.013, 1.0))
    satin = Material("satin aluminum wear surfaces", rgba=(0.62, 0.60, 0.55, 1.0))
    rubber = Material("black rubber feet", rgba=(0.004, 0.004, 0.004, 1.0))
    warm_lens = Material("warm translucent LED lens", rgba=(1.0, 0.82, 0.34, 0.72))
    led_white = Material("pale LED emitter strip", rgba=(0.92, 0.90, 0.82, 1.0))

    hub_post = model.part("hub_post")

    # Fixed tripod hub and post.  The single root part keeps the fixed central
    # post visibly welded/clamped to the hinge hub rather than articulated.
    hub_post.visual(
        Cylinder(radius=0.075, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.620)),
        material=dark,
        name="base_hub",
    )
    hub_post.visual(
        Cylinder(radius=0.026, length=0.930),
        origin=Origin(xyz=(0.0, 0.0, 1.105)),
        material=satin,
        name="central_post",
    )
    hub_post.visual(
        Cylinder(radius=0.045, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.670)),
        material=dark,
        name="lower_collar",
    )
    hub_post.visual(
        Cylinder(radius=0.040, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 1.510)),
        material=dark,
        name="top_collar",
    )

    # Top yoke for the floodlight tilt hinge.
    hub_post.visual(
        Box((0.050, 0.480, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 1.530)),
        material=dark,
        name="top_yoke_bridge",
    )
    for side, y in enumerate((-0.225, 0.225)):
        hub_post.visual(
            Box((0.040, 0.020, 0.160)),
            origin=Origin(xyz=(0.0, y, 1.625)),
            material=dark,
            name=f"top_yoke_arm_{side}",
        )
    hub_post.visual(
        Cylinder(radius=0.012, length=0.500),
        origin=Origin(xyz=(0.0, 0.0, 1.665), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin,
        name="top_pin",
    )

    # Three clevises around the base hub.  Each clevis is welded into the hub
    # and carries a captured pin for its folding leg.
    hinge_radius = 0.145
    hinge_z = 0.620
    for i, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        # Two side plates leave a central gap for the leg hinge eye.
        for side, y_local in enumerate((-0.037, 0.037)):
            x = c * 0.105 - s * y_local
            y = s * 0.105 + c * y_local
            hub_post.visual(
                Box((0.080, 0.016, 0.052)),
                origin=Origin(xyz=(x, y, hinge_z), rpy=(0.0, 0.0, angle)),
                material=dark,
                name=f"base_clevis_{i}_{side}",
            )
        hub_post.visual(
            Cylinder(radius=0.011, length=0.105),
            origin=Origin(
                xyz=(c * hinge_radius, s * hinge_radius, hinge_z),
                rpy=(-math.pi / 2.0, 0.0, angle),
            ),
            material=satin,
            name=f"base_pin_{i}",
        )

    # Folding legs.  At q=0 the stand is deployed; positive rotation folds the
    # leg upward beside the fixed post.
    leg_length = 0.880
    tube_start = 0.032
    tube_length = leg_length - tube_start
    leg_down_angle = math.radians(42.0)
    leg_dir = (math.cos(leg_down_angle), 0.0, -math.sin(leg_down_angle))
    leg_axis_tilt = math.pi / 2.0 + leg_down_angle

    for i, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        leg = model.part(f"leg_{i}")
        leg.visual(
            Cylinder(radius=0.019, length=0.044),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark,
            name="hinge_eye",
        )
        leg.visual(
            Box((0.030, 0.030, 0.030)),
            origin=Origin(xyz=(0.032, 0.0, -0.015)),
            material=dark,
            name="hinge_neck",
        )
        leg.visual(
            Cylinder(radius=0.014, length=tube_length),
            origin=Origin(
                xyz=(
                    leg_dir[0] * (tube_start + tube_length / 2.0),
                    0.0,
                    leg_dir[2] * (tube_start + tube_length / 2.0),
                ),
                rpy=(0.0, leg_axis_tilt, 0.0),
            ),
            material=satin,
            name="leg_tube",
        )
        leg.visual(
            Box((0.145, 0.075, 0.026)),
            origin=Origin(
                xyz=(
                    leg_dir[0] * leg_length,
                    0.0,
                    leg_dir[2] * leg_length - 0.010,
                )
            ),
            material=rubber,
            name="foot_pad",
        )

        model.articulation(
            f"leg_hinge_{i}",
            ArticulationType.REVOLUTE,
            parent=hub_post,
            child=leg,
            origin=Origin(
                xyz=(math.cos(angle) * hinge_radius, math.sin(angle) * hinge_radius, hinge_z),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=2.20),
        )

    lamp = model.part("lamp_head")
    # Rectangular temporary-work floodlight with a black heat-sink housing,
    # warm front lens, bezel, and side trunnions.
    lamp.visual(
        Box((0.170, 0.340, 0.220)),
        origin=Origin(xyz=(0.115, 0.0, 0.0)),
        material=dark,
        name="rear_housing",
    )
    lamp.visual(
        Box((0.012, 0.286, 0.158)),
        origin=Origin(xyz=(0.203, 0.0, 0.0)),
        material=warm_lens,
        name="front_lens",
    )
    lamp.visual(
        Box((0.004, 0.240, 0.116)),
        origin=Origin(xyz=(0.205, 0.0, 0.0)),
        material=led_white,
        name="led_panel",
    )
    lamp.visual(
        Box((0.024, 0.350, 0.032)),
        origin=Origin(xyz=(0.216, 0.0, 0.086)),
        material=dark,
        name="top_bezel",
    )
    lamp.visual(
        Box((0.024, 0.350, 0.032)),
        origin=Origin(xyz=(0.216, 0.0, -0.086)),
        material=dark,
        name="bottom_bezel",
    )
    for side, y in enumerate((-0.151, 0.151)):
        lamp.visual(
            Box((0.024, 0.035, 0.184)),
            origin=Origin(xyz=(0.216, y, 0.0)),
            material=dark,
            name=f"side_bezel_{side}",
        )
    for idx, y in enumerate((-0.105, -0.070, -0.035, 0.0, 0.035, 0.070, 0.105)):
        lamp.visual(
            Box((0.026, 0.008, 0.190)),
            origin=Origin(xyz=(0.050, y, 0.0)),
            material=satin,
            name=f"heat_sink_fin_{idx}",
        )
    for side, y in enumerate((-0.180, 0.180)):
        lamp.visual(
            Box((0.030, 0.025, 0.064)),
            origin=Origin(xyz=(0.035, -0.164 if y < 0.0 else 0.164, 0.0)),
            material=dark,
            name=f"pivot_boss_{side}",
        )
        lamp.visual(
            Cylinder(radius=0.025, length=0.040),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=f"pivot_sleeve_{side}",
        )

    model.articulation(
        "lamp_tilt",
        ArticulationType.REVOLUTE,
        parent=hub_post,
        child=lamp,
        origin=Origin(xyz=(0.0, 0.0, 1.665)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.70, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hub = object_model.get_part("hub_post")
    lamp = object_model.get_part("lamp_head")
    lamp_tilt = object_model.get_articulation("lamp_tilt")

    def _aabb_center_z(aabb) -> float | None:
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) / 2.0

    # Captured hinge pins intentionally pass through hinge eyes/sleeves.  The
    # overlap is local, coaxial, and is the visible mechanical capture.
    for i in range(3):
        leg = object_model.get_part(f"leg_{i}")
        ctx.allow_overlap(
            hub,
            leg,
            elem_a=f"base_pin_{i}",
            elem_b="hinge_eye",
            reason="The base hinge pin is intentionally captured through the folding leg eye.",
        )
        ctx.expect_overlap(
            hub,
            leg,
            axes="xyz",
            elem_a=f"base_pin_{i}",
            elem_b="hinge_eye",
            min_overlap=0.018,
            name=f"leg {i} hinge pin is captured in its eye",
        )

        hinge = object_model.get_articulation(f"leg_hinge_{i}")
        rest_foot = ctx.part_element_world_aabb(leg, elem="foot_pad")
        with ctx.pose({hinge: 2.20}):
            folded_foot = ctx.part_element_world_aabb(leg, elem="foot_pad")
        rest_z = _aabb_center_z(rest_foot)
        folded_z = _aabb_center_z(folded_foot)
        ctx.check(
            f"leg {i} folds upward from deployed pose",
            rest_z is not None and folded_z is not None and folded_z > rest_z + 0.35,
            details=f"rest_z={rest_z}, folded_z={folded_z}",
        )

    for sleeve in ("pivot_sleeve_0", "pivot_sleeve_1"):
        ctx.allow_overlap(
            hub,
            lamp,
            elem_a="top_pin",
            elem_b=sleeve,
            reason="The lamp tilt pin is intentionally captured inside the side trunnion sleeve.",
        )
        ctx.expect_overlap(
            hub,
            lamp,
            axes="xyz",
            elem_a="top_pin",
            elem_b=sleeve,
            min_overlap=0.020,
            name=f"lamp top pin captured in {sleeve}",
        )

    level_lens = ctx.part_element_world_aabb(lamp, elem="front_lens")
    with ctx.pose({lamp_tilt: 0.85}):
        down_lens = ctx.part_element_world_aabb(lamp, elem="front_lens")
    with ctx.pose({lamp_tilt: -0.70}):
        up_lens = ctx.part_element_world_aabb(lamp, elem="front_lens")
    level_z = _aabb_center_z(level_lens)
    down_z = _aabb_center_z(down_lens)
    up_z = _aabb_center_z(up_lens)
    ctx.check(
        "lamp head tilts down at positive limit",
        level_z is not None and down_z is not None and down_z < level_z - 0.08,
        details=f"level_z={level_z}, down_z={down_z}",
    )
    ctx.check(
        "lamp head tilts up at negative limit",
        level_z is not None and up_z is not None and up_z > level_z + 0.08,
        details=f"level_z={level_z}, up_z={up_z}",
    )

    return ctx.report()


object_model = build_object_model()
