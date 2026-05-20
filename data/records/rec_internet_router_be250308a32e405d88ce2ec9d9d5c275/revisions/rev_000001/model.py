from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BODY_W = 0.115
BODY_D = 0.036
BODY_H = 0.085
FRONT_Y = -BODY_D / 2.0
REAR_Y = BODY_D / 2.0


def _rounded_body_mesh():
    """Vertical rounded-rectangle router housing, extruded through its depth."""
    body = ExtrudeGeometry.centered(
        rounded_rect_profile(BODY_W, BODY_H, 0.010, corner_segments=10),
        BODY_D,
        cap=True,
        closed=True,
    )
    # The extrusion helper builds along local Z.  Rotate so the rounded profile
    # becomes the front silhouette (X/Z) and the extrusion becomes router depth.
    body.rotate_x(math.pi / 2.0).translate(0.0, 0.0, BODY_H / 2.0)
    return mesh_from_geometry(body, "rounded_router_housing")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_travel_router")

    white = Material("warm_white_plastic", rgba=(0.86, 0.88, 0.86, 1.0))
    dark = Material("soft_black_plastic", rgba=(0.015, 0.017, 0.018, 1.0))
    rubber = Material("matte_rubber", rgba=(0.025, 0.026, 0.025, 1.0))
    charcoal = Material("charcoal_trim", rgba=(0.08, 0.085, 0.09, 1.0))
    blue = Material("blue_status_lens", rgba=(0.10, 0.45, 1.00, 1.0))
    metal = Material("dark_hinge_pin", rgba=(0.18, 0.18, 0.17, 1.0))

    housing = model.part("housing")
    housing.visual(
        _rounded_body_mesh(),
        material=white,
        name="rounded_shell",
    )
    housing.visual(
        Box((0.070, 0.0022, 0.018)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.0010, 0.037)),
        material=charcoal,
        name="front_service_panel",
    )
    housing.visual(
        Box((0.018, 0.0028, 0.010)),
        origin=Origin(xyz=(-0.022, FRONT_Y - 0.0020, 0.036)),
        material=dark,
        name="ethernet_port",
    )
    housing.visual(
        Box((0.014, 0.0028, 0.006)),
        origin=Origin(xyz=(0.006, FRONT_Y - 0.0020, 0.039)),
        material=dark,
        name="usb_port",
    )
    for i, x in enumerate((0.019, 0.027, 0.034)):
        housing.visual(
            Cylinder(radius=0.0020, length=0.0012),
            origin=Origin(
                xyz=(x, FRONT_Y - 0.0021, 0.045),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=blue,
            name=f"status_led_{i}",
        )
    housing.visual(
        Box((0.060, 0.010, 0.0012)),
        origin=Origin(xyz=(0.0, -0.002, BODY_H + 0.0003)),
        material=charcoal,
        name="top_label_inset",
    )

    antenna_pivots = [(-0.046, 0.006, BODY_H + 0.010), (0.046, 0.006, BODY_H + 0.010)]
    for idx, (x, y, z) in enumerate(antenna_pivots):
        housing.visual(
            Box((0.018, 0.014, 0.004)),
            origin=Origin(xyz=(x, y, BODY_H + 0.0010)),
            material=charcoal,
            name=f"antenna_base_{idx}",
        )
        for side, dx in enumerate((-0.0068, 0.0068)):
            housing.visual(
                Box((0.0030, 0.010, 0.012)),
                origin=Origin(xyz=(x + dx, y, z - 0.0012)),
                material=charcoal,
                name=f"antenna_yoke_{idx}_{side}",
            )

    # Rear hinge clips keep the kickstand visibly captured on its lower axis.
    housing.visual(
        Box((0.007, 0.008, 0.016)),
        origin=Origin(xyz=(-0.041, REAR_Y + 0.004, 0.018)),
        material=charcoal,
        name="kickstand_clip_0",
    )
    housing.visual(
        Box((0.007, 0.008, 0.016)),
        origin=Origin(xyz=(0.041, REAR_Y + 0.004, 0.018)),
        material=charcoal,
        name="kickstand_clip_1",
    )
    housing.visual(
        Box((0.085, 0.0025, 0.010)),
        origin=Origin(xyz=(0.0, REAR_Y + 0.0012, 0.018)),
        material=charcoal,
        name="rear_hinge_strip",
    )

    for idx, (x, y, z) in enumerate(antenna_pivots):
        antenna = model.part(f"antenna_{idx}")
        antenna.visual(
            Cylinder(radius=0.0030, length=0.0140),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name="hinge_barrel",
        )
        antenna.visual(
            Cylinder(radius=0.0028, length=0.057),
            origin=Origin(xyz=(0.0, 0.0, 0.0315)),
            material=rubber,
            name="rubber_whip",
        )
        antenna.visual(
            Sphere(radius=0.0031),
            origin=Origin(xyz=(0.0, 0.0, 0.061)),
            material=rubber,
            name="rounded_tip",
        )
        model.articulation(
            f"antenna_hinge_{idx}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=antenna,
            origin=Origin(xyz=(x, y, z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=2.0, lower=-0.15, upper=1.45),
        )

    kickstand = model.part("kickstand")
    kickstand.visual(
        Cylinder(radius=0.0030, length=0.077),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hinge_rod",
    )
    kickstand.visual(
        Box((0.066, 0.004, 0.062)),
        origin=Origin(xyz=(0.0, 0.0045, 0.031)),
        material=charcoal,
        name="support_blade",
    )
    kickstand.visual(
        Box((0.050, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, 0.0060, 0.064)),
        material=dark,
        name="rubber_foot",
    )
    model.articulation(
        "kickstand_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=kickstand,
        origin=Origin(xyz=(0.0, REAR_Y + 0.006, 0.018)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.0, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    kickstand = object_model.get_part("kickstand")
    kickstand_hinge = object_model.get_articulation("kickstand_hinge")

    ctx.allow_overlap(
        kickstand,
        housing,
        elem_a="hinge_rod",
        elem_b="kickstand_clip_0",
        reason="The metal kickstand rod is intentionally captured inside the rear hinge clip.",
    )
    ctx.allow_overlap(
        kickstand,
        housing,
        elem_a="hinge_rod",
        elem_b="kickstand_clip_1",
        reason="The metal kickstand rod is intentionally captured inside the opposite rear hinge clip.",
    )

    ctx.expect_contact(
        kickstand,
        housing,
        elem_a="hinge_rod",
        elem_b="kickstand_clip_0",
        contact_tol=0.0015,
        name="kickstand rod is retained by one rear clip",
    )
    ctx.expect_contact(
        kickstand,
        housing,
        elem_a="hinge_rod",
        elem_b="kickstand_clip_1",
        contact_tol=0.0015,
        name="kickstand rod is retained by the opposite rear clip",
    )

    closed_foot = ctx.part_element_world_aabb(kickstand, elem="rubber_foot")
    with ctx.pose({kickstand_hinge: 1.05}):
        opened_foot = ctx.part_element_world_aabb(kickstand, elem="rubber_foot")
    ctx.check(
        "kickstand folds rearward and downward",
        closed_foot is not None
        and opened_foot is not None
        and opened_foot[1][1] > closed_foot[1][1] + 0.035
        and opened_foot[0][2] < closed_foot[0][2] - 0.020,
        details=f"closed={closed_foot}, opened={opened_foot}",
    )

    for idx in (0, 1):
        antenna = object_model.get_part(f"antenna_{idx}")
        hinge = object_model.get_articulation(f"antenna_hinge_{idx}")
        ctx.allow_overlap(
            antenna,
            housing,
            elem_a="hinge_barrel",
            elem_b=f"antenna_yoke_{idx}_0",
            reason="The antenna hinge barrel is intentionally nested inside the molded top yoke.",
        )
        ctx.allow_overlap(
            antenna,
            housing,
            elem_a="hinge_barrel",
            elem_b=f"antenna_yoke_{idx}_1",
            reason="The antenna hinge barrel is intentionally nested inside the opposite side of the yoke.",
        )
        ctx.expect_contact(
            antenna,
            housing,
            elem_a="hinge_barrel",
            elem_b=f"antenna_yoke_{idx}_0",
            contact_tol=0.0015,
            name=f"antenna {idx} barrel sits in its top hinge yoke",
        )
        ctx.expect_contact(
            antenna,
            housing,
            elem_a="hinge_barrel",
            elem_b=f"antenna_yoke_{idx}_1",
            contact_tol=0.0015,
            name=f"antenna {idx} barrel is retained by the opposite yoke",
        )
        closed_tip = ctx.part_element_world_aabb(antenna, elem="rounded_tip")
        with ctx.pose({hinge: 1.05}):
            folded_tip = ctx.part_element_world_aabb(antenna, elem="rounded_tip")
        ctx.check(
            f"antenna {idx} rotates rearward on its hinge",
            closed_tip is not None
            and folded_tip is not None
            and folded_tip[1][1] > closed_tip[1][1] + 0.030
            and folded_tip[1][2] < closed_tip[1][2] - 0.015,
            details=f"closed={closed_tip}, folded={folded_tip}",
        )

    return ctx.report()


object_model = build_object_model()
