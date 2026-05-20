from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


TRAVEL = 0.024
SLIDER_OPEN_X = 0.056
GUIDE_Y = 0.034


def _jaw_profile(sign: float) -> list[tuple[float, float]]:
    """Tapered top-view jaw profile, mirrored by the sign of X."""

    points = [
        (0.000, -0.014),
        (0.016, -0.012),
        (0.030, -0.005),
        (0.030, 0.005),
        (0.016, 0.012),
        (0.000, 0.014),
    ]
    mirrored = [(sign * x, y) for x, y in points]
    if sign < 0.0:
        mirrored.reverse()
    return mirrored


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_precision_gripper")

    model.material("dark_anodized", rgba=(0.12, 0.13, 0.15, 1.0))
    model.material("matte_black", rgba=(0.035, 0.036, 0.040, 1.0))
    model.material("ground_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    model.material("brushed_steel", rgba=(0.48, 0.50, 0.53, 1.0))
    model.material("bronze_pad", rgba=(0.72, 0.52, 0.25, 1.0))
    model.material("hardened_tip", rgba=(0.10, 0.105, 0.11, 1.0))

    body = model.part("center_body")
    body.visual(
        Box((0.205, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material="dark_anodized",
        name="backbone",
    )
    for idx, y in enumerate((-GUIDE_Y, GUIDE_Y)):
        body.visual(
            Box((0.168, 0.012, 0.010)),
            origin=Origin(xyz=(0.0, y, 0.021)),
            material="dark_anodized",
            name=f"guide_mount_{idx}",
        )
    for idx, x in enumerate((-0.060, 0.0, 0.060)):
        body.visual(
            Box((0.012, 0.080, 0.012)),
            origin=Origin(xyz=(x, 0.0, 0.018)),
            material="dark_anodized",
            name=f"cross_web_{idx}",
        )
    body.visual(
        Box((0.018, 0.022, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0255)),
        material="matte_black",
        name="work_zone_floor",
    )
    for idx, x in enumerate((-0.092, 0.092)):
        body.visual(
            Box((0.008, 0.086, 0.024)),
            origin=Origin(xyz=(x, 0.0, 0.032)),
            material="dark_anodized",
            name=f"end_stop_{idx}",
        )

    for idx, sign in enumerate((-1.0, 1.0)):
        guide = model.part(f"guide_{idx}")
        guide.visual(
            Box((0.160, 0.010, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, 0.031)),
            material="ground_steel",
            name="rail",
        )
        guide.visual(
            Box((0.145, 0.006, 0.004)),
            origin=Origin(xyz=(0.0, sign * 0.008, 0.036)),
            material="brushed_steel",
            name="cover",
        )
        for bolt_idx, x in enumerate((-0.058, 0.058)):
            guide.visual(
                Cylinder(radius=0.0023, length=0.0016),
                origin=Origin(xyz=(x, sign * 0.008, 0.0383)),
                material="hardened_tip",
                name=f"cover_bolt_{bolt_idx}",
            )
        model.articulation(
            f"body_to_guide_{idx}",
            ArticulationType.FIXED,
            parent=body,
            child=guide,
            origin=Origin(xyz=(0.0, sign * GUIDE_Y, 0.0)),
        )

    for idx, sign in enumerate((1.0, -1.0)):
        slider = model.part(f"jaw_slider_{idx}")
        slider.visual(
            Box((0.036, 0.086, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, 0.047)),
            material="brushed_steel",
            name="carriage_shoe",
        )
        for pad_idx, y in enumerate((-GUIDE_Y, GUIDE_Y)):
            slider.visual(
                Box((0.030, 0.008, 0.004)),
                origin=Origin(xyz=(0.0, y, 0.0380)),
                material="bronze_pad",
                name=f"slide_pad_{pad_idx}",
            )
        slider.visual(
            mesh_from_geometry(
                ExtrudeGeometry(_jaw_profile(sign), 0.016, center=True),
                f"jaw_taper_{idx}",
            ),
            origin=Origin(xyz=(0.0, 0.0, 0.0615)),
            material="matte_black",
            name="tapered_jaw",
        )
        slider.visual(
            Box((0.004, 0.012, 0.020)),
            origin=Origin(xyz=(sign * 0.029, 0.0, 0.062)),
            material="hardened_tip",
            name="tip_pad",
        )
        for bolt_idx, (x, y) in enumerate(
            ((-0.010, -0.028), (-0.010, 0.028), (0.010, -0.028), (0.010, 0.028))
        ):
            slider.visual(
                Cylinder(radius=0.0036, length=0.004),
                origin=Origin(xyz=(x, y, 0.0558)),
                material="hardened_tip",
                name=f"shoulder_bolt_{bolt_idx}",
            )

        model.articulation(
            f"body_to_jaw_slider_{idx}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=slider,
            origin=Origin(xyz=(-SLIDER_OPEN_X if idx == 0 else SLIDER_OPEN_X, 0.0, 0.0)),
            axis=(1.0 if idx == 0 else -1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=70.0, velocity=0.18, lower=0.0, upper=TRAVEL),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("center_body")
    guide_0 = object_model.get_part("guide_0")
    guide_1 = object_model.get_part("guide_1")
    slider_0 = object_model.get_part("jaw_slider_0")
    slider_1 = object_model.get_part("jaw_slider_1")
    slide_0 = object_model.get_articulation("body_to_jaw_slider_0")
    slide_1 = object_model.get_articulation("body_to_jaw_slider_1")

    ctx.expect_gap(
        guide_0,
        body,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.000001,
        positive_elem="rail",
        negative_elem="guide_mount_0",
        name="guide 0 is seated on its machined mount",
    )
    ctx.expect_gap(
        guide_1,
        body,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.000001,
        positive_elem="rail",
        negative_elem="guide_mount_1",
        name="guide 1 is seated on its machined mount",
    )
    for slider in (slider_0, slider_1):
        ctx.expect_gap(
            slider,
            guide_0,
            axis="z",
            min_gap=0.0,
            max_gap=0.0002,
            positive_elem="slide_pad_0",
            negative_elem="rail",
            name=f"{slider.name} lower pad has guide clearance",
        )
        ctx.expect_gap(
            slider,
            guide_1,
            axis="z",
            min_gap=0.0,
            max_gap=0.0002,
            positive_elem="slide_pad_1",
            negative_elem="rail",
            name=f"{slider.name} upper pad has guide clearance",
        )

    ctx.expect_gap(
        slider_1,
        slider_0,
        axis="x",
        min_gap=0.045,
        max_gap=0.060,
        positive_elem="tip_pad",
        negative_elem="tip_pad",
        name="open jaws leave a usable central gap",
    )
    ctx.expect_gap(
        slider_0,
        body,
        axis="x",
        min_gap=0.008,
        positive_elem="carriage_shoe",
        negative_elem="end_stop_0",
        name="slider 0 clears its outer end stop",
    )
    ctx.expect_gap(
        body,
        slider_1,
        axis="x",
        min_gap=0.008,
        positive_elem="end_stop_1",
        negative_elem="carriage_shoe",
        name="slider 1 clears its outer end stop",
    )

    rest_0 = ctx.part_world_position(slider_0)
    rest_1 = ctx.part_world_position(slider_1)
    with ctx.pose({slide_0: TRAVEL, slide_1: TRAVEL}):
        ctx.expect_gap(
            slider_1,
            slider_0,
            axis="x",
            min_gap=0.0010,
            max_gap=0.0060,
            positive_elem="tip_pad",
            negative_elem="tip_pad",
            name="closed jaws retain a small rectangular work-zone clearance",
        )
        ctx.expect_overlap(
            slider_0,
            guide_0,
            axes="x",
            min_overlap=0.020,
            elem_a="slide_pad_0",
            elem_b="rail",
            name="slider 0 remains engaged on guide 0 at full travel",
        )
        ctx.expect_overlap(
            slider_1,
            guide_1,
            axes="x",
            min_overlap=0.020,
            elem_a="slide_pad_1",
            elem_b="rail",
            name="slider 1 remains engaged on guide 1 at full travel",
        )
        moved_0 = ctx.part_world_position(slider_0)
        moved_1 = ctx.part_world_position(slider_1)

    ctx.check(
        "mirrored sliders move toward center",
        rest_0 is not None
        and rest_1 is not None
        and moved_0 is not None
        and moved_1 is not None
        and moved_0[0] > rest_0[0] + 0.020
        and moved_1[0] < rest_1[0] - 0.020,
        details=f"rest_0={rest_0}, moved_0={moved_0}, rest_1={rest_1}, moved_1={moved_1}",
    )

    return ctx.report()


object_model = build_object_model()
