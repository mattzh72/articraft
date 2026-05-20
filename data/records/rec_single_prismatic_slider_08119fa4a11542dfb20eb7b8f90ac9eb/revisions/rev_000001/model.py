from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_pad_linear_slider")

    model.material("dark_anodized_aluminum", color=(0.05, 0.055, 0.06, 1.0))
    model.material("brushed_steel", color=(0.62, 0.64, 0.62, 1.0))
    model.material("bearing_bronze", color=(0.76, 0.52, 0.22, 1.0))
    model.material("carriage_aluminum", color=(0.78, 0.80, 0.82, 1.0))
    model.material("black_oxide", color=(0.015, 0.015, 0.014, 1.0))
    model.material("blue_top_pad", color=(0.02, 0.16, 0.62, 1.0))

    rail = model.part("rail_body")
    rail.visual(
        Box((0.78, 0.20, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material="dark_anodized_aluminum",
        name="base_plate",
    )
    rail.visual(
        Box((0.70, 0.032, 0.035)),
        origin=Origin(xyz=(0.0, -0.052, 0.0475)),
        material="dark_anodized_aluminum",
        name="guide_rail_0",
    )
    rail.visual(
        Box((0.68, 0.004, 0.030)),
        origin=Origin(xyz=(0.0, -0.034, 0.050)),
        material="brushed_steel",
        name="guide_face_0",
    )
    rail.visual(
        Box((0.70, 0.032, 0.035)),
        origin=Origin(xyz=(0.0, 0.052, 0.0475)),
        material="dark_anodized_aluminum",
        name="guide_rail_1",
    )
    rail.visual(
        Box((0.68, 0.004, 0.030)),
        origin=Origin(xyz=(0.0, 0.034, 0.050)),
        material="brushed_steel",
        name="guide_face_1",
    )

    for i, x in enumerate((-0.365, 0.365)):
        rail.visual(
            Box((0.025, 0.180, 0.070)),
            origin=Origin(xyz=(x, 0.0, 0.065)),
            material="dark_anodized_aluminum",
            name=f"end_stop_{i}",
        )

    for i, (x, y) in enumerate(((-0.30, -0.075), (-0.30, 0.075), (0.30, -0.075), (0.30, 0.075))):
        rail.visual(
            Cylinder(radius=0.007, length=0.005),
            origin=Origin(xyz=(x, y, 0.0325)),
            material="black_oxide",
            name=f"rail_screw_{i}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.220, 0.150, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.089)),
        material="carriage_aluminum",
        name="bridge",
    )
    carriage.visual(
        Box((0.195, 0.050, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material="carriage_aluminum",
        name="center_tongue",
    )
    carriage.visual(
        Box((0.170, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, -0.029, 0.050)),
        material="bearing_bronze",
        name="bearing_pad_0",
    )
    carriage.visual(
        Box((0.170, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, 0.029, 0.050)),
        material="bearing_bronze",
        name="bearing_pad_1",
    )
    carriage.visual(
        Box((0.145, 0.095, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.113)),
        material="blue_top_pad",
        name="top_pad",
    )
    for i, (x, y) in enumerate(((-0.055, -0.030), (-0.055, 0.030), (0.055, -0.030), (0.055, 0.030))):
        carriage.visual(
            Cylinder(radius=0.007, length=0.004),
            origin=Origin(xyz=(x, y, 0.1215)),
            material="black_oxide",
            name=f"pad_screw_{i}",
        )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=carriage,
        origin=Origin(xyz=(-0.20, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.40),
        motion_properties=MotionProperties(damping=3.0, friction=1.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail = object_model.get_part("rail_body")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("rail_to_carriage")

    ctx.allow_overlap(
        rail,
        carriage,
        elem_a="guide_face_0",
        elem_b="bearing_pad_0",
        reason="The bronze pad is intentionally seated with a tiny preload into the lower guide face.",
    )
    ctx.allow_overlap(
        rail,
        carriage,
        elem_a="guide_face_1",
        elem_b="bearing_pad_1",
        reason="The bronze pad is intentionally seated with a tiny preload into the upper guide face.",
    )

    for i in (0, 1):
        ctx.expect_overlap(
            carriage,
            rail,
            axes="xz",
            elem_a=f"bearing_pad_{i}",
            elem_b=f"guide_face_{i}",
            min_overlap=0.025,
            name=f"bearing pad {i} overlaps guide face",
        )
        ctx.expect_within(
            carriage,
            rail,
            axes="x",
            inner_elem=f"bearing_pad_{i}",
            outer_elem=f"guide_face_{i}",
            margin=0.0,
            name=f"bearing pad {i} starts inside guide face length",
        )

    ctx.expect_gap(
        carriage,
        rail,
        axis="y",
        positive_elem="bearing_pad_0",
        negative_elem="guide_face_0",
        max_gap=0.001,
        max_penetration=0.004,
        name="lower pad guide preload is small",
    )
    ctx.expect_gap(
        rail,
        carriage,
        axis="y",
        positive_elem="guide_face_1",
        negative_elem="bearing_pad_1",
        max_gap=0.001,
        max_penetration=0.004,
        name="upper pad guide preload is small",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.40}):
        for i in (0, 1):
            ctx.expect_overlap(
                carriage,
                rail,
                axes="xz",
                elem_a=f"bearing_pad_{i}",
                elem_b=f"guide_face_{i}",
                min_overlap=0.025,
                name=f"bearing pad {i} remains on guide face at full travel",
            )
            ctx.expect_within(
                carriage,
                rail,
                axes="x",
                inner_elem=f"bearing_pad_{i}",
                outer_elem=f"guide_face_{i}",
                margin=0.0,
                name=f"bearing pad {i} remains inside guide face length",
            )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage translates along rail axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.39
        and abs(extended_pos[1] - rest_pos[1]) < 1e-6
        and abs(extended_pos[2] - rest_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
