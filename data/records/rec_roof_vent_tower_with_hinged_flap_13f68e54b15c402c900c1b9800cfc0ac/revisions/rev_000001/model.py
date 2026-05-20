from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_vent_tower")

    galvanized = model.material("galvanized_steel", rgba=(0.73, 0.75, 0.77, 1.0))
    shadow = model.material("shadow_finish", rgba=(0.22, 0.24, 0.26, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.62, 0.50, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=galvanized,
        name="roof_curb",
    )
    body.visual(
        Box((0.03, 0.34, 0.72)),
        origin=Origin(xyz=(-0.205, 0.0, 0.44)),
        material=galvanized,
        name="left_wall",
    )
    body.visual(
        Box((0.03, 0.34, 0.72)),
        origin=Origin(xyz=(0.205, 0.0, 0.44)),
        material=galvanized,
        name="right_wall",
    )
    body.visual(
        Box((0.38, 0.03, 0.72)),
        origin=Origin(xyz=(0.0, -0.155, 0.44)),
        material=galvanized,
        name="back_wall",
    )
    body.visual(
        Box((0.38, 0.03, 0.46)),
        origin=Origin(xyz=(0.0, 0.155, 0.31)),
        material=galvanized,
        name="front_lower_wall",
    )
    body.visual(
        Box((0.44, 0.22, 0.03)),
        origin=Origin(xyz=(0.0, -0.06, 0.815)),
        material=galvanized,
        name="roof_cap",
    )
    body.visual(
        Box((0.44, 0.10, 0.03)),
        origin=Origin(xyz=(0.0, 0.22, 0.555)),
        material=galvanized,
        name="outlet_sill",
    )
    body.visual(
        Box((0.03, 0.10, 0.23)),
        origin=Origin(xyz=(-0.205, 0.22, 0.655)),
        material=galvanized,
        name="left_jamb",
    )
    body.visual(
        Box((0.03, 0.10, 0.23)),
        origin=Origin(xyz=(0.205, 0.22, 0.655)),
        material=galvanized,
        name="right_jamb",
    )
    body.visual(
        Box((0.44, 0.10, 0.03)),
        origin=Origin(xyz=(0.0, 0.22, 0.785)),
        material=galvanized,
        name="outlet_top_beam",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.62, 0.50, 0.80)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
    )

    flap = model.part("flap")
    flap.visual(
        Box((0.44, 0.20, 0.012)),
        origin=Origin(xyz=(0.0, 0.10, -0.006)),
        material=galvanized,
        name="canopy",
    )
    flap.visual(
        Box((0.44, 0.012, 0.11)),
        origin=Origin(xyz=(0.0, 0.194, -0.067)),
        material=galvanized,
        name="front_lip",
    )
    flap.visual(
        Box((0.012, 0.20, 0.11)),
        origin=Origin(xyz=(-0.214, 0.10, -0.067)),
        material=galvanized,
        name="left_cheek",
    )
    flap.visual(
        Box((0.012, 0.20, 0.11)),
        origin=Origin(xyz=(0.214, 0.10, -0.067)),
        material=galvanized,
        name="right_cheek",
    )
    flap.visual(
        Cylinder(radius=0.008, length=0.40),
        origin=Origin(xyz=(0.0, 0.010, -0.008), rpy=(0.0, 1.57079632679, 0.0)),
        material=shadow,
        name="hinge_barrel",
    )
    flap.inertial = Inertial.from_geometry(
        Box((0.44, 0.20, 0.12)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.10, -0.06)),
    )

    model.articulation(
        "body_to_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(0.0, 0.27, 0.80)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("body_to_flap")

    ctx.expect_overlap(
        flap,
        body,
        axes="x",
        min_overlap=0.42,
        elem_a="front_lip",
        elem_b="outlet_top_beam",
        name="flap spans the outlet width",
    )
    ctx.expect_gap(
        flap,
        body,
        axis="y",
        min_gap=0.16,
        max_gap=0.22,
        positive_elem="front_lip",
        negative_elem="outlet_top_beam",
        name="closed flap projects ahead of the framed outlet",
    )

    closed_lip = ctx.part_element_world_aabb(flap, elem="front_lip")
    with ctx.pose({hinge: 1.0}):
        opened_lip = ctx.part_element_world_aabb(flap, elem="front_lip")
        ctx.check(
            "flap opens upward from the top hinge",
            closed_lip is not None
            and opened_lip is not None
            and opened_lip[0][2] > closed_lip[0][2] + 0.12,
            details=f"closed={closed_lip}, opened={opened_lip}",
        )
        ctx.expect_gap(
            flap,
            body,
            axis="y",
            min_gap=0.10,
            positive_elem="front_lip",
            negative_elem="outlet_top_beam",
            name="opened flap still stays in front of the outlet frame",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
