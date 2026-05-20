from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_feed_juicer")

    body_black = model.material("body_black", rgba=(0.16, 0.17, 0.18, 1.0))
    smoked_clear = model.material("smoked_clear", rgba=(0.72, 0.82, 0.88, 0.35))
    pusher_black = model.material("pusher_black", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.82, 0.84, 0.86, 1.0))

    body = model.part("body")

    body.visual(
        Box((0.42, 0.28, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=body_black,
        name="base_plate",
    )
    body.visual(
        Box((0.32, 0.04, 0.08)),
        origin=Origin(xyz=(0.0, -0.12, 0.08)),
        material=body_black,
        name="side_wall_0",
    )
    body.visual(
        Box((0.32, 0.04, 0.08)),
        origin=Origin(xyz=(0.0, 0.12, 0.08)),
        material=body_black,
        name="side_wall_1",
    )
    body.visual(
        Box((0.050, 0.20, 0.08)),
        origin=Origin(xyz=(-0.185, 0.0, 0.08)),
        material=body_black,
        name="rear_wall",
    )
    body.visual(
        Box((0.050, 0.20, 0.06)),
        origin=Origin(xyz=(0.185, 0.0, 0.07)),
        material=body_black,
        name="front_wall",
    )
    body.visual(
        Box((0.22, 0.03, 0.012)),
        origin=Origin(xyz=(0.0, -0.125, 0.126)),
        material=body_black,
        name="deck_band_0",
    )
    body.visual(
        Box((0.22, 0.03, 0.012)),
        origin=Origin(xyz=(0.0, 0.125, 0.126)),
        material=body_black,
        name="deck_band_1",
    )
    body.visual(
        Box((0.10, 0.22, 0.012)),
        origin=Origin(xyz=(-0.16, 0.0, 0.126)),
        material=body_black,
        name="deck_band_2",
    )
    body.visual(
        Box((0.10, 0.22, 0.012)),
        origin=Origin(xyz=(0.16, 0.0, 0.126)),
        material=body_black,
        name="deck_band_3",
    )
    body.visual(
        Box((0.055, 0.070, 0.030)),
        origin=Origin(xyz=(0.222, 0.0, 0.070)),
        material=body_black,
        name="spout",
    )
    body.visual(
        Box((0.040, 0.160, 0.082)),
        origin=Origin(xyz=(-0.178, 0.0, 0.161)),
        material=body_black,
        name="rear_pedestal",
    )
    body.visual(
        Cylinder(radius=0.045, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=body_black,
        name="motor_hub",
    )

    basket = model.part("basket")
    basket_shell = LatheGeometry.from_shell_profiles(
        [
            (0.018, 0.0),
            (0.045, 0.010),
            (0.082, 0.032),
            (0.096, 0.080),
            (0.100, 0.095),
        ],
        [
            (0.0, 0.004),
            (0.040, 0.014),
            (0.074, 0.033),
            (0.089, 0.084),
            (0.094, 0.090),
        ],
        segments=56,
    )
    basket.visual(
        mesh_from_geometry(basket_shell, "basket_shell"),
        material=steel,
        name="basket_shell",
    )
    basket.visual(
        Cylinder(radius=0.098, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.093)),
        material=steel,
        name="cutter",
    )
    basket.visual(
        Cylinder(radius=0.020, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=steel,
        name="hub",
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.085, 0.120, 0.010)),
        origin=Origin(xyz=(0.2625, 0.0, -0.005)),
        material=smoked_clear,
        name="front_band",
    )
    lid.visual(
        Box((0.220, 0.040, 0.010)),
        origin=Origin(xyz=(0.157, -0.080, -0.005)),
        material=smoked_clear,
        name="deck_band_0",
    )
    lid.visual(
        Box((0.220, 0.040, 0.010)),
        origin=Origin(xyz=(0.157, 0.080, -0.005)),
        material=smoked_clear,
        name="deck_band_1",
    )
    lid.visual(
        Box((0.018, 0.220, 0.050)),
        origin=Origin(xyz=(0.301, 0.0, -0.035)),
        material=smoked_clear,
        name="front_skirt",
    )
    lid.visual(
        Box((0.290, 0.012, 0.050)),
        origin=Origin(xyz=(0.155, -0.104, -0.035)),
        material=smoked_clear,
        name="side_skirt_0",
    )
    lid.visual(
        Box((0.290, 0.012, 0.050)),
        origin=Origin(xyz=(0.155, 0.104, -0.035)),
        material=smoked_clear,
        name="side_skirt_1",
    )

    chute = model.part("chute")
    chute.visual(
        Box((0.010, 0.120, 0.130)),
        origin=Origin(xyz=(0.095, 0.0, 0.065)),
        material=smoked_clear,
        name="rear_wall",
    )
    chute.visual(
        Box((0.010, 0.120, 0.130)),
        origin=Origin(xyz=(0.225, 0.0, 0.065)),
        material=smoked_clear,
        name="front_wall",
    )
    chute.visual(
        Box((0.130, 0.010, 0.130)),
        origin=Origin(xyz=(0.160, -0.065, 0.065)),
        material=smoked_clear,
        name="side_wall_0",
    )
    chute.visual(
        Box((0.130, 0.010, 0.130)),
        origin=Origin(xyz=(0.160, 0.065, 0.065)),
        material=smoked_clear,
        name="side_wall_1",
    )

    pusher = model.part("pusher")
    pusher.visual(
        Box((0.120, 0.120, 0.110)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=pusher_black,
        name="block",
    )
    pusher.visual(
        Box((0.054, 0.040, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=pusher_black,
        name="stem",
    )
    pusher.visual(
        Box((0.150, 0.120, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.154)),
        material=pusher_black,
        name="cap",
    )
    pusher.visual(
        Cylinder(radius=0.016, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.176), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pusher_black,
        name="grip",
    )

    flap = model.part("flap")
    flap.visual(
        Box((0.006, 0.096, 0.120)),
        origin=Origin(xyz=(0.003, 0.0, 0.060)),
        material=smoked_clear,
        name="panel",
    )
    flap.visual(
        Box((0.014, 0.040, 0.010)),
        origin=Origin(xyz=(0.003, 0.0, 0.112)),
        material=smoked_clear,
        name="tab",
    )

    model.articulation(
        "body_to_basket",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=30.0),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.205, 0.0, 0.210)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "lid_to_chute",
        ArticulationType.FIXED,
        parent=lid,
        child=chute,
        origin=Origin(),
    )
    model.articulation(
        "chute_to_pusher",
        ArticulationType.PRISMATIC,
        parent=chute,
        child=pusher,
        origin=Origin(xyz=(0.160, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.45,
            lower=0.0,
            upper=0.165,
        ),
    )
    model.articulation(
        "chute_to_flap",
        ArticulationType.REVOLUTE,
        parent=chute,
        child=flap,
        origin=Origin(xyz=(0.080, 0.0, 0.130)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    basket = object_model.get_part("basket")
    lid = object_model.get_part("lid")
    chute = object_model.get_part("chute")
    pusher = object_model.get_part("pusher")
    flap = object_model.get_part("flap")

    lid_hinge = object_model.get_articulation("body_to_lid")
    pusher_slide = object_model.get_articulation("chute_to_pusher")
    flap_hinge = object_model.get_articulation("chute_to_flap")

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.20,
        name="lid covers the body opening footprint",
    )
    ctx.expect_within(
        basket,
        body,
        axes="xy",
        margin=0.01,
        inner_elem="basket_shell",
        name="basket stays centered inside the body well",
    )
    ctx.expect_within(
        pusher,
        chute,
        axes="xy",
        margin=0.01,
        inner_elem="block",
        name="pusher block stays guided inside the chute",
    )

    rest_front = ctx.part_element_world_aabb(lid, elem="front_skirt")
    with ctx.pose({lid_hinge: math.radians(70.0)}):
        opened_front = ctx.part_element_world_aabb(lid, elem="front_skirt")
    ctx.check(
        "lid opens upward from the rear hinge",
        rest_front is not None
        and opened_front is not None
        and opened_front[0][2] > rest_front[0][2] + 0.10,
        details=f"rest={rest_front}, opened={opened_front}",
    )

    with ctx.pose({pusher_slide: 0.165}):
        ctx.expect_gap(
            pusher,
            chute,
            axis="z",
            min_gap=0.004,
            positive_elem="block",
            name="raised pusher clears the chute opening",
        )

    with ctx.pose({pusher_slide: 0.165, flap_hinge: math.pi / 2.0}):
        ctx.expect_overlap(
            flap,
            chute,
            axes="xy",
            min_overlap=0.09,
            elem_a="panel",
            name="flap can cover the chute when the pusher is lifted",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
