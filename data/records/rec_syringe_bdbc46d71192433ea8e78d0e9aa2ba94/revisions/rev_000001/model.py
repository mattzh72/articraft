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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BARREL_INNER_RADIUS = 0.0132
BARREL_OUTER_RADIUS = 0.0160
BARREL_STRAIGHT_LENGTH = 0.106
NOZZLE_LENGTH = 0.037
PLUNGER_TRAVEL = 0.078


def _barrel_shell():
    outer_profile = [
        (BARREL_OUTER_RADIUS, 0.000),
        (BARREL_OUTER_RADIUS, BARREL_STRAIGHT_LENGTH),
        (0.0125, 0.112),
        (0.0088, 0.121),
        (0.0050, 0.134),
        (0.0031, BARREL_STRAIGHT_LENGTH + NOZZLE_LENGTH),
    ]
    inner_profile = [
        (BARREL_INNER_RADIUS, 0.0015),
        (BARREL_INNER_RADIUS, 0.103),
        (0.0102, 0.111),
        (0.0061, 0.121),
        (0.0015, 0.1345),
        (0.0009, BARREL_STRAIGHT_LENGTH + NOZZLE_LENGTH),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "syringe_barrel_shell",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hand_syringe")

    clear_poly = model.material("clear_poly", rgba=(0.86, 0.95, 1.0, 0.38))
    plunger_white = model.material("plunger_white", rgba=(0.94, 0.95, 0.96, 1.0))
    rubber_gray = model.material("rubber_gray", rgba=(0.18, 0.19, 0.22, 1.0))
    marking_blue = model.material("marking_blue", rgba=(0.30, 0.53, 0.77, 1.0))

    barrel = model.part("barrel")
    barrel.visual(
        _barrel_shell(),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clear_poly,
        name="barrel_shell",
    )
    barrel.visual(
        Box((0.012, 0.034, 0.006)),
        origin=Origin(xyz=(0.002, 0.024, 0.0)),
        material=clear_poly,
        name="finger_flange_left",
    )
    barrel.visual(
        Box((0.012, 0.034, 0.006)),
        origin=Origin(xyz=(0.002, -0.024, 0.0)),
        material=clear_poly,
        name="finger_flange_right",
    )
    barrel.visual(
        Cylinder(radius=0.0102, length=0.010),
        origin=Origin(xyz=(0.111, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clear_poly,
        name="front_luer_collar",
    )
    barrel.visual(
        Cylinder(radius=0.0032, length=0.016),
        origin=Origin(xyz=(0.136, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=marking_blue,
        name="nozzle_tip",
    )
    barrel.inertial = Inertial.from_geometry(
        Box((0.152, 0.085, 0.040)),
        mass=0.06,
        origin=Origin(xyz=(0.064, 0.0, 0.0)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0042, length=0.112),
        origin=Origin(xyz=(-0.039, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plunger_white,
        name="plunger_rod",
    )
    plunger.visual(
        Cylinder(radius=0.0060, length=0.014),
        origin=Origin(xyz=(-0.093, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plunger_white,
        name="thumb_stem",
    )
    plunger.visual(
        Box((0.008, 0.038, 0.012)),
        origin=Origin(xyz=(-0.101, 0.0, 0.0)),
        material=plunger_white,
        name="thumb_pad",
    )
    plunger.visual(
        Cylinder(radius=0.0118, length=0.010),
        origin=Origin(xyz=(0.019, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_gray,
        name="plunger_head_core",
    )
    plunger.visual(
        Cylinder(radius=0.0132, length=0.004),
        origin=Origin(xyz=(0.0155, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_gray,
        name="plunger_seal_rear",
    )
    plunger.visual(
        Cylinder(radius=0.0132, length=0.004),
        origin=Origin(xyz=(0.0225, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_gray,
        name="plunger_seal_front",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.132, 0.040, 0.026)),
        mass=0.02,
        origin=Origin(xyz=(-0.034, 0.0, 0.0)),
    )

    model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.18,
            lower=0.0,
            upper=PLUNGER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("barrel_to_plunger")
    upper = slide.motion_limits.upper if slide.motion_limits is not None else 0.0

    with ctx.pose({slide: 0.0}):
        ctx.expect_within(
            plunger,
            barrel,
            axes="yz",
            inner_elem="plunger_seal_front",
            outer_elem="barrel_shell",
            margin=0.0,
            name="retracted plunger head stays inside barrel diameter",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="x",
            elem_a="plunger_head_core",
            elem_b="barrel_shell",
            min_overlap=0.010,
            name="retracted plunger head remains inserted in barrel",
        )

    rest_position = ctx.part_world_position(plunger)
    with ctx.pose({slide: upper}):
        ctx.expect_within(
            plunger,
            barrel,
            axes="yz",
            inner_elem="plunger_seal_front",
            outer_elem="barrel_shell",
            margin=0.0,
            name="full-stroke plunger head stays centered in barrel",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="x",
            elem_a="plunger_head_core",
            elem_b="barrel_shell",
            min_overlap=0.006,
            name="full-stroke plunger head remains retained in barrel",
        )
        ctx.expect_gap(
            barrel,
            plunger,
            axis="x",
            positive_elem="front_luer_collar",
            negative_elem="plunger_seal_front",
            min_gap=0.002,
            max_gap=0.012,
            name="plunger stops short of nozzle collar",
        )
        advanced_position = ctx.part_world_position(plunger)

    ctx.check(
        "plunger advances toward nozzle",
        rest_position is not None
        and advanced_position is not None
        and advanced_position[0] > rest_position[0] + 0.06,
        details=f"rest={rest_position}, advanced={advanced_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
