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


def _barrel_shell_mesh():
    outer_profile = [
        (0.0046, 0.000),
        (0.0062, 0.004),
        (0.0084, 0.011),
        (0.0085, 0.050),
        (0.0087, 0.070),
        (0.0092, 0.074),
    ]
    inner_profile = [
        (0.0022, 0.000),
        (0.0042, 0.004),
        (0.0068, 0.012),
        (0.0068, 0.051),
        (0.0067, 0.068),
        (0.0065, 0.0715),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="flat",
        ),
        "syringe_barrel_shell",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hand_syringe")

    clear_plastic = model.material("clear_plastic", rgba=(0.88, 0.93, 0.97, 0.40))
    white_plastic = model.material("white_plastic", rgba=(0.94, 0.95, 0.96, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.18, 0.18, 0.19, 1.0))
    nozzle_gray = model.material("nozzle_gray", rgba=(0.82, 0.84, 0.86, 1.0))

    barrel = model.part("barrel")
    barrel.visual(
        _barrel_shell_mesh(),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clear_plastic,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=0.0052, length=0.004),
        origin=Origin(xyz=(-0.0015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nozzle_gray,
        name="luer_collar",
    )
    barrel.visual(
        Cylinder(radius=0.0020, length=0.011),
        origin=Origin(xyz=(-0.0090, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nozzle_gray,
        name="nozzle_tip",
    )
    barrel.visual(
        Cylinder(radius=0.0108, length=0.0045),
        origin=Origin(xyz=(0.0755, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clear_plastic,
        name="rear_collar",
    )
    barrel.visual(
        Box((0.004, 0.024, 0.003)),
        origin=Origin(xyz=(0.0765, 0.017, 0.0)),
        material=clear_plastic,
        name="finger_flange_left",
    )
    barrel.visual(
        Box((0.004, 0.024, 0.003)),
        origin=Origin(xyz=(0.0765, -0.017, 0.0)),
        material=clear_plastic,
        name="finger_flange_right",
    )
    barrel.inertial = Inertial.from_geometry(
        Box((0.104, 0.050, 0.026)),
        mass=0.045,
        origin=Origin(xyz=(0.036, 0.0, 0.0)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0017, length=0.052),
        origin=Origin(xyz=(-0.026, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white_plastic,
        name="rod",
    )
    plunger.visual(
        Cylinder(radius=0.0061, length=0.009),
        origin=Origin(xyz=(-0.056, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="seal_core",
    )
    plunger.visual(
        Cylinder(radius=0.00635, length=0.0018),
        origin=Origin(xyz=(-0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="seal_front",
    )
    plunger.visual(
        Cylinder(radius=0.00635, length=0.0018),
        origin=Origin(xyz=(-0.052, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="seal_rear",
    )
    plunger.visual(
        Cylinder(radius=0.0035, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white_plastic,
        name="thumb_stem",
    )
    plunger.visual(
        Box((0.008, 0.024, 0.004)),
        origin=Origin(xyz=(0.0215, 0.0, 0.0)),
        material=white_plastic,
        name="thumb_pad",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.090, 0.024, 0.014)),
        mass=0.018,
        origin=Origin(xyz=(-0.018, 0.0, 0.0)),
    )

    model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(0.074, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=0.030,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("barrel_to_plunger")
    upper = slide.motion_limits.upper if slide.motion_limits is not None else 0.03

    with ctx.pose({slide: 0.0}):
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="yz",
            elem_a="seal_core",
            elem_b="barrel_shell",
            min_overlap=0.011,
            name="seal stays centered in the barrel at rest",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="x",
            elem_a="seal_core",
            elem_b="barrel_shell",
            min_overlap=0.008,
            name="seal is inserted in the barrel at rest",
        )
        rest_pos = ctx.part_world_position(plunger)

    with ctx.pose({slide: upper}):
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="yz",
            elem_a="seal_core",
            elem_b="barrel_shell",
            min_overlap=0.011,
            name="seal stays centered in the barrel when retracted",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="x",
            elem_a="seal_core",
            elem_b="barrel_shell",
            min_overlap=0.008,
            name="seal remains retained at full retraction",
        )
        extended_pos = ctx.part_world_position(plunger)

    ctx.check(
        "plunger retracts backward along the barrel axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.02,
        details=f"rest={rest_pos}, retracted={extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
