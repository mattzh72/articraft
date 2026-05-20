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


X_ALIGNED = Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hand_syringe")

    clear_poly = model.material("clear_poly", rgba=(0.92, 0.97, 1.0, 0.35))
    white_plastic = model.material("white_plastic", rgba=(0.95, 0.95, 0.95, 1.0))
    gray_plastic = model.material("gray_plastic", rgba=(0.76, 0.78, 0.80, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.12, 0.12, 0.13, 1.0))

    barrel = model.part("barrel")

    barrel_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.0105, 0.000),
            (0.0105, 0.005),
            (0.0082, 0.008),
            (0.0082, 0.086),
            (0.0060, 0.095),
            (0.0033, 0.105),
            (0.0022, 0.112),
        ],
        inner_profile=[
            (0.0025, 0.000),
            (0.0025, 0.004),
            (0.0048, 0.0065),
            (0.0072, 0.010),
            (0.0072, 0.087),
            (0.0040, 0.097),
            (0.0010, 0.106),
            (0.0007, 0.112),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    barrel.visual(
        mesh_from_geometry(barrel_shell, "syringe_barrel_shell"),
        origin=X_ALIGNED,
        material=clear_poly,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=0.0108, length=0.006),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=X_ALIGNED.rpy),
        material=gray_plastic,
        name="rear_guide",
    )
    barrel.visual(
        Box((0.004, 0.022, 0.009)),
        origin=Origin(xyz=(0.004, 0.014, 0.0)),
        material=gray_plastic,
        name="left_finger_flange",
    )
    barrel.visual(
        Box((0.004, 0.022, 0.009)),
        origin=Origin(xyz=(0.004, -0.014, 0.0)),
        material=gray_plastic,
        name="right_finger_flange",
    )
    barrel.visual(
        Cylinder(radius=0.0042, length=0.008),
        origin=Origin(xyz=(0.096, 0.0, 0.0), rpy=X_ALIGNED.rpy),
        material=gray_plastic,
        name="tip_hub",
    )
    barrel.inertial = Inertial.from_geometry(
        Box((0.118, 0.060, 0.028)),
        mass=0.040,
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0018, length=0.114),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=X_ALIGNED.rpy),
        material=white_plastic,
        name="guide_rod",
    )
    plunger.visual(
        Cylinder(radius=0.0043, length=0.014),
        origin=Origin(xyz=(-0.049, 0.0, 0.0), rpy=X_ALIGNED.rpy),
        material=white_plastic,
        name="thumb_neck",
    )
    plunger.visual(
        Cylinder(radius=0.013, length=0.006),
        origin=Origin(xyz=(-0.058, 0.0, 0.0), rpy=X_ALIGNED.rpy),
        material=white_plastic,
        name="thumb_pad",
    )
    plunger.visual(
        Cylinder(radius=0.0032, length=0.010),
        origin=Origin(xyz=(0.071, 0.0, 0.0), rpy=X_ALIGNED.rpy),
        material=white_plastic,
        name="seal_stem",
    )
    plunger.visual(
        Cylinder(radius=0.0066, length=0.010),
        origin=Origin(xyz=(0.078, 0.0, 0.0), rpy=X_ALIGNED.rpy),
        material=black_rubber,
        name="plunger_core",
    )
    plunger.visual(
        Cylinder(radius=0.0069, length=0.002),
        origin=Origin(xyz=(0.074, 0.0, 0.0), rpy=X_ALIGNED.rpy),
        material=black_rubber,
        name="seal_rib_rear",
    )
    plunger.visual(
        Cylinder(radius=0.0069, length=0.002),
        origin=Origin(xyz=(0.082, 0.0, 0.0), rpy=X_ALIGNED.rpy),
        material=black_rubber,
        name="seal_rib_front",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.146, 0.028, 0.028)),
        mass=0.012,
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
    )

    model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.20,
            lower=0.0,
            upper=0.055,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("barrel_to_plunger")
    upper = slide.motion_limits.upper if slide.motion_limits is not None else 0.0

    ctx.expect_within(
        plunger,
        barrel,
        axes="yz",
        inner_elem="seal_rib_front",
        outer_elem="barrel_shell",
        margin=0.0,
        name="front seal stays inside barrel diameter at rest",
    )
    ctx.expect_within(
        plunger,
        barrel,
        axes="yz",
        inner_elem="guide_rod",
        outer_elem="barrel_shell",
        margin=0.0,
        name="rod stays centered within the barrel guide at rest",
    )
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="x",
        elem_a="plunger_core",
        elem_b="barrel_shell",
        min_overlap=0.010,
        name="seal remains inserted in the barrel at rest",
    )

    rest_pos = ctx.part_world_position(plunger)

    with ctx.pose({slide: upper}):
        ctx.expect_within(
            plunger,
            barrel,
            axes="yz",
            inner_elem="seal_rib_rear",
            outer_elem="barrel_shell",
            margin=0.0,
            name="rear seal stays inside barrel diameter when retracted",
        )
        ctx.expect_within(
            plunger,
            barrel,
            axes="yz",
            inner_elem="guide_rod",
            outer_elem="barrel_shell",
            margin=0.0,
            name="rod stays aligned with the barrel when retracted",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="x",
            elem_a="plunger_core",
            elem_b="barrel_shell",
            min_overlap=0.010,
            name="seal retains insertion at full draw",
        )
        retracted_pos = ctx.part_world_position(plunger)

    ctx.check(
        "positive travel retracts the plunger backward",
        rest_pos is not None
        and retracted_pos is not None
        and retracted_pos[0] < rest_pos[0] - 0.030,
        details=f"rest={rest_pos}, retracted={retracted_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
