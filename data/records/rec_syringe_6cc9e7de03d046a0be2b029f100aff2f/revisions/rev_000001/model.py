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


CYLINDER_X = (0.0, math.pi / 2.0, 0.0)


def _build_barrel_shell_mesh():
    outer_profile = [
        (0.0021, 0.000),
        (0.0023, 0.006),
        (0.0032, 0.014),
        (0.0047, 0.022),
        (0.0067, 0.030),
        (0.0081, 0.036),
        (0.0108, 0.040),
        (0.0108, 0.112),
        (0.0128, 0.116),
    ]
    inner_profile = [
        (0.00065, 0.000),
        (0.00080, 0.006),
        (0.00115, 0.014),
        (0.0021, 0.022),
        (0.0047, 0.030),
        (0.0072, 0.036),
        (0.0091, 0.040),
        (0.0091, 0.112),
        (0.0110, 0.116),
    ]
    shell = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    shell.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(shell, "syringe_barrel_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hand_syringe")

    clear_plastic = model.material("clear_plastic", rgba=(0.82, 0.87, 0.93, 0.40))
    milky_plastic = model.material("milky_plastic", rgba=(0.92, 0.94, 0.96, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.18, 0.20, 0.22, 1.0))
    blue_grip = model.material("blue_grip", rgba=(0.27, 0.49, 0.83, 1.0))

    barrel_shell_mesh = _build_barrel_shell_mesh()

    barrel = model.part("barrel")
    barrel.visual(
        barrel_shell_mesh,
        origin=Origin(xyz=(-0.058, 0.0, 0.0)),
        material=clear_plastic,
        name="barrel_shell",
    )
    barrel.visual(
        Box((0.005, 0.028, 0.013)),
        origin=Origin(xyz=(0.056, 0.024, 0.0)),
        material=milky_plastic,
        name="finger_flange_left",
    )
    barrel.visual(
        Box((0.005, 0.028, 0.013)),
        origin=Origin(xyz=(0.056, -0.024, 0.0)),
        material=milky_plastic,
        name="finger_flange_right",
    )
    barrel.visual(
        Cylinder(radius=0.0134, length=0.004),
        origin=Origin(xyz=(0.054, 0.0, 0.0), rpy=CYLINDER_X),
        material=milky_plastic,
        name="rear_collar",
    )
    barrel.inertial = Inertial.from_geometry(
        Box((0.126, 0.076, 0.028)),
        mass=0.08,
        origin=Origin(xyz=(0.002, 0.0, 0.0)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0022, length=0.132),
        origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=CYLINDER_X),
        material=blue_grip,
        name="plunger_rod",
    )
    plunger.visual(
        Box((0.008, 0.034, 0.014)),
        origin=Origin(xyz=(0.084, 0.0, 0.0)),
        material=blue_grip,
        name="thumb_pad",
    )
    plunger.visual(
        Cylinder(radius=0.0084, length=0.003),
        origin=Origin(xyz=(-0.050, 0.0, 0.0), rpy=CYLINDER_X),
        material=dark_rubber,
        name="seal_front",
    )
    plunger.visual(
        Cylinder(radius=0.0084, length=0.003),
        origin=Origin(xyz=(-0.044, 0.0, 0.0), rpy=CYLINDER_X),
        material=dark_rubber,
        name="seal_rear",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.138, 0.036, 0.016)),
        mass=0.03,
        origin=Origin(xyz=(0.017, 0.0, 0.0)),
    )

    model.articulation(
        "plunger_slide",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.25,
            lower=0.0,
            upper=0.035,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("plunger_slide")

    ctx.expect_within(
        plunger,
        barrel,
        axes="yz",
        inner_elem="seal_front",
        outer_elem="barrel_shell",
        name="front seal stays centered in barrel bore",
    )
    ctx.expect_within(
        plunger,
        barrel,
        axes="yz",
        inner_elem="seal_rear",
        outer_elem="barrel_shell",
        name="rear seal stays centered in barrel bore",
    )
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="x",
        elem_a="seal_front",
        elem_b="barrel_shell",
        min_overlap=0.002,
        name="front seal remains inserted at rest",
    )
    ctx.expect_gap(
        plunger,
        barrel,
        axis="x",
        positive_elem="thumb_pad",
        negative_elem="barrel_shell",
        min_gap=0.012,
        name="thumb pad sits behind the barrel",
    )

    rest_pos = ctx.part_world_position(plunger)
    upper = slide.motion_limits.upper if slide.motion_limits is not None else 0.0
    with ctx.pose({slide: upper}):
        ctx.expect_within(
            plunger,
            barrel,
            axes="yz",
            inner_elem="seal_front",
            outer_elem="barrel_shell",
            name="front seal stays centered when retracted",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="x",
            elem_a="seal_rear",
            elem_b="barrel_shell",
            min_overlap=0.002,
            name="rear seal remains inserted at max retraction",
        )
        ctx.expect_gap(
            plunger,
            barrel,
            axis="x",
            positive_elem="thumb_pad",
            negative_elem="barrel_shell",
            min_gap=0.040,
            name="thumb pad clears the barrel at max retraction",
        )
        extended_pos = ctx.part_world_position(plunger)

    ctx.check(
        "plunger retracts rearward along barrel axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.02,
        details=f"rest={rest_pos}, retracted={extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
