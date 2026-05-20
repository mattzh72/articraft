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
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.0145, 0.000),
            (0.0145, 0.108),
            (0.0102, 0.123),
            (0.0040, 0.144),
            (0.0022, 0.157),
        ],
        [
            (0.0120, 0.002),
            (0.0120, 0.104),
            (0.0060, 0.126),
            (0.0010, 0.151),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(shell, "syringe_barrel_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hand_syringe")

    clear_poly = model.material("clear_poly", rgba=(0.82, 0.88, 0.93, 0.30))
    white_plastic = model.material("white_plastic", rgba=(0.95, 0.95, 0.95, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.18, 0.19, 0.21, 1.0))
    steel = model.material("steel", rgba=(0.60, 0.62, 0.64, 1.0))

    barrel_shell = _barrel_shell_mesh()

    body = model.part("body")
    body.visual(
        barrel_shell,
        origin=Origin(rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=clear_poly,
        name="barrel_shell",
    )
    body.visual(
        Box((0.010, 0.019, 0.006)),
        origin=Origin(xyz=(0.001, 0.0, 0.015)),
        material=white_plastic,
        name="rear_collar_top",
    )
    body.visual(
        Box((0.010, 0.019, 0.006)),
        origin=Origin(xyz=(0.001, 0.0, -0.015)),
        material=white_plastic,
        name="rear_collar_bottom",
    )
    body.visual(
        Box((0.010, 0.005, 0.018)),
        origin=Origin(xyz=(0.001, 0.013, 0.0)),
        material=white_plastic,
        name="rear_collar_right",
    )
    body.visual(
        Box((0.010, 0.005, 0.018)),
        origin=Origin(xyz=(0.001, -0.013, 0.0)),
        material=white_plastic,
        name="rear_collar_left",
    )
    body.visual(
        Box((0.010, 0.024, 0.004)),
        origin=Origin(xyz=(-0.002, 0.021, -0.012)),
        material=white_plastic,
        name="finger_flange_right",
    )
    body.visual(
        Box((0.010, 0.024, 0.004)),
        origin=Origin(xyz=(-0.002, -0.021, -0.012)),
        material=white_plastic,
        name="finger_flange_left",
    )
    body.visual(
        Box((0.014, 0.010, 0.010)),
        origin=Origin(xyz=(0.011, 0.0, 0.020)),
        material=white_plastic,
        name="guide_front_top",
    )
    body.visual(
        Box((0.014, 0.010, 0.010)),
        origin=Origin(xyz=(0.011, 0.0, -0.020)),
        material=white_plastic,
        name="guide_front_bottom",
    )
    body.visual(
        Box((0.070, 0.007, 0.006)),
        origin=Origin(xyz=(0.047, 0.0, 0.025)),
        material=white_plastic,
        name="guide_upper_rail",
    )
    body.visual(
        Box((0.070, 0.007, 0.006)),
        origin=Origin(xyz=(0.047, 0.0, -0.025)),
        material=white_plastic,
        name="guide_lower_rail",
    )
    body.visual(
        Box((0.008, 0.010, 0.056)),
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
        material=white_plastic,
        name="guide_rear_crossbar",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.250, 0.070, 0.072)),
        mass=0.11,
        origin=Origin(xyz=(-0.036, 0.0, 0.0)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0108, length=0.012),
        origin=Origin(xyz=(-0.092, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="plunger_head",
    )
    plunger.visual(
        Cylinder(radius=0.0022, length=0.210),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="plunger_rod",
    )
    plunger.visual(
        Box((0.016, 0.006, 0.034)),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material=white_plastic,
        name="slider_block",
    )
    plunger.visual(
        Cylinder(radius=0.017, length=0.008),
        origin=Origin(xyz=(0.118, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white_plastic,
        name="thumb_pad",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.230, 0.040, 0.040)),
        mass=0.02,
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
    )

    model.articulation(
        "plunger_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=0.20, lower=0.0, upper=0.055),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("plunger_slide")

    ctx.expect_within(
        plunger,
        body,
        axes="yz",
        inner_elem="plunger_head",
        outer_elem="barrel_shell",
        margin=0.002,
        name="plunger head stays centered inside barrel",
    )
    ctx.expect_overlap(
        plunger,
        body,
        axes="x",
        elem_a="plunger_head",
        elem_b="barrel_shell",
        min_overlap=0.010,
        name="rest plunger head remains inserted in barrel",
    )

    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_within(
            plunger,
            body,
            axes="yz",
            inner_elem="plunger_head",
            outer_elem="barrel_shell",
            margin=0.002,
            name="extended plunger head stays centered inside barrel",
        )
        ctx.expect_overlap(
            plunger,
            body,
            axes="x",
            elem_a="plunger_head",
            elem_b="barrel_shell",
            min_overlap=0.010,
            name="extended plunger head still retains insertion",
        )
        extended_pos = ctx.part_world_position(plunger)

    ctx.check(
        "plunger retracts rearward along barrel axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.04,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
