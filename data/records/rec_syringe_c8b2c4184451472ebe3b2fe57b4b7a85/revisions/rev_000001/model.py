from __future__ import annotations

from math import pi

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


BARREL_LENGTH = 0.105
BARREL_OUTER_R = 0.0125
BARREL_INNER_R = 0.0110
PLUNGER_HEAD_R = 0.01115
PLUNGER_HEAD_LENGTH = 0.012
PLUNGER_TRAVEL = 0.055


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _barrel_shell_geometry():
    """Clear hollow barrel, shoulder, and tapered luer-style nozzle along +X."""
    outer_profile = [
        (BARREL_OUTER_R, 0.000),
        (BARREL_OUTER_R, BARREL_LENGTH),
        (0.0090, BARREL_LENGTH + 0.004),
        (0.0050, BARREL_LENGTH + 0.019),
        (0.0027, BARREL_LENGTH + 0.032),
    ]
    inner_profile = [
        (BARREL_INNER_R, 0.000),
        (BARREL_INNER_R, BARREL_LENGTH),
        (0.0064, BARREL_LENGTH + 0.004),
        (0.0030, BARREL_LENGTH + 0.019),
        (0.0012, BARREL_LENGTH + 0.032),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="round",
        end_cap="flat",
        lip_samples=8,
    ).rotate_y(pi / 2.0)



def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hand_syringe")

    clear_plastic = model.material("clear_plastic", rgba=(0.78, 0.92, 1.00, 0.34))
    blue_marking = model.material("blue_marking", rgba=(0.08, 0.25, 0.62, 1.0))
    dark_ink = model.material("dark_ink", rgba=(0.02, 0.03, 0.04, 1.0))
    rubber = model.material("black_rubber", rgba=(0.03, 0.03, 0.035, 1.0))
    white_plastic = model.material("white_plastic", rgba=(0.94, 0.96, 0.98, 1.0))

    barrel = model.part("barrel")
    barrel.visual(
        _save_mesh("barrel_shell", _barrel_shell_geometry()),
        material=clear_plastic,
        name="barrel_shell",
    )
    for tab_name, y_pos in (("finger_tab_0", -0.023), ("finger_tab_1", 0.023)):
        barrel.visual(
            Box((0.006, 0.022, 0.012)),
            origin=Origin(xyz=(0.0, y_pos, 0.0)),
            material=clear_plastic,
            name=tab_name,
        )
    barrel.visual(
        Cylinder(radius=0.0108, length=0.0025),
        origin=Origin(xyz=(BARREL_LENGTH + 0.0008, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=clear_plastic,
        name="front_collar",
    )

    # Printed graduations are slightly proud and overlap the outer wall by a hair
    # so they read as ink bonded to the barrel instead of floating marks.
    for index, x_pos in enumerate([0.018, 0.030, 0.042, 0.054, 0.066, 0.078, 0.090]):
        is_major = index % 2 == 0
        barrel.visual(
            Box((0.0012, 0.010 if is_major else 0.006, 0.0008)),
            origin=Origin(xyz=(x_pos, 0.0, BARREL_OUTER_R + 0.0001)),
            material=dark_ink if is_major else blue_marking,
            name=f"graduation_{index}",
        )
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=BARREL_OUTER_R, length=BARREL_LENGTH + 0.032),
        mass=0.012,
        origin=Origin(xyz=((BARREL_LENGTH + 0.032) * 0.5, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=PLUNGER_HEAD_R, length=PLUNGER_HEAD_LENGTH),
        origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="rubber_head",
    )
    plunger.visual(
        Cylinder(radius=0.0026, length=0.088),
        origin=Origin(xyz=(-0.034, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=white_plastic,
        name="center_rod",
    )
    plunger.visual(
        Box((0.096, 0.0022, 0.0070)),
        origin=Origin(xyz=(-0.034, 0.0, 0.0)),
        material=white_plastic,
        name="vertical_rib",
    )
    plunger.visual(
        Box((0.096, 0.0070, 0.0022)),
        origin=Origin(xyz=(-0.034, 0.0, 0.0)),
        material=white_plastic,
        name="flat_rib",
    )
    plunger.visual(
        Cylinder(radius=0.015, length=0.006),
        origin=Origin(xyz=(-0.083, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=white_plastic,
        name="thumb_pad",
    )
    plunger.inertial = Inertial.from_geometry(
        Cylinder(radius=0.011, length=0.11),
        mass=0.006,
        origin=Origin(xyz=(-0.030, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "plunger_slide",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        # The child frame starts at the rear barrel opening; positive motion
        # pushes the plunger head deeper into the fixed cylindrical guide.
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=0.25, lower=0.0, upper=PLUNGER_TRAVEL),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("plunger_slide")

    ctx.allow_overlap(
        barrel,
        plunger,
        elem_a="barrel_shell",
        elem_b="rubber_head",
        reason="The rubber piston seal is intentionally modeled as a tiny interference fit against the clear barrel bore.",
    )

    ctx.expect_within(
        plunger,
        barrel,
        axes="yz",
        inner_elem="rubber_head",
        outer_elem="barrel_shell",
        margin=0.0,
        name="rubber head is radially contained by barrel",
    )
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="x",
        elem_a="rubber_head",
        elem_b="barrel_shell",
        min_overlap=PLUNGER_HEAD_LENGTH * 0.90,
        name="rubber head is inserted in barrel at rest",
    )
    ctx.expect_gap(
        barrel,
        plunger,
        axis="x",
        positive_elem="finger_tab_0",
        negative_elem="thumb_pad",
        min_gap=0.010,
        name="thumb pad starts behind the finger flange",
    )

    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: PLUNGER_TRAVEL}):
        ctx.expect_within(
            plunger,
            barrel,
            axes="yz",
            inner_elem="rubber_head",
            outer_elem="barrel_shell",
            margin=0.0,
            name="rubber head stays guided at full stroke",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="x",
            elem_a="rubber_head",
            elem_b="barrel_shell",
            min_overlap=PLUNGER_HEAD_LENGTH * 0.90,
            name="rubber head remains inserted at full stroke",
        )
        ctx.expect_gap(
            barrel,
            plunger,
            axis="x",
            positive_elem="finger_tab_0",
            negative_elem="thumb_pad",
            min_gap=0.010,
            name="thumb pad stops short of the finger flange",
        )
        pushed_pos = ctx.part_world_position(plunger)

    ctx.check(
        "prismatic joint pushes along barrel axis",
        rest_pos is not None and pushed_pos is not None and pushed_pos[0] > rest_pos[0] + 0.050,
        details=f"rest={rest_pos}, pushed={pushed_pos}",
    )

    return ctx.report()


object_model = build_object_model()
