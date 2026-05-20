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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hand_syringe")

    clear_poly = model.material("clear_poly", rgba=(0.86, 0.91, 0.98, 0.42))
    translucent_flange = model.material("translucent_flange", rgba=(0.80, 0.88, 0.98, 0.72))
    plunger_white = model.material("plunger_white", rgba=(0.97, 0.97, 0.97, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.12, 0.12, 0.13, 1.0))

    barrel_outer_profile = [
        (0.0009, 0.000),
        (0.0015, 0.008),
        (0.0030, 0.016),
        (0.0048, 0.022),
        (0.0087, 0.030),
        (0.0089, 0.100),
        (0.0104, 0.106),
        (0.0104, 0.110),
    ]
    barrel_inner_profile = [
        (0.0004, 0.000),
        (0.0009, 0.008),
        (0.0018, 0.016),
        (0.0032, 0.022),
        (0.0072, 0.030),
        (0.0074, 0.100),
        (0.0087, 0.106),
        (0.0087, 0.110),
    ]
    barrel_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            barrel_outer_profile,
            barrel_inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=10,
        ),
        "syringe_barrel_shell",
    )

    barrel = model.part("barrel")
    barrel.visual(
        barrel_shell,
        material=clear_poly,
        name="barrel_shell",
    )
    barrel.visual(
        Box((0.030, 0.009, 0.004)),
        origin=Origin(xyz=(0.024, 0.0, 0.106)),
        material=translucent_flange,
        name="finger_flange_right",
    )
    barrel.visual(
        Cylinder(radius=0.0045, length=0.009),
        origin=Origin(xyz=(0.039, 0.0, 0.106), rpy=(1.57079632679, 0.0, 0.0)),
        material=translucent_flange,
        name="finger_flange_right_tip",
    )
    barrel.visual(
        Box((0.030, 0.009, 0.004)),
        origin=Origin(xyz=(-0.024, 0.0, 0.106)),
        material=translucent_flange,
        name="finger_flange_left",
    )
    barrel.visual(
        Cylinder(radius=0.0045, length=0.009),
        origin=Origin(xyz=(-0.039, 0.0, 0.106), rpy=(1.57079632679, 0.0, 0.0)),
        material=translucent_flange,
        name="finger_flange_left_tip",
    )
    barrel.inertial = Inertial.from_geometry(
        Box((0.082, 0.022, 0.110)),
        mass=0.030,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0145, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=plunger_white,
        name="thumb_pad",
    )
    plunger.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=plunger_white,
        name="thumb_hub",
    )
    plunger.visual(
        Cylinder(radius=0.0029, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=plunger_white,
        name="plunger_rod",
    )
    plunger.visual(
        Cylinder(radius=0.0098, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, -0.0245)),
        material=plunger_white,
        name="retainer_stop",
    )
    plunger.visual(
        Cylinder(radius=0.0069, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=gasket_black,
        name="piston_head",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.029, 0.029, 0.112)),
        mass=0.010,
        origin=Origin(xyz=(0.0, 0.0, -0.038)),
    )

    model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.136)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.10,
            lower=0.0,
            upper=0.032,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("barrel_to_plunger")

    ctx.expect_within(
        plunger,
        barrel,
        axes="xy",
        inner_elem="piston_head",
        outer_elem="barrel_shell",
        margin=0.0015,
        name="piston head stays centered in the barrel",
    )
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="z",
        elem_a="plunger_rod",
        elem_b="barrel_shell",
        min_overlap=0.060,
        name="retracted plunger keeps generous rod overlap in the barrel",
    )

    rest_pos = ctx.part_world_position(plunger)
    upper = slide.motion_limits.upper if slide.motion_limits is not None else None
    with ctx.pose({slide: upper if upper is not None else 0.032}):
        ctx.expect_within(
            plunger,
            barrel,
            axes="xy",
            inner_elem="piston_head",
            outer_elem="barrel_shell",
            margin=0.0015,
            name="piston head stays centered when fully pressed",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="z",
            elem_a="plunger_rod",
            elem_b="barrel_shell",
            min_overlap=0.085,
            name="pressed plunger still retains a long sliding engagement",
        )
        pressed_pos = ctx.part_world_position(plunger)

    ctx.check(
        "plunger motion drives toward the nozzle",
        rest_pos is not None
        and pressed_pos is not None
        and pressed_pos[2] < rest_pos[2] - 0.020,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
