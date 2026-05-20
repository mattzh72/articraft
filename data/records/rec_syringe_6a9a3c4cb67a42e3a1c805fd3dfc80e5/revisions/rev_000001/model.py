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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hand_syringe")

    clear_barrel = model.material("clear_barrel", rgba=(0.80, 0.88, 0.98, 0.34))
    white_plastic = model.material("white_plastic", rgba=(0.95, 0.95, 0.96, 1.0))
    gray_rubber = model.material("gray_rubber", rgba=(0.27, 0.29, 0.31, 1.0))
    translucent_blue = model.material("translucent_blue", rgba=(0.46, 0.64, 0.90, 0.45))

    barrel = model.part("barrel")

    barrel_shell = LatheGeometry.from_shell_profiles(
        [
            (0.0140, -0.0560),
            (0.0139, -0.0300),
            (0.0138, 0.0180),
            (0.0134, 0.0440),
            (0.0108, 0.0560),
            (0.0065, 0.0660),
            (0.0038, 0.0750),
            (0.0020, 0.0830),
        ],
        [
            (0.0118, -0.0560),
            (0.0118, -0.0300),
            (0.0117, 0.0180),
            (0.0114, 0.0420),
            (0.0088, 0.0540),
            (0.0040, 0.0640),
            (0.0010, 0.0715),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(math.pi / 2.0)
    barrel.visual(
        mesh_from_geometry(barrel_shell, "barrel_shell"),
        material=clear_barrel,
        name="barrel_shell",
    )
    barrel.visual(
        Box((0.0180, 0.0200, 0.0032)),
        origin=Origin(xyz=(-0.0490, 0.0240, -0.0020)),
        material=translucent_blue,
        name="finger_flange_left",
    )
    barrel.visual(
        Box((0.0180, 0.0200, 0.0032)),
        origin=Origin(xyz=(-0.0490, -0.0240, -0.0020)),
        material=translucent_blue,
        name="finger_flange_right",
    )
    barrel.visual(
        Box((0.0220, 0.0050, 0.0140)),
        origin=Origin(xyz=(-0.0580, 0.0148, 0.0)),
        material=translucent_blue,
        name="guide_cheek_left",
    )
    barrel.visual(
        Box((0.0220, 0.0050, 0.0140)),
        origin=Origin(xyz=(-0.0580, -0.0148, 0.0)),
        material=translucent_blue,
        name="guide_cheek_right",
    )
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0170, length=0.1400),
        mass=0.032,
        origin=Origin(xyz=(0.0120, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0036, length=0.0760),
        origin=Origin(xyz=(-0.0350, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white_plastic,
        name="plunger_rod",
    )
    plunger.visual(
        Cylinder(radius=0.0053, length=0.0120),
        origin=Origin(xyz=(0.0050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white_plastic,
        name="rod_boss",
    )
    plunger.visual(
        Cylinder(radius=0.0115, length=0.0110),
        origin=Origin(xyz=(0.0135, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gray_rubber,
        name="plunger_head",
    )
    plunger.visual(
        Box((0.0100, 0.0246, 0.0100)),
        origin=Origin(xyz=(-0.0070, 0.0, 0.0)),
        material=white_plastic,
        name="guide_slider",
    )
    plunger.visual(
        Box((0.0080, 0.0300, 0.0240)),
        origin=Origin(xyz=(-0.0800, 0.0, 0.0)),
        material=white_plastic,
        name="thumb_pad",
    )
    plunger.visual(
        Cylinder(radius=0.0075, length=0.0100),
        origin=Origin(xyz=(-0.0710, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white_plastic,
        name="thumb_hub",
    )
    plunger.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0140, length=0.1060),
        mass=0.010,
        origin=Origin(xyz=(-0.0260, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "plunger_slide",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(-0.0560, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.20,
            lower=0.0,
            upper=0.0800,
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
        inner_elem="plunger_head",
        outer_elem="barrel_shell",
        margin=0.0015,
        name="plunger head is centered inside the barrel at rest",
    )
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="x",
        elem_a="plunger_head",
        elem_b="barrel_shell",
        min_overlap=0.010,
        name="plunger head remains inserted into the barrel at rest",
    )

    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_within(
            plunger,
            barrel,
            axes="yz",
            inner_elem="plunger_head",
            outer_elem="barrel_shell",
            margin=0.0015,
            name="plunger head stays centered inside the barrel when fully depressed",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="x",
            elem_a="plunger_head",
            elem_b="barrel_shell",
            min_overlap=0.008,
            name="plunger head still overlaps the barrel length when fully depressed",
        )
        pushed_pos = ctx.part_world_position(plunger)

    ctx.check(
        "positive prismatic motion drives the plunger toward the nozzle",
        rest_pos is not None
        and pushed_pos is not None
        and pushed_pos[0] > rest_pos[0] + 0.07,
        details=f"rest={rest_pos}, pushed={pushed_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
