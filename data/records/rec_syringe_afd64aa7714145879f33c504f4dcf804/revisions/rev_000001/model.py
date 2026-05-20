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


def _x_axis_origin(x: float, y: float = 0.0, z: float = 0.0) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def _build_barrel_shell_mesh():
    outer_profile = [
        (0.0118, 0.0000),
        (0.0118, 0.0910),
        (0.0058, 0.0960),
        (0.0032, 0.1020),
        (0.0017, 0.1120),
        (0.0012, 0.1160),
    ]
    inner_profile = [
        (0.0103, 0.0000),
        (0.0103, 0.0900),
        (0.0023, 0.1000),
        (0.0005, 0.1160),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
        "syringe_barrel_shell",
    )


def _build_rear_collar_shell_mesh():
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.0130, 0.0000),
                (0.0130, 0.0060),
            ],
            [
                (0.0038, 0.0000),
                (0.0038, 0.0060),
            ],
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ),
        "syringe_rear_collar",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hand_syringe")

    clear_body = model.material("clear_body", rgba=(0.84, 0.91, 0.98, 0.34))
    plunger_plastic = model.material("plunger_plastic", rgba=(0.93, 0.93, 0.95, 1.0))
    stopper_rubber = model.material("stopper_rubber", rgba=(0.16, 0.17, 0.18, 1.0))

    barrel_body = model.part("barrel_body")
    barrel_body.visual(
        _build_barrel_shell_mesh(),
        origin=_x_axis_origin(0.0),
        material=clear_body,
        name="barrel_shell",
    )
    barrel_body.visual(
        _build_rear_collar_shell_mesh(),
        origin=_x_axis_origin(0.0),
        material=clear_body,
        name="rear_collar",
    )
    barrel_body.visual(
        Box((0.0140, 0.0200, 0.0028)),
        origin=Origin(xyz=(0.0050, 0.0200, -0.0016)),
        material=clear_body,
        name="left_finger_flange",
    )
    barrel_body.visual(
        Box((0.0140, 0.0200, 0.0028)),
        origin=Origin(xyz=(0.0050, -0.0200, -0.0016)),
        material=clear_body,
        name="right_finger_flange",
    )
    barrel_body.visual(
        Box((0.0260, 0.0030, 0.0040)),
        origin=Origin(xyz=(-0.0070, 0.0095, 0.0080)),
        material=clear_body,
        name="left_guide_rail",
    )
    barrel_body.visual(
        Box((0.0260, 0.0030, 0.0040)),
        origin=Origin(xyz=(-0.0070, -0.0095, 0.0080)),
        material=clear_body,
        name="right_guide_rail",
    )
    barrel_body.inertial = Inertial.from_geometry(
        Box((0.1180, 0.0520, 0.0260)),
        mass=0.030,
        origin=Origin(xyz=(0.0480, 0.0, 0.0010)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0095, length=0.0090),
        origin=_x_axis_origin(0.0230),
        material=stopper_rubber,
        name="plunger_head",
    )
    plunger.visual(
        Cylinder(radius=0.0028, length=0.0550),
        origin=_x_axis_origin(-0.0045),
        material=plunger_plastic,
        name="plunger_rod",
    )
    plunger.visual(
        Box((0.0140, 0.0130, 0.0045)),
        origin=Origin(xyz=(-0.0060, 0.0, 0.0082)),
        material=plunger_plastic,
        name="guide_carriage",
    )
    plunger.visual(
        Box((0.0060, 0.0040, 0.0105)),
        origin=Origin(xyz=(-0.0060, 0.0, 0.0047)),
        material=plunger_plastic,
        name="carriage_stem",
    )
    plunger.visual(
        Cylinder(radius=0.0060, length=0.0140),
        origin=_x_axis_origin(-0.0385),
        material=plunger_plastic,
        name="thumb_boss",
    )
    plunger.visual(
        Cylinder(radius=0.0160, length=0.0030),
        origin=_x_axis_origin(-0.0460),
        material=plunger_plastic,
        name="thumb_pad",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.0820, 0.0320, 0.0200)),
        mass=0.012,
        origin=Origin(xyz=(-0.0120, 0.0, 0.0030)),
    )

    model.articulation(
        "plunger_slide",
        ArticulationType.PRISMATIC,
        parent=barrel_body,
        child=plunger,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.20,
            lower=0.0,
            upper=0.064,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel_body = object_model.get_part("barrel_body")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("plunger_slide")

    ctx.expect_within(
        plunger,
        barrel_body,
        axes="yz",
        inner_elem="plunger_head",
        outer_elem="barrel_shell",
        margin=0.0008,
        name="plunger head stays centered inside barrel bore at rest",
    )
    ctx.expect_overlap(
        plunger,
        barrel_body,
        axes="x",
        elem_a="plunger_head",
        elem_b="barrel_shell",
        min_overlap=0.008,
        name="withdrawn plunger head still sits inside the barrel",
    )

    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: 0.064}):
        ctx.expect_within(
            plunger,
            barrel_body,
            axes="yz",
            inner_elem="plunger_head",
            outer_elem="barrel_shell",
            margin=0.0008,
            name="pressed plunger head stays centered inside barrel bore",
        )
        ctx.expect_overlap(
            plunger,
            barrel_body,
            axes="x",
            elem_a="plunger_head",
            elem_b="barrel_shell",
            min_overlap=0.008,
            name="pressed plunger head remains inside the barrel shell",
        )
        pressed_pos = ctx.part_world_position(plunger)

    ctx.check(
        "plunger advances toward nozzle",
        rest_pos is not None and pressed_pos is not None and pressed_pos[0] > rest_pos[0] + 0.05,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
