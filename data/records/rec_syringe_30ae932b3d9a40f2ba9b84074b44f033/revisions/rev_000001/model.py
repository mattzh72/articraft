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

    clear_plastic = model.material("clear_plastic", rgba=(0.82, 0.88, 0.96, 0.35))
    plunger_plastic = model.material("plunger_plastic", rgba=(0.92, 0.95, 0.98, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.12, 0.12, 0.13, 1.0))

    body = model.part("body")

    barrel_shell = LatheGeometry.from_shell_profiles(
        [
            (0.0018, -0.006),
            (0.0026, 0.001),
            (0.0045, 0.010),
            (0.0062, 0.017),
            (0.0108, 0.021),
            (0.0108, 0.116),
            (0.0125, 0.120),
        ],
        [
            (0.0008, -0.002),
            (0.0014, 0.005),
            (0.0036, 0.014),
            (0.00835, 0.021),
            (0.00835, 0.116),
            (0.0066, 0.119),
            (0.0046, 0.120),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    body.visual(
        mesh_from_geometry(barrel_shell, "syringe_barrel_shell"),
        material=clear_plastic,
        name="barrel_shell",
    )
    body.visual(
        Box((0.026, 0.007, 0.003)),
        origin=Origin(xyz=(0.0, 0.0145, 0.119)),
        material=clear_plastic,
        name="finger_flange_left",
    )
    body.visual(
        Box((0.026, 0.007, 0.003)),
        origin=Origin(xyz=(0.0, -0.0145, 0.119)),
        material=clear_plastic,
        name="finger_flange_right",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.052, 0.052, 0.128)),
        mass=0.030,
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0076, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.088)),
        material=rubber_black,
        name="plunger_head",
    )
    plunger.visual(
        Cylinder(radius=0.00818, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, -0.0908)),
        material=rubber_black,
        name="seal_rib_front",
    )
    plunger.visual(
        Cylinder(radius=0.00818, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, -0.0852)),
        material=rubber_black,
        name="seal_rib_rear",
    )
    plunger.visual(
        Cylinder(radius=0.0022, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, -0.029)),
        material=plunger_plastic,
        name="plunger_rod",
    )
    plunger.visual(
        Cylinder(radius=0.0108, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=plunger_plastic,
        name="stop_collar",
    )
    plunger.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=plunger_plastic,
        name="thumb_collar",
    )
    plunger.visual(
        Cylinder(radius=0.014, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0315)),
        material=plunger_plastic,
        name="thumb_pad",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.030, 0.030, 0.126)),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, -0.027)),
    )

    model.articulation(
        "body_to_plunger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.20,
            lower=0.0,
            upper=0.078,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("body_to_plunger")

    ctx.expect_contact(
        plunger,
        body,
        elem_a="stop_collar",
        elem_b="barrel_shell",
        name="plunger stop collar seats against the rear barrel ring at rest",
    )
    ctx.expect_origin_distance(
        body,
        plunger,
        axes="xy",
        max_dist=1e-6,
        name="plunger origin stays centered on barrel axis at rest",
    )
    ctx.expect_within(
        plunger,
        body,
        axes="xy",
        inner_elem="plunger_head",
        outer_elem="barrel_shell",
        margin=0.0006,
        name="plunger head is centered within barrel at rest",
    )
    ctx.expect_overlap(
        plunger,
        body,
        axes="z",
        elem_a="plunger_head",
        elem_b="barrel_shell",
        min_overlap=0.007,
        name="plunger head remains inserted at rest",
    )

    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_origin_distance(
            body,
            plunger,
            axes="xy",
            max_dist=1e-6,
            name="plunger origin stays centered on barrel axis when retracted",
        )
        ctx.expect_within(
            plunger,
            body,
            axes="xy",
            inner_elem="plunger_head",
            outer_elem="barrel_shell",
            margin=0.0006,
            name="plunger head stays centered within barrel when retracted",
        )
        ctx.expect_overlap(
            plunger,
            body,
            axes="z",
            elem_a="plunger_head",
            elem_b="barrel_shell",
            min_overlap=0.007,
            name="plunger head retains insertion at max travel",
        )
        extended_pos = ctx.part_world_position(plunger)

    ctx.check(
        "plunger translates along barrel axis",
        rest_pos is not None
        and extended_pos is not None
        and abs(extended_pos[0] - rest_pos[0]) < 1e-6
        and abs(extended_pos[1] - rest_pos[1]) < 1e-6
        and extended_pos[2] > rest_pos[2] + 0.06,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
