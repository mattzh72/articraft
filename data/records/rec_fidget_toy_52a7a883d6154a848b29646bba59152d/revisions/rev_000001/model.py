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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spinning_top_fidget")

    body_metal = model.material("body_metal", rgba=(0.23, 0.24, 0.26, 1.0))
    tip_metal = model.material("tip_metal", rgba=(0.78, 0.68, 0.38, 1.0))

    tip = model.part("tip")
    tip.visual(
        Sphere(radius=0.0014),
        origin=Origin(xyz=(0.0, 0.0, 0.0014)),
        material=tip_metal,
        name="tip_contact",
    )
    tip.visual(
        Cylinder(radius=0.0025, length=0.0042),
        origin=Origin(xyz=(0.0, 0.0, 0.0041)),
        material=tip_metal,
        name="tip_shaft",
    )
    tip.visual(
        Cylinder(radius=0.0034, length=0.0014),
        origin=Origin(xyz=(0.0, 0.0, 0.0055)),
        material=tip_metal,
        name="tip_flange",
    )
    tip.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0034, length=0.0062),
        mass=0.01,
        origin=Origin(xyz=(0.0, 0.0, 0.0031)),
    )

    spinner = model.part("spinner")

    body_profile = [
        (0.0, 0.0062),
        (0.0038, 0.0062),
        (0.0075, 0.0068),
        (0.0150, 0.0094),
        (0.0240, 0.0125),
        (0.0310, 0.0144),
        (0.0325, 0.0151),
        (0.0325, 0.0177),
        (0.0240, 0.0180),
        (0.0100, 0.0183),
        (0.0, 0.0183),
        (0.0, 0.0062),
    ]
    body_mesh = mesh_from_geometry(LatheGeometry(body_profile, segments=72), "body_shell")
    spinner.visual(body_mesh, material=body_metal, name="body_shell")
    spinner.visual(
        Cylinder(radius=0.0039, length=0.0018),
        origin=Origin(xyz=(0.0, 0.0, 0.0071)),
        material=body_metal,
        name="hub_collar",
    )
    spinner.visual(
        Cylinder(radius=0.0054, length=0.0115),
        origin=Origin(xyz=(0.0, 0.0, 0.02375)),
        material=body_metal,
        name="grip_core",
    )
    for index, z_center in enumerate((0.0208, 0.0237, 0.0266, 0.0295)):
        spinner.visual(
            Cylinder(radius=0.0072, length=0.0016),
            origin=Origin(xyz=(0.0, 0.0, z_center)),
            material=body_metal,
            name=f"grip_rib_{index}",
        )
    spinner.visual(
        Sphere(radius=0.0056),
        origin=Origin(xyz=(0.0, 0.0, 0.0310)),
        material=body_metal,
        name="grip_cap",
    )
    spinner.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0325, length=0.0350),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
    )

    model.articulation(
        "tip_to_spinner",
        ArticulationType.CONTINUOUS,
        parent=tip,
        child=spinner,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=40.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tip = object_model.get_part("tip")
    spinner = object_model.get_part("spinner")
    spin = object_model.get_articulation("tip_to_spinner")

    ctx.check(
        "spinner uses continuous vertical spin",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(spin.axis) == (0.0, 0.0, 1.0)
        and spin.motion_limits is not None
        and spin.motion_limits.lower is None
        and spin.motion_limits.upper is None,
        details=(
            f"type={spin.articulation_type}, axis={spin.axis}, "
            f"limits={spin.motion_limits}"
        ),
    )

    ctx.expect_contact(
        spinner,
        tip,
        elem_a="hub_collar",
        elem_b="tip_shaft",
        contact_tol=1e-6,
        name="spinner seats on the blunt tip insert",
    )

    body_aabb = ctx.part_element_world_aabb(spinner, elem="body_shell")
    grip_aabb = ctx.part_element_world_aabb(spinner, elem="grip_core")
    if body_aabb is not None:
        body_size = tuple(body_aabb[1][axis] - body_aabb[0][axis] for axis in range(3))
        ctx.check(
            "body reads as a wide flat disc",
            body_size[0] > 0.060 and body_size[1] > 0.060 and body_size[2] < 0.013,
            details=f"body_size={body_size}",
        )
    else:
        ctx.fail("body shell measurable", "Missing body_shell world AABB")

    if body_aabb is not None and grip_aabb is not None:
        ctx.check(
            "finger grip rises above the disc",
            grip_aabb[0][2] < body_aabb[1][2] and grip_aabb[1][2] > body_aabb[1][2] + 0.010,
            details=f"body_aabb={body_aabb}, grip_aabb={grip_aabb}",
        )
    else:
        ctx.fail("grip measurable", f"body_aabb={body_aabb}, grip_aabb={grip_aabb}")

    rest_pos = ctx.part_world_position(spinner)
    with ctx.pose({spin: math.pi / 2.0}):
        quarter_turn_pos = ctx.part_world_position(spinner)
        ctx.expect_contact(
            spinner,
            tip,
            elem_a="hub_collar",
            elem_b="tip_shaft",
            contact_tol=1e-6,
            name="spinner stays seated while turning",
        )

    ctx.check(
        "spin stays centered on the contact point",
        rest_pos is not None
        and quarter_turn_pos is not None
        and abs(rest_pos[0] - quarter_turn_pos[0]) < 1e-9
        and abs(rest_pos[1] - quarter_turn_pos[1]) < 1e-9
        and abs(rest_pos[2] - quarter_turn_pos[2]) < 1e-9,
        details=f"rest={rest_pos}, quarter_turn={quarter_turn_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
