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
    model = ArticulatedObject(name="screw_cap_bottle")

    clear_pet = model.material("clear_pet", rgba=(0.83, 0.92, 0.98, 0.35))
    cap_blue = model.material("cap_blue", rgba=(0.16, 0.34, 0.78, 1.0))
    label_white = model.material("label_white", rgba=(0.96, 0.97, 0.98, 1.0))
    label_teal = model.material("label_teal", rgba=(0.18, 0.63, 0.63, 1.0))

    bottle = model.part("bottle")

    bottle_outer = [
        (0.006, 0.000),
        (0.016, 0.002),
        (0.030, 0.010),
        (0.034, 0.050),
        (0.034, 0.135),
        (0.032, 0.165),
        (0.026, 0.186),
        (0.019, 0.198),
        (0.015, 0.206),
        (0.0145, 0.222),
        (0.0138, 0.229),
    ]
    bottle_inner = [
        (0.000, 0.007),
        (0.014, 0.010),
        (0.029, 0.017),
        (0.0305, 0.050),
        (0.0305, 0.135),
        (0.0285, 0.164),
        (0.0235, 0.184),
        (0.0170, 0.196),
        (0.0120, 0.204),
        (0.0110, 0.219),
        (0.0105, 0.224),
    ]
    bottle.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                bottle_outer,
                bottle_inner,
                segments=88,
                start_cap="flat",
                end_cap="flat",
            ),
            "bottle_shell",
        ),
        material=clear_pet,
        name="bottle_shell",
    )
    bottle.visual(
        Cylinder(radius=0.0148, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.214)),
        material=clear_pet,
        name="neck_finish",
    )
    for index, z_pos in enumerate((0.208, 0.2125, 0.217), start=1):
        bottle.visual(
            Cylinder(radius=0.0152, length=0.0018),
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material=clear_pet,
            name=f"thread_ring_{index}",
        )
    bottle.visual(
        Cylinder(radius=0.0345, length=0.068),
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
        material=label_white,
        name="label_sleeve",
    )
    bottle.visual(
        Box((0.046, 0.0012, 0.050)),
        origin=Origin(xyz=(0.0, 0.0345, 0.112)),
        material=label_teal,
        name="label_stripe",
    )
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.035, length=0.229),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, 0.1145)),
    )

    cap = model.part("cap")
    cap_outer = [
        (0.0176, -0.0015),
        (0.0188, 0.0015),
        (0.0188, 0.0255),
        (0.0178, 0.0290),
        (0.0080, 0.0310),
        (0.0000, 0.0310),
    ]
    cap_inner = [
        (0.0176, -0.0015),
        (0.0152, 0.0018),
        (0.0152, 0.0240),
        (0.0070, 0.0250),
        (0.0000, 0.0250),
    ]
    cap.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                cap_outer,
                cap_inner,
                segments=88,
                start_cap="flat",
                end_cap="flat",
            ),
            "cap_shell",
        ),
        material=cap_blue,
        name="cap_shell",
    )
    for rib_index in range(16):
        angle = (2.0 * math.pi * rib_index) / 16.0
        cap.visual(
            Box((0.0020, 0.0017, 0.020)),
            origin=Origin(
                xyz=(0.01915 * math.cos(angle), 0.01915 * math.sin(angle), 0.0135),
                rpy=(0.0, 0.0, angle),
            ),
            material=cap_blue,
            name=f"grip_rib_{rib_index:02d}",
        )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.019, length=0.031),
        mass=0.006,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    model.articulation(
        "cap_spin",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.206)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottle = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    cap_spin = object_model.get_articulation("cap_spin")

    ctx.expect_within(
        bottle,
        cap,
        axes="xy",
        inner_elem="neck_finish",
        outer_elem="cap_shell",
        margin=0.0,
        name="neck finish stays inside cap skirt footprint",
    )
    ctx.expect_overlap(
        bottle,
        cap,
        axes="xy",
        elem_a="neck_finish",
        elem_b="cap_shell",
        min_overlap=0.028,
        name="cap remains centered over bottle neck",
    )
    ctx.expect_origin_gap(
        cap,
        bottle,
        axis="z",
        min_gap=0.19,
        max_gap=0.215,
        name="cap articulation sits at the bottle top",
    )

    limits = cap_spin.motion_limits
    ctx.check(
        "cap joint is continuous without angular stops",
        limits is not None and limits.lower is None and limits.upper is None,
        details=f"motion_limits={limits}",
    )

    rest_pos = ctx.part_world_position(cap)
    with ctx.pose({cap_spin: math.pi / 2.0}):
        spun_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap spins in place around the neck axis",
        rest_pos is not None
        and spun_pos is not None
        and abs(spun_pos[0] - rest_pos[0]) <= 1e-6
        and abs(spun_pos[1] - rest_pos[1]) <= 1e-6
        and abs(spun_pos[2] - rest_pos[2]) <= 1e-6,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
