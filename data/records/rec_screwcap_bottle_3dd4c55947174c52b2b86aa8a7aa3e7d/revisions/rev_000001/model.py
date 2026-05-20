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

    clear_pet = model.material("clear_pet", rgba=(0.82, 0.91, 0.98, 0.34))
    cap_blue = model.material("cap_blue", rgba=(0.18, 0.42, 0.78, 1.0))
    neck_plastic = model.material("neck_plastic", rgba=(0.78, 0.88, 0.97, 0.48))

    bottle_outer = [
        (0.004, 0.000),
        (0.012, 0.002),
        (0.029, 0.006),
        (0.0335, 0.020),
        (0.0340, 0.078),
        (0.0315, 0.130),
        (0.0345, 0.164),
        (0.0310, 0.183),
        (0.0240, 0.194),
        (0.0190, 0.1995),
        (0.0170, 0.2015),
    ]
    bottle_inner = [
        (0.000, 0.002),
        (0.010, 0.004),
        (0.027, 0.007),
        (0.0310, 0.020),
        (0.0310, 0.078),
        (0.0288, 0.130),
        (0.0316, 0.164),
        (0.0286, 0.183),
        (0.0214, 0.194),
        (0.0168, 0.1995),
        (0.0152, 0.2005),
    ]
    bottle_geom = LatheGeometry(
        bottle_outer + list(reversed(bottle_inner)),
        segments=72,
    )
    bottle_shell_mesh = mesh_from_geometry(bottle_geom, "bottle_shell")
    neck_outer = [
        (0.0160, 0.2006),
        (0.0164, 0.2034),
        (0.0168, 0.2062),
        (0.0168, 0.2100),
        (0.0152, 0.2140),
        (0.0159, 0.2161),
        (0.0154, 0.2181),
        (0.0159, 0.2201),
        (0.0154, 0.2221),
        (0.0157, 0.2242),
    ]
    neck_inner = [
        (0.0142, 0.2010),
        (0.0142, 0.2060),
        (0.0141, 0.2120),
        (0.0140, 0.2180),
        (0.0140, 0.2240),
    ]
    neck_geom = LatheGeometry(
        neck_outer + list(reversed(neck_inner)),
        segments=72,
    )
    neck_finish_mesh = mesh_from_geometry(neck_geom, "neck_finish")

    cap_outer = [
        (0.0000, 0.0119),
        (0.0055, 0.0120),
        (0.0130, 0.0119),
        (0.0168, 0.0112),
        (0.0182, 0.0094),
        (0.0186, 0.0060),
        (0.0187, 0.0010),
        (0.0187, -0.0055),
        (0.0185, -0.0088),
        (0.0182, -0.0110),
    ]
    cap_inner = [
        (0.0000, 0.0094),
        (0.0055, 0.0095),
        (0.0128, 0.0094),
        (0.0162, 0.0088),
        (0.0171, 0.0065),
        (0.0173, 0.0010),
        (0.0173, -0.0055),
        (0.0171, -0.0088),
        (0.0169, -0.0110),
    ]
    cap_geom = LatheGeometry(
        cap_outer + list(reversed(cap_inner)),
        segments=72,
    )
    cap_shell_mesh = mesh_from_geometry(
        cap_geom,
        "cap_shell",
    )

    bottle_body = model.part("bottle_body")
    bottle_body.visual(bottle_shell_mesh, material=clear_pet, name="bottle_shell")
    bottle_body.visual(
        neck_finish_mesh,
        material=neck_plastic,
        name="neck_finish",
    )
    bottle_body.visual(
        Box((0.0060, 0.0070, 0.0054)),
        origin=Origin(xyz=(0.0178, 0.0, 0.2048)),
        material=neck_plastic,
        name="support_flange_right",
    )
    bottle_body.visual(
        Box((0.0060, 0.0070, 0.0054)),
        origin=Origin(xyz=(-0.0178, 0.0, 0.2048)),
        material=neck_plastic,
        name="support_flange_left",
    )
    bottle_body.inertial = Inertial.from_geometry(
        Box((0.070, 0.070, 0.226)),
        mass=0.040,
        origin=Origin(xyz=(0.0, 0.0, 0.113)),
    )

    cap = model.part("cap")
    cap.visual(cap_shell_mesh, material=cap_blue, name="cap_shell")
    rib_radius = 0.01855
    for rib_index in range(18):
        angle = math.tau * rib_index / 18.0
        cap.visual(
            Box((0.0016, 0.0030, 0.0155)),
            origin=Origin(
                xyz=(
                    rib_radius * math.cos(angle),
                    rib_radius * math.sin(angle),
                    -0.0004,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=cap_blue,
            name=f"grip_rib_{rib_index:02d}",
        )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0185, length=0.024),
        mass=0.004,
    )

    model.articulation(
        "cap_spin",
        ArticulationType.CONTINUOUS,
        parent=bottle_body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.2185)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=14.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle_body = object_model.get_part("bottle_body")
    cap = object_model.get_part("cap")
    cap_spin = object_model.get_articulation("cap_spin")

    ctx.expect_within(
        bottle_body,
        cap,
        axes="xy",
        inner_elem="neck_finish",
        outer_elem="cap_shell",
        margin=0.0005,
        name="cap shell envelops the threaded neck",
    )
    ctx.expect_contact(
        cap,
        bottle_body,
        elem_a="cap_shell",
        elem_b="support_flange_left",
        name="cap is retained by the left support flange",
    )
    ctx.expect_contact(
        cap,
        bottle_body,
        elem_a="cap_shell",
        elem_b="support_flange_right",
        name="cap is retained by the right support flange",
    )
    ctx.expect_gap(
        cap,
        bottle_body,
        axis="z",
        positive_elem="cap_shell",
        negative_elem="support_flange_left",
        max_gap=0.0015,
        max_penetration=1e-5,
        name="cap clears the left support flange",
    )
    ctx.expect_gap(
        cap,
        bottle_body,
        axis="z",
        positive_elem="cap_shell",
        negative_elem="support_flange_right",
        max_gap=0.0015,
        max_penetration=1e-5,
        name="cap clears the right support flange",
    )

    rest_pos = ctx.part_world_position(cap)
    with ctx.pose({cap_spin: math.pi}):
        ctx.expect_within(
            bottle_body,
            cap,
            axes="xy",
            inner_elem="neck_finish",
            outer_elem="cap_shell",
            margin=0.0005,
            name="cap remains centered on the neck after rotation",
        )
        spun_pos = ctx.part_world_position(cap)

    ctx.check(
        "cap rotates in place about the bottle neck axis",
        rest_pos is not None
        and spun_pos is not None
        and abs(rest_pos[0] - spun_pos[0]) < 1e-6
        and abs(rest_pos[1] - spun_pos[1]) < 1e-6
        and abs(rest_pos[2] - spun_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
