from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, *, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _annular_plate(
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    *,
    z_center: float,
    segments: int = 48,
):
    return ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments=segments),
        [_circle_profile(inner_radius, segments=segments)],
        thickness,
        center=True,
    ).translate(0.0, 0.0, z_center)


def _build_bottle_shell():
    outer_profile = [
        (0.004, 0.000),
        (0.020, 0.002),
        (0.031, 0.006),
        (0.034, 0.015),
        (0.034, 0.145),
        (0.032, 0.173),
        (0.027, 0.196),
        (0.022, 0.210),
        (0.0180, 0.215),
        (0.0161, 0.218),
        (0.0172, 0.221),
        (0.0161, 0.224),
        (0.0155, 0.228),
        (0.0150, 0.238),
        (0.0153, 0.242),
        (0.0148, 0.245),
    ]
    inner_profile = [
        (0.000, 0.004),
        (0.028, 0.012),
        (0.030, 0.145),
        (0.028, 0.173),
        (0.023, 0.196),
        (0.018, 0.210),
        (0.0130, 0.220),
        (0.0106, 0.239),
        (0.0103, 0.243),
    ]
    shell = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    return shell


def _build_support_sleeve():
    support = CylinderGeometry(radius=0.0188, height=0.018, radial_segments=56, closed=False).translate(0.0, 0.0, 0.009)
    support.merge(
        CylinderGeometry(radius=0.0170, height=0.014, radial_segments=56, closed=False).translate(0.0, 0.0, 0.011)
    )
    support.merge(_annular_plate(0.0188, 0.0155, 0.004, z_center=0.002, segments=56))
    return support


def _build_cap_shell():
    cap = CylinderGeometry(radius=0.0227, height=0.028, radial_segments=64, closed=False).translate(0.0, 0.0, 0.012)
    cap.merge(CylinderGeometry(radius=0.0227, height=0.005, radial_segments=64).translate(0.0, 0.0, 0.0235))
    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_cap_bottle")

    bottle_pet = model.material("bottle_pet", rgba=(0.77, 0.89, 0.96, 0.42))
    cap_plastic = model.material("cap_plastic", rgba=(0.16, 0.34, 0.85, 1.0))
    sleeve_plastic = model.material("sleeve_plastic", rgba=(0.84, 0.86, 0.90, 1.0))

    bottle_body = model.part("bottle_body")
    bottle_body.visual(
        mesh_from_geometry(_build_bottle_shell(), "bottle_shell"),
        material=bottle_pet,
        name="bottle_shell",
    )
    bottle_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.034, length=0.245),
        mass=0.055,
        origin=Origin(xyz=(0.0, 0.0, 0.1225)),
    )

    support_sleeve = model.part("support_sleeve")
    support_sleeve.visual(
        mesh_from_geometry(_build_support_sleeve(), "support_sleeve"),
        material=sleeve_plastic,
        name="support_sleeve",
    )
    support_sleeve.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0188, length=0.018),
        mass=0.006,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )

    cap = model.part("cap")
    cap.visual(
        mesh_from_geometry(_build_cap_shell(), "bottle_cap"),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=cap_plastic,
        name="cap_shell",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0227, length=0.028),
        mass=0.010,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    model.articulation(
        "body_to_support",
        ArticulationType.FIXED,
        parent=bottle_body,
        child=support_sleeve,
        origin=Origin(xyz=(0.0, 0.0, 0.224)),
    )
    model.articulation(
        "cap_spin",
        ArticulationType.CONTINUOUS,
        parent=support_sleeve,
        child=cap,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=18.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle_body = object_model.get_part("bottle_body")
    support_sleeve = object_model.get_part("support_sleeve")
    cap = object_model.get_part("cap")
    cap_spin = object_model.get_articulation("cap_spin")

    ctx.expect_contact(
        support_sleeve,
        bottle_body,
        elem_a="support_sleeve",
        elem_b="bottle_shell",
        name="support sleeve seats on the bottle neck",
    )
    ctx.expect_within(
        support_sleeve,
        cap,
        axes="xy",
        margin=0.0015,
        name="support sleeve remains inside cap footprint",
    )
    ctx.expect_overlap(
        cap,
        bottle_body,
        axes="xy",
        min_overlap=0.028,
        name="cap stays centered over the bottle neck",
    )

    rest_pos = ctx.part_world_position(cap)
    with ctx.pose({cap_spin: 1.8}):
        spun_pos = ctx.part_world_position(cap)
        ctx.expect_within(
            support_sleeve,
            cap,
            axes="xy",
            margin=0.0015,
            name="support sleeve remains inside cap while spun",
        )
    ctx.check(
        "cap rotates in place about the neck axis",
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
