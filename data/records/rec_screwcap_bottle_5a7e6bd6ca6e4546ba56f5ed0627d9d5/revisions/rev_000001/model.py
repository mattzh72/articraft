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
    CylinderGeometry,
    DomeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)


def _build_bottle_shell():
    outer_profile = [
        (0.006, 0.000),
        (0.020, 0.003),
        (0.031, 0.010),
        (0.033, 0.030),
        (0.033, 0.148),
        (0.032, 0.168),
        (0.028, 0.184),
        (0.022, 0.195),
        (0.018, 0.200),
        (0.0170, 0.202),
        (0.0154, 0.205),
        (0.0168, 0.207),
        (0.0155, 0.209),
        (0.0169, 0.211),
        (0.0156, 0.213),
        (0.0168, 0.215),
        (0.0154, 0.217),
        (0.0151, 0.221),
        (0.0152, 0.224),
    ]
    inner_profile = [
        (0.000, 0.004),
        (0.022, 0.008),
        (0.0310, 0.018),
        (0.0312, 0.150),
        (0.0295, 0.171),
        (0.0215, 0.190),
        (0.0155, 0.198),
        (0.0118, 0.206),
        (0.0108, 0.214),
        (0.0105, 0.221),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _build_cap_shell():
    cap_outer_radius = 0.0194
    cap_inner_radius = 0.0176
    cap_height = 0.022
    top_thickness = 0.003

    skirt = boolean_difference(
        CylinderGeometry(radius=cap_outer_radius, height=cap_height, radial_segments=56).translate(
            0.0,
            0.0,
            cap_height * 0.5,
        ),
        CylinderGeometry(
            radius=cap_inner_radius,
            height=cap_height + 0.004,
            radial_segments=56,
        ).translate(0.0, 0.0, cap_height * 0.5),
    )

    crown = DomeGeometry(cap_outer_radius, radial_segments=40, height_segments=12, closed=True)
    crown.scale(1.0, 1.0, 0.20).translate(0.0, 0.0, cap_height - top_thickness)

    top_disk = CylinderGeometry(
        radius=cap_outer_radius,
        height=top_thickness,
        radial_segments=56,
    ).translate(0.0, 0.0, cap_height - (top_thickness * 0.5))

    top_pad = CylinderGeometry(
        radius=cap_outer_radius * 0.86,
        height=0.0014,
        radial_segments=48,
    ).translate(0.0, 0.0, cap_height - 0.0007)

    skirt.merge(top_disk)
    skirt.merge(crown)
    skirt.merge(top_pad)
    return skirt


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_cap_bottle")

    bottle_clear = model.material("bottle_clear", rgba=(0.79, 0.90, 0.98, 0.40))
    cap_white = model.material("cap_white", rgba=(0.94, 0.95, 0.97, 1.0))

    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_geometry(_build_bottle_shell(), "bottle_shell"),
        material=bottle_clear,
        name="body_shell",
    )
    bottle.inertial = Inertial.from_geometry(
        Box((0.068, 0.068, 0.224)),
        mass=0.040,
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
    )

    cap = model.part("cap")
    cap.visual(
        mesh_from_geometry(_build_cap_shell(), "cap_shell"),
        material=cap_white,
        name="cap_shell",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0194, length=0.022),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )

    model.articulation(
        "cap_spin",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.202)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    cap_spin = object_model.get_articulation("cap_spin")

    ctx.expect_origin_distance(
        cap,
        bottle,
        axes="xy",
        max_dist=1e-6,
        name="cap is concentric with the bottle axis",
    )
    ctx.expect_origin_gap(
        cap,
        bottle,
        axis="z",
        min_gap=0.201,
        max_gap=0.203,
        name="cap sits at the top of the neck finish",
    )
    ctx.expect_overlap(
        cap,
        bottle,
        axes="xy",
        min_overlap=0.034,
        name="cap remains inside the bottle body footprint",
    )

    rest_position = ctx.part_world_position(cap)
    with ctx.pose({cap_spin: math.tau / 3.0}):
        turned_position = ctx.part_world_position(cap)

    ctx.check(
        "continuous spin keeps the cap mounted in place",
        rest_position is not None
        and turned_position is not None
        and all(abs(a - b) <= 1e-6 for a, b in zip(rest_position, turned_position)),
        details=f"rest={rest_position}, turned={turned_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
