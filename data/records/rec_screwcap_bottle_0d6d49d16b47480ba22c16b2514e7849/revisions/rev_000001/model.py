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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="squat_screw_cap_bottle")

    amber_body = model.material("amber_body", rgba=(0.58, 0.34, 0.14, 1.0))
    cream_label = model.material("cream_label", rgba=(0.90, 0.86, 0.73, 1.0))
    charcoal_cap = model.material("charcoal_cap", rgba=(0.12, 0.12, 0.13, 1.0))
    shadow_gray = model.material("shadow_gray", rgba=(0.18, 0.18, 0.20, 1.0))

    body_outer_profile = [
        (0.016, 0.000),
        (0.046, 0.002),
        (0.056, 0.010),
        (0.061, 0.022),
        (0.061, 0.060),
        (0.058, 0.069),
        (0.050, 0.076),
        (0.040, 0.082),
        (0.032, 0.090),
        (0.031, 0.126),
        (0.034, 0.130),
    ]
    body_inner_profile = [
        (0.000, 0.007),
        (0.043, 0.012),
        (0.049, 0.020),
        (0.049, 0.058),
        (0.046, 0.067),
        (0.038, 0.074),
        (0.026, 0.084),
        (0.022, 0.118),
    ]
    bottle_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            body_outer_profile,
            body_inner_profile,
            segments=88,
            start_cap="flat",
            end_cap="flat",
            lip_samples=10,
        ),
        "bottle_shell",
    )

    cap_outer_profile = [
        (0.042, 0.000),
        (0.045, 0.004),
        (0.046, 0.012),
        (0.046, 0.048),
        (0.043, 0.056),
        (0.000, 0.060),
    ]
    cap_inner_profile = [
        (0.036, 0.000),
        (0.036, 0.046),
        (0.031, 0.052),
        (0.000, 0.052),
    ]
    cap_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            cap_outer_profile,
            cap_inner_profile,
            segments=88,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "cap_shell",
    )
    heel_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.0545, tube=0.0040, radial_segments=18, tubular_segments=72),
        "heel_ring",
    )
    thread_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.0326, tube=0.0018, radial_segments=14, tubular_segments=56),
        "thread_ring",
    )

    bottle_body = model.part("bottle_body")
    bottle_body.visual(
        bottle_shell_mesh,
        material=amber_body,
        name="body_shell",
    )
    bottle_body.visual(
        heel_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=amber_body,
        name="base_pushup_ring",
    )
    bottle_body.visual(
        Cylinder(radius=0.0615, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.046), rpy=(0.0, 0.0, 0.0)),
        material=cream_label,
        name="label_band",
    )
    bottle_body.visual(
        Cylinder(radius=0.0308, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
        material=amber_body,
        name="neck_finish",
    )
    for index, z_pos in enumerate((0.097, 0.105, 0.113, 0.121)):
        bottle_body.visual(
            thread_ring_mesh,
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material=amber_body,
            name=f"thread_ring_{index}",
        )
    bottle_body.inertial = Inertial.from_geometry(
        Box((0.126, 0.126, 0.130)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
    )

    screw_cap = model.part("screw_cap")
    screw_cap.visual(
        cap_shell_mesh,
        material=charcoal_cap,
        name="cap_shell",
    )
    screw_cap.visual(
        Cylinder(radius=0.0305, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=shadow_gray,
        name="cap_liner",
    )
    for index in range(16):
        angle = (2.0 * math.pi * index) / 16.0
        screw_cap.visual(
            Box((0.007, 0.005, 0.044)),
            origin=Origin(
                xyz=(0.0435 * math.cos(angle), 0.0435 * math.sin(angle), 0.024),
                rpy=(0.0, 0.0, angle),
            ),
            material=shadow_gray,
            name=f"grip_rib_{index:02d}",
        )
    screw_cap.visual(
        Box((0.018, 0.006, 0.0035)),
        origin=Origin(xyz=(0.018, 0.0, 0.060)),
        material=shadow_gray,
        name="top_orientation_tab",
    )
    screw_cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.046, length=0.060),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    model.articulation(
        "cap_spin",
        ArticulationType.CONTINUOUS,
        parent=bottle_body,
        child=screw_cap,
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle_body = object_model.get_part("bottle_body")
    screw_cap = object_model.get_part("screw_cap")
    cap_spin = object_model.get_articulation("cap_spin")

    ctx.expect_origin_distance(
        screw_cap,
        bottle_body,
        axes="xy",
        max_dist=0.001,
        name="cap stays centered over the bottle axis",
    )
    ctx.expect_origin_gap(
        screw_cap,
        bottle_body,
        axis="z",
        min_gap=0.090,
        max_gap=0.100,
        name="cap mount sits above the body centerline",
    )
    ctx.expect_overlap(
        screw_cap,
        bottle_body,
        axes="xy",
        min_overlap=0.080,
        name="cap visually dominates the bottle front footprint",
    )
    ctx.expect_contact(
        screw_cap,
        bottle_body,
        elem_a="cap_liner",
        elem_b="neck_finish",
        name="cap liner seats on the bottle finish",
    )

    rest_pos = ctx.part_world_position(screw_cap)
    with ctx.pose({cap_spin: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(screw_cap)
        ctx.expect_origin_distance(
            screw_cap,
            bottle_body,
            axes="xy",
            max_dist=0.001,
            name="cap remains coaxial while rotated",
        )

    ctx.check(
        "cap rotation preserves its mounted position",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) <= 1e-6
        and abs(rest_pos[1] - turned_pos[1]) <= 1e-6
        and abs(rest_pos[2] - turned_pos[2]) <= 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
