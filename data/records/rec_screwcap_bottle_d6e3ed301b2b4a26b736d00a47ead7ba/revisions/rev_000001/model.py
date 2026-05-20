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

    bottle_pet = model.material("bottle_pet", rgba=(0.78, 0.90, 0.98, 0.52))
    cap_hdpe = model.material("cap_hdpe", rgba=(0.96, 0.97, 0.98, 1.0))
    label_ink = model.material("label_ink", rgba=(0.18, 0.43, 0.78, 1.0))

    body = model.part("body")

    body_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.0, 0.0),
                (0.012, 0.002),
                (0.026, 0.006),
                (0.033, 0.012),
                (0.035, 0.028),
                (0.035, 0.103),
                (0.034, 0.127),
                (0.031, 0.150),
                (0.026, 0.171),
                (0.021, 0.186),
                (0.0165, 0.198),
                (0.0153, 0.214),
                (0.0148, 0.223),
            ],
            [
                (0.0, 0.010),
                (0.010, 0.012),
                (0.024, 0.016),
                (0.030, 0.022),
                (0.031, 0.102),
                (0.030, 0.126),
                (0.027, 0.149),
                (0.0225, 0.169),
                (0.0175, 0.184),
                (0.0132, 0.196),
                (0.0120, 0.214),
            ],
            segments=72,
        ),
        "bottle_body_shell",
    )
    body.visual(body_shell, material=bottle_pet, name="body_shell")
    body.visual(
        Cylinder(radius=0.0175, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        material=bottle_pet,
        name="neck_support_collar",
    )
    body.visual(
        Cylinder(radius=0.0160, length=0.0045),
        origin=Origin(xyz=(0.0, 0.0, 0.1995)),
        material=bottle_pet,
        name="neck_thread_lower",
    )
    body.visual(
        Cylinder(radius=0.0160, length=0.0045),
        origin=Origin(xyz=(0.0, 0.0, 0.208)),
        material=bottle_pet,
        name="neck_thread_upper",
    )
    body.visual(
        Cylinder(radius=0.0332, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
        material=label_ink,
        name="label_band",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.074, 0.074, 0.223)),
        mass=0.055,
        origin=Origin(xyz=(0.0, 0.0, 0.1115)),
    )

    cap = model.part("cap")

    cap_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.0, 0.034),
                (0.010, 0.034),
                (0.0155, 0.033),
                (0.0179, 0.030),
                (0.0185, 0.025),
                (0.0185, 0.005),
                (0.0178, 0.0015),
                (0.0170, 0.0),
            ],
            [
                (0.0, 0.0295),
                (0.010, 0.0295),
                (0.0150, 0.0288),
                (0.0170, 0.0260),
                (0.0176, 0.0200),
                (0.0176, 0.0060),
                (0.0172, 0.0018),
                (0.0168, 0.0),
            ],
            segments=72,
        ),
        "bottle_cap_shell",
    )
    cap.visual(cap_shell, material=cap_hdpe, name="cap_shell")
    cap.visual(
        Cylinder(radius=0.0173, length=0.0030),
        origin=Origin(xyz=(0.0, 0.0, 0.0315)),
        material=cap_hdpe,
        name="cap_top_collar",
    )

    rib_height = 0.022
    rib_center_z = 0.013
    rib_radius = 0.0176
    for rib_index in range(18):
        angle = rib_index * math.tau / 18.0
        cap.visual(
            Box((0.0028, 0.0030, rib_height)),
            origin=Origin(
                xyz=(rib_radius * math.cos(angle), rib_radius * math.sin(angle), rib_center_z),
                rpy=(0.0, 0.0, angle),
            ),
            material=cap_hdpe,
            name=f"grip_rib_{rib_index:02d}",
        )

    cap.visual(
        Box((0.0030, 0.0060, 0.016)),
        origin=Origin(xyz=(0.0178, 0.0, 0.014), rpy=(0.0, 0.0, 0.0)),
        material=cap_hdpe,
        name="mold_seam_pad",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0182, length=0.034),
        mass=0.0045,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
    )

    model.articulation(
        "cap_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.198)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    cap = object_model.get_part("cap")
    cap_spin = object_model.get_articulation("cap_spin")

    limits = cap_spin.motion_limits
    ctx.check(
        "cap uses a continuous neck-axis articulation",
        cap_spin.articulation_type == ArticulationType.CONTINUOUS
        and cap_spin.axis == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        details=(
            f"type={cap_spin.articulation_type}, axis={cap_spin.axis}, "
            f"limits=({None if limits is None else limits.lower}, {None if limits is None else limits.upper})"
        ),
    )
    ctx.expect_within(
        body,
        cap,
        axes="xy",
        inner_elem="neck_thread_upper",
        outer_elem="cap_shell",
        margin=0.003,
        name="cap shell surrounds the threaded neck in plan",
    )
    ctx.expect_gap(
        cap,
        body,
        axis="z",
        positive_elem="cap_shell",
        negative_elem="neck_support_collar",
        min_gap=0.0005,
        max_gap=0.014,
        name="cap skirt starts just above the shoulder collar",
    )

    with ctx.pose({cap_spin: 1.35}):
        ctx.expect_within(
            body,
            cap,
            axes="xy",
            inner_elem="neck_thread_upper",
            outer_elem="cap_shell",
            margin=0.003,
            name="rotated cap remains coaxial with the threaded neck",
        )
        ctx.expect_gap(
            cap,
            body,
            axis="z",
            positive_elem="cap_shell",
            negative_elem="neck_support_collar",
            min_gap=0.0005,
            max_gap=0.014,
            name="rotated cap keeps the same seated height",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
