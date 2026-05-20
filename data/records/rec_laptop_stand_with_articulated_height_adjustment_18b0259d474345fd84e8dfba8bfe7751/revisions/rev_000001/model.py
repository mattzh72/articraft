from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _pitch_for_vector(dx: float, dz: float) -> float:
    """Pitch a local +X bar so it points to the requested X/Z vector."""

    return -math.atan2(dz, dx)


def _circle_intersections(
    a: tuple[float, float],
    b: tuple[float, float],
    radius_a: float,
    radius_b: float,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """Return the two X/Z elbow points for fixed-length links between pivots."""

    ax, az = a
    bx, bz = b
    dx = bx - ax
    dz = bz - az
    span = math.hypot(dx, dz)
    along = (radius_a * radius_a - radius_b * radius_b + span * span) / (2.0 * span)
    height = math.sqrt(max(0.0, radius_a * radius_a - along * along))
    mid_x = ax + along * dx / span
    mid_z = az + along * dz / span
    off_x = -dz * height / span
    off_z = dx * height / span
    return (mid_x + off_x, mid_z + off_z), (mid_x - off_x, mid_z - off_z)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_laptop_stand")

    aluminum = model.material("satin_aluminum", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    black_plastic = model.material("black_pivot_caps", rgba=(0.025, 0.028, 0.030, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.42, 0.26, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=aluminum,
        name="base_plate",
    )

    # Front hinge knuckles on the base.  They are split along Y so the tray's
    # center knuckle can rotate between them on the same transverse hinge line.
    hinge_x = -0.155
    hinge_z = 0.036
    for idx, y in enumerate((-0.088, 0.088)):
        base.visual(
            Box((0.034, 0.070, 0.016)),
            origin=Origin(xyz=(hinge_x, y, 0.024)),
            material=aluminum,
            name=f"front_hinge_saddle_{idx}",
        )
        base.visual(
            Cylinder(radius=0.012, length=0.066),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=aluminum,
            name=f"front_hinge_barrel_{idx}",
        )

    # A clevis at the rear of the base for the lower member of the lift arm.
    base_pivot_x = 0.140
    base_pivot_z = 0.041
    base_pivot_ear_names = ("base_pivot_ear_0", "base_pivot_ear_1")
    base_pivot_bushing_names = ("base_pivot_bushing_0", "base_pivot_bushing_1")
    for idx, y in enumerate((-0.032, 0.032)):
        base.visual(
            Box((0.030, 0.020, 0.050)),
            origin=Origin(xyz=(base_pivot_x, y, 0.032)),
            material=aluminum,
            name=base_pivot_ear_names[idx],
        )
        base.visual(
            Cylinder(radius=0.013, length=0.020),
            origin=Origin(
                xyz=(base_pivot_x, y, base_pivot_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=black_plastic,
            name=base_pivot_bushing_names[idx],
        )

    tray_pitch = 0.38
    cos_pitch = math.cos(tray_pitch)
    sin_pitch = math.sin(tray_pitch)
    tray = model.part("tray")
    tray.visual(
        Box((0.36, 0.24, 0.016)),
        origin=Origin(xyz=(0.180, 0.0, 0.016)),
        material=aluminum,
        name="tray_panel",
    )
    tray.visual(
        Cylinder(radius=0.010, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="front_hinge_barrel",
    )
    tray.visual(
        Box((0.014, 0.220, 0.040)),
        origin=Origin(xyz=(0.045, 0.0, 0.044)),
        material=aluminum,
        name="front_lip",
    )
    for idx, y in enumerate((-0.070, 0.070)):
        tray.visual(
            Box((0.250, 0.022, 0.004)),
            origin=Origin(xyz=(0.205, y, 0.026)),
            material=dark_rubber,
            name=f"rubber_strip_{idx}",
        )

    tray_pivot_local_x = 0.300
    tray_pivot_local_z = -0.010
    tray_pivot_ear_names = ("tray_pivot_ear_0", "tray_pivot_ear_1")
    tray_pivot_bushing_names = ("tray_pivot_bushing_0", "tray_pivot_bushing_1")
    for idx, y in enumerate((-0.018, 0.018)):
        tray.visual(
            Box((0.032, 0.016, 0.030)),
            origin=Origin(xyz=(tray_pivot_local_x, y, -0.001)),
            material=aluminum,
            name=tray_pivot_ear_names[idx],
        )
        tray.visual(
            Cylinder(radius=0.013, length=0.016),
            origin=Origin(
                xyz=(tray_pivot_local_x, y, tray_pivot_local_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=black_plastic,
            name=tray_pivot_bushing_names[idx],
        )

    # The rear lifting arm is laid out as a real two-link mechanism.  The
    # member lengths are fixed and the rest/raised elbow points are chosen from
    # circle intersections so the two members remain pinned together while the
    # tray height changes.
    raise_reference = 0.35
    lower_member_len = 0.130
    upper_member_len = 0.110
    tray_pivot_world_x = hinge_x + cos_pitch * tray_pivot_local_x - sin_pitch * tray_pivot_local_z
    tray_pivot_world_z = hinge_z + sin_pitch * tray_pivot_local_x + cos_pitch * tray_pivot_local_z
    raised_pitch = tray_pitch + raise_reference
    raised_tray_pivot_x = (
        hinge_x
        + math.cos(raised_pitch) * tray_pivot_local_x
        - math.sin(raised_pitch) * tray_pivot_local_z
    )
    raised_tray_pivot_z = (
        hinge_z
        + math.sin(raised_pitch) * tray_pivot_local_x
        + math.cos(raised_pitch) * tray_pivot_local_z
    )
    rest_elbows = _circle_intersections(
        (base_pivot_x, base_pivot_z),
        (tray_pivot_world_x, tray_pivot_world_z),
        lower_member_len,
        upper_member_len,
    )
    raised_elbows = _circle_intersections(
        (base_pivot_x, base_pivot_z),
        (raised_tray_pivot_x, raised_tray_pivot_z),
        lower_member_len,
        upper_member_len,
    )
    elbow_world_x, elbow_world_z = max(rest_elbows, key=lambda point: point[0])
    raised_elbow_x, raised_elbow_z = max(raised_elbows, key=lambda point: point[0])

    lower_rest_angle = math.atan2(elbow_world_z - base_pivot_z, elbow_world_x - base_pivot_x)
    lower_raised_angle = math.atan2(raised_elbow_z - base_pivot_z, raised_elbow_x - base_pivot_x)
    lower_mimic_multiplier = (lower_raised_angle - lower_rest_angle) / raise_reference
    upper_local_angle = (
        math.atan2(elbow_world_z - tray_pivot_world_z, elbow_world_x - tray_pivot_world_x)
        - tray_pitch
    )
    upper_raised_angle = math.atan2(
        raised_elbow_z - raised_tray_pivot_z,
        raised_elbow_x - raised_tray_pivot_x,
    )
    upper_mimic_multiplier = (upper_raised_angle - raised_pitch - upper_local_angle) / raise_reference

    lower_arm = model.part("lower_arm")
    lower_dx = elbow_world_x - base_pivot_x
    lower_dz = elbow_world_z - base_pivot_z
    lower_len = math.hypot(lower_dx, lower_dz)
    lower_pitch = _pitch_for_vector(lower_dx, lower_dz)
    side_link_names = ("side_link_0", "side_link_1")
    base_fork_names = ("base_fork_0", "base_fork_1")
    elbow_fork_names = ("elbow_fork_0", "elbow_fork_1")
    for idx, y in enumerate((-0.016, 0.016)):
        lower_arm.visual(
            Box((lower_len, 0.012, 0.010)),
            origin=Origin(
                xyz=(lower_dx / 2.0, y, lower_dz / 2.0),
                rpy=(0.0, lower_pitch, 0.0),
            ),
            material=aluminum,
            name=side_link_names[idx],
        )
        lower_arm.visual(
            Cylinder(radius=0.013, length=0.012),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black_plastic,
            name=base_fork_names[idx],
        )
        lower_arm.visual(
            Cylinder(radius=0.013, length=0.012),
            origin=Origin(
                xyz=(lower_dx, y, lower_dz),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=black_plastic,
            name=elbow_fork_names[idx],
        )
    lower_arm.visual(
        Box((0.018, 0.044, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=black_plastic,
        name="base_cross_pin",
    )

    upper_arm = model.part("upper_arm")
    delta_x = elbow_world_x - tray_pivot_world_x
    delta_z = elbow_world_z - tray_pivot_world_z
    upper_dx = cos_pitch * delta_x + sin_pitch * delta_z
    upper_dz = -sin_pitch * delta_x + cos_pitch * delta_z
    upper_len = math.hypot(upper_dx, upper_dz)
    upper_pitch = _pitch_for_vector(upper_dx, upper_dz)
    upper_arm.visual(
        Box((upper_len, 0.020, 0.010)),
        origin=Origin(
            xyz=(upper_dx / 2.0, 0.0, upper_dz / 2.0),
            rpy=(0.0, upper_pitch, 0.0),
        ),
        material=aluminum,
        name="center_link",
    )
    upper_arm.visual(
        Cylinder(radius=0.013, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="tray_boss",
    )
    upper_arm.visual(
        Cylinder(radius=0.013, length=0.020),
        origin=Origin(
            xyz=(upper_dx, 0.0, upper_dz),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=black_plastic,
        name="elbow_boss",
    )

    model.articulation(
        "front_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=tray,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z), rpy=(0.0, -tray_pitch, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.25, upper=0.55),
    )
    model.articulation(
        "lower_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(base_pivot_x, 0.0, base_pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=-0.55, upper=1.10),
        mimic=Mimic(joint="front_hinge", multiplier=lower_mimic_multiplier, offset=0.0),
    )
    model.articulation(
        "upper_pivot",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=upper_arm,
        origin=Origin(xyz=(tray_pivot_local_x, 0.0, tray_pivot_local_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=-1.40, upper=0.70),
        mimic=Mimic(joint="front_hinge", multiplier=upper_mimic_multiplier, offset=0.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    tray = object_model.get_part("tray")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    front_hinge = object_model.get_articulation("front_hinge")
    lower_pivot = object_model.get_articulation("lower_pivot")
    upper_pivot = object_model.get_articulation("upper_pivot")

    ctx.check(
        "tray is one rigid part",
        tray is not None and len([p for p in object_model.parts if p.name == "tray"]) == 1,
        details="The upper tray should not be split into separate hinged panels.",
    )
    ctx.check(
        "primary revolute joints",
        all(
            joint is not None and joint.articulation_type == ArticulationType.REVOLUTE
            for joint in (front_hinge, lower_pivot, upper_pivot)
        ),
        details="The front tray hinge, lower base pivot, and upper tray-side pivot must all rotate.",
    )

    ctx.expect_contact(
        base,
        lower_arm,
        elem_a="base_pivot_bushing_1",
        elem_b="base_fork_1",
        contact_tol=0.001,
        name="lower arm seated in base pivot",
    )
    ctx.expect_contact(
        tray,
        upper_arm,
        elem_a="tray_pivot_bushing_1",
        elem_b="tray_boss",
        contact_tol=0.001,
        name="upper arm seated in tray pivot",
    )
    ctx.expect_contact(
        lower_arm,
        upper_arm,
        elem_a="elbow_fork_1",
        elem_b="elbow_boss",
        contact_tol=0.001,
        name="two arm members meet at elbow",
    )

    rest_tray_aabb = ctx.part_world_aabb(tray)
    rest_lower_aabb = ctx.part_world_aabb(lower_arm)
    with ctx.pose({front_hinge: 0.35}):
        raised_tray_aabb = ctx.part_world_aabb(tray)
        raised_lower_aabb = ctx.part_world_aabb(lower_arm)
        ctx.expect_contact(
            lower_arm,
            upper_arm,
            elem_a="elbow_fork_1",
            elem_b="elbow_boss",
            contact_tol=0.002,
            name="raised arm members remain pinned at elbow",
        )

    ctx.check(
        "front hinge raises tray rear",
        rest_tray_aabb is not None
        and raised_tray_aabb is not None
        and raised_tray_aabb[1][2] > rest_tray_aabb[1][2] + 0.03,
        details=f"rest={rest_tray_aabb}, raised={raised_tray_aabb}",
    )
    ctx.check(
        "mimicked lower arm rotates with tray",
        rest_lower_aabb is not None
        and raised_lower_aabb is not None
        and raised_lower_aabb[1][2] > rest_lower_aabb[1][2] + 0.005,
        details=f"rest={rest_lower_aabb}, raised={raised_lower_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
