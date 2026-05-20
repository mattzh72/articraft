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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
)


def _circle_profile(radius: float, *, segments: int = 56) -> list[tuple[float, float]]:
    return superellipse_profile(
        radius * 2.0,
        radius * 2.0,
        exponent=2.0,
        segments=segments,
    )


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_cap_bottle")

    bottle_plastic = model.material("bottle_plastic", rgba=(0.60, 0.44, 0.22, 0.96))
    cap_plastic = model.material("cap_plastic", rgba=(0.12, 0.12, 0.13, 1.0))

    bottle = model.part("bottle")

    body_outer_radius = 0.038
    body_inner_radius = 0.034
    body_wall_height = 0.133
    body_wall_center_z = 0.0765
    neck_joint_z = 0.1809

    bottle.visual(
        Cylinder(radius=body_outer_radius, length=0.0104),
        origin=Origin(xyz=(0.0, 0.0, 0.0052)),
        material=bottle_plastic,
        name="base_disk",
    )

    body_wall = ExtrudeWithHolesGeometry(
        _circle_profile(body_outer_radius),
        [_circle_profile(body_inner_radius)],
        body_wall_height,
        center=True,
    )
    bottle.visual(
        _save_mesh("bottle_body_wall", body_wall.translate(0.0, 0.0, body_wall_center_z)),
        material=bottle_plastic,
        name="body_wall",
    )

    shoulder_shell = LatheGeometry.from_shell_profiles(
        [
            (body_outer_radius, 0.1415),
            (0.0375, 0.1480),
            (0.0340, 0.1580),
            (0.0285, 0.1680),
            (0.0225, 0.1755),
            (0.0188, 0.1800),
            (0.0186, 0.1828),
            (0.0172, 0.1840),
            (0.0178, 0.1854),
            (0.0169, 0.1868),
            (0.0178, 0.1882),
            (0.0169, 0.1896),
            (0.0176, 0.1910),
            (0.0169, 0.1920),
            (0.0169, 0.1930),
        ],
        [
            (body_inner_radius, 0.1415),
            (0.0333, 0.1480),
            (0.0308, 0.1580),
            (0.0253, 0.1680),
            (0.0198, 0.1755),
            (0.0150, 0.1800),
            (0.0134, 0.1828),
            (0.0131, 0.1930),
        ],
        segments=80,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    bottle.visual(
        _save_mesh("bottle_shoulder_neck", shoulder_shell),
        material=bottle_plastic,
        name="shoulder_neck",
    )
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=body_outer_radius, length=0.193),
        mass=0.085,
        origin=Origin(xyz=(0.0, 0.0, 0.0965)),
    )

    cap = model.part("cap")

    cap_outer_radius = 0.024
    cap_height = 0.016

    cap.visual(
        Cylinder(radius=cap_outer_radius, length=0.0034),
        origin=Origin(xyz=(0.0, 0.0, 0.0143)),
        material=cap_plastic,
        name="cap_top",
    )
    cap.visual(
        Cylinder(radius=0.0170, length=0.0018),
        origin=Origin(xyz=(0.0, 0.0, 0.0130)),
        material=cap_plastic,
        name="seal_land",
    )

    flute_count = 32
    flute_radius = 0.02195
    for index in range(flute_count):
        angle = math.tau * index / flute_count
        cap.visual(
            Box((0.0046, 0.0037, 0.0135)),
            origin=Origin(
                xyz=(
                    flute_radius * math.cos(angle),
                    flute_radius * math.sin(angle),
                    0.00675,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=cap_plastic,
            name=f"flute_{index:02d}",
        )

    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=cap_outer_radius, length=cap_height),
        mass=0.010,
        origin=Origin(xyz=(0.0, 0.0, cap_height * 0.5)),
    )

    model.articulation(
        "cap_spin",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, neck_joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=12.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    bottle = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    cap_spin = object_model.get_articulation("cap_spin")

    limits = cap_spin.motion_limits
    ctx.check(
        "cap uses continuous spin",
        cap_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(cap_spin.axis) == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        details=(
            f"type={cap_spin.articulation_type}, axis={cap_spin.axis}, "
            f"limits={None if limits is None else (limits.lower, limits.upper)}"
        ),
    )
    ctx.expect_origin_gap(
        cap,
        bottle,
        axis="z",
        min_gap=0.178,
        max_gap=0.185,
        name="cap is mounted at the neck height",
    )
    ctx.expect_overlap(
        cap,
        bottle,
        axes="xy",
        min_overlap=0.045,
        name="cap stays centered over the bottle",
    )
    ctx.expect_gap(
        cap,
        bottle,
        axis="z",
        positive_elem="cap_top",
        negative_elem="shoulder_neck",
        min_gap=0.0004,
        max_gap=0.0030,
        name="cap top sits just above the bottle lip",
    )
    ctx.expect_contact(
        cap,
        bottle,
        elem_a="seal_land",
        elem_b="shoulder_neck",
        name="cap seals against the neck finish",
    )

    rest_pos = ctx.part_world_position(cap)
    with ctx.pose({cap_spin: 1.7}):
        ctx.expect_overlap(
            cap,
            bottle,
            axes="xy",
            min_overlap=0.045,
            name="rotated cap remains coaxial",
        )
        spun_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap rotation does not translate the closure",
        rest_pos is not None
        and spun_pos is not None
        and all(abs(a - b) < 1e-9 for a, b in zip(rest_pos, spun_pos)),
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
