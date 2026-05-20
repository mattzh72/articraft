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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_cap_bottle")

    def ring_band(
        *,
        outer_radius: float,
        inner_radius: float,
        height: float,
        z_center: float,
        radial_segments: int = 56,
    ):
        outer = CylinderGeometry(radius=outer_radius, height=height, radial_segments=radial_segments)
        inner = CylinderGeometry(
            radius=inner_radius,
            height=height + 0.002,
            radial_segments=radial_segments,
        )
        return boolean_difference(outer, inner).translate(0.0, 0.0, z_center)

    bottle_pet = model.material("bottle_pet", rgba=(0.72, 0.85, 0.98, 0.45))
    cap_plastic = model.material("cap_plastic", rgba=(0.10, 0.12, 0.15, 1.0))

    bottle = model.part("bottle_body")

    bottle_shell = LatheGeometry.from_shell_profiles(
        [
            (0.005, 0.000),
            (0.028, 0.000),
            (0.034, 0.006),
            (0.036, 0.020),
            (0.036, 0.086),
            (0.031, 0.118),
            (0.034, 0.148),
            (0.028, 0.168),
            (0.022, 0.178),
            (0.018, 0.188),
            (0.0155, 0.198),
            (0.015, 0.208),
            (0.015, 0.214),
            (0.016, 0.218),
        ],
        [
            (0.000, 0.005),
            (0.025, 0.008),
            (0.030, 0.020),
            (0.030, 0.084),
            (0.026, 0.118),
            (0.028, 0.146),
            (0.023, 0.166),
            (0.018, 0.177),
            (0.0145, 0.188),
            (0.0125, 0.198),
            (0.0120, 0.208),
            (0.0120, 0.214),
        ],
        segments=88,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    bottle.visual(
        mesh_from_geometry(bottle_shell, "bottle_shell"),
        material=bottle_pet,
        name="body_shell",
    )

    threaded_neck = ring_band(
        outer_radius=0.0157,
        inner_radius=0.0144,
        height=0.026,
        z_center=0.201,
        radial_segments=64,
    )
    threaded_neck.merge(
        ring_band(
            outer_radius=0.0170,
            inner_radius=0.0144,
            height=0.004,
            z_center=0.191,
            radial_segments=64,
        )
    )
    for z_center in (0.197, 0.202, 0.207, 0.212):
        threaded_neck.merge(
            ring_band(
                outer_radius=0.0172,
                inner_radius=0.0148,
                height=0.0022,
                z_center=z_center,
                radial_segments=64,
            )
        )
    bottle.visual(
        mesh_from_geometry(threaded_neck, "threaded_neck"),
        material=bottle_pet,
        name="threaded_neck",
    )
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.036, length=0.218),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, 0.109)),
    )

    cap = model.part("cap")
    cap_shell = boolean_difference(
        CylinderGeometry(radius=0.0205, height=0.060, radial_segments=64).translate(0.0, 0.0, 0.030),
        CylinderGeometry(radius=0.0178, height=0.056, radial_segments=64).translate(0.0, 0.0, 0.028),
    )
    cap_shell.merge(
        ring_band(
            outer_radius=0.0216,
            inner_radius=0.0200,
            height=0.006,
            z_center=0.007,
            radial_segments=64,
        )
    )
    for z_center in (0.016, 0.026, 0.036, 0.046):
        cap_shell.merge(
            ring_band(
                outer_radius=0.0213,
                inner_radius=0.0200,
                height=0.0032,
                z_center=z_center,
                radial_segments=64,
            )
        )
    for z_center in (0.011, 0.016, 0.021):
        cap_shell.merge(
            ring_band(
                outer_radius=0.0179,
                inner_radius=0.0172,
                height=0.0024,
                z_center=z_center,
                radial_segments=64,
            )
        )
    cap.visual(
        mesh_from_geometry(cap_shell, "cap_shell"),
        material=cap_plastic,
        name="cap_shell",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0205, length=0.060),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    model.articulation(
        "cap_spin",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.191)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottle = object_model.get_part("bottle_body")
    cap = object_model.get_part("cap")
    cap_spin = object_model.get_articulation("cap_spin")

    with ctx.pose({cap_spin: 0.0}):
        ctx.expect_within(
            bottle,
            cap,
            axes="xy",
            inner_elem="threaded_neck",
            outer_elem="cap_shell",
            margin=0.004,
            name="threaded neck stays inside the cap skirt",
        )
        ctx.expect_overlap(
            cap,
            bottle,
            axes="z",
            elem_a="cap_shell",
            elem_b="threaded_neck",
            min_overlap=0.020,
            name="cap covers the bottle threads",
        )
        rest_pos = ctx.part_world_position(cap)

    with ctx.pose({cap_spin: math.tau * 0.33}):
        ctx.expect_within(
            bottle,
            cap,
            axes="xy",
            inner_elem="threaded_neck",
            outer_elem="cap_shell",
            margin=0.004,
            name="threaded neck stays centered during rotation",
        )
        rotated_pos = ctx.part_world_position(cap)

    same_position = (
        rest_pos is not None
        and rotated_pos is not None
        and max(abs(a - b) for a, b in zip(rest_pos, rotated_pos)) <= 1e-6
    )
    ctx.check(
        "cap rotates in place about the neck axis",
        same_position,
        details=f"rest={rest_pos}, rotated={rotated_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
