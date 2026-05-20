from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _fluted_cap_geometry(
    *,
    outer_radius: float,
    inner_radius: float,
    bottom_z: float,
    top_z: float,
    top_thickness: float,
    flutes: int = 24,
    flute_depth: float = 0.00045,
    theta_segments: int = 96,
) -> MeshGeometry:
    """Build one clean screw-cap mesh: open at the bottom, closed at the top."""

    shoulder_z = top_z - 0.004
    inner_top_z = top_z - top_thickness
    profile = [
        (inner_radius, bottom_z, False),
        (outer_radius - 0.00025, bottom_z, True),
        (outer_radius, bottom_z + 0.004, True),
        (outer_radius, shoulder_z, True),
        (outer_radius - 0.0012, top_z, False),
        (0.0, top_z, False),
        (0.0, inner_top_z, False),
        (inner_radius, inner_top_z, False),
    ]

    mesh = MeshGeometry()

    def cap_radius(radius: float, theta: float, modulate: bool) -> float:
        if not modulate or radius <= 0.0:
            return radius
        # Shallow vertical grip flutes: enough to show the rotating cap without
        # adding stacked rings or extra secondary breaks around the closure.
        groove = 0.5 + 0.5 * math.cos(flutes * theta)
        return radius - flute_depth * groove

    for theta_index in range(theta_segments):
        theta = 2.0 * math.pi * theta_index / theta_segments
        c = math.cos(theta)
        s = math.sin(theta)
        for radius, z, modulate in profile:
            r = cap_radius(radius, theta, modulate)
            mesh.add_vertex(r * c, r * s, z)

    profile_count = len(profile)

    def vid(theta_index: int, profile_index: int) -> int:
        return (theta_index % theta_segments) * profile_count + profile_index

    for theta_index in range(theta_segments):
        next_theta = (theta_index + 1) % theta_segments
        for profile_index in range(profile_count):
            next_profile = (profile_index + 1) % profile_count
            a = vid(theta_index, profile_index)
            b = vid(next_theta, profile_index)
            c = vid(next_theta, next_profile)
            d = vid(theta_index, next_profile)
            mesh.add_face(a, b, c)
            mesh.add_face(a, c, d)

    return mesh


def _neck_thread_geometry() -> MeshGeometry:
    points = []
    turns = 2.25
    samples = 96
    for index in range(samples + 1):
        t = index / samples
        theta = turns * 2.0 * math.pi * t
        radius = 0.01285
        z = 0.190 + 0.014 * t
        points.append((radius * math.cos(theta), radius * math.sin(theta), z))
    return tube_from_spline_points(
        points,
        radius=0.00075,
        samples_per_segment=2,
        radial_segments=12,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_cap_bottle")

    clear_pet = model.material("clear_pet", rgba=(0.70, 0.88, 1.0, 0.42))
    label_white = model.material("label_white", rgba=(0.92, 0.96, 1.0, 0.82))
    cap_blue = model.material("cap_blue", rgba=(0.05, 0.30, 0.78, 1.0))

    bottle = model.part("bottle")

    outer_profile = [
        (0.018, 0.006),
        (0.030, 0.008),
        (0.034, 0.015),
        (0.035, 0.032),
        (0.035, 0.132),
        (0.033, 0.150),
        (0.027, 0.165),
        (0.019, 0.178),
        (0.0122, 0.188),
    ]
    inner_profile = [
        (0.012, 0.011),
        (0.026, 0.013),
        (0.031, 0.020),
        (0.032, 0.034),
        (0.032, 0.130),
        (0.030, 0.146),
        (0.024, 0.160),
        (0.016, 0.173),
        (0.0092, 0.187),
    ]
    bottle_shell = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=96,
        start_cap="flat",
        end_cap="flat",
    )
    bottle.visual(
        mesh_from_geometry(bottle_shell, "bottle_shell"),
        material=clear_pet,
        name="bottle_shell",
    )
    neck_finish = LatheGeometry.from_shell_profiles(
        [(0.0122, 0.186), (0.0118, 0.214), (0.0130, 0.218)],
        [(0.0092, 0.186), (0.0092, 0.213), (0.0102, 0.216)],
        segments=96,
        start_cap="flat",
        end_cap="flat",
    )
    bottle.visual(
        mesh_from_geometry(neck_finish, "neck_finish"),
        material=clear_pet,
        name="neck_finish",
    )

    label_profile = [
        (0.0347, 0.064),
        (0.0351, 0.064),
        (0.0351, 0.116),
        (0.0347, 0.116),
    ]
    bottle.visual(
        mesh_from_geometry(LatheGeometry(label_profile, segments=96), "label_band"),
        material=label_white,
        name="label_band",
    )
    bottle.visual(
        mesh_from_geometry(_neck_thread_geometry(), "neck_thread"),
        material=clear_pet,
        name="neck_thread",
    )

    cap = model.part("cap")
    cap.visual(
        mesh_from_geometry(
            _fluted_cap_geometry(
                outer_radius=0.0164,
                inner_radius=0.0145,
                bottom_z=-0.012,
                top_z=0.016,
                top_thickness=0.0055,
            ),
            "cap_shell",
        ),
        material=cap_blue,
        name="cap_shell",
    )

    model.articulation(
        "cap_spin",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.2075)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.45, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    cap_spin = object_model.get_articulation("cap_spin")

    ctx.allow_overlap(
        bottle,
        cap,
        elem_a="neck_finish",
        elem_b="cap_shell",
        reason=(
            "The closed screw cap's single clean shell intentionally represents the "
            "hidden top seal seated against the bottle mouth."
        ),
    )
    ctx.check(
        "cap uses continuous screw rotation",
        cap_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(cap_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={cap_spin.articulation_type}, axis={cap_spin.axis}",
    )
    ctx.expect_origin_distance(
        cap,
        bottle,
        axes="xy",
        max_dist=0.001,
        name="cap is centered on the bottle neck axis",
    )
    ctx.expect_within(
        bottle,
        cap,
        axes="xy",
        inner_elem="neck_thread",
        outer_elem="cap_shell",
        margin=0.0,
        name="threaded neck sits inside the cap footprint",
    )
    ctx.expect_contact(
        bottle,
        cap,
        elem_a="neck_finish",
        elem_b="cap_shell",
        name="cap seal is seated on the mouth lip",
    )
    ctx.expect_overlap(
        cap,
        bottle,
        axes="z",
        elem_a="cap_shell",
        elem_b="neck_thread",
        min_overlap=0.006,
        name="cap axially covers the screw thread",
    )

    with ctx.pose({cap_spin: math.pi / 2.0}):
        ctx.expect_origin_distance(
            cap,
            bottle,
            axes="xy",
            max_dist=0.001,
            name="rotated cap stays coaxial with the neck",
        )

    return ctx.report()


object_model = build_object_model()
