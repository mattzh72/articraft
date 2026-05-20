from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


BELL_SPECS = (
    # name suffix, x position in tower bay, mouth radius, shell height
    (0, -0.72, 0.34, 0.68),
    (1, 0.00, 0.285, 0.58),
    (2, 0.62, 0.235, 0.49),
)


def _tube_mesh(points, radius: float, name: str, *, segments: int = 14):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=2,
            radial_segments=segments,
            cap_ends=True,
        ),
        name,
    )


def _bell_shell_mesh(mouth_radius: float, height: float, name: str):
    top_z = -0.17
    lip_z = top_z - height
    outer_profile = [
        (mouth_radius * 0.98, lip_z),
        (mouth_radius * 1.06, lip_z + 0.035),
        (mouth_radius * 0.95, lip_z + 0.105),
        (mouth_radius * 0.72, lip_z + height * 0.38),
        (mouth_radius * 0.50, lip_z + height * 0.66),
        (mouth_radius * 0.31, lip_z + height * 0.86),
        (mouth_radius * 0.18, top_z + 0.012),
        (mouth_radius * 0.10, top_z + 0.035),
    ]
    inner_profile = [
        (mouth_radius * 0.82, lip_z + 0.055),
        (mouth_radius * 0.73, lip_z + 0.125),
        (mouth_radius * 0.54, lip_z + height * 0.42),
        (mouth_radius * 0.34, lip_z + height * 0.69),
        (mouth_radius * 0.18, lip_z + height * 0.88),
        (mouth_radius * 0.075, top_z + 0.002),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="round",
            end_cap="flat",
            lip_samples=8,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="steel_frame_carillon_bell_tower")

    galvanized = model.material("galvanized_steel", rgba=(0.52, 0.55, 0.57, 1.0))
    dark_steel = model.material("dark_bearing_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    bronze = model.material("aged_bell_bronze", rgba=(0.72, 0.48, 0.20, 1.0))
    dark_bronze = model.material("dark_bronze", rgba=(0.25, 0.16, 0.08, 1.0))
    concrete = model.material("weathered_concrete", rgba=(0.45, 0.46, 0.45, 1.0))

    frame = model.part("frame")

    # Concrete footings anchor an open bolted steel tower.
    for ix, x in enumerate((-1.18, 1.18)):
        for iy, y in enumerate((-0.58, 0.58)):
            frame.visual(
                Box((0.34, 0.28, 0.10)),
                origin=Origin(xyz=(x, y, 0.05)),
                material=concrete,
                name=f"footing_{ix}_{iy}",
            )
            frame.visual(
                Box((0.18, 0.16, 0.035)),
                origin=Origin(xyz=(x, y, 0.1175)),
                material=dark_steel,
                name=f"base_plate_{ix}_{iy}",
            )

    # Four columns and the principal rectangular tower bays.
    for ix, x in enumerate((-1.18, 1.18)):
        for iy, y in enumerate((-0.58, 0.58)):
            frame.visual(
                Cylinder(radius=0.045, length=3.72),
                origin=Origin(xyz=(x, y, 1.98)),
                material=galvanized,
                name=f"column_{ix}_{iy}",
            )

    for z, label, radius in ((0.30, "bottom", 0.032), (1.65, "middle", 0.026), (3.10, "bell", 0.04), (3.80, "top", 0.04)):
        frame.visual(
            Cylinder(radius=radius, length=2.46),
            origin=Origin(xyz=(0.0, -0.58, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=galvanized,
            name=f"{label}_front_rail",
        )
        frame.visual(
            Cylinder(radius=radius, length=2.46),
            origin=Origin(xyz=(0.0, 0.58, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=galvanized,
            name=f"{label}_rear_rail",
        )
        frame.visual(
            Cylinder(radius=radius, length=1.24),
            origin=Origin(xyz=(-1.18, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"{label}_left_side_rail",
        )
        frame.visual(
            Cylinder(radius=radius, length=1.24),
            origin=Origin(xyz=(1.18, 0.0, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"{label}_right_side_rail",
        )

    # Lattice diagonals in each face give the tower the open truss silhouette.
    diag_specs = [
        ((-1.18, -0.58, 0.30), (1.18, -0.58, 1.65), "front_low_a"),
        ((1.18, -0.58, 0.30), (-1.18, -0.58, 1.65), "front_low_b"),
        ((-1.18, -0.58, 1.65), (1.18, -0.58, 3.10), "front_high_a"),
        ((1.18, -0.58, 1.65), (-1.18, -0.58, 3.10), "front_high_b"),
        ((-1.18, 0.58, 0.30), (1.18, 0.58, 1.65), "rear_low_a"),
        ((1.18, 0.58, 0.30), (-1.18, 0.58, 1.65), "rear_low_b"),
        ((-1.18, 0.58, 1.65), (1.18, 0.58, 3.10), "rear_high_a"),
        ((1.18, 0.58, 1.65), (-1.18, 0.58, 3.10), "rear_high_b"),
        ((-1.18, -0.58, 0.30), (-1.18, 0.58, 1.65), "left_low_a"),
        ((-1.18, 0.58, 0.30), (-1.18, -0.58, 1.65), "left_low_b"),
        ((1.18, -0.58, 0.30), (1.18, 0.58, 1.65), "right_low_a"),
        ((1.18, 0.58, 0.30), (1.18, -0.58, 1.65), "right_low_b"),
        ((-1.18, -0.58, 1.65), (-1.18, 0.58, 3.10), "left_high_a"),
        ((-1.18, 0.58, 1.65), (-1.18, -0.58, 3.10), "left_high_b"),
        ((1.18, -0.58, 1.65), (1.18, 0.58, 3.10), "right_high_a"),
        ((1.18, 0.58, 1.65), (1.18, -0.58, 3.10), "right_high_b"),
    ]
    for start, end, name in diag_specs:
        frame.visual(
            _tube_mesh([start, end], 0.018, f"lattice_{name}"),
            material=galvanized,
            name=f"lattice_{name}",
        )

    # Bearing blocks are welded to the bell cross-beam; the yoke axles seat in them.
    for idx, x, _mouth, _height in BELL_SPECS:
        frame.visual(
            Box((0.20, 0.09, 0.18)),
            origin=Origin(xyz=(x, -0.515, 3.10)),
            material=dark_steel,
            name=f"front_bearing_{idx}",
        )
        frame.visual(
            Box((0.20, 0.09, 0.18)),
            origin=Origin(xyz=(x, 0.515, 3.10)),
            material=dark_steel,
            name=f"rear_bearing_{idx}",
        )
        frame.visual(
            Box((0.10, 0.05, 0.25)),
            origin=Origin(xyz=(x, -0.515, 3.275)),
            material=galvanized,
            name=f"front_hanger_{idx}",
        )
        frame.visual(
            Box((0.10, 0.05, 0.25)),
            origin=Origin(xyz=(x, 0.515, 3.275)),
            material=galvanized,
            name=f"rear_hanger_{idx}",
        )

    for idx, x, mouth_radius, height in BELL_SPECS:
        bell = model.part(f"bell_{idx}")
        bell.visual(
            Cylinder(radius=0.036, length=1.00),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="axle",
        )
        bell.visual(
            Box((0.26, 0.70, 0.10)),
            origin=Origin(xyz=(0.0, 0.0, -0.015)),
            material=dark_steel,
            name="yoke",
        )
        bell.visual(
            Box((0.075, 0.12, 0.22)),
            origin=Origin(xyz=(-0.075, 0.0, -0.125)),
            material=dark_steel,
            name="side_strap_0",
        )
        bell.visual(
            Box((0.075, 0.12, 0.22)),
            origin=Origin(xyz=(0.075, 0.0, -0.125)),
            material=dark_steel,
            name="side_strap_1",
        )
        bell.visual(
            Cylinder(radius=0.095, length=0.11),
            origin=Origin(xyz=(0.0, 0.0, -0.170)),
            material=dark_bronze,
            name="crown",
        )
        bell.visual(
            _bell_shell_mesh(mouth_radius, height, f"bell_shell_{idx}"),
            material=bronze,
            name="bell_shell",
        )
        bell.visual(
            mesh_from_geometry(
                TorusGeometry(
                    radius=mouth_radius * 0.92,
                    tube=0.022,
                    radial_segments=18,
                    tubular_segments=72,
                ),
                f"bell_lip_{idx}",
            ),
            origin=Origin(xyz=(0.0, 0.0, -0.17 - height + 0.035)),
            material=bronze,
            name="mouth_lip",
        )

        model.articulation(
            f"frame_to_bell_{idx}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=bell,
            origin=Origin(xyz=(x, 0.0, 3.10)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=120.0,
                velocity=1.2,
                lower=-math.radians(28.0),
                upper=math.radians(28.0),
            ),
        )

        clapper = model.part(f"clapper_{idx}")
        rod_len = height * 0.64
        ball_radius = mouth_radius * 0.16
        clapper.visual(
            Cylinder(radius=0.012, length=0.18),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="pin",
        )
        clapper.visual(
            Cylinder(radius=0.011, length=rod_len),
            origin=Origin(xyz=(0.0, 0.0, -rod_len / 2.0)),
            material=dark_steel,
            name="rod",
        )
        clapper.visual(
            Sphere(radius=ball_radius),
            origin=Origin(xyz=(0.0, 0.0, -rod_len - ball_radius * 0.15)),
            material=dark_bronze,
            name="ball",
        )

        model.articulation(
            f"bell_to_clapper_{idx}",
            ArticulationType.REVOLUTE,
            parent=bell,
            child=clapper,
            origin=Origin(xyz=(0.0, 0.0, -0.285)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=3.0,
                lower=-math.radians(38.0),
                upper=math.radians(38.0),
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")

    ctx.check(
        "three graduated bells",
        len(BELL_SPECS) == 3 and BELL_SPECS[0][2] > BELL_SPECS[1][2] > BELL_SPECS[2][2],
        details=str(BELL_SPECS),
    )

    for idx, _x, _mouth_radius, _height in BELL_SPECS:
        bell = object_model.get_part(f"bell_{idx}")
        clapper = object_model.get_part(f"clapper_{idx}")
        bell_joint = object_model.get_articulation(f"frame_to_bell_{idx}")
        clapper_joint = object_model.get_articulation(f"bell_to_clapper_{idx}")

        for bearing_name in (f"front_bearing_{idx}", f"rear_bearing_{idx}"):
            ctx.allow_overlap(
                frame,
                bell,
                elem_a=bearing_name,
                elem_b="axle",
                reason="The yoke axle is intentionally captured inside the steel bearing block at the frame cross-beam.",
            )
            ctx.expect_overlap(
                bell,
                frame,
                axes="xyz",
                elem_a="axle",
                elem_b=bearing_name,
                min_overlap=0.004,
                name=f"axle seated in {bearing_name}",
            )

        ctx.expect_within(
            clapper,
            bell,
            axes="xy",
            inner_elem="ball",
            outer_elem="bell_shell",
            margin=0.01,
            name=f"clapper ball hangs inside bell {idx}",
        )
        ctx.allow_overlap(
            bell,
            clapper,
            elem_a="bell_shell",
            elem_b="pin",
            reason="The clapper suspension pin is intentionally captured through the upper bell casting.",
        )
        ctx.expect_overlap(
            bell,
            clapper,
            axes="xyz",
            elem_a="bell_shell",
            elem_b="pin",
            min_overlap=0.004,
            name=f"clapper pin captured in bell {idx}",
        )

        def elem_center(part, elem):
            bounds = ctx.part_element_world_aabb(part, elem=elem)
            if bounds is None:
                return None
            lo, hi = bounds
            return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

        rest_center = elem_center(clapper, "ball")
        with ctx.pose({clapper_joint: math.radians(26.0)}):
            swung_center = elem_center(clapper, "ball")
            ctx.expect_within(
                clapper,
                bell,
                axes="xy",
                inner_elem="ball",
                outer_elem="bell_shell",
                margin=0.035,
                name=f"swung clapper remains in bell {idx}",
            )
        ctx.check(
            f"clapper {idx} articulates on secondary pin",
            rest_center is not None
            and swung_center is not None
            and abs(swung_center[0] - rest_center[0]) > 0.04,
            details=f"rest={rest_center}, swung={swung_center}",
        )

        with ctx.pose({bell_joint: math.radians(20.0)}):
            ctx.expect_within(
                bell,
                frame,
                axes="x",
                inner_elem="bell_shell",
                margin=0.06,
                name=f"bell {idx} swings within tower width",
            )

    return ctx.report()


object_model = build_object_model()
