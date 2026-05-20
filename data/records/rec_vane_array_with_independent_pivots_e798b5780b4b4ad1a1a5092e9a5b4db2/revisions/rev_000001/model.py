from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


FRAME_WIDTH = 0.780
FRAME_HEIGHT = 0.560
FRAME_DEPTH = 0.110
SIDE_RAIL_WIDTH = 0.055
CROSS_RAIL_HEIGHT = 0.050
BLADE_COUNT = 6
BLADE_PITCH = 0.076
BLADE_LENGTH = 0.592
BLADE_LIMIT = 0.62


def _annular_sleeve_x(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    segments: int = 36,
) -> MeshGeometry:
    """A hollow bearing sleeve with its bore running along local +X."""

    geom = MeshGeometry()
    outer_left = []
    outer_right = []
    inner_left = []
    inner_right = []
    x0 = -length / 2.0
    x1 = length / 2.0

    for i in range(segments):
        a = 2.0 * pi * i / segments
        y = cos(a)
        z = sin(a)
        outer_left.append(geom.add_vertex(x0, outer_radius * y, outer_radius * z))
        outer_right.append(geom.add_vertex(x1, outer_radius * y, outer_radius * z))
        inner_left.append(geom.add_vertex(x0, inner_radius * y, inner_radius * z))
        inner_right.append(geom.add_vertex(x1, inner_radius * y, inner_radius * z))

    for i in range(segments):
        j = (i + 1) % segments
        # Outer cylindrical wall.
        geom.add_face(outer_left[i], outer_left[j], outer_right[j])
        geom.add_face(outer_left[i], outer_right[j], outer_right[i])
        # Inner bore wall.
        geom.add_face(inner_left[j], inner_left[i], inner_right[i])
        geom.add_face(inner_left[j], inner_right[i], inner_right[j])
        # End annuli.
        geom.add_face(outer_left[j], outer_left[i], inner_left[i])
        geom.add_face(outer_left[j], inner_left[i], inner_left[j])
        geom.add_face(outer_right[i], outer_right[j], inner_right[j])
        geom.add_face(outer_right[i], inner_right[j], inner_right[i])

    return geom


def _blade_body_x(length: float) -> MeshGeometry:
    """Extruded formed shutter blade with rolled lips and a cambered section."""

    # Points are a closed Y/Z section.  The pivot axis is at (Y=0, Z=0).
    profile = [
        (-0.052, -0.002),
        (-0.046, 0.006),
        (-0.030, 0.011),
        (-0.006, 0.013),
        (0.024, 0.011),
        (0.048, 0.005),
        (0.054, 0.000),
        (0.047, -0.005),
        (0.022, -0.010),
        (-0.010, -0.011),
        (-0.036, -0.008),
        (-0.051, -0.004),
    ]

    geom = MeshGeometry()
    left = []
    right = []
    x0 = -length / 2.0
    x1 = length / 2.0
    for y, z in profile:
        left.append(geom.add_vertex(x0, y, z))
        right.append(geom.add_vertex(x1, y, z))

    n = len(profile)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(left[i], left[j], right[j])
        geom.add_face(left[i], right[j], right[i])

    # Convex-ish end caps, triangulated as fans.
    for i in range(1, n - 1):
        geom.add_face(left[0], left[i], left[i + 1])
        geom.add_face(right[0], right[i + 1], right[i])

    return geom


def _blade_z_positions() -> list[float]:
    first = -0.5 * (BLADE_COUNT - 1) * BLADE_PITCH
    return [first + i * BLADE_PITCH for i in range(BLADE_COUNT)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ventilation_shutter_module")

    frame_mat = Material("dark_powder_coated_steel", color=(0.11, 0.12, 0.12, 1.0))
    stiffener_mat = Material("slightly_worn_edges", color=(0.16, 0.17, 0.16, 1.0))
    blade_mat = Material("satin_aluminum_blades", color=(0.72, 0.76, 0.76, 1.0))
    boss_mat = Material("cast_zinc_pivot_bosses", color=(0.54, 0.57, 0.56, 1.0))
    bushing_mat = Material("black_nylon_bearing_sleeves", color=(0.015, 0.014, 0.013, 1.0))
    stop_mat = Material("matte_rubber_stop_pads", color=(0.025, 0.025, 0.022, 1.0))

    frame = model.part("frame")

    rail_x = FRAME_WIDTH / 2.0 - SIDE_RAIL_WIDTH / 2.0
    rail_size = (SIDE_RAIL_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)
    for side, sx, rail_name in (
        (0, -rail_x, "side_rail_0"),
        (1, rail_x, "side_rail_1"),
    ):
        frame.visual(
            Box(rail_size),
            origin=Origin(xyz=(sx, 0.0, 0.0)),
            material=frame_mat,
            name=rail_name,
        )
        # Pressed stiffening beads along the exposed front and rear faces.
        for face, y in enumerate((-FRAME_DEPTH / 2.0 - 0.004, FRAME_DEPTH / 2.0 + 0.004)):
            frame.visual(
                Box((0.014, 0.010, FRAME_HEIGHT - 0.075)),
                origin=Origin(xyz=(sx, y, 0.0)),
                material=stiffener_mat,
                name=f"rail_stiffener_{side}_{face}",
            )

    frame.visual(
        Box((FRAME_WIDTH, FRAME_DEPTH, CROSS_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT / 2.0 - CROSS_RAIL_HEIGHT / 2.0)),
        material=frame_mat,
        name="top_rail",
    )
    frame.visual(
        Box((FRAME_WIDTH, FRAME_DEPTH, CROSS_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, -FRAME_HEIGHT / 2.0 + CROSS_RAIL_HEIGHT / 2.0)),
        material=frame_mat,
        name="bottom_rail",
    )
    # Narrow return flanges that make the module read as a formed sheet-metal frame.
    frame.visual(
        Box((FRAME_WIDTH - 0.080, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -FRAME_DEPTH / 2.0 - 0.006, FRAME_HEIGHT / 2.0 - 0.072)),
        material=stiffener_mat,
        name="top_return_lip",
    )
    frame.visual(
        Box((FRAME_WIDTH - 0.080, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -FRAME_DEPTH / 2.0 - 0.006, -FRAME_HEIGHT / 2.0 + 0.072)),
        material=stiffener_mat,
        name="bottom_return_lip",
    )

    sleeve_mesh = _annular_sleeve_x(0.019, 0.010, 0.022)
    sleeve_x = FRAME_WIDTH / 2.0 - SIDE_RAIL_WIDTH - 0.009
    for blade_index, z in enumerate(_blade_z_positions()):
        for side, sx in enumerate((-sleeve_x, sleeve_x)):
            frame.visual(
                mesh_from_geometry(sleeve_mesh, f"bearing_sleeve_mesh_{blade_index}_{side}"),
                origin=Origin(xyz=(sx, 0.0, z)),
                material=bushing_mat,
                name=f"bearing_sleeve_{blade_index}_{side}",
            )
            # Two small saddle ribs tie each sleeve into the rail without filling the bore.
            for rib, rz in enumerate((-0.026, 0.026)):
                frame.visual(
                    Box((0.020, 0.020, 0.008)),
                    origin=Origin(xyz=(sx, 0.0, z + rz)),
                    material=frame_mat,
                    name=f"sleeve_saddle_{blade_index}_{side}_{rib}",
                )

        # Low stop pads live behind the rotation envelope and show practical travel stops.
        for side, sx in ((0, -sleeve_x - 0.004), (1, sleeve_x + 0.004)):
            frame.visual(
                Box((0.014, 0.018, 0.011)),
                origin=Origin(xyz=(sx, -0.047, z - 0.034)),
                material=stop_mat,
                name=f"stop_pad_{blade_index}_{side}",
            )

    blade_body_mesh = _blade_body_x(BLADE_LENGTH)
    boss_x = BLADE_LENGTH / 2.0 + 0.006
    shaft_x = BLADE_LENGTH / 2.0 + 0.024

    for blade_index, z in enumerate(_blade_z_positions()):
        blade = model.part(f"blade_{blade_index}")
        blade.visual(
            mesh_from_geometry(blade_body_mesh, f"formed_blade_mesh_{blade_index}"),
            material=blade_mat,
            name="formed_blade",
        )
        for side, sign, boss_name, shaft_name in (
            (0, -1.0, "pivot_boss_0", "pivot_shaft_0"),
            (1, 1.0, "pivot_boss_1", "pivot_shaft_1"),
        ):
            blade.visual(
                Cylinder(radius=0.018, length=0.016),
                origin=Origin(xyz=(sign * boss_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
                material=boss_mat,
                name=boss_name,
            )
            blade.visual(
                Cylinder(radius=0.0105, length=0.028),
                origin=Origin(xyz=(sign * shaft_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
                material=boss_mat,
                name=shaft_name,
            )

        model.articulation(
            f"blade_pivot_{blade_index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=blade,
            origin=Origin(xyz=(0.0, 0.0, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.6, velocity=2.5, lower=-BLADE_LIMIT, upper=BLADE_LIMIT),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    blades = [object_model.get_part(f"blade_{i}") for i in range(BLADE_COUNT)]
    joints = [object_model.get_articulation(f"blade_pivot_{i}") for i in range(BLADE_COUNT)]

    ctx.check(
        "six independent shutter blades",
        len(blades) == BLADE_COUNT and len(joints) == BLADE_COUNT,
        details=f"blades={len(blades)}, joints={len(joints)}",
    )

    # Each blade has two coaxial shafts retained inside the repeated sleeve pattern,
    # with small end clearances to the solid side rails.
    for i, blade in enumerate(blades):
        ctx.allow_overlap(
            blade,
            frame,
            elem_a="pivot_shaft_0",
            elem_b=f"bearing_sleeve_{i}_0",
            reason="The blade trunnion is intentionally captured in the nylon sleeve bore with a tiny interference proxy so the revolute support is grounded.",
        )
        ctx.allow_overlap(
            blade,
            frame,
            elem_a="pivot_shaft_1",
            elem_b=f"bearing_sleeve_{i}_1",
            reason="The blade trunnion is intentionally captured in the nylon sleeve bore with a tiny interference proxy so the revolute support is grounded.",
        )
        ctx.expect_overlap(
            blade,
            frame,
            axes="x",
            elem_a="pivot_shaft_0",
            elem_b=f"bearing_sleeve_{i}_0",
            min_overlap=0.014,
            name=f"blade_{i} shaft_0 retained in sleeve",
        )
        ctx.expect_overlap(
            blade,
            frame,
            axes="x",
            elem_a="pivot_shaft_1",
            elem_b=f"bearing_sleeve_{i}_1",
            min_overlap=0.014,
            name=f"blade_{i} shaft_1 retained in sleeve",
        )
        ctx.expect_within(
            blade,
            frame,
            axes="yz",
            inner_elem="pivot_shaft_0",
            outer_elem=f"bearing_sleeve_{i}_0",
            margin=0.0,
            name=f"blade_{i} shaft_0 centered in sleeve",
        )
        ctx.expect_within(
            blade,
            frame,
            axes="yz",
            inner_elem="pivot_shaft_1",
            outer_elem=f"bearing_sleeve_{i}_1",
            margin=0.0,
            name=f"blade_{i} shaft_1 centered in sleeve",
        )
        ctx.expect_gap(
            blade,
            frame,
            axis="x",
            positive_elem="pivot_shaft_0",
            negative_elem="side_rail_0",
            min_gap=0.0005,
            name=f"blade_{i} shaft clears side_rail_0",
        )
        ctx.expect_gap(
            frame,
            blade,
            axis="x",
            positive_elem="side_rail_1",
            negative_elem="pivot_shaft_1",
            min_gap=0.0005,
            name=f"blade_{i} shaft clears side_rail_1",
        )

    for pose_value, label in ((BLADE_LIMIT, "upper"), (-BLADE_LIMIT, "lower")):
        with ctx.pose({joint: pose_value for joint in joints}):
            for lower_index in range(BLADE_COUNT - 1):
                ctx.expect_gap(
                    blades[lower_index + 1],
                    blades[lower_index],
                    axis="z",
                    min_gap=0.003,
                    name=f"{label} travel gap blade_{lower_index}_to_{lower_index + 1}",
                )
            ctx.expect_gap(
                frame,
                blades[-1],
                axis="z",
                positive_elem="top_rail",
                negative_elem="formed_blade",
                min_gap=0.006,
                name=f"{label} travel clears top rail",
            )
            ctx.expect_gap(
                blades[0],
                frame,
                axis="z",
                positive_elem="formed_blade",
                negative_elem="bottom_rail",
                min_gap=0.006,
                name=f"{label} travel clears bottom rail",
            )

    # Adjacent independently-driven vanes must also clear when they are driven
    # to opposite limits, which is the tightest practical louver pattern.
    with ctx.pose({joint: (BLADE_LIMIT if i % 2 else -BLADE_LIMIT) for i, joint in enumerate(joints)}):
        for lower_index in range(BLADE_COUNT - 1):
            ctx.expect_gap(
                blades[lower_index + 1],
                blades[lower_index],
                axis="z",
                positive_elem="formed_blade",
                negative_elem="formed_blade",
                min_gap=0.006,
                name=f"alternating travel gap blade_{lower_index}_to_{lower_index + 1}",
            )

    return ctx.report()


object_model = build_object_model()
