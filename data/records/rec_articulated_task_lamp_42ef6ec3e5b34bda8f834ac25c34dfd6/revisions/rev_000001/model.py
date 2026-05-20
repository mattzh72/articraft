from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


POST_RADIUS = 0.028
COLLAR_OUTER_RADIUS = 0.039
COLLAR_INNER_RADIUS = POST_RADIUS
COLLAR_HEIGHT = 0.046
ARM_SWING_LIMIT = math.radians(150.0)
SHADE_TILT_LIMIT = math.radians(42.0)


def _ring_points(
    *,
    axis: str,
    coord: float,
    radius: float,
    segments: int,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for index in range(segments):
        angle = math.tau * index / segments
        c = math.cos(angle)
        s = math.sin(angle)
        if axis == "z":
            points.append((radius * c, radius * s, coord))
        elif axis == "x":
            points.append((coord, radius * c, radius * s))
        else:
            raise ValueError(f"Unsupported ring axis: {axis}")
    return points


def _add_ring(
    geom: MeshGeometry,
    *,
    axis: str,
    coord: float,
    radius: float,
    segments: int,
) -> list[int]:
    return [geom.add_vertex(x, y, z) for x, y, z in _ring_points(axis=axis, coord=coord, radius=radius, segments=segments)]


def _bridge_rings(geom: MeshGeometry, ring_a: list[int], ring_b: list[int], *, flip: bool = False) -> None:
    ring_count = len(ring_a)
    for index in range(ring_count):
        next_index = (index + 1) % ring_count
        a0 = ring_a[index]
        a1 = ring_a[next_index]
        b0 = ring_b[index]
        b1 = ring_b[next_index]
        if flip:
            geom.add_face(a0, b1, a1)
            geom.add_face(a0, b0, b1)
        else:
            geom.add_face(a0, a1, b1)
            geom.add_face(a0, b1, b0)


def _cylindrical_shell_mesh(
    *,
    axis: str,
    outer_radius: float,
    inner_radius: float,
    length: float,
    segments: int = 40,
) -> MeshGeometry:
    geom = MeshGeometry()
    half = length * 0.5
    outer_low = _add_ring(geom, axis=axis, coord=-half, radius=outer_radius, segments=segments)
    outer_high = _add_ring(geom, axis=axis, coord=half, radius=outer_radius, segments=segments)
    inner_low = _add_ring(geom, axis=axis, coord=-half, radius=inner_radius, segments=segments)
    inner_high = _add_ring(geom, axis=axis, coord=half, radius=inner_radius, segments=segments)

    _bridge_rings(geom, outer_low, outer_high)
    _bridge_rings(geom, inner_high, inner_low)
    _bridge_rings(geom, outer_high, inner_high)
    _bridge_rings(geom, inner_low, outer_low)
    return geom


def _conical_shade_shell_mesh(
    *,
    length: float,
    outer_tip_radius: float,
    outer_mouth_radius: float,
    wall_thickness: float,
    segments: int = 40,
) -> MeshGeometry:
    geom = MeshGeometry()
    inner_tip_radius = max(outer_tip_radius - wall_thickness, wall_thickness * 1.5)
    inner_mouth_radius = max(outer_mouth_radius - wall_thickness, inner_tip_radius + 0.002)

    outer_tip = _add_ring(geom, axis="x", coord=0.0, radius=outer_tip_radius, segments=segments)
    outer_mouth = _add_ring(geom, axis="x", coord=length, radius=outer_mouth_radius, segments=segments)
    inner_tip = _add_ring(geom, axis="x", coord=0.0, radius=inner_tip_radius, segments=segments)
    inner_mouth = _add_ring(geom, axis="x", coord=length, radius=inner_mouth_radius, segments=segments)

    _bridge_rings(geom, outer_tip, outer_mouth)
    _bridge_rings(geom, inner_mouth, inner_tip)
    _bridge_rings(geom, outer_tip, inner_tip)
    return geom


def _mesh(geometry: MeshGeometry, name: str):
    return mesh_from_geometry(geometry, name)


def _arm_specs() -> tuple[dict[str, float], ...]:
    return (
        {"height": 0.78, "azimuth": math.radians(18.0), "reach": 0.48, "rise": 0.10, "pitch": -0.78},
        {"height": 0.98, "azimuth": math.radians(92.0), "reach": 0.56, "rise": 0.17, "pitch": -0.68},
        {"height": 1.18, "azimuth": math.radians(165.0), "reach": 0.44, "rise": 0.08, "pitch": -0.84},
        {"height": 1.38, "azimuth": math.radians(236.0), "reach": 0.60, "rise": 0.15, "pitch": -0.72},
        {"height": 1.56, "azimuth": math.radians(314.0), "reach": 0.52, "rise": 0.12, "pitch": -0.62},
    )


def _build_arm_mesh(*, reach: float, rise: float) -> MeshGeometry:
    geom = MeshGeometry()
    geom.merge(BoxGeometry((0.055, 0.030, 0.030)).translate(0.064, 0.0, 0.0))
    geom.merge(CylinderGeometry(radius=0.018, height=0.034, radial_segments=24).rotate_y(math.pi / 2.0).translate(0.093, 0.0, 0.0))

    tube_end_x = reach - 0.070
    arm_tube = tube_from_spline_points(
        [
            (0.092, 0.0, 0.0),
            (0.170, 0.0, 0.016),
            (reach * 0.44, 0.0, rise * 0.45),
            (reach * 0.74, 0.0, rise * 0.82),
            (tube_end_x, 0.0, rise),
        ],
        radius=0.012,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    geom.merge(arm_tube)

    geom.merge(
        BoxGeometry((0.050, 0.024, 0.018))
        .translate(reach - 0.045, 0.0, rise)
    )
    geom.merge(
        BoxGeometry((0.024, 0.008, 0.026)).translate(reach - 0.019, 0.016, rise)
    )
    geom.merge(BoxGeometry((0.024, 0.008, 0.026)).translate(reach - 0.019, -0.016, rise))
    return geom


def _build_shade_mesh() -> MeshGeometry:
    shade_length = 0.190
    shell = _conical_shade_shell_mesh(
        length=shade_length,
        outer_tip_radius=0.028,
        outer_mouth_radius=0.078,
        wall_thickness=0.0035,
        segments=42,
    )
    shell.translate(0.032, 0.0, 0.0)

    geom = MeshGeometry()
    geom.merge(BoxGeometry((0.026, 0.024, 0.018)).translate(0.013, 0.0, 0.0))
    geom.merge(CylinderGeometry(radius=0.026, height=0.036, radial_segments=24).rotate_y(math.pi / 2.0).translate(0.033, 0.0, 0.0))
    geom.merge(shell)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tree_floor_lamp")

    matte_black = model.material("matte_black", rgba=(0.16, 0.15, 0.15, 1.0))
    satin_brass = model.material("satin_brass", rgba=(0.68, 0.57, 0.38, 1.0))
    warm_cream = model.material("warm_cream", rgba=(0.92, 0.90, 0.84, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.180, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=matte_black,
        name="base_disc",
    )
    stand.visual(
        Cylinder(radius=0.055, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=matte_black,
        name="base_hub",
    )
    stand.visual(
        Cylinder(radius=POST_RADIUS, length=1.620),
        origin=Origin(xyz=(0.0, 0.0, 0.845)),
        material=satin_brass,
        name="post",
    )
    stand.visual(
        Cylinder(radius=0.034, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 1.680)),
        material=satin_brass,
        name="top_cap",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.40, 0.40, 1.74)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.87)),
    )

    for index, spec in enumerate(_arm_specs(), start=1):
        arm = model.part(f"arm_{index}")
        arm.visual(
            _mesh(_build_arm_mesh(reach=spec["reach"], rise=spec["rise"]), f"tree_lamp_arm_{index}"),
            material=satin_brass,
            name="arm_structure",
        )
        arm.visual(
            Box((0.050, 0.060, COLLAR_HEIGHT)),
            origin=Origin(xyz=(POST_RADIUS + 0.025, 0.0, 0.0)),
            material=matte_black,
            name="collar_shell",
        )
        arm.inertial = Inertial.from_geometry(
            Box((spec["reach"] + 0.08, 0.10, spec["rise"] + 0.10)),
            mass=1.8,
            origin=Origin(xyz=(spec["reach"] * 0.48, 0.0, spec["rise"] * 0.5)),
        )

        shade = model.part(f"shade_{index}")
        shade.visual(
            Cylinder(radius=0.007, length=0.008),
            origin=Origin(xyz=(0.0, 0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_brass,
            name="left_trunnion",
        )
        shade.visual(
            Cylinder(radius=0.007, length=0.008),
            origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_brass,
            name="right_trunnion",
        )
        shade.visual(
            Box((0.024, 0.024, 0.018)),
            origin=Origin(xyz=(0.014, 0.0, 0.0)),
            material=satin_brass,
            name="pivot_block",
        )
        shade.visual(
            _mesh(_build_shade_mesh(), f"tree_lamp_shade_{index}"),
            origin=Origin(xyz=(0.024, 0.0, 0.0), rpy=(0.0, spec["pitch"], 0.0)),
            material=warm_cream,
            name="shade_shell",
        )
        shade.inertial = Inertial.from_geometry(
            Box((0.26, 0.18, 0.22)),
            mass=0.65,
            origin=Origin(xyz=(0.12, 0.0, -0.05)),
        )

        model.articulation(
            f"stand_to_arm_{index}",
            ArticulationType.REVOLUTE,
            parent=stand,
            child=arm,
            origin=Origin(xyz=(0.0, 0.0, spec["height"]), rpy=(0.0, 0.0, spec["azimuth"])),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=1.6,
                lower=-ARM_SWING_LIMIT,
                upper=ARM_SWING_LIMIT,
            ),
        )
        model.articulation(
            f"arm_{index}_to_shade_{index}",
            ArticulationType.REVOLUTE,
            parent=arm,
            child=shade,
            origin=Origin(xyz=(spec["reach"], 0.0, spec["rise"])),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=1.8,
                lower=-SHADE_TILT_LIMIT,
                upper=SHADE_TILT_LIMIT,
            ),
        )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return (
        0.5 * (lower[0] + upper[0]),
        0.5 * (lower[1] + upper[1]),
        0.5 * (lower[2] + upper[2]),
    )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")

    stand_box = ctx.part_world_aabb(stand)
    stand_ok = False
    if stand_box is not None:
        lower, upper = stand_box
        stand_ok = (
            1.65 <= upper[2] <= 1.75
            and 0.34 <= (upper[0] - lower[0]) <= 0.38
            and 0.34 <= (upper[1] - lower[1]) <= 0.38
        )
    ctx.check("stand proportions read as a tall floor lamp", stand_ok, details=f"stand_aabb={stand_box}")

    for index in range(1, 6):
        arm = object_model.get_part(f"arm_{index}")
        shade = object_model.get_part(f"shade_{index}")
        arm_joint = object_model.get_articulation(f"stand_to_arm_{index}")
        shade_joint = object_model.get_articulation(f"arm_{index}_to_shade_{index}")

        ctx.expect_contact(
            arm,
            stand,
            elem_a="collar_shell",
            elem_b="post",
            contact_tol=0.001,
            name=f"arm {index} collar is mounted against the post",
        )
        ctx.expect_contact(
            arm,
            shade,
            contact_tol=0.001,
            name=f"arm {index} supports shade {index} at the pivot",
        )

        rest_pos = None
        swung_pos = None
        with ctx.pose({arm_joint: 0.0}):
            rest_pos = ctx.part_world_position(shade)
        with ctx.pose({arm_joint: 0.85}):
            swung_pos = ctx.part_world_position(shade)

        arm_moves_ok = False
        if rest_pos is not None and swung_pos is not None:
            dx = swung_pos[0] - rest_pos[0]
            dy = swung_pos[1] - rest_pos[1]
            dz = swung_pos[2] - rest_pos[2]
            arm_moves_ok = math.hypot(dx, dy) > 0.12 and abs(dz) < 0.04
        ctx.check(
            f"arm {index} revolves around the post",
            arm_moves_ok,
            details=f"rest={rest_pos}, swung={swung_pos}",
        )

        low_aabb = None
        high_aabb = None
        with ctx.pose({shade_joint: -0.60}):
            low_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
        with ctx.pose({shade_joint: 0.60}):
            high_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")

        low_center = _aabb_center(low_aabb)
        high_center = _aabb_center(high_aabb)
        shade_moves_ok = False
        if low_center is not None and high_center is not None:
            delta = math.dist(low_center, high_center)
            shade_moves_ok = delta > 0.06
        ctx.check(
            f"shade {index} tilts through a visible arc",
            shade_moves_ok,
            details=f"low_center={low_center}, high_center={high_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
