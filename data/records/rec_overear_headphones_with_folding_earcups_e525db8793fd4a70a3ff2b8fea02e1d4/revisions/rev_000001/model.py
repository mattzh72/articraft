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


def _signed_power(value: float, power: float) -> float:
    return (1.0 if value >= 0.0 else -1.0) * (abs(value) ** power)


def _superellipse_loop_yz(
    x: float,
    radius_y: float,
    radius_z: float,
    *,
    exponent: float = 2.7,
    segments: int = 48,
) -> list[tuple[float, float, float]]:
    """Closed oval loop in the YZ plane, centered on the X axis."""
    power = 2.0 / exponent
    return [
        (
            x,
            radius_y * _signed_power(cos(2.0 * pi * i / segments), power),
            radius_z * _signed_power(sin(2.0 * pi * i / segments), power),
        )
        for i in range(segments)
    ]


def _add_loop(geom: MeshGeometry, loop: list[tuple[float, float, float]]) -> list[int]:
    return [geom.add_vertex(x, y, z) for x, y, z in loop]


def _connect_loops(geom: MeshGeometry, first: list[int], second: list[int]) -> None:
    count = len(first)
    for i in range(count):
        j = (i + 1) % count
        geom.add_face(first[i], second[i], second[j])
        geom.add_face(first[i], second[j], first[j])


def _cap_loop(geom: MeshGeometry, loop: list[int], *, flip: bool = False) -> None:
    center = geom.add_vertex(0.0, 0.0, 0.0)
    # Move center to the average of the loop vertices.
    vertices = geom.vertices
    cx = sum(vertices[i][0] for i in loop) / len(loop)
    cy = sum(vertices[i][1] for i in loop) / len(loop)
    cz = sum(vertices[i][2] for i in loop) / len(loop)
    vertices[center] = (cx, cy, cz)
    for i in range(len(loop)):
        j = (i + 1) % len(loop)
        if flip:
            geom.add_face(center, loop[j], loop[i])
        else:
            geom.add_face(center, loop[i], loop[j])


def _oval_solid_x(
    thickness: float,
    radius_y: float,
    radius_z: float,
    *,
    center_x: float = 0.0,
    bulge: float = 0.0,
    segments: int = 56,
) -> MeshGeometry:
    """Pillow-like superellipse solid whose main axis is X."""
    geom = MeshGeometry()
    stations = [
        (-0.5 * thickness, 0.94),
        (-0.22 * thickness, 1.0 + bulge),
        (0.22 * thickness, 1.0 + bulge),
        (0.5 * thickness, 0.94),
    ]
    loops: list[list[int]] = []
    for x, scale in stations:
        loops.append(
            _add_loop(
                geom,
                _superellipse_loop_yz(
                    center_x + x,
                    radius_y * scale,
                    radius_z * scale,
                    segments=segments,
                ),
            )
        )
    for a, b in zip(loops, loops[1:]):
        _connect_loops(geom, a, b)
    _cap_loop(geom, loops[0], flip=True)
    _cap_loop(geom, loops[-1], flip=False)
    return geom


def _oval_ring_x(
    inner_side: int,
    shell_half_thickness: float,
    pad_thickness: float,
    *,
    outer_y: float,
    outer_z: float,
    inner_y: float,
    inner_z: float,
    segments: int = 56,
) -> MeshGeometry:
    """Soft raised oval earpad ring extruded along X."""
    geom = MeshGeometry()
    x0 = inner_side * (shell_half_thickness - 0.001)
    x1 = inner_side * (shell_half_thickness + pad_thickness)
    outer0 = _add_loop(geom, _superellipse_loop_yz(x0, outer_y, outer_z, segments=segments))
    outer1 = _add_loop(geom, _superellipse_loop_yz(x1, outer_y, outer_z, segments=segments))
    inner0 = _add_loop(geom, _superellipse_loop_yz(x0, inner_y, inner_z, segments=segments))
    inner1 = _add_loop(geom, _superellipse_loop_yz(x1, inner_y, inner_z, segments=segments))

    _connect_loops(geom, outer0, outer1)
    # Reverse the inner wall so its normals face into the ear opening.
    _connect_loops(geom, inner1, inner0)
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(outer1[i], inner1[i], inner1[j])
        geom.add_face(outer1[i], inner1[j], outer1[j])
        geom.add_face(outer0[j], inner0[j], inner0[i])
        geom.add_face(outer0[j], inner0[i], outer0[i])
    return geom


def _curved_band(
    points: list[tuple[float, float, float]],
    *,
    half_width_y: float,
    half_thickness: float,
) -> MeshGeometry:
    """Sweep a rounded-rectangle-like section along the headband arch."""
    geom = MeshGeometry()
    loops: list[list[int]] = []
    for index, point in enumerate(points):
        if index == 0:
            tangent = (
                points[1][0] - point[0],
                0.0,
                points[1][2] - point[2],
            )
        elif index == len(points) - 1:
            tangent = (
                point[0] - points[index - 1][0],
                0.0,
                point[2] - points[index - 1][2],
            )
        else:
            tangent = (
                points[index + 1][0] - points[index - 1][0],
                0.0,
                points[index + 1][2] - points[index - 1][2],
            )
        length = (tangent[0] ** 2 + tangent[2] ** 2) ** 0.5 or 1.0
        tx, tz = tangent[0] / length, tangent[2] / length
        # Normal in the XZ arch plane.
        nx, nz = -tz, tx
        x, y, z = point
        section = [
            (x - nx * half_thickness, y - half_width_y, z - nz * half_thickness),
            (x + nx * half_thickness, y - half_width_y, z + nz * half_thickness),
            (x + nx * half_thickness, y + half_width_y, z + nz * half_thickness),
            (x - nx * half_thickness, y + half_width_y, z - nz * half_thickness),
        ]
        loops.append(_add_loop(geom, section))
    for a, b in zip(loops, loops[1:]):
        _connect_loops(geom, a, b)
    _cap_loop(geom, loops[0], flip=True)
    _cap_loop(geom, loops[-1], flip=False)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_headphone")

    model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    model.material("soft_leather", rgba=(0.025, 0.024, 0.023, 1.0))
    model.material("dark_fabric", rgba=(0.035, 0.038, 0.042, 1.0))
    model.material("gunmetal", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("brushed_metal", rgba=(0.62, 0.64, 0.66, 1.0))
    model.material("rubber_mark", rgba=(0.08, 0.08, 0.075, 1.0))

    headband = model.part("headband")
    arch_points = [
        (-0.116, 0.0, 0.146),
        (-0.108, 0.0, 0.178),
        (-0.072, 0.0, 0.212),
        (0.0, 0.0, 0.238),
        (0.072, 0.0, 0.212),
        (0.108, 0.0, 0.178),
        (0.116, 0.0, 0.146),
    ]
    cushion_points = [
        (-0.082, 0.0, 0.167),
        (-0.055, 0.0, 0.208),
        (0.0, 0.0, 0.228),
        (0.055, 0.0, 0.208),
        (0.082, 0.0, 0.167),
    ]
    headband.visual(
        mesh_from_geometry(
            _curved_band(arch_points, half_width_y=0.018, half_thickness=0.0075),
            "outer_headband",
        ),
        material="matte_black",
        name="outer_band",
    )
    headband.visual(
        mesh_from_geometry(
            _curved_band(cushion_points, half_width_y=0.024, half_thickness=0.008),
            "headband_pad",
        ),
        material="soft_leather",
        name="head_pad",
    )
    for side, name_prefix in ((-1, "left"), (1, "right")):
        headband.visual(
            Box((0.034, 0.046, 0.084)),
            origin=Origin(xyz=(side * 0.112, 0.0, 0.108)),
            material="matte_black",
            name=f"{name_prefix}_socket",
        )
        headband.visual(
            Cylinder(radius=0.005, length=0.006),
            origin=Origin(xyz=(side * 0.112, -0.026, 0.120), rpy=(pi / 2.0, 0.0, 0.0)),
            material="brushed_metal",
            name=f"{name_prefix}_rivet",
        )

    sliders = {}
    yokes = {}
    cups = {}
    for side, side_name, inner_side in ((-1, "left", 1), (1, "right", -1)):
        slider = model.part(f"{side_name}_slider")
        slider.visual(
            Box((0.016, 0.012, 0.100)),
            origin=Origin(xyz=(0.0, 0.0, -0.025)),
            material="brushed_metal",
            name="rail",
        )
        slider.visual(
            Box((0.022, 0.018, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, -0.080)),
            material="gunmetal",
            name="fold_leaf",
        )
        slider.visual(
            Cylinder(radius=0.010, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, -0.078)),
            material="gunmetal",
            name="upper_knuckle",
        )
        for mark_z in (-0.006, -0.023, -0.040):
            slider.visual(
                Cylinder(radius=0.0025, length=0.0015),
                origin=Origin(xyz=(0.0, -0.0064, mark_z), rpy=(pi / 2.0, 0.0, 0.0)),
                material="rubber_mark",
                name=f"detent_{int(abs(mark_z) * 1000):02d}",
            )

        yoke = model.part(f"{side_name}_yoke")
        yoke.visual(
            Cylinder(radius=0.011, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, -0.008)),
            material="gunmetal",
            name="fold_barrel",
        )
        yoke.visual(
            Box((0.016, 0.024, 0.022)),
            origin=Origin(xyz=(0.0, 0.0, -0.012)),
            material="gunmetal",
            name="stem",
        )
        yoke.visual(
            Box((0.020, 0.118, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.027)),
            material="gunmetal",
            name="upper_bridge",
        )
        yoke.visual(
            Box((0.016, 0.010, 0.074)),
            origin=Origin(xyz=(0.0, 0.055, -0.060)),
            material="gunmetal",
            name="front_arm",
        )
        yoke.visual(
            Box((0.016, 0.010, 0.074)),
            origin=Origin(xyz=(0.0, -0.055, -0.060)),
            material="gunmetal",
            name="rear_arm",
        )
        for y_pos, visual_name in ((0.061, "front_boss"), (-0.061, "rear_boss")):
            yoke.visual(
                Cylinder(radius=0.012, length=0.008),
                origin=Origin(xyz=(0.0, y_pos, -0.090), rpy=(pi / 2.0, 0.0, 0.0)),
                material="gunmetal",
                name=visual_name,
            )

        cup = model.part(f"{side_name}_cup")
        cup.visual(
            mesh_from_geometry(
                _oval_solid_x(
                    0.042,
                    0.045,
                    0.054,
                    bulge=0.04,
                    segments=64,
                ),
                f"{side_name}_cup_shell",
            ),
            material="matte_black",
            name="shell",
        )
        cup.visual(
            mesh_from_geometry(
                _oval_ring_x(
                    inner_side,
                    shell_half_thickness=0.021,
                    pad_thickness=0.018,
                    outer_y=0.046,
                    outer_z=0.055,
                    inner_y=0.028,
                    inner_z=0.036,
                    segments=64,
                ),
                f"{side_name}_ear_pad",
            ),
            material="soft_leather",
            name="ear_pad",
        )
        cup.visual(
            mesh_from_geometry(
                _oval_solid_x(
                    0.004,
                    0.031,
                    0.039,
                    center_x=inner_side * 0.039,
                    segments=48,
                ),
                f"{side_name}_driver_cloth",
            ),
            material="dark_fabric",
            name="driver_cloth",
        )
        cup.visual(
            Cylinder(radius=0.006, length=0.1005),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material="brushed_metal",
            name="trunnion_pin",
        )
        cup.visual(
            Cylinder(radius=0.016, length=0.004),
            origin=Origin(xyz=(-inner_side * 0.023, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material="brushed_metal",
            name="outer_badge",
        )

        sliders[side_name] = slider
        yokes[side_name] = yoke
        cups[side_name] = cup

        model.articulation(
            f"{side_name}_extension",
            ArticulationType.PRISMATIC,
            parent=headband,
            child=slider,
            origin=Origin(xyz=(side * 0.112, 0.0, 0.105)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(lower=0.0, upper=0.040, effort=18.0, velocity=0.20),
        )
        model.articulation(
            f"{side_name}_fold",
            ArticulationType.REVOLUTE,
            parent=slider,
            child=yoke,
            origin=Origin(xyz=(0.0, 0.0, -0.085)),
            axis=(0.0, 0.0, -side),
            motion_limits=MotionLimits(lower=0.0, upper=1.55, effort=3.0, velocity=2.0),
        )
        model.articulation(
            f"{side_name}_swivel",
            ArticulationType.REVOLUTE,
            parent=yoke,
            child=cup,
            origin=Origin(xyz=(0.0, 0.0, -0.090)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(lower=-0.55, upper=0.55, effort=2.0, velocity=2.5),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    headband = object_model.get_part("headband")
    for side_name in ("left", "right"):
        slider = object_model.get_part(f"{side_name}_slider")
        yoke = object_model.get_part(f"{side_name}_yoke")
        cup = object_model.get_part(f"{side_name}_cup")
        extension = object_model.get_articulation(f"{side_name}_extension")
        fold = object_model.get_articulation(f"{side_name}_fold")
        swivel = object_model.get_articulation(f"{side_name}_swivel")

        ctx.allow_overlap(
            headband,
            slider,
            elem_a=f"{side_name}_socket",
            elem_b="rail",
            reason="The telescoping headband rail is intentionally retained inside the solid socket proxy.",
        )
        ctx.expect_within(
            slider,
            headband,
            axes="xy",
            inner_elem="rail",
            outer_elem=f"{side_name}_socket",
            margin=0.001,
            name=f"{side_name} slider centered in socket",
        )
        ctx.expect_overlap(
            slider,
            headband,
            axes="z",
            elem_a="rail",
            elem_b=f"{side_name}_socket",
            min_overlap=0.028,
            name=f"{side_name} collapsed rail remains inserted",
        )
        rest_pos = ctx.part_world_position(slider)
        with ctx.pose({extension: 0.040}):
            ctx.expect_within(
                slider,
                headband,
                axes="xy",
                inner_elem="rail",
                outer_elem=f"{side_name}_socket",
                margin=0.001,
                name=f"{side_name} extended rail stays captured",
            )
            ctx.expect_overlap(
                slider,
                headband,
                axes="z",
                elem_a="rail",
                elem_b=f"{side_name}_socket",
                min_overlap=0.022,
                name=f"{side_name} extended rail retains insertion",
            )
            extended_pos = ctx.part_world_position(slider)
        ctx.check(
            f"{side_name} extension moves downward",
            rest_pos is not None and extended_pos is not None and extended_pos[2] < rest_pos[2] - 0.030,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

        ctx.expect_contact(
            cup,
            yoke,
            elem_a="trunnion_pin",
            elem_b="front_arm",
            contact_tol=0.0015,
            name=f"{side_name} front trunnion seated in yoke",
        )
        ctx.expect_contact(
            cup,
            yoke,
            elem_a="trunnion_pin",
            elem_b="rear_arm",
            contact_tol=0.0015,
            name=f"{side_name} rear trunnion seated in yoke",
        )

        folded_before = ctx.part_world_aabb(cup)
        with ctx.pose({fold: 1.55}):
            folded_after = ctx.part_world_aabb(cup)
        if folded_before is not None and folded_after is not None:
            before_size = tuple(folded_before[1][i] - folded_before[0][i] for i in range(3))
            after_size = tuple(folded_after[1][i] - folded_after[0][i] for i in range(3))
            ctx.check(
                f"{side_name} fold hinge turns cup for storage",
                after_size[0] > before_size[0] + 0.025 and after_size[1] < before_size[1] - 0.025,
                details=f"before={before_size}, after={after_size}",
            )

        rest_cup = ctx.part_world_aabb(cup)
        with ctx.pose({swivel: 0.45}):
            swivel_cup = ctx.part_world_aabb(cup)
        if rest_cup is not None and swivel_cup is not None:
            rest_size = tuple(rest_cup[1][i] - rest_cup[0][i] for i in range(3))
            swivel_size = tuple(swivel_cup[1][i] - swivel_cup[0][i] for i in range(3))
            ctx.check(
                f"{side_name} cup swivels on yoke pivots",
                swivel_size[0] > rest_size[0] + 0.010 and swivel_size[2] > rest_size[2] + 0.010,
                details=f"rest={rest_size}, swivel={swivel_size}",
            )

    return ctx.report()


object_model = build_object_model()
