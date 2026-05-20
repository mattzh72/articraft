from __future__ import annotations

import math
from typing import Iterable

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def _superellipse_loop(
    width: float,
    depth: float,
    z: float,
    *,
    center_y: float = 0.0,
    exponent: float = 4.2,
    segments: int = 72,
) -> list[tuple[float, float, float]]:
    """Rounded-rectangle section loop in X/Y at height z."""
    pts: list[tuple[float, float, float]] = []
    for i in range(segments):
        t = 2.0 * math.pi * i / segments
        ct = math.cos(t)
        st = math.sin(t)
        x = math.copysign(abs(ct) ** (2.0 / exponent), ct) * width * 0.5
        y = center_y + math.copysign(abs(st) ** (2.0 / exponent), st) * depth * 0.5
        pts.append((x, y, z))
    return pts


def _add_loop(vertices: list[tuple[float, float, float]], loop: Iterable[tuple[float, float, float]]) -> list[int]:
    indices: list[int] = []
    for p in loop:
        indices.append(len(vertices))
        vertices.append(p)
    return indices


def _quad(faces: list[tuple[int, int, int]], a: int, b: int, c: int, d: int) -> None:
    faces.append((a, b, c))
    faces.append((a, c, d))


def _build_bin_tub_mesh() -> MeshGeometry:
    """Thin-walled tapered tub: open at the top, closed with an internal floor."""
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []
    wall = 0.028
    seg = 72

    # z, width, depth, center_y.  The slight forward center shift at the base is
    # typical of wheelie bins, keeping the rear axle tucked under the handle.
    outer_specs = (
        (0.095, 0.440, 0.475, 0.045),
        (0.320, 0.500, 0.555, 0.036),
        (0.700, 0.575, 0.655, 0.018),
        (1.020, 0.625, 0.720, 0.000),
    )
    inner_specs = (
        (0.160, 0.440 - 2 * wall, 0.475 - 2 * wall, 0.045),
        (0.320, 0.500 - 2 * wall, 0.555 - 2 * wall, 0.036),
        (0.700, 0.575 - 2 * wall, 0.655 - 2 * wall, 0.018),
        (1.020, 0.625 - 2 * wall, 0.720 - 2 * wall, 0.000),
    )

    outer = [
        _add_loop(vertices, _superellipse_loop(w, d, z, center_y=cy, segments=seg))
        for z, w, d, cy in outer_specs
    ]
    inner = [
        _add_loop(vertices, _superellipse_loop(w, d, z, center_y=cy, segments=seg))
        for z, w, d, cy in inner_specs
    ]

    # Outer skin and inner liner.
    for loops, reverse in ((outer, False), (inner, True)):
        for lower, upper in zip(loops[:-1], loops[1:]):
            n = len(lower)
            for i in range(n):
                a = lower[i]
                b = lower[(i + 1) % n]
                c = upper[(i + 1) % n]
                d = upper[i]
                if reverse:
                    _quad(faces, a, d, c, b)
                else:
                    _quad(faces, a, b, c, d)

    # Rolled top rim: outer wall folds inward to the liner.
    top_outer = outer[-1]
    top_inner = inner[-1]
    for i in range(seg):
        _quad(faces, top_outer[i], top_outer[(i + 1) % seg], top_inner[(i + 1) % seg], top_inner[i])

    # Sloped underbody joining outer shell to the raised interior floor.
    bottom_outer = outer[0]
    floor_inner = inner[0]
    for i in range(seg):
        _quad(faces, bottom_outer[i], floor_inner[i], floor_inner[(i + 1) % seg], bottom_outer[(i + 1) % seg])

    # Interior floor cap: the bin is hollow, but it has a real bottom panel.
    center_idx = len(vertices)
    vertices.append((0.0, 0.045, 0.160))
    for i in range(seg):
        faces.append((center_idx, floor_inner[(i + 1) % seg], floor_inner[i]))

    return MeshGeometry(vertices=vertices, faces=faces)


def _build_lid_cap_mesh() -> MeshGeometry:
    """Low, softly rounded lid slab with a clean overhanging silhouette."""
    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []
    seg = 72
    bottom = _add_loop(vertices, _superellipse_loop(0.700, 0.770, 0.000, center_y=0.405, segments=seg))
    top = _add_loop(vertices, _superellipse_loop(0.700, 0.770, 0.032, center_y=0.405, segments=seg))
    for i in range(seg):
        _quad(faces, bottom[i], bottom[(i + 1) % seg], top[(i + 1) % seg], top[i])

    top_center = len(vertices)
    vertices.append((0.0, 0.405, 0.036))
    bottom_center = len(vertices)
    vertices.append((0.0, 0.405, -0.002))
    for i in range(seg):
        faces.append((top_center, top[i], top[(i + 1) % seg]))
        faces.append((bottom_center, bottom[(i + 1) % seg], bottom[i]))
    return MeshGeometry(vertices=vertices, faces=faces)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_wheelie_bin")

    polymer = model.material("deep_graphite_polymer", rgba=(0.055, 0.070, 0.068, 1.0))
    lid_polymer = model.material("satin_lid_polymer", rgba=(0.075, 0.090, 0.086, 1.0))
    inset_polymer = model.material("subtle_inset_polymer", rgba=(0.040, 0.050, 0.048, 1.0))
    painted_metal = model.material("powder_coated_metal", rgba=(0.18, 0.19, 0.18, 1.0))
    elastomer = model.material("matte_black_elastomer", rgba=(0.010, 0.011, 0.010, 1.0))
    hub_polymer = model.material("warm_grey_hub_polymer", rgba=(0.34, 0.35, 0.33, 1.0))

    shell = model.part("bin_shell")
    shell.visual(
        mesh_from_geometry(_build_bin_tub_mesh(), "bin_tub"),
        material=polymer,
        name="bin_tub",
    )

    # Crisp, restrained surface strategy: a proud front datum panel, low rim
    # bead, and rubber feet are all mounted into the molded shell.
    shell.visual(
        Box((0.360, 0.018, 0.300)),
        origin=Origin(xyz=(0.0, 0.336, 0.570)),
        material=inset_polymer,
        name="front_inset",
    )
    shell.visual(
        Box((0.210, 0.035, 0.045)),
        origin=Origin(xyz=(0.0, 0.360, 0.940)),
        material=painted_metal,
        name="front_badge",
    )
    shell.visual(
        Box((0.160, 0.110, 0.120)),
        origin=Origin(xyz=(-0.165, 0.250, 0.060)),
        material=elastomer,
        name="front_foot_0",
    )
    shell.visual(
        Box((0.160, 0.110, 0.120)),
        origin=Origin(xyz=(0.165, 0.250, 0.060)),
        material=elastomer,
        name="front_foot_1",
    )

    # Rear rolling hardware: the axle is a visible powder-coated metal shaft,
    # carried by a polymer cradle molded into the bin body.
    shell.visual(
        Box((0.590, 0.180, 0.115)),
        origin=Origin(xyz=(0.0, -0.225, 0.160)),
        material=polymer,
        name="axle_cradle",
    )
    shell.visual(
        Cylinder(radius=0.018, length=0.840),
        origin=Origin(xyz=(0.0, -0.310, 0.125), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_metal,
        name="rear_axle",
    )

    # Full-width rear handle and hinge pin with molded support ears in the real
    # load path from shell to lid.
    shell.visual(
        Cylinder(radius=0.021, length=0.720),
        origin=Origin(xyz=(0.0, -0.430, 0.970), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_metal,
        name="rear_handle",
    )
    shell.visual(
        Cylinder(radius=0.010, length=0.720),
        origin=Origin(xyz=(0.0, -0.390, 1.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_metal,
        name="hinge_pin",
    )
    for i, x in enumerate((-0.335, -0.150, 0.150, 0.335)):
        shell.visual(
            Box((0.045, 0.080, 0.085)),
            origin=Origin(xyz=(x, -0.392, 1.005)),
            material=polymer,
            name=f"hinge_ear_{i}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_build_lid_cap_mesh(), "lid_cap"),
        material=lid_polymer,
        name="lid_cap",
    )
    lid.visual(
        Box((0.520, 0.430, 0.012)),
        origin=Origin(xyz=(0.0, 0.455, 0.041)),
        material=inset_polymer,
        name="lid_inset",
    )
    lid.visual(
        Box((0.640, 0.026, 0.032)),
        origin=Origin(xyz=(0.0, 0.030, 0.017)),
        material=lid_polymer,
        name="rear_lid_rail",
    )
    for i, (x, length) in enumerate(((-0.250, 0.115), (0.0, 0.165), (0.250, 0.115))):
        lid.visual(
            Cylinder(radius=0.024, length=length),
            origin=Origin(xyz=(x, 0.000, 0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=lid_polymer,
            name=f"hinge_sleeve_{i}",
        )
        lid.visual(
            Box((length, 0.028, 0.026)),
            origin=Origin(xyz=(x, 0.023, 0.010)),
            material=lid_polymer,
            name=f"hinge_bridge_{i}",
        )

    hinge = model.articulation(
        "shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=lid,
        origin=Origin(xyz=(0.0, -0.390, 1.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.2, lower=0.0, upper=1.55),
    )

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.125,
            0.058,
            inner_radius=0.087,
            carcass=TireCarcass(belt_width_ratio=0.68, sidewall_bulge=0.035),
            tread=TireTread(style="block", depth=0.006, count=20, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.0025),),
            sidewall=TireSidewall(style="rounded", bulge=0.045),
            shoulder=TireShoulder(width=0.006, radius=0.003),
        ),
        "wheel_tire",
    )
    rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.091,
            0.052,
            rim=WheelRim(inner_radius=0.058, flange_height=0.006, flange_thickness=0.003),
            hub=WheelHub(
                radius=0.033,
                width=0.046,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.040, hole_diameter=0.004),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.004, window_radius=0.011),
            bore=WheelBore(style="round", diameter=0.034),
        ),
        "wheel_rim",
    )

    for i, x in enumerate((-0.365, 0.365)):
        wheel = model.part(f"wheel_{i}")
        wheel.visual(tire_mesh, material=elastomer, name="tire")
        wheel.visual(rim_mesh, material=hub_polymer, name="rim_core")
        model.articulation(
            f"axle_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=shell,
            child=wheel,
            origin=Origin(xyz=(x, -0.310, 0.125)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=20.0),
        )

    # Names kept as intrinsic mechanism labels; local variable prevents lint
    # tools from considering the hinge construction accidental.
    assert hinge.name == "shell_to_lid"
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("bin_shell")
    lid = object_model.get_part("lid")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    hinge = object_model.get_articulation("shell_to_lid")

    for sleeve in ("hinge_sleeve_0", "hinge_sleeve_1", "hinge_sleeve_2"):
        ctx.allow_overlap(
            shell,
            lid,
            elem_a="hinge_pin",
            elem_b=sleeve,
            reason="The lid sleeve is intentionally captured around the full-width metal hinge pin.",
        )
        ctx.expect_overlap(
            shell,
            lid,
            axes="x",
            elem_a="hinge_pin",
            elem_b=sleeve,
            min_overlap=0.090,
            name=f"{sleeve} engages hinge pin",
        )

    for wheel_name in ("wheel_0", "wheel_1"):
        ctx.allow_overlap(
            shell,
            wheel_name,
            elem_a="rear_axle",
            elem_b="rim_core",
            reason="The wheel hub is represented as a captured bearing around the metal axle.",
        )
        ctx.expect_overlap(
            shell,
            wheel_name,
            axes="x",
            elem_a="rear_axle",
            elem_b="rim_core",
            min_overlap=0.040,
            name=f"{wheel_name} hub spans axle",
        )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            shell,
            axis="z",
            positive_elem="lid_cap",
            negative_elem="bin_tub",
            min_gap=0.010,
            max_gap=0.055,
            name="closed lid sits just above rolled rim",
        )
        ctx.expect_overlap(
            lid,
            shell,
            axes="xy",
            elem_a="lid_cap",
            elem_b="bin_tub",
            min_overlap=0.560,
            name="closed lid covers bin opening",
        )
        closed_aabb = ctx.part_world_aabb(lid)

    with ctx.pose({hinge: 1.25}):
        open_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "lid swings upward from rear hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.45,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    ctx.check(
        "wheel articulations roll about rear axle",
        all(
            object_model.get_articulation(f"axle_to_wheel_{i}").axis == (1.0, 0.0, 0.0)
            for i in (0, 1)
        ),
        details="Wheel joint axes should align with the full-width rear axle.",
    )
    for i, wheel in enumerate((wheel_0, wheel_1)):
        aabb = ctx.part_world_aabb(wheel)
        ctx.check(
            f"wheel_{i} tire reaches ground plane",
            aabb is not None and abs(aabb[0][2]) < 0.010,
            details=f"aabb={aabb}",
        )

    return ctx.report()


object_model = build_object_model()
