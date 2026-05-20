from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


CASE_LENGTH = 0.86
HINGE_Y = 0.190
HINGE_Z = 0.086
FRONT_Y = -0.190
HANDLE_Z = 0.072


def _case_profile(count: int = 96) -> list[tuple[float, float]]:
    """A violin-case plan view: broad bouts, pinched waist, tapered ends."""
    pts: list[tuple[float, float]] = []
    half_len = CASE_LENGTH * 0.5
    for i in range(count):
        t = 2.0 * math.pi * i / count
        x = half_len * math.cos(t)
        # Asymmetric violin-family bouts, with a narrow waist at the bridge area.
        lower_bout = 0.104 * math.exp(-((x + 0.155) / 0.165) ** 2)
        upper_bout = 0.092 * math.exp(-((x - 0.170) / 0.150) ** 2)
        waist_cut = 0.048 * math.exp(-(x / 0.085) ** 2)
        shoulder = 0.076
        half_width = max(0.038, shoulder + lower_bout + upper_bout - waist_cut)
        y = half_width * math.sin(t)
        pts.append((x, y))
    return pts


def _scaled_profile(
    profile: list[tuple[float, float]],
    sx: float,
    sy: float,
    *,
    y_shift: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x * sx, y * sy + y_shift) for x, y in profile]


def _add_ring(
    geom: MeshGeometry,
    profile: list[tuple[float, float]],
    z: float,
) -> list[int]:
    return [geom.add_vertex(x, y, z) for x, y in profile]


def _bridge_rings(geom: MeshGeometry, a: list[int], b: list[int]) -> None:
    n = len(a)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(a[i], b[i], b[j])
        geom.add_face(a[i], b[j], a[j])


def _cap_ring(geom: MeshGeometry, ring: list[int], *, reverse: bool = False) -> None:
    n = len(ring)
    cx = sum(geom.vertices[i][0] for i in ring) / n
    cy = sum(geom.vertices[i][1] for i in ring) / n
    cz = sum(geom.vertices[i][2] for i in ring) / n
    c = geom.add_vertex(cx, cy, cz)
    for i in range(n):
        j = (i + 1) % n
        if reverse:
            geom.add_face(c, ring[j], ring[i])
        else:
            geom.add_face(c, ring[i], ring[j])


def _lower_shell_geometry() -> MeshGeometry:
    profile = _case_profile()
    geom = MeshGeometry()
    outer_bottom = _add_ring(geom, _scaled_profile(profile, 0.91, 0.90), 0.000)
    outer_belly = _add_ring(geom, _scaled_profile(profile, 0.975, 0.965), 0.044)
    outer_rim = _add_ring(geom, profile, 0.075)
    inner_rim = _add_ring(geom, _scaled_profile(profile, 0.865, 0.815), 0.072)
    inner_floor = _add_ring(geom, _scaled_profile(profile, 0.735, 0.660), 0.026)

    _bridge_rings(geom, outer_bottom, outer_belly)
    _bridge_rings(geom, outer_belly, outer_rim)
    _bridge_rings(geom, outer_rim, inner_rim)
    _bridge_rings(geom, inner_rim, inner_floor)
    _cap_ring(geom, inner_floor)
    _cap_ring(geom, outer_bottom, reverse=True)
    return geom


def _lid_shell_geometry() -> MeshGeometry:
    profile = _case_profile()
    geom = MeshGeometry()
    # The child frame sits on the rear hinge pin; shift the matching case outline
    # so the closed lid falls directly over the lower shell.
    bottom = _add_ring(geom, _scaled_profile(profile, 0.985, 0.970, y_shift=-HINGE_Y), 0.000)
    side = _add_ring(geom, _scaled_profile(profile, 1.000, 0.990, y_shift=-HINGE_Y), 0.038)
    shoulder = _add_ring(geom, _scaled_profile(profile, 0.905, 0.890, y_shift=-HINGE_Y), 0.060)
    crown = _add_ring(geom, _scaled_profile(profile, 0.520, 0.500, y_shift=-HINGE_Y), 0.078)

    _bridge_rings(geom, bottom, side)
    _bridge_rings(geom, side, shoulder)
    _bridge_rings(geom, shoulder, crown)
    _cap_ring(geom, crown)
    return geom


def _flat_extrusion(
    profile: list[tuple[float, float]],
    z0: float,
    height: float,
) -> MeshGeometry:
    geom = MeshGeometry()
    low = _add_ring(geom, profile, z0)
    high = _add_ring(geom, profile, z0 + height)
    _bridge_rings(geom, low, high)
    _cap_ring(geom, high)
    _cap_ring(geom, low, reverse=True)
    return geom


def _instrument_body_profile(count: int = 72) -> list[tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    for i in range(count):
        t = 2.0 * math.pi * i / count
        x = 0.155 * math.cos(t) - 0.075
        lower = 0.048 * math.exp(-((x + 0.140) / 0.070) ** 2)
        upper = 0.040 * math.exp(-((x - 0.000) / 0.060) ** 2)
        waist = 0.022 * math.exp(-((x + 0.070) / 0.035) ** 2)
        half_width = max(0.020, 0.026 + lower + upper - waist)
        y = half_width * math.sin(t)
        pts.append((x, y))
    return pts


def _hinge_segments() -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    # Three fixed lower knuckles alternating with two moving lid knuckles.
    lower = [(-0.335, 0.140), (-0.005, 0.195), (0.335, 0.140)]
    lid = [(-0.185, 0.135), (0.175, 0.135)]
    return lower, lid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="violin_case")

    black_shell = model.material("black_shell", rgba=(0.035, 0.033, 0.030, 1.0))
    edge_trim = model.material("rubber_edge_trim", rgba=(0.010, 0.010, 0.009, 1.0))
    plush_blue = model.material("plush_blue_lining", rgba=(0.045, 0.070, 0.145, 1.0))
    dark_recess = model.material("dark_molded_recess", rgba=(0.012, 0.018, 0.045, 1.0))
    satin_metal = model.material("satin_nickel_hardware", rgba=(0.58, 0.55, 0.50, 1.0))
    handle_mat = model.material("black_padded_handle", rgba=(0.020, 0.018, 0.016, 1.0))

    lower = model.part("lower_shell")
    lower.visual(
        mesh_from_geometry(_lower_shell_geometry(), "lower_shaped_shell"),
        material=black_shell,
        name="lower_shell_body",
    )
    lower.visual(
        mesh_from_geometry(
            _flat_extrusion(_scaled_profile(_case_profile(), 0.805, 0.760), 0.018, 0.014),
            "instrument_bed",
        ),
        material=plush_blue,
        name="instrument_bed",
    )
    lower.visual(
        mesh_from_geometry(_flat_extrusion(_instrument_body_profile(), 0.031, 0.003), "violin_body_recess"),
        material=dark_recess,
        name="body_recess",
    )
    lower.visual(
        Box((0.285, 0.030, 0.004)),
        origin=Origin(xyz=(0.140, 0.0, 0.033)),
        material=dark_recess,
        name="neck_channel",
    )
    lower.visual(
        Sphere(0.028),
        origin=Origin(xyz=(0.305, 0.0, 0.034)),
        material=dark_recess,
        name="scroll_pocket",
    )
    lower.visual(
        Box((0.690, 0.018, 0.004)),
        origin=Origin(xyz=(0.000, -0.088, 0.033)),
        material=dark_recess,
        name="bow_channel",
    )
    lower.visual(
        Box((0.760, 0.014, 0.006)),
        origin=Origin(xyz=(0.0, HINGE_Y - 0.026, 0.074)),
        material=edge_trim,
        name="rear_hinge_rail",
    )
    lower.visual(
        Box((0.265, 0.030, 0.008)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.036, HANDLE_Z - 0.014)),
        material=satin_metal,
        name="front_handle_plate",
    )

    lower_segments, lid_segments = _hinge_segments()
    for index, (x, length) in enumerate(lower_segments):
        lower.visual(
            Box((length, 0.024, 0.006)),
            origin=Origin(xyz=(x, HINGE_Y - 0.014, HINGE_Z - 0.006)),
            material=satin_metal,
            name=f"lower_hinge_tab_{index}",
        )
        lower.visual(
            Cylinder(radius=0.0065, length=length),
            origin=Origin(xyz=(x, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_metal,
            name=f"lower_hinge_knuckle_{index}",
        )
    lower.visual(
        Cylinder(radius=0.0025, length=0.760),
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="hinge_pin",
    )

    # Front handle pivot brackets are fixed to the lower shell.
    for index, x in enumerate((-0.105, 0.105)):
        lower.visual(
            Box((0.038, 0.031, 0.040)),
            origin=Origin(xyz=(x, FRONT_Y + 0.0105, HANDLE_Z - 0.010)),
            material=satin_metal,
            name=f"handle_bracket_{index}",
        )
        lower.visual(
            Cylinder(radius=0.0045, length=0.034),
            origin=Origin(xyz=(x, FRONT_Y, HANDLE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_metal,
            name=f"handle_pivot_pin_{index}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_lid_shell_geometry(), "top_shaped_lid"),
        material=black_shell,
        name="lid_shell",
    )
    lid.visual(
        mesh_from_geometry(
            _flat_extrusion(_scaled_profile(_case_profile(), 0.985, 0.970, y_shift=-HINGE_Y), 0.000, 0.006),
            "lid_plush_lining",
        ),
        material=plush_blue,
        name="lid_lining",
    )
    lid.visual(
        Box((0.735, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.344, 0.010)),
        material=edge_trim,
        name="front_lid_lip",
    )
    for index, (x, length) in enumerate(lid_segments):
        lid.visual(
            Box((length, 0.050, 0.005)),
            origin=Origin(xyz=(x, -0.020, 0.004)),
            material=satin_metal,
            name=f"lid_hinge_tab_{index}",
        )
        lid.visual(
            Cylinder(radius=0.0060, length=length),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_metal,
            name=f"lid_hinge_knuckle_{index}",
        )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        # The closed lid extends from the hinge toward local -Y; -X makes
        # positive motion lift the free/front edge upward.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=0.0, upper=1.35),
    )

    handle = model.part("handle")
    for index, x in enumerate((-0.105, 0.105)):
        handle.visual(
            mesh_from_geometry(TorusGeometry(radius=0.011, tube=0.0028), f"handle_eyelet_{index}"),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_metal,
            name=f"pivot_eyelet_{index}",
        )
        handle.visual(
            Cylinder(radius=0.0055, length=0.063),
            origin=Origin(xyz=(x, -0.043, -0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=handle_mat,
            name=f"handle_arm_{index}",
        )
        handle.visual(
            Sphere(0.007),
            origin=Origin(xyz=(x, -0.074, -0.010)),
            material=handle_mat,
            name=f"handle_corner_{index}",
        )
    handle.visual(
        Cylinder(radius=0.012, length=0.210),
        origin=Origin(xyz=(0.0, -0.074, -0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_mat,
        name="padded_grip",
    )

    model.articulation(
        "handle_pivots",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=handle,
        origin=Origin(xyz=(0.0, FRONT_Y, HANDLE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-0.65, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_shell")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    lid_hinge = object_model.get_articulation("lid_hinge")
    handle_pivots = object_model.get_articulation("handle_pivots")

    for index in range(2):
        ctx.allow_overlap(
            lower,
            lid,
            elem_a="hinge_pin",
            elem_b=f"lid_hinge_knuckle_{index}",
            reason="The continuous metal hinge pin is intentionally captured inside the moving lid knuckle proxy.",
        )
        ctx.expect_within(
            lower,
            lid,
            axes="yz",
            inner_elem="hinge_pin",
            outer_elem=f"lid_hinge_knuckle_{index}",
            margin=0.001,
            name=f"hinge pin centered in lid knuckle {index}",
        )
        ctx.expect_overlap(
            lower,
            lid,
            axes="x",
            elem_a="hinge_pin",
            elem_b=f"lid_hinge_knuckle_{index}",
            min_overlap=0.10,
            name=f"hinge pin passes through lid knuckle {index}",
        )
        ctx.allow_overlap(
            handle,
            lower,
            elem_a=f"pivot_eyelet_{index}",
            elem_b=f"handle_bracket_{index}",
            reason="The simplified handle clevis locally captures the eyelet at the pivot instead of modeling the fork cutout.",
        )
        ctx.expect_overlap(
            handle,
            lower,
            axes="xz",
            elem_a=f"pivot_eyelet_{index}",
            elem_b=f"handle_bracket_{index}",
            min_overlap=0.005,
            name=f"handle eyelet seated in bracket {index}",
        )

    ctx.check(
        "lid hinge is rear longitudinal revolute",
        lid_hinge.axis == (-1.0, 0.0, 0.0)
        and lid_hinge.motion_limits is not None
        and lid_hinge.motion_limits.lower == 0.0
        and lid_hinge.motion_limits.upper is not None
        and lid_hinge.motion_limits.upper > 1.2,
        details=f"axis={lid_hinge.axis}, limits={lid_hinge.motion_limits}",
    )
    ctx.check(
        "handle uses a pivoting revolute joint",
        handle_pivots.axis == (-1.0, 0.0, 0.0)
        and handle_pivots.motion_limits is not None
        and handle_pivots.motion_limits.lower is not None
        and handle_pivots.motion_limits.lower < 0.0
        and handle_pivots.motion_limits.upper is not None
        and handle_pivots.motion_limits.upper > 0.9,
        details=f"axis={handle_pivots.axis}, limits={handle_pivots.motion_limits}",
    )

    with ctx.pose({lid_hinge: 0.0, handle_pivots: 0.0}):
        ctx.expect_gap(
            lid,
            lower,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="lower_shell_body",
            min_gap=0.002,
            max_gap=0.020,
            name="closed lid sits just above lower rim",
        )
        ctx.expect_overlap(
            lid,
            lower,
            axes="xy",
            elem_a="lid_shell",
            elem_b="lower_shell_body",
            min_overlap=0.20,
            name="lid outline matches lower shell footprint",
        )

    rest_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_hinge: 1.10}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid opens upward around rear hinge",
        rest_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > rest_lid_aabb[1][2] + 0.10,
        details=f"rest={rest_lid_aabb}, open={open_lid_aabb}",
    )

    rest_handle_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({handle_pivots: 0.90}):
        raised_handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "handle swings on the two front pivots",
        rest_handle_aabb is not None
        and raised_handle_aabb is not None
        and raised_handle_aabb[1][2] > rest_handle_aabb[1][2] + 0.025,
        details=f"rest={rest_handle_aabb}, raised={raised_handle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
