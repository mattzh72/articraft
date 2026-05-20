from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import inspect
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    params = inspect.signature(Material).parameters
    if "rgba" in params:
        return Material(name=name, rgba=rgba)
    if "color" in params:
        return Material(name=name, color=rgba)
    try:
        return Material(name, rgba)
    except TypeError:
        return Material(name=name)


GLASS_CLEAR = _make_material("glass_clear", (0.93, 0.97, 1.0, 0.24))
PRESSED_GLASS = _make_material("pressed_glass", (0.86, 0.94, 0.98, 0.50))
BRUSHED_ALUMINUM = _make_material("brushed_aluminum", (0.76, 0.77, 0.79, 1.0))
NICKEL_PLATE = _make_material("nickel_plate", (0.72, 0.74, 0.77, 1.0))
BRASS_CONTACT = _make_material("brass_contact", (0.77, 0.62, 0.24, 1.0))
CERAMIC_OFF_WHITE = _make_material("ceramic_off_white", (0.84, 0.83, 0.79, 1.0))
PHENOLIC_BLACK = _make_material("phenolic_black", (0.13, 0.12, 0.11, 1.0))
STEEL_WIRE = _make_material("steel_wire", (0.66, 0.68, 0.71, 1.0))
TUNGSTEN = _make_material("tungsten", (0.53, 0.41, 0.18, 1.0))


def _mesh_path(name: str):
    path = ASSETS.mesh_path(name) if hasattr(ASSETS, "mesh_path") else ASSETS.mesh_dir / name
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def _save_mesh(geometry, name: str):
    return mesh_from_geometry(geometry, _mesh_path(name))


def _circle_profile(radius: float, segments: int = 56, phase: float = 0.0):
    return [
        (
            radius * math.cos(phase + math.tau * i / segments),
            radius * math.sin(phase + math.tau * i / segments),
        )
        for i in range(segments)
    ]


def _ring_geometry(
    outer_radius: float,
    inner_radius: float,
    height: float,
    *,
    z0: float = 0.0,
    segments: int = 56,
):
    return ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments),
        [_circle_profile(inner_radius, segments)],
        height,
        center=False,
    ).translate(0.0, 0.0, z0)


def _helix_points(
    radius: float,
    z0: float,
    z1: float,
    turns: float,
    *,
    samples_per_turn: int = 28,
    start_angle: float = 0.0,
):
    count = max(2, int(math.ceil(turns * samples_per_turn)))
    points = []
    for i in range(count + 1):
        frac = i / count
        angle = start_angle + turns * math.tau * frac
        points.append(
            (
                radius * math.cos(angle),
                radius * math.sin(angle),
                z0 + (z1 - z0) * frac,
            )
        )
    return points


def _coil_points(
    *,
    x0: float,
    x1: float,
    radius: float,
    turns: float,
    z_center: float,
    y_center: float = 0.0,
    samples_per_turn: int = 20,
):
    count = max(2, int(math.ceil(turns * samples_per_turn)))
    points = []
    for i in range(count + 1):
        frac = i / count
        angle = turns * math.tau * frac
        points.append(
            (
                x0 + (x1 - x0) * frac,
                y_center + radius * math.cos(angle),
                z_center + radius * math.sin(angle),
            )
        )
    return points


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_in_light_bulb_fixture", assets=ASSETS)

    socket = model.part("socket")
    socket.visual(
        _save_mesh(_ring_geometry(0.0225, 0.0168, 0.019, z0=-0.016), "socket_upper_collar.obj"),
        material=PHENOLIC_BLACK,
    )
    socket.visual(
        _save_mesh(_ring_geometry(0.0245, 0.0175, 0.028, z0=-0.053), "socket_lower_body.obj"),
        material=CERAMIC_OFF_WHITE,
    )
    socket.visual(
        _save_mesh(_ring_geometry(0.0255, 0.0217, 0.004, z0=-0.001), "socket_retaining_ring.obj"),
        material=NICKEL_PLATE,
    )
    for index, angle in enumerate((0.0, math.tau / 3.0, 2.0 * math.tau / 3.0)):
        socket.visual(
            Box((0.006, 0.012, 0.011)),
            origin=Origin(
                xyz=(0.0195 * math.cos(angle), 0.0195 * math.sin(angle), -0.0205),
                rpy=(0.0, 0.0, angle),
            ),
            material=CERAMIC_OFF_WHITE,
            name=f"socket_rib_{index}",
        )
    for index, angle in enumerate((0.0, math.tau / 3.0, 2.0 * math.tau / 3.0)):
        socket.visual(
            Box((0.014, 0.0035, 0.004)),
            origin=Origin(
                xyz=(0.012 * math.cos(angle), 0.012 * math.sin(angle), -0.047),
                rpy=(0.0, 0.0, angle),
            ),
            material=PHENOLIC_BLACK,
            name=f"socket_spoke_{index}",
        )
    socket.visual(
        Cylinder(radius=0.0055, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=PHENOLIC_BLACK,
    )
    socket.visual(
        Cylinder(radius=0.0075, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.068)),
        material=NICKEL_PLATE,
    )
    socket.visual(
        Cylinder(radius=0.0022, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.043)),
        material=BRASS_CONTACT,
    )
    socket.visual(
        Cylinder(radius=0.0048, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -0.037)),
        material=BRASS_CONTACT,
    )
    socket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.025, length=0.075),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, -0.037)),
    )

    bulb_body = model.part("bulb_body")
    bulb_body_profile = [
        (0.0, -0.0365),
        (0.0038, -0.0365),
        (0.0050, -0.0352),
        (0.0086, -0.0328),
        (0.0118, -0.0306),
        (0.0127, -0.0075),
        (0.0140, -0.0013),
        (0.0150, 0.0015),
        (0.0146, 0.0046),
        (0.0130, 0.0060),
        (0.0, 0.0060),
    ]
    bulb_body.visual(
        _save_mesh(LatheGeometry(bulb_body_profile, segments=72), "bulb_base_shell.obj"),
        material=BRUSHED_ALUMINUM,
    )
    bulb_body.visual(
        _save_mesh(
            wire_from_points(
                _helix_points(
                    0.01325,
                    -0.0305,
                    -0.0038,
                    5.2,
                    samples_per_turn=30,
                    start_angle=0.35,
                ),
                radius=0.00095,
                radial_segments=14,
                cap_ends=True,
                corner_mode="miter",
            ),
            "bulb_thread.obj",
        ),
        material=NICKEL_PLATE,
    )
    bulb_body.visual(
        Cylinder(radius=0.0063, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -0.0352)),
        material=PHENOLIC_BLACK,
    )
    bulb_body.visual(
        Cylinder(radius=0.0032, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0, -0.0344)),
        material=BRASS_CONTACT,
    )
    bulb_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0155, length=0.044),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
    )

    bulb_glass = model.part("bulb_glass")
    glass_outer = [
        (0.0124, 0.0),
        (0.0170, 0.010),
        (0.0248, 0.028),
        (0.0303, 0.054),
        (0.0288, 0.079),
        (0.0190, 0.100),
        (0.0069, 0.111),
        (0.0019, 0.114),
    ]
    glass_inner = [
        (0.0008, 0.112),
        (0.0059, 0.109),
        (0.0168, 0.099),
        (0.0261, 0.078),
        (0.0275, 0.054),
        (0.0233, 0.029),
        (0.0156, 0.011),
        (0.0109, 0.0010),
    ]
    bulb_glass.visual(
        _save_mesh(LatheGeometry(glass_outer + glass_inner, segments=88), "bulb_glass_shell.obj"),
        material=GLASS_CLEAR,
    )
    bulb_glass.inertial = Inertial.from_geometry(
        Sphere(radius=0.031),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
    )

    filament = model.part("filament_assembly")
    filament.visual(
        Cylinder(radius=0.0024, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=PRESSED_GLASS,
    )
    filament.visual(
        _save_mesh(
            wire_from_points(
                [(-0.0015, 0.0, 0.011), (-0.0038, 0.0007, 0.026), (-0.0060, 0.0011, 0.044)],
                radius=0.00055,
                radial_segments=10,
                cap_ends=True,
                corner_mode="miter",
            ),
            "filament_left_support.obj",
        ),
        material=STEEL_WIRE,
    )
    filament.visual(
        _save_mesh(
            wire_from_points(
                [(0.0015, 0.0, 0.011), (0.0038, 0.0007, 0.026), (0.0060, 0.0011, 0.044)],
                radius=0.00055,
                radial_segments=10,
                cap_ends=True,
                corner_mode="miter",
            ),
            "filament_right_support.obj",
        ),
        material=STEEL_WIRE,
    )
    filament.visual(
        _save_mesh(
            wire_from_points(
                _coil_points(
                    x0=-0.0060,
                    x1=0.0060,
                    radius=0.0011,
                    turns=8.5,
                    z_center=0.044,
                    y_center=0.0011,
                    samples_per_turn=18,
                ),
                radius=0.00033,
                radial_segments=8,
                cap_ends=True,
                corner_mode="miter",
            ),
            "filament_coil.obj",
        ),
        material=TUNGSTEN,
    )
    filament.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0045, length=0.050),
        mass=0.003,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    model.articulation(
        "bulb_screw",
        ArticulationType.REVOLUTE,
        parent="socket",
        child="bulb_body",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=10.0,
            lower=0.0,
            upper=math.tau * 2.5,
        ),
    )
    model.articulation(
        "body_to_glass",
        ArticulationType.FIXED,
        parent="bulb_body",
        child="bulb_glass",
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )
    model.articulation(
        "body_to_filament",
        ArticulationType.FIXED,
        parent="bulb_body",
        child="filament_assembly",
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "bulb_body",
        "socket",
        reason="the bulb shell seats inside a hollow threaded socket throat; conservative collision hulls around the socket collar can fill that opening",
    )
    ctx.allow_overlap(
        "bulb_glass",
        "filament_assembly",
        reason="the filament sits inside a thin hollow glass shell and collision hulls on the glass can be conservative",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.004, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("bulb_body", "socket", axes="xy", max_dist=0.0035)
    ctx.expect_aabb_overlap("bulb_body", "socket", axes="xy", min_overlap=0.026)
    ctx.expect_origin_distance("bulb_glass", "socket", axes="xy", max_dist=0.004)
    ctx.expect_aabb_overlap("bulb_glass", "socket", axes="xy", min_overlap=0.040)
    ctx.expect_aabb_gap("bulb_glass", "socket", axis="z", max_gap=0.010, max_penetration=0.0)
    ctx.expect_origin_distance("filament_assembly", "bulb_glass", axes="xy", max_dist=0.0025)
    ctx.expect_aabb_overlap("filament_assembly", "bulb_glass", axes="xy", min_overlap=0.0045)

    for angle in (0.0, math.tau * 0.5, math.tau * 1.25, math.tau * 2.5):
        with ctx.pose(bulb_screw=angle):
            ctx.expect_origin_distance("bulb_body", "socket", axes="xy", max_dist=0.0035)
            ctx.expect_aabb_overlap("bulb_body", "socket", axes="xy", min_overlap=0.026)
            ctx.expect_origin_distance("bulb_glass", "socket", axes="xy", max_dist=0.004)
            ctx.expect_aabb_overlap("bulb_glass", "socket", axes="xy", min_overlap=0.040)
            ctx.expect_aabb_gap("bulb_glass", "socket", axis="z", max_gap=0.010, max_penetration=0.0)
            ctx.expect_origin_distance("filament_assembly", "bulb_glass", axes="xy", max_dist=0.0025)
            ctx.expect_aabb_overlap("filament_assembly", "bulb_glass", axes="xy", min_overlap=0.0045)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
