from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _frustum_mesh(
    *,
    bottom_size: tuple[float, float],
    top_size: tuple[float, float],
    bottom_z: float,
    top_z: float,
) -> MeshGeometry:
    bottom_x, bottom_y = bottom_size[0] * 0.5, bottom_size[1] * 0.5
    top_x, top_y = top_size[0] * 0.5, top_size[1] * 0.5
    vertices = [
        (-bottom_x, -bottom_y, bottom_z),
        (bottom_x, -bottom_y, bottom_z),
        (bottom_x, bottom_y, bottom_z),
        (-bottom_x, bottom_y, bottom_z),
        (-top_x, -top_y, top_z),
        (top_x, -top_y, top_z),
        (top_x, top_y, top_z),
        (-top_x, top_y, top_z),
    ]
    faces = [
        (0, 1, 2),
        (0, 2, 3),
        (4, 6, 5),
        (4, 7, 6),
        (0, 4, 5),
        (0, 5, 1),
        (1, 5, 6),
        (1, 6, 2),
        (2, 6, 7),
        (2, 7, 3),
        (3, 7, 4),
        (3, 4, 0),
    ]
    return MeshGeometry(vertices=vertices, faces=faces)


def _gable_roof_mesh(width: float, length: float, base_z: float, peak_z: float) -> MeshGeometry:
    half_w = width * 0.5
    half_l = length * 0.5
    vertices = [
        (-half_w, -half_l, base_z),
        (half_w, -half_l, base_z),
        (0.0, -half_l, peak_z),
        (-half_w, half_l, base_z),
        (half_w, half_l, base_z),
        (0.0, half_l, peak_z),
    ]
    faces = [
        (0, 1, 2),
        (3, 5, 4),
        (0, 3, 4),
        (0, 4, 1),
        (1, 4, 5),
        (1, 5, 2),
        (2, 5, 3),
        (2, 3, 0),
    ]
    return MeshGeometry(vertices=vertices, faces=faces)


def _ring_band(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    radial_segments: int = 48,
) -> MeshGeometry:
    outer = CylinderGeometry(radius=outer_radius, height=length, radial_segments=radial_segments)
    inner = CylinderGeometry(
        radius=inner_radius,
        height=length + 0.004,
        radial_segments=radial_segments,
    )
    return boolean_difference(outer, inner)


def _blade_lattice_mesh() -> MeshGeometry:
    """A connected flat blade lattice in local coordinates.

    Local +Z is radial outward from the hub, local +Y is the shaft direction,
    and local +X is the root hinge/fold axis.
    """
    lattice = MeshGeometry()

    def cyl(a, b, radius):
        geom = CylinderGeometry(radius=radius, height=_distance(a, b), radial_segments=12)
        # MeshGeometry has no direct rpy helper; use the same two rotations as the
        # visual-cylinder helper formula resolves to: first pitch about Y, then yaw about Z.
        _, pitch, yaw = _rpy_for_cylinder(a, b)
        geom.rotate_y(pitch)
        geom.rotate_z(yaw)
        geom.translate(*_midpoint(a, b))
        lattice.merge(geom)

    # Root barrel and light lattice spars. Slight physical overlap at joints
    # makes the authored blade one manufactured assembly instead of loose sticks.
    cyl((-0.018, 0.0, 0.0), (0.018, 0.0, 0.0), 0.0048)
    cyl((0.0, 0.0, -0.002), (0.0, 0.0, 0.128), 0.0036)
    cyl((-0.017, 0.0, 0.015), (-0.019, 0.0, 0.124), 0.0022)
    cyl((0.017, 0.0, 0.015), (0.019, 0.0, 0.124), 0.0022)
    for z in (0.020, 0.048, 0.077, 0.106, 0.126):
        width = 0.034 + 0.010 * (z / 0.126)
        cyl((-width * 0.5, 0.0, z), (width * 0.5, 0.0, z), 0.0021)
    cyl((-0.016, 0.0, 0.022), (0.019, 0.0, 0.078), 0.0017)
    cyl((0.016, 0.0, 0.048), (-0.019, 0.0, 0.106), 0.0017)
    cyl((-0.019, 0.0, 0.077), (0.019, 0.0, 0.126), 0.0017)
    return lattice


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_traditional_windmill")

    plaster = model.material("warm_plaster", rgba=(0.83, 0.78, 0.66, 1.0))
    stone = model.material("brushed_stone", rgba=(0.48, 0.48, 0.44, 1.0))
    dark_wood = model.material("dark_oak", rgba=(0.30, 0.18, 0.10, 1.0))
    light_wood = model.material("oiled_spruce", rgba=(0.74, 0.56, 0.34, 1.0))
    sail_cloth = model.material("aged_canvas", rgba=(0.88, 0.82, 0.68, 1.0))
    iron = model.material("blackened_iron", rgba=(0.08, 0.09, 0.10, 1.0))
    shadow = model.material("shadow_glass", rgba=(0.05, 0.06, 0.07, 1.0))

    yaw_bearing_mesh = mesh_from_geometry(
        _ring_band(outer_radius=0.043, inner_radius=0.023, length=0.027),
        "yaw_bearing_seat",
    )
    tower_mesh = mesh_from_geometry(
        _frustum_mesh(
            bottom_size=(0.116, 0.100),
            top_size=(0.068, 0.058),
            bottom_z=0.020,
            top_z=0.280,
        ),
        "tapered_tower_shell",
    )
    cap_roof_mesh = mesh_from_geometry(
        _gable_roof_mesh(width=0.112, length=0.150, base_z=0.070, peak_z=0.103),
        "gable_cap_roof",
    )
    front_bearing_mesh = mesh_from_geometry(
        _ring_band(outer_radius=0.020, inner_radius=0.0074, length=0.024).rotate_x(math.pi / 2.0),
        "front_shaft_bearing",
    )
    rear_bearing_mesh = mesh_from_geometry(
        _ring_band(outer_radius=0.018, inner_radius=0.0074, length=0.022).rotate_x(math.pi / 2.0),
        "rear_shaft_bearing",
    )
    blade_mesh = mesh_from_geometry(_blade_lattice_mesh(), "folding_blade_lattice")

    tower = model.part("tower")
    tower.visual(Box((0.240, 0.200, 0.020)), origin=Origin(xyz=(0.0, 0.0, 0.010)), material=stone, name="desktop_base")
    tower.visual(Box((0.160, 0.136, 0.018)), origin=Origin(xyz=(0.0, 0.0, 0.027)), material=stone, name="stone_plinth")
    tower.visual(tower_mesh, material=plaster, name="tapered_tower")
    tower.visual(
        yaw_bearing_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.2915)),
        material=iron,
        name="yaw_bearing",
    )
    tower.visual(Box((0.034, 0.006, 0.060)), origin=Origin(xyz=(0.0, -0.052, 0.060)), material=dark_wood, name="front_door")
    tower.visual(Box((0.028, 0.006, 0.026)), origin=Origin(xyz=(0.0, -0.040, 0.168)), material=shadow, name="front_window")
    tower.visual(Box((0.006, 0.024, 0.024)), origin=Origin(xyz=(-0.045, 0.0, 0.150)), material=shadow, name="side_window")
    tower.visual(Box((0.018, 0.070, 0.012)), origin=Origin(xyz=(-0.033, 0.0, 0.284)), material=light_wood, name="cap_table_side_0")
    tower.visual(Box((0.018, 0.070, 0.012)), origin=Origin(xyz=(0.033, 0.0, 0.284)), material=light_wood, name="cap_table_side_1")
    tower.visual(Box((0.082, 0.012, 0.012)), origin=Origin(xyz=(0.0, -0.031, 0.284)), material=light_wood, name="cap_table_front")
    tower.visual(Box((0.082, 0.012, 0.012)), origin=Origin(xyz=(0.0, 0.031, 0.284)), material=light_wood, name="cap_table_rear")

    cap = model.part("cap")
    cap.visual(
        Cylinder(radius=0.016, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=iron,
        name="yaw_stem",
    )
    cap.visual(
        Cylinder(radius=0.036, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=iron,
        name="yaw_collar",
    )
    cap.visual(Box((0.092, 0.140, 0.012)), origin=Origin(xyz=(0.0, -0.025, 0.020)), material=dark_wood, name="cap_floor")
    for x in (-0.044, 0.044):
        cap.visual(Box((0.016, 0.142, 0.055)), origin=Origin(xyz=(x, -0.025, 0.045)), material=dark_wood, name=f"side_cheek_{x:+.0f}")
    cap.visual(Box((0.092, 0.016, 0.050)), origin=Origin(xyz=(0.0, 0.052, 0.047)), material=dark_wood, name="rear_bulkhead")
    cap.visual(cap_roof_mesh, origin=Origin(xyz=(0.0, -0.025, 0.0)), material=light_wood, name="gable_roof")
    cap.visual(front_bearing_mesh, origin=Origin(xyz=(0.0, -0.085, 0.052)), material=iron, name="front_bearing")
    cap.visual(rear_bearing_mesh, origin=Origin(xyz=(0.0, -0.025, 0.052)), material=iron, name="rear_bearing")
    for y in (-0.085, -0.025):
        for x in (-0.029, 0.029):
            cap.visual(Box((0.018, 0.026, 0.012)), origin=Origin(xyz=(x, y, 0.052)), material=iron, name=f"bearing_yoke_{y:+.3f}_{x:+.3f}")
    cap.visual(Box((0.030, 0.080, 0.006)), origin=Origin(xyz=(0.0, 0.100, 0.058)), material=light_wood, name="folded_tail_grip")

    rotor_hub = model.part("rotor_hub")
    rotor_hub.visual(
        Cylinder(radius=0.0075, length=0.120),
        origin=Origin(xyz=(0.0, 0.048, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="rotor_shaft",
    )
    rotor_hub.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="hub_disk",
    )
    rotor_hub.visual(
        Cylinder(radius=0.017, length=0.016),
        origin=Origin(xyz=(0.0, -0.033, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=light_wood,
        name="front_spinner",
    )
    root_radius = 0.034
    spoke_radius = 0.0032
    for index, theta in enumerate((0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)):
        end = (math.sin(theta) * (root_radius - 0.003), -0.034, math.cos(theta) * (root_radius - 0.003))
        _add_member(rotor_hub, (0.0, -0.034, 0.0), end, spoke_radius, iron, name=f"hub_spoke_{index}")

    blade_angles = (0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)
    for index, theta in enumerate(blade_angles):
        blade = model.part(f"blade_{index}")
        blade.visual(blade_mesh, material=light_wood, name="lattice")
        # Thin canvas strips are inset into the lattice but leave the ladder-like
        # traditional frame visible.
        blade.visual(Box((0.028, 0.002, 0.036)), origin=Origin(xyz=(0.0, -0.001, 0.042)), material=sail_cloth, name="root_canvas")
        blade.visual(Box((0.032, 0.002, 0.034)), origin=Origin(xyz=(0.0, -0.001, 0.092)), material=sail_cloth, name="tip_canvas")
        model.articulation(
            f"hub_to_blade_{index}",
            ArticulationType.REVOLUTE,
            parent=rotor_hub,
            child=blade,
            origin=Origin(
                xyz=(math.sin(theta) * root_radius, -0.034, math.cos(theta) * root_radius),
                rpy=(0.0, theta, 0.0),
            ),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.6, velocity=1.0, lower=0.0, upper=1.32),
        )

    model.articulation(
        "tower_to_cap",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.305)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.45, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "cap_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=rotor_hub,
        origin=Origin(xyz=(0.0, -0.105, 0.052)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    cap = object_model.get_part("cap")
    rotor = object_model.get_part("rotor_hub")
    blade_0 = object_model.get_part("blade_0")
    blade_1 = object_model.get_part("blade_1")
    yaw = object_model.get_articulation("tower_to_cap")
    rotor_spin = object_model.get_articulation("cap_to_rotor")
    fold_0 = object_model.get_articulation("hub_to_blade_0")
    fold_1 = object_model.get_articulation("hub_to_blade_1")
    fold_2 = object_model.get_articulation("hub_to_blade_2")
    fold_3 = object_model.get_articulation("hub_to_blade_3")

    ctx.allow_overlap(
        cap,
        rotor,
        elem_a="front_bearing",
        elem_b="rotor_shaft",
        reason="The visible front bearing is a compact bushing proxy intentionally capturing the rotating shaft.",
    )
    ctx.allow_overlap(
        cap,
        rotor,
        elem_a="rear_bearing",
        elem_b="rotor_shaft",
        reason="The rear bearing proxy seats the same continuous rotor shaft for shaft-supported rotation.",
    )
    ctx.expect_within(rotor, cap, axes="xz", inner_elem="rotor_shaft", outer_elem="front_bearing", margin=0.0, name="front bearing surrounds shaft")
    ctx.expect_within(rotor, cap, axes="xz", inner_elem="rotor_shaft", outer_elem="rear_bearing", margin=0.0, name="rear bearing surrounds shaft")
    ctx.expect_overlap(rotor, cap, axes="y", elem_a="rotor_shaft", elem_b="front_bearing", min_overlap=0.015, name="shaft retained in front bearing")
    ctx.expect_overlap(rotor, cap, axes="y", elem_a="rotor_shaft", elem_b="rear_bearing", min_overlap=0.014, name="shaft retained in rear bearing")
    ctx.expect_within(cap, tower, axes="xy", inner_elem="yaw_stem", outer_elem="yaw_bearing", margin=0.0, name="yaw stem seated in bearing")

    rest_blade = ctx.part_world_position(blade_0)
    with ctx.pose({rotor_spin: math.pi / 2.0}):
        spun_blade = ctx.part_world_position(blade_0)
    ctx.check(
        "rotor spin carries blade around horizontal shaft",
        rest_blade is not None
        and spun_blade is not None
        and abs(spun_blade[0] - rest_blade[0]) > 0.025
        and spun_blade[2] < rest_blade[2] - 0.020,
        details=f"rest={rest_blade}, spun={spun_blade}",
    )

    with ctx.pose({yaw: 0.60}):
        yawed_rotor = ctx.part_world_position(rotor)
    rest_rotor = ctx.part_world_position(rotor)
    ctx.check(
        "cap yaws rotor as one nacelle stage",
        rest_rotor is not None
        and yawed_rotor is not None
        and abs(yawed_rotor[0] - rest_rotor[0]) > 0.040,
        details=f"rest={rest_rotor}, yawed={yawed_rotor}",
    )

    deployed_0 = ctx.part_world_aabb(blade_0)
    deployed_1 = ctx.part_world_aabb(blade_1)
    with ctx.pose({fold_0: 1.32, fold_1: 1.32, fold_2: 1.32, fold_3: 1.32}):
        stowed_0 = ctx.part_world_aabb(blade_0)
        stowed_1 = ctx.part_world_aabb(blade_1)
        ctx.expect_gap(cap, blade_0, axis="y", max_penetration=0.0, name="folded blade clears cap")
    ctx.check(
        "top blade folds forward for reduced height",
        deployed_0 is not None
        and stowed_0 is not None
        and (deployed_0[1][2] - deployed_0[0][2]) > (stowed_0[1][2] - stowed_0[0][2]) + 0.060,
        details=f"deployed={deployed_0}, stowed={stowed_0}",
    )
    ctx.check(
        "side blade folds forward for narrow stow width",
        deployed_1 is not None
        and stowed_1 is not None
        and (deployed_1[1][0] - deployed_1[0][0]) > (stowed_1[1][0] - stowed_1[0][0]) + 0.055,
        details=f"deployed={deployed_1}, stowed={stowed_1}",
    )

    return ctx.report()


object_model = build_object_model()
