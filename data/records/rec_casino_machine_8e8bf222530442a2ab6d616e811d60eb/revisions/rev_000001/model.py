from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _prism_from_yz_profile(width: float, yz_profile: list[tuple[float, float]]) -> MeshGeometry:
    """Closed x-extruded prism from a side profile written as (y, z) vertices."""
    half = width / 2.0
    vertices: list[tuple[float, float, float]] = []
    for x in (-half, half):
        for y, z in yz_profile:
            vertices.append((x, y, z))

    n = len(yz_profile)
    faces: list[tuple[int, int, int]] = []

    # -X cap and +X cap.
    for i in range(1, n - 1):
        faces.append((0, i, i + 1))
        faces.append((n, n + i + 1, n + i))

    # Side quads.
    for i in range(n):
        j = (i + 1) % n
        faces.append((i, j, n + j))
        faces.append((i, n + j, n + i))

    return MeshGeometry(vertices=vertices, faces=faces)


def _oriented_panel_on_front(
    width: float,
    height: float,
    thickness: float,
    center_yz: tuple[float, float],
    *,
    inset: float = 0.0,
) -> MeshGeometry:
    """Thin rectangular solid following the sloped front cabinet face."""
    # Unit vector up the front face from lower front to upper front.
    vy, vz = (0.055, 0.350)
    length = (vy * vy + vz * vz) ** 0.5
    vy, vz = vy / length, vz / length
    # Outward normal points toward the player, slightly upward.
    ny, nz = -vz, vy
    cy, cz = center_yz
    cy += ny * inset
    cz += nz * inset

    vertices: list[tuple[float, float, float]] = []
    for x in (-width / 2.0, width / 2.0):
        for s in (-height / 2.0, height / 2.0):
            for t in (-thickness / 2.0, thickness / 2.0):
                y = cy + vy * s + ny * t
                z = cz + vz * s + nz * t
                vertices.append((x, y, z))

    def idx(ix: int, is_: int, it: int) -> int:
        return ix * 4 + is_ * 2 + it

    quads = [
        (idx(0, 0, 0), idx(1, 0, 0), idx(1, 1, 0), idx(0, 1, 0)),
        (idx(0, 0, 1), idx(0, 1, 1), idx(1, 1, 1), idx(1, 0, 1)),
        (idx(0, 0, 0), idx(0, 0, 1), idx(1, 0, 1), idx(1, 0, 0)),
        (idx(0, 1, 0), idx(1, 1, 0), idx(1, 1, 1), idx(0, 1, 1)),
        (idx(0, 0, 0), idx(0, 1, 0), idx(0, 1, 1), idx(0, 0, 1)),
        (idx(1, 0, 0), idx(1, 0, 1), idx(1, 1, 1), idx(1, 1, 0)),
    ]
    faces: list[tuple[int, int, int]] = []
    for a, b, c, d in quads:
        faces.append((a, b, c))
        faces.append((a, c, d))
    return MeshGeometry(vertices=vertices, faces=faces)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bar_mounted_casino_machine")

    cabinet_mat = model.material("deep_blue_cabinet", color=(0.035, 0.050, 0.095, 1.0))
    shelf_mat = model.material("satin_black_shelf", color=(0.015, 0.016, 0.018, 1.0))
    bezel_mat = model.material("black_bezel", color=(0.0, 0.0, 0.0, 1.0))
    glass_mat = model.material("dim_green_glass", color=(0.02, 0.32, 0.24, 0.92))
    steel_mat = model.material("brushed_steel", color=(0.62, 0.60, 0.55, 1.0))
    tray_mat = model.material("coin_tray_metal", color=(0.36, 0.34, 0.30, 1.0))
    red_mat = model.material("casino_red", color=(0.78, 0.02, 0.02, 1.0))
    amber_mat = model.material("amber_button", color=(1.0, 0.58, 0.08, 1.0))

    cabinet = model.part("cabinet")

    # A compact, shallow wedge cabinet: narrow in depth, with the screen face
    # leaning back over a short bar-top shelf.
    side_profile = [
        (-0.160, 0.105),
        (0.130, 0.090),
        (0.130, 0.505),
        (-0.105, 0.490),
    ]
    cabinet.visual(
        mesh_from_geometry(_prism_from_yz_profile(0.420, side_profile), "cabinet_shell"),
        material=cabinet_mat,
        name="cabinet_shell",
    )
    cabinet.visual(
        mesh_from_geometry(
            _oriented_panel_on_front(0.350, 0.245, 0.016, (-0.128, 0.325), inset=0.000),
            "screen_bezel",
        ),
        material=bezel_mat,
        name="screen_bezel",
    )
    cabinet.visual(
        mesh_from_geometry(
            _oriented_panel_on_front(0.300, 0.185, 0.010, (-0.131, 0.325), inset=0.007),
            "screen_glass",
        ),
        material=glass_mat,
        name="screen_glass",
    )

    # Short front control shelf and a bar clamp/pedestal underneath it.
    cabinet.visual(
        Box((0.460, 0.160, 0.045)),
        origin=Origin(xyz=(0.0, -0.185, 0.115)),
        material=shelf_mat,
        name="button_shelf",
    )
    cabinet.visual(
        Box((0.180, 0.120, 0.014)),
        origin=Origin(xyz=(0.0, -0.135, 0.0855)),
        material=steel_mat,
        name="mounting_plate",
    )
    cabinet.visual(
        Cylinder(radius=0.030, length=0.072),
        origin=Origin(xyz=(0.0, -0.135, 0.043), rpy=(0.0, 0.0, 0.0)),
        material=steel_mat,
        name="bar_post",
    )
    cabinet.visual(
        Box((0.160, 0.115, 0.018)),
        origin=Origin(xyz=(0.0, -0.135, 0.002)),
        material=steel_mat,
        name="bar_clamp_pad",
    )
    cabinet.visual(
        Box((0.018, 0.115, 0.062)),
        origin=Origin(xyz=(-0.071, -0.135, 0.034)),
        material=steel_mat,
        name="clamp_cheek_0",
    )
    cabinet.visual(
        Box((0.018, 0.115, 0.062)),
        origin=Origin(xyz=(0.071, -0.135, 0.034)),
        material=steel_mat,
        name="clamp_cheek_1",
    )

    # Coin return tray below the shelf, with raised sides and a movable front flap.
    cabinet.visual(
        Box((0.320, 0.125, 0.012)),
        origin=Origin(xyz=(0.0, -0.198, 0.041)),
        material=tray_mat,
        name="tray_floor",
    )
    cabinet.visual(
        Box((0.014, 0.125, 0.058)),
        origin=Origin(xyz=(-0.153, -0.198, 0.069)),
        material=tray_mat,
        name="tray_side_0",
    )
    cabinet.visual(
        Box((0.014, 0.125, 0.058)),
        origin=Origin(xyz=(0.153, -0.198, 0.069)),
        material=tray_mat,
        name="tray_side_1",
    )
    cabinet.visual(
        Box((0.320, 0.014, 0.058)),
        origin=Origin(xyz=(0.0, -0.137, 0.069)),
        material=tray_mat,
        name="tray_back",
    )
    cabinet.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(-0.164, -0.263, 0.095), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_mat,
        name="tray_hinge_socket_0",
    )
    cabinet.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(0.164, -0.263, 0.095), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_mat,
        name="tray_hinge_socket_1",
    )

    # Rear hinge fixed leaf. The rotating panel carries the mating barrel.
    cabinet.visual(
        Box((0.020, 0.006, 0.340)),
        origin=Origin(xyz=(-0.198, 0.133, 0.302)),
        material=steel_mat,
        name="rear_hinge_leaf",
    )

    # Small printed label plaque makes the single-screen front read as a casino unit.
    cabinet.visual(
        mesh_from_geometry(
            _oriented_panel_on_front(0.200, 0.034, 0.008, (-0.153, 0.170), inset=-0.004),
            "payline_plaque",
        ),
        material=red_mat,
        name="payline_plaque",
    )

    selector_knob = model.part("selector_knob")
    selector_mesh = mesh_from_geometry(
        KnobGeometry(
            0.055,
            0.030,
            body_style="faceted",
            grip=KnobGrip(style="ribbed", count=16, depth=0.0012, width=0.0020),
            indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "selector_knob",
    )
    selector_knob.visual(
        selector_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=red_mat,
        name="selector_cap",
    )
    selector_knob.visual(
        Cylinder(radius=0.010, length=0.032),
        origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_mat,
        name="selector_shaft",
    )

    model.articulation(
        "cabinet_to_selector",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=selector_knob,
        origin=Origin(xyz=(0.230, -0.185, 0.124)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.7, velocity=9.0),
    )

    tray_flap = model.part("coin_tray_flap")
    tray_flap.visual(
        Box((0.300, 0.012, 0.075)),
        origin=Origin(xyz=(0.0, -0.006, -0.0435)),
        material=tray_mat,
        name="flap_plate",
    )
    tray_flap.visual(
        Cylinder(radius=0.006, length=0.286),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_mat,
        name="flap_pin",
    )
    model.articulation(
        "cabinet_to_tray_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=tray_flap,
        origin=Origin(xyz=(0.0, -0.263, 0.095)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.10),
    )

    service_panel = model.part("service_panel")
    service_panel.visual(
        Box((0.370, 0.012, 0.325)),
        origin=Origin(xyz=(0.185, 0.006, 0.1625)),
        material=cabinet_mat,
        name="service_door",
    )
    service_panel.visual(
        Box((0.030, 0.008, 0.300)),
        origin=Origin(xyz=(0.016, 0.007, 0.162)),
        material=steel_mat,
        name="service_hinge_leaf",
    )
    service_panel.visual(
        Cylinder(radius=0.006, length=0.340),
        origin=Origin(xyz=(0.0, 0.006, 0.170)),
        material=steel_mat,
        name="service_hinge_pin",
    )
    service_panel.visual(
        Box((0.055, 0.010, 0.020)),
        origin=Origin(xyz=(0.315, 0.014, 0.160)),
        material=steel_mat,
        name="service_pull",
    )
    for i, z in enumerate((0.085, 0.105, 0.125, 0.235, 0.255, 0.275)):
        service_panel.visual(
            Box((0.090, 0.004, 0.006)),
            origin=Origin(xyz=(0.210, 0.014, z)),
            material=bezel_mat,
            name=f"vent_slot_{i}",
        )
    model.articulation(
        "cabinet_to_service_panel",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=service_panel,
        origin=Origin(xyz=(-0.198, 0.132, 0.140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.85),
    )

    # Three short, real pushbuttons on the shelf. They are separate prismatic
    # controls rather than painted bumps.
    for i, x in enumerate((-0.080, 0.0, 0.080)):
        button = model.part(f"button_{i}")
        mat = amber_mat if i != 1 else red_mat
        button.visual(
            Cylinder(radius=0.023, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
            material=mat,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=bezel_mat,
            name="button_bezel",
        )
        model.articulation(
            f"cabinet_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x, -0.195, 0.1375)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=1.0, velocity=0.2, lower=0.0, upper=0.004),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    selector = object_model.get_part("selector_knob")
    flap = object_model.get_part("coin_tray_flap")
    service = object_model.get_part("service_panel")
    selector_joint = object_model.get_articulation("cabinet_to_selector")
    flap_joint = object_model.get_articulation("cabinet_to_tray_flap")
    service_joint = object_model.get_articulation("cabinet_to_service_panel")

    ctx.check(
        "primary mechanisms are articulated",
        selector_joint.articulation_type == ArticulationType.CONTINUOUS
        and flap_joint.articulation_type == ArticulationType.REVOLUTE
        and service_joint.articulation_type == ArticulationType.REVOLUTE,
        details=(
            f"selector={selector_joint.articulation_type}, "
            f"flap={flap_joint.articulation_type}, service={service_joint.articulation_type}"
        ),
    )
    ctx.expect_gap(
        selector,
        cabinet,
        axis="x",
        min_gap=-0.0005,
        max_gap=0.004,
        positive_elem="selector_cap",
        negative_elem="button_shelf",
        name="selector knob is seated on shelf side",
    )
    ctx.expect_overlap(
        selector,
        cabinet,
        axes="yz",
        min_overlap=0.020,
        elem_a="selector_cap",
        elem_b="button_shelf",
        name="selector shaft aligns with shelf side",
    )
    ctx.expect_gap(
        cabinet,
        flap,
        axis="z",
        min_gap=0.0,
        max_gap=0.004,
        positive_elem="tray_hinge_socket_0",
        negative_elem="flap_plate",
        name="tray flap top edge hangs from the front hinge",
    )
    ctx.expect_gap(
        service,
        cabinet,
        axis="y",
        min_gap=0.0,
        max_gap=0.010,
        positive_elem="service_door",
        negative_elem="cabinet_shell",
        name="rear service panel sits just behind cabinet",
    )

    flap_closed = ctx.part_world_aabb(flap)
    with ctx.pose({flap_joint: 0.85}):
        flap_open = ctx.part_world_aabb(flap)
    ctx.check(
        "coin tray flap swings outward on its front hinge",
        flap_closed is not None
        and flap_open is not None
        and flap_open[0][1] < flap_closed[0][1] - 0.020,
        details=f"closed={flap_closed}, open={flap_open}",
    )

    service_closed = ctx.part_world_aabb(service)
    with ctx.pose({service_joint: 1.20}):
        service_open = ctx.part_world_aabb(service)
    ctx.check(
        "rear service panel swings behind the machine",
        service_closed is not None
        and service_open is not None
        and service_open[1][1] > service_closed[1][1] + 0.150,
        details=f"closed={service_closed}, open={service_open}",
    )

    return ctx.report()


object_model = build_object_model()
